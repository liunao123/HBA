#pragma once
// -----------------------------------------------------------------------------
// 最小 PCD 读写器 (binary / ascii)
//
// 不依赖 PCL: 本机 apt 装的 vtk-9.1 CMake 配置是坏的, PCLConfig 一旦请求 io 组件就会
// 强制 find_package(VTK) 而失败 (lt-mapper/session_align/CMakeLists.txt 里有同样记录)。
// WG_wuling 数据集的 fuse_lidar PCD 格式固定为:
//   FIELDS x y z intensity timestamp sensor_id
//   SIZE   4 4 4 4         8         4
//   TYPE   F F F U         F         U
//   DATA   binary
// 但这里仍按 header 解析字段偏移, 以免个别文件字段顺序不同。
// -----------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <atomic>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace ialign {

struct PcdCloud {
  std::vector<Eigen::Vector4d> points;  // (x, y, z, 1)
  std::vector<double> intensities;      // 可能为空
};

namespace detail {

struct PcdField {
  std::string name;
  int size = 0;
  char type = 'F';
  int count = 1;
  int offset = 0;
};

inline std::vector<std::string> splitWs(const std::string& s) {
  std::istringstream iss(s);
  std::vector<std::string> out;
  std::string tok;
  while (iss >> tok) out.push_back(tok);
  return out;
}

/// @brief 按 type/size 读一个标量并转成 double
inline double readScalar(const char* p, char type, int size) {
  switch (type) {
    case 'F':
      if (size == 4) {
        float v;
        std::memcpy(&v, p, 4);
        return v;
      }
      if (size == 8) {
        double v;
        std::memcpy(&v, p, 8);
        return v;
      }
      break;
    case 'U': {
      std::uint64_t v = 0;
      std::memcpy(&v, p, std::min(size, 8));
      return static_cast<double>(v);
    }
    case 'I': {
      std::int64_t v = 0;
      // 有符号需要按宽度做符号扩展
      if (size == 1) {
        std::int8_t t;
        std::memcpy(&t, p, 1);
        v = t;
      } else if (size == 2) {
        std::int16_t t;
        std::memcpy(&t, p, 2);
        v = t;
      } else if (size == 4) {
        std::int32_t t;
        std::memcpy(&t, p, 4);
        v = t;
      } else if (size == 8) {
        std::memcpy(&v, p, 8);
      }
      return static_cast<double>(v);
    }
    default:
      break;
  }
  return 0.0;
}

/// @brief LZF 解压 (liblzf 的 lzf_decompress, PCL 的 binary_compressed 用的就是它)。
///        算法本身很短, 但两处边界不能省: 越界写和 back-reference 指到缓冲区之前,
///        都会在损坏文件上变成静默的内存破坏而不是报错。
inline std::size_t lzfDecompress(
  const unsigned char* in, std::size_t in_len, unsigned char* out, std::size_t out_len) {
  const unsigned char* ip = in;
  const unsigned char* const in_end = in + in_len;
  unsigned char* op = out;
  unsigned char* const out_end = out + out_len;

  while (ip < in_end) {
    unsigned int ctrl = *ip++;
    if (ctrl < (1u << 5)) {
      // 字面量: 后面紧跟 ctrl+1 个原始字节
      ctrl++;
      if (op + ctrl > out_end) return 0;
      if (ip + ctrl > in_end) return 0;
      std::memcpy(op, ip, ctrl);
      op += ctrl;
      ip += ctrl;
    } else {
      // 回溯引用: 高 3 位是长度, 低 5 位是偏移的高位
      std::size_t len = ctrl >> 5;
      if (ip >= in_end) return 0;
      std::size_t ref_back = ((ctrl & 0x1fu) << 8) + 1;
      if (len == 7) {
        len += *ip++;
        if (ip >= in_end) return 0;
      }
      ref_back += *ip++;
      if (static_cast<std::size_t>(op - out) < ref_back) return 0;  // 指到缓冲区之前
      unsigned char* ref = op - ref_back;
      if (op + len + 2 > out_end) return 0;
      // 必须逐字节复制: 源和目标可能重叠(这正是 LZF 表达重复模式的方式), memcpy 不行
      *op++ = *ref++;
      *op++ = *ref++;
      while (len--) *op++ = *ref++;
    }
  }
  return static_cast<std::size_t>(op - out);
}

}  // namespace detail

/// @brief 读取 PCD。失败返回 false。
/// @param want_intensity 是否解析 intensity 字段
inline bool loadPcd(const std::string& path, PcdCloud& out, bool want_intensity = true) {
  out.points.clear();
  out.intensities.clear();

  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    // 限流: 数据盘掉线时这里会被调几万次, 刷屏本身就掩盖了问题 (实测一次掉线 34566 行,
    // 而且多线程交错输出到半行)。只打前 20 条, 之后每 5000 条报一次累计。
    // 真正的把关在 local_align 的 guardIo: 失败率超阈值就中止且不写存档。
    static std::atomic<std::size_t> n_err{0};
    const std::size_t e = n_err.fetch_add(1) + 1;
    if (e <= 20) {
      std::cerr << "[pcd] cannot open " << path << "\n";
    } else if (e == 21) {
      std::cerr << "[pcd] cannot open ... (后续同类错误不再逐条打印)\n";
    } else if (e % 5000 == 0) {
      std::cerr << "[pcd] cannot open: 累计 " << e << " 次 —— 数据盘可能掉线了\n";
    }
    return false;
  }

  std::vector<detail::PcdField> fields;
  std::vector<std::string> names;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  std::size_t num_points = 0;
  std::string data_kind;

  // ---- header ----
  std::string line;
  while (std::getline(ifs, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;

    const auto tok = detail::splitWs(line);
    if (tok.empty()) continue;
    const std::string& key = tok[0];

    if (key == "FIELDS") {
      names.assign(tok.begin() + 1, tok.end());
    } else if (key == "SIZE") {
      for (size_t i = 1; i < tok.size(); i++) sizes.push_back(std::stoi(tok[i]));
    } else if (key == "TYPE") {
      for (size_t i = 1; i < tok.size(); i++) types.push_back(tok[i][0]);
    } else if (key == "COUNT") {
      for (size_t i = 1; i < tok.size(); i++) counts.push_back(std::stoi(tok[i]));
    } else if (key == "POINTS") {
      num_points = std::stoull(tok[1]);
    } else if (key == "WIDTH" && num_points == 0) {
      num_points = std::stoull(tok[1]);  // 兜底: 有些文件 POINTS 在 WIDTH*HEIGHT 之后
    } else if (key == "DATA") {
      data_kind = tok.size() > 1 ? tok[1] : "";
      break;
    }
  }

  if (names.empty() || sizes.size() != names.size() || types.size() != names.size()) {
    std::cerr << "[pcd] malformed header: " << path << "\n";
    return false;
  }
  if (counts.empty()) counts.assign(names.size(), 1);

  int stride = 0;
  for (size_t i = 0; i < names.size(); i++) {
    detail::PcdField f;
    f.name = names[i];
    f.size = sizes[i];
    f.type = types[i];
    f.count = i < counts.size() ? counts[i] : 1;
    f.offset = stride;
    stride += f.size * f.count;
    fields.push_back(f);
  }

  const detail::PcdField* fx = nullptr;
  const detail::PcdField* fy = nullptr;
  const detail::PcdField* fz = nullptr;
  const detail::PcdField* fi = nullptr;
  for (const auto& f : fields) {
    if (f.name == "x") fx = &f;
    else if (f.name == "y") fy = &f;
    else if (f.name == "z") fz = &f;
    else if (f.name == "intensity") fi = &f;
  }
  if (!fx || !fy || !fz) {
    std::cerr << "[pcd] missing x/y/z: " << path << "\n";
    return false;
  }
  if (!want_intensity) fi = nullptr;

  // ---- body ----
  if (data_kind == "binary") {
    std::vector<char> buf(static_cast<std::size_t>(stride) * num_points);
    ifs.read(buf.data(), static_cast<std::streamsize>(buf.size()));
    const std::size_t got = static_cast<std::size_t>(ifs.gcount());
    const std::size_t n = got / static_cast<std::size_t>(stride);
    if (n != num_points) {
      // 文件被截断时按实际读到的点数处理, 不直接失败
      std::cerr << "[pcd] short read " << path << " (" << n << "/" << num_points << ")\n";
    }

    out.points.reserve(n);
    if (fi) out.intensities.reserve(n);
    for (std::size_t i = 0; i < n; i++) {
      const char* p = buf.data() + i * static_cast<std::size_t>(stride);
      const double x = detail::readScalar(p + fx->offset, fx->type, fx->size);
      const double y = detail::readScalar(p + fy->offset, fy->type, fy->size);
      const double z = detail::readScalar(p + fz->offset, fz->type, fz->size);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      out.points.emplace_back(x, y, z, 1.0);
      if (fi) out.intensities.push_back(detail::readScalar(p + fi->offset, fi->type, fi->size));
    }
    return true;
  }

  if (data_kind == "ascii") {
    out.points.reserve(num_points);
    // ascii 下按 FIELDS 顺序逐列解析
    int ix = -1, iy = -1, iz = -1, ii = -1;
    for (size_t i = 0; i < fields.size(); i++) {
      if (fields[i].name == "x") ix = static_cast<int>(i);
      else if (fields[i].name == "y") iy = static_cast<int>(i);
      else if (fields[i].name == "z") iz = static_cast<int>(i);
      else if (fields[i].name == "intensity") ii = static_cast<int>(i);
    }
    while (std::getline(ifs, line)) {
      const auto tok = detail::splitWs(line);
      if (static_cast<int>(tok.size()) <= std::max(ix, std::max(iy, iz))) continue;
      try {
        const double x = std::stod(tok[ix]);
        const double y = std::stod(tok[iy]);
        const double z = std::stod(tok[iz]);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        out.points.emplace_back(x, y, z, 1.0);
        if (fi && ii >= 0 && ii < static_cast<int>(tok.size())) out.intensities.push_back(std::stod(tok[ii]));
      } catch (...) {
        continue;
      }
    }
    return true;
  }

  if (data_kind == "binary_compressed") {
    // PCL 的 binary_compressed: 4 字节压缩后长度 + 4 字节原始长度 + LZF 数据。
    // !! 解压后的排布是**按字段连续**(SoA), 不是按点连续 !!
    //    即先所有点的 x, 再所有点的 y ... 与 binary 的 AoS 完全不同。
    //    照 binary 的 stride 去索引会读出一堆看似合理但完全错乱的坐标 —— 不会报错, 只会静默错。
    std::uint32_t csize = 0, usize = 0;
    ifs.read(reinterpret_cast<char*>(&csize), 4);
    ifs.read(reinterpret_cast<char*>(&usize), 4);
    if (!ifs || csize == 0 || usize == 0) {
      std::cerr << "[pcd] binary_compressed 头部损坏: " << path << "\n";
      return false;
    }
    std::vector<unsigned char> cbuf(csize), ubuf(usize);
    ifs.read(reinterpret_cast<char*>(cbuf.data()), csize);
    if (static_cast<std::uint32_t>(ifs.gcount()) != csize) {
      std::cerr << "[pcd] binary_compressed 数据被截断: " << path << "\n";
      return false;
    }
    const std::size_t got = detail::lzfDecompress(cbuf.data(), csize, ubuf.data(), usize);
    if (got != usize) {
      std::cerr << "[pcd] LZF 解压失败 " << path << " (" << got << "/" << usize << ")\n";
      return false;
    }

    // 每个字段的起始偏移 = 前面所有字段占的总字节数 (count 只支持 1)
    std::size_t off = 0;
    std::vector<std::size_t> foff(fields.size(), 0);
    for (std::size_t i = 0; i < fields.size(); i++) {
      foff[i] = off;
      off += static_cast<std::size_t>(fields[i].size) * num_points;
    }
    if (off > usize) {
      std::cerr << "[pcd] binary_compressed 字段总长 " << off << " > 解压长度 " << usize << ": " << path << "\n";
      return false;
    }
    const auto base = [&](const detail::PcdField* f) -> const char* {
      if (!f) return nullptr;
      for (std::size_t i = 0; i < fields.size(); i++) {
        if (&fields[i] == f) return reinterpret_cast<const char*>(ubuf.data()) + foff[i];
      }
      return nullptr;
    };
    const char* bx = base(fx);
    const char* by = base(fy);
    const char* bz = base(fz);
    const char* bi = base(fi);
    if (!bx || !by || !bz) return false;

    out.points.reserve(num_points);
    if (bi) out.intensities.reserve(num_points);
    for (std::size_t i = 0; i < num_points; i++) {
      const double x = detail::readScalar(bx + i * fx->size, fx->type, fx->size);
      const double y = detail::readScalar(by + i * fy->size, fy->type, fy->size);
      const double z = detail::readScalar(bz + i * fz->size, fz->type, fz->size);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      out.points.emplace_back(x, y, z, 1.0);
      if (bi) out.intensities.push_back(detail::readScalar(bi + i * fi->size, fi->type, fi->size));
    }
    return true;
  }

  std::cerr << "[pcd] unsupported DATA type '" << data_kind << "' in " << path << "\n";
  return false;
}

/// @brief 写 binary PCD (x y z float + 可选 intensity float)
inline bool savePcdBinary(
  const std::string& path,
  const std::vector<Eigen::Vector4d>& points,
  const std::vector<double>& intensities = {}) {
  const bool has_i = intensities.size() == points.size() && !intensities.empty();

  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) {
    std::cerr << "[pcd] cannot write " << path << "\n";
    return false;
  }

  ofs << "# .PCD v0.7 - Point Cloud Data file format\n";
  ofs << "VERSION 0.7\n";
  ofs << (has_i ? "FIELDS x y z intensity\n" : "FIELDS x y z\n");
  ofs << (has_i ? "SIZE 4 4 4 4\n" : "SIZE 4 4 4\n");
  ofs << (has_i ? "TYPE F F F F\n" : "TYPE F F F\n");
  ofs << (has_i ? "COUNT 1 1 1 1\n" : "COUNT 1 1 1\n");
  ofs << "WIDTH " << points.size() << "\n";
  ofs << "HEIGHT 1\n";
  ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";
  ofs << "POINTS " << points.size() << "\n";
  ofs << "DATA binary\n";

  const int stride = has_i ? 16 : 12;
  std::vector<char> buf(static_cast<std::size_t>(stride) * points.size());
  for (std::size_t i = 0; i < points.size(); i++) {
    char* p = buf.data() + i * static_cast<std::size_t>(stride);
    const float x = static_cast<float>(points[i].x());
    const float y = static_cast<float>(points[i].y());
    const float z = static_cast<float>(points[i].z());
    std::memcpy(p + 0, &x, 4);
    std::memcpy(p + 4, &y, 4);
    std::memcpy(p + 8, &z, 4);
    if (has_i) {
      const float v = static_cast<float>(intensities[i]);
      std::memcpy(p + 12, &v, 4);
    }
  }
  ofs.write(buf.data(), static_cast<std::streamsize>(buf.size()));
  return ofs.good();
}

/// @brief 写带颜色的 PCD。字段用 PCL 的老约定 `FIELDS x y z rgb` + `TYPE F`, 其中那 4 个
///        字节实际是 0x00RRGGBB 的 uint32 按位重解释成 float —— 这是 pcl_viewer 和
///        CloudCompare 都认的写法。写成 `TYPE U` 或拆成 r/g/b 三个字段, CloudCompare
///        会当成三个标量字段而不上色。
inline bool savePcdRgb(
  const std::string& path,
  const std::vector<Eigen::Vector4d>& points,
  const std::vector<Eigen::Matrix<std::uint8_t, 3, 1>>& rgb) {
  if (rgb.size() != points.size()) {
    std::cerr << "[pcd] savePcdRgb: 颜色数 " << rgb.size() << " != 点数 " << points.size() << "\n";
    return false;
  }
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) {
    std::cerr << "[pcd] cannot write " << path << "\n";
    return false;
  }
  ofs << "# .PCD v0.7 - Point Cloud Data file format\n";
  ofs << "VERSION 0.7\n";
  ofs << "FIELDS x y z rgb\n";
  ofs << "SIZE 4 4 4 4\n";
  ofs << "TYPE F F F F\n";
  ofs << "COUNT 1 1 1 1\n";
  ofs << "WIDTH " << points.size() << "\n";
  ofs << "HEIGHT 1\n";
  ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";
  ofs << "POINTS " << points.size() << "\n";
  ofs << "DATA binary\n";

  std::vector<char> buf(16 * points.size());
  for (std::size_t i = 0; i < points.size(); i++) {
    char* p = buf.data() + i * 16;
    const float x = static_cast<float>(points[i].x());
    const float y = static_cast<float>(points[i].y());
    const float z = static_cast<float>(points[i].z());
    std::memcpy(p + 0, &x, 4);
    std::memcpy(p + 4, &y, 4);
    std::memcpy(p + 8, &z, 4);
    const std::uint32_t c = (static_cast<std::uint32_t>(rgb[i][0]) << 16) |
                            (static_cast<std::uint32_t>(rgb[i][1]) << 8) |
                            static_cast<std::uint32_t>(rgb[i][2]);
    std::memcpy(p + 12, &c, 4);
  }
  ofs.write(buf.data(), static_cast<std::streamsize>(buf.size()));
  return ofs.good();
}

/// @brief 写 LAS 1.2 / Point Data Format 2 (x y z + intensity + RGB, 26 字节/点)。
///
/// 坐标写**绝对 UTM**: 传进来的 pts 是相对 utm_offset 的局部坐标, 这里加回去。
/// LAS 的 scale/offset 机制天生就是干这个的 —— 文件里存的是 int32 的小数值(1mm 刻度),
/// 阅读器解出来是绝对坐标, 两边都不损失精度。int32 在 1mm 刻度下可表达 ±2147km。
///
/// @param ints  强度; 长度与 pts 一致才写, 否则全 0。这批数据实测是 0~255,
///              直接放进 uint16 不缩放 —— 缩放会让"强度值"不再是原始测量值。
/// @param rgb   颜色; 长度与 pts 一致才写, 否则全 0。8 位左移 8 位填进 uint16
///              (LAS 的颜色字段是 16 位, 这是通行做法)。
/// @param utm_zone  >0 才写 GeoKeyDirectory VLR (EPSG = 32600 + zone, WGS84 UTM 北半球)。
///              **0 = 不写 CRS**, 坐标仍是绝对 UTM, 只是不声明属于哪个带。
///              为什么不自动推: 一个点相对自己带的中央经线偏移总在 ±3° 内, 所以任何带号
///              都"自洽" —— 只凭 (easting, northing) 在数学上无法确定带号。猜错会让整张图
///              落到地球上错误的位置, 而且不会有任何报错。必须由外部给。
inline bool saveLas(const std::string& path,
                    const std::vector<Eigen::Vector4d>& pts,
                    const std::vector<double>& ints,
                    const std::vector<Eigen::Matrix<std::uint8_t, 3, 1>>& rgb,
                    const Eigen::Vector2d& utm_offset,
                    int utm_zone = 0) {
  if (pts.empty()) return false;
  std::ofstream o(path, std::ios::binary);
  if (!o) {
    std::cerr << "[las] cannot write " << path << "\n";
    return false;
  }
  const bool has_i = ints.size() == pts.size();
  const bool has_c = rgb.size() == pts.size();

  double xmn = 1e18, xmx = -1e18, ymn = 1e18, ymx = -1e18, zmn = 1e18, zmx = -1e18;
  for (const auto& p : pts) {
    const double ux = p.x() + utm_offset.x(), uy = p.y() + utm_offset.y(), uz = p.z();
    xmn = std::min(xmn, ux); xmx = std::max(xmx, ux);
    ymn = std::min(ymn, uy); ymx = std::max(ymx, uy);
    zmn = std::min(zmn, uz); zmx = std::max(zmx, uz);
  }
  const double sc = 0.001;                 // 1mm
  const double xo = std::floor(xmn), yo = std::floor(ymn), zo = std::floor(zmn);

  const auto w1 = [&](std::uint8_t v) { o.write(reinterpret_cast<const char*>(&v), 1); };
  const auto w2 = [&](std::uint16_t v) { o.write(reinterpret_cast<const char*>(&v), 2); };
  const auto w4u = [&](std::uint32_t v) { o.write(reinterpret_cast<const char*>(&v), 4); };
  const auto w4s = [&](std::int32_t v) { o.write(reinterpret_cast<const char*>(&v), 4); };
  const auto w8 = [&](double v) { o.write(reinterpret_cast<const char*>(&v), 8); };

  const bool has_zone = utm_zone > 0;
  // 头 227 字节; VLR 54(头)+32(数据)=86 -> 有 VLR 时点数据从 313 开始
  const std::uint32_t pt_offset = has_zone ? 313u : 227u;

  // ---- Public Header Block (必须正好 227 字节) ----
  o.write("LASF", 4);
  w2(0); w2(0);                                  // File Source ID, Global Encoding
  w4u(0); w2(0); w2(0);                          // Project ID GUID 1/2/3
  { char g[8] = {}; o.write(g, 8); }             // GUID 4
  w1(1); w1(2);                                  // Version 1.2
  { char v[32] = {}; o.write(v, 32); }           // System Identifier
  { char v[32] = "incremental_align/local_align"; o.write(v, 32); }
  w2(0); w2(0);                                  // Creation Day, Year
  w2(227);                                       // Header Size
  w4u(pt_offset);
  w4u(has_zone ? 1u : 0u);                       // Number of VLRs
  w1(2);                                         // Point Data Format 2
  w2(26);                                        // Record Length
  w4u(static_cast<std::uint32_t>(pts.size()));
  for (int r = 0; r < 5; r++) w4u(r == 0 ? static_cast<std::uint32_t>(pts.size()) : 0u);
  w8(sc); w8(sc); w8(sc);
  w8(xo); w8(yo); w8(zo);
  w8(xmx); w8(xmn);
  w8(ymx); w8(ymn);
  w8(zmx); w8(zmn);

  // ---- GeoKeyDirectory VLR (86 字节, 只在给了带号时写) ----
  if (has_zone) {
    w2(0);                                       // reserved
    { char u[16] = "LASF_Projection"; o.write(u, 16); }
    w2(34735);                                   // GeoKeyDirectoryTag
    w2(32);                                      // record length after header
    { char d[32] = "GeoKeyDirectory"; o.write(d, 32); }
    w2(1); w2(1); w2(0); w2(3);                  // version/revision/minor/nkeys
    w2(1024); w2(0); w2(1); w2(1);               // GTModelType = Projected
    w2(1025); w2(0); w2(1); w2(1);               // GTRasterType = PixelIsArea
    w2(3072); w2(0); w2(1);
    w2(static_cast<std::uint16_t>(32600 + utm_zone));   // ProjectedCSType = EPSG
  }

  // ---- Point Data Records ----
  for (std::size_t k = 0; k < pts.size(); k++) {
    w4s(static_cast<std::int32_t>(std::lround((pts[k].x() + utm_offset.x() - xo) / sc)));
    w4s(static_cast<std::int32_t>(std::lround((pts[k].y() + utm_offset.y() - yo) / sc)));
    w4s(static_cast<std::int32_t>(std::lround((pts[k].z() - zo) / sc)));
    // 四舍五入不截断: 体素导出的强度是格内均值(小数), 截断会系统性地偏低最多 1
    w2(has_i ? static_cast<std::uint16_t>(std::lround(std::min(65535.0, std::max(0.0, ints[k]))))
             : 0);
    w1(0); w1(0);                                // return bits, classification
    w1(0); w1(0);                                // scan angle, user data
    w2(0);                                       // point source id
    if (has_c) {
      w2(static_cast<std::uint16_t>(rgb[k][0]) << 8);
      w2(static_cast<std::uint16_t>(rgb[k][1]) << 8);
      w2(static_cast<std::uint16_t>(rgb[k][2]) << 8);
    } else {
      w2(0); w2(0); w2(0);
    }
  }
  return o.good();
}

// -----------------------------------------------------------------------------
// 流式写入器 —— 导出**原始点**(不做体素滤波)时必须用它。
//
// 为什么: 全量原始点是 2446 帧 x 6.7 万点 = 1.69 亿点。先攒进 vector 再写要
//   坐标 1.69e8 x 32B = 5.4GB + 强度 1.4GB + 颜色 0.5GB > 7GB, 还没算中间副本 ——
//   这个项目已经被 OOM 看守终止过四次。流式写的峰值内存只和**单个分块**有关。
//
// 两种格式的头部都要先写、后回填(点数/包围盒), 手法不同:
//   PCD  头是 ASCII, 点数字段先写成定宽的零填充数字, 结束时 seek 回去覆盖。
//        (前导零对 stoull/atoi 无害, PCL 和 CloudCompare 都能读)
//   LAS  头是定长二进制, 点数在 107/111 偏移、包围盒在 179 偏移, 结束时 seek 回去写。
//        **scale/offset 必须一开始就定** (点是相对 offset 的 int32), 所以 offset 直接
//        取 floor(utm_center) —— 不需要预先扫一遍数据, 而 int32 在 1mm 刻度下能表达
//        +-2147km, 相对 utm_center 的局部坐标最多几公里, 完全够。
// -----------------------------------------------------------------------------

/// @brief 流式 PCD 写入。带强度或带 rgb 二选一 (PCD 里两者字段布局不同)。
class PcdStream {
 public:
  /// @param with_rgb true = FIELDS x y z rgb; false = FIELDS x y z intensity
  bool open(const std::string& path, bool with_rgb) {
    rgb_ = with_rgb;
    os_.open(path, std::ios::binary);
    if (!os_) {
      std::cerr << "[pcd] cannot write " << path << "\n";
      return false;
    }
    os_ << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n";
    os_ << (rgb_ ? "FIELDS x y z rgb\n" : "FIELDS x y z intensity\n");
    os_ << "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH ";
    wpos_ = os_.tellp();
    os_ << "000000000000\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ";
    ppos_ = os_.tellp();
    os_ << "000000000000\nDATA binary\n";
    return true;
  }
  /// @param ok 与 pts 等长; 只在 rgb 模式下有意义 —— ok[k]=0 且 drop=true 时跳过该点
  void add(const std::vector<Eigen::Vector4d>& pts, const std::vector<double>* ints,
           const std::vector<Eigen::Matrix<std::uint8_t, 3, 1>>* rgb,
           const std::vector<char>* ok = nullptr, bool drop = false) {
    const bool hi = ints && ints->size() == pts.size();
    const bool hc = rgb && rgb->size() == pts.size();
    buf_.clear();
    buf_.reserve(pts.size() * 4);
    for (std::size_t k = 0; k < pts.size(); k++) {
      if (rgb_ && drop && ok && !(*ok)[k]) continue;
      buf_.push_back(static_cast<float>(pts[k].x()));
      buf_.push_back(static_cast<float>(pts[k].y()));
      buf_.push_back(static_cast<float>(pts[k].z()));
      if (rgb_) {
        const std::uint32_t v = hc ? ((static_cast<std::uint32_t>((*rgb)[k][0]) << 16) |
                                      (static_cast<std::uint32_t>((*rgb)[k][1]) << 8) |
                                      static_cast<std::uint32_t>((*rgb)[k][2]))
                                   : 0u;
        float f;
        std::memcpy(&f, &v, 4);
        buf_.push_back(f);
      } else {
        buf_.push_back(hi ? static_cast<float>((*ints)[k]) : 0.0f);
      }
      n_++;
    }
    os_.write(reinterpret_cast<const char*>(buf_.data()),
              static_cast<std::streamsize>(buf_.size() * 4));
  }
  /// @brief 回填 WIDTH / POINTS
  bool finish() {
    if (!os_) return false;
    char b[13];
    std::snprintf(b, sizeof(b), "%012zu", n_);
    os_.seekp(wpos_);
    os_.write(b, 12);
    os_.seekp(ppos_);
    os_.write(b, 12);
    os_.close();
    return true;
  }
  std::size_t size() const { return n_; }

 private:
  std::ofstream os_;
  std::streampos wpos_{}, ppos_{};
  std::size_t n_ = 0;
  bool rgb_ = false;
  std::vector<float> buf_;
};

/// @brief 流式 LAS 1.2 / Point Format 2 写入 (xyz + 强度 + 颜色, 绝对 UTM)。
/// 参数含义与 saveLas 一致, 见那里的注释 (特别是 utm_zone 为什么不能自动推)。
class LasStream {
 public:
  bool open(const std::string& path, const Eigen::Vector2d& utm_offset, int utm_zone = 0) {
    os_.open(path, std::ios::binary);
    if (!os_) {
      std::cerr << "[las] cannot write " << path << "\n";
      return false;
    }
    uo_ = utm_offset;
    // offset 一开始就定死: 不需要预扫数据。取 floor(utm_center) 而不是 floor(实际 min),
    // 差别只是 int32 的取值范围偏了几公里, 而它能表达 +-2147km。
    xo_ = std::floor(utm_offset.x());
    yo_ = std::floor(utm_offset.y());
    zo_ = 0.0;
    const bool hz = utm_zone > 0;
    const std::uint32_t pt_off = hz ? 313u : 227u;
    const auto w1 = [&](std::uint8_t v) { os_.write(reinterpret_cast<const char*>(&v), 1); };
    const auto w2 = [&](std::uint16_t v) { os_.write(reinterpret_cast<const char*>(&v), 2); };
    const auto w4 = [&](std::uint32_t v) { os_.write(reinterpret_cast<const char*>(&v), 4); };
    const auto w8 = [&](double v) { os_.write(reinterpret_cast<const char*>(&v), 8); };
    os_.write("LASF", 4);
    w2(0); w2(0); w4(0); w2(0); w2(0);
    { char g[8] = {}; os_.write(g, 8); }
    w1(1); w1(2);
    { char v[32] = {}; os_.write(v, 32); }
    { char v[32] = "incremental_align/local_align"; os_.write(v, 32); }
    w2(0); w2(0); w2(227);
    w4(pt_off);
    w4(hz ? 1u : 0u);
    w1(2); w2(26);
    w4(0);                                  // 点数, finish() 回填 (偏移 107)
    for (int r = 0; r < 5; r++) w4(0);       // by-return 点数 (偏移 111)
    w8(kSc); w8(kSc); w8(kSc);
    w8(xo_); w8(yo_); w8(zo_);
    for (int r = 0; r < 6; r++) w8(0.0);     // 包围盒, finish() 回填 (偏移 179)
    if (hz) {
      w2(0);
      { char u[16] = "LASF_Projection"; os_.write(u, 16); }
      w2(34735); w2(32);
      { char d[32] = "GeoKeyDirectory"; os_.write(d, 32); }
      w2(1); w2(1); w2(0); w2(3);
      w2(1024); w2(0); w2(1); w2(1);
      w2(1025); w2(0); w2(1); w2(1);
      w2(3072); w2(0); w2(1);
      w2(static_cast<std::uint16_t>(32600 + utm_zone));
    }
    return true;
  }
  /// @param ok   与 pts 等长; ok[k]=0 = 这个点没投到任何图像上
  /// @param drop true = **未上色的点直接不写** (需要 rgb 和 ok 都给了才生效)。
  ///             为什么要有这个: 不删的话它们以黑色写进 las, 而 las 里颜色和强度并存 ——
  ///             按颜色渲染时那些黑点会盖在真实结构上, 看起来像洞或阴影。单相机只覆盖
  ///             前视一小片, 所以"未上色"是多数而不是少数。
  void add(const std::vector<Eigen::Vector4d>& pts, const std::vector<double>* ints,
           const std::vector<Eigen::Matrix<std::uint8_t, 3, 1>>* rgb,
           const std::vector<char>* ok = nullptr, bool drop = false) {
    const bool hi = ints && ints->size() == pts.size();
    const bool hc = rgb && rgb->size() == pts.size();
    // drop 只在真的有颜色和 ok 时生效 —— 否则 --no_color 下会把所有点删光
    const bool do_drop = drop && hc && ok && ok->size() == pts.size();
    buf_.clear();
    buf_.reserve(pts.size() * 26);
    const auto put = [&](const void* p, std::size_t n) {
      const char* c = static_cast<const char*>(p);
      buf_.insert(buf_.end(), c, c + n);
    };
    for (std::size_t k = 0; k < pts.size(); k++) {
      // 必须在更新包围盒和 n_ **之前**跳过, 否则被删的点仍会撑大 las 头里的包围盒
      if (do_drop && !(*ok)[k]) continue;
      const double ux = pts[k].x() + uo_.x(), uy = pts[k].y() + uo_.y(), uz = pts[k].z();
      xmn_ = std::min(xmn_, ux); xmx_ = std::max(xmx_, ux);
      ymn_ = std::min(ymn_, uy); ymx_ = std::max(ymx_, uy);
      zmn_ = std::min(zmn_, uz); zmx_ = std::max(zmx_, uz);
      const std::int32_t ix = static_cast<std::int32_t>(std::lround((ux - xo_) / kSc));
      const std::int32_t iy = static_cast<std::int32_t>(std::lround((uy - yo_) / kSc));
      const std::int32_t iz = static_cast<std::int32_t>(std::lround((uz - zo_) / kSc));
      put(&ix, 4); put(&iy, 4); put(&iz, 4);
      const std::uint16_t iv =
        hi ? static_cast<std::uint16_t>(std::lround(std::min(65535.0, std::max(0.0, (*ints)[k]))))
           : 0;
      put(&iv, 2);
      const std::uint8_t z0 = 0;
      put(&z0, 1); put(&z0, 1); put(&z0, 1); put(&z0, 1);
      const std::uint16_t psid = 0;
      put(&psid, 2);
      // 没上到色的给黑 (不给灰占位: las 里强度和颜色并存, 灰会和浅色路面混)
      const bool col = hc && (!ok || (*ok)[k]);
      for (int c = 0; c < 3; c++) {
        const std::uint16_t v =
          col ? static_cast<std::uint16_t>(static_cast<std::uint16_t>((*rgb)[k][c]) << 8) : 0;
        put(&v, 2);
      }
      n_++;
    }
    os_.write(buf_.data(), static_cast<std::streamsize>(buf_.size()));
  }
  /// @brief 回填点数 (偏移 107 / 111) 和包围盒 (偏移 179)
  bool finish() {
    if (!os_ || n_ == 0) {
      if (os_) os_.close();
      return n_ > 0;
    }
    const std::uint32_t n32 = static_cast<std::uint32_t>(n_);
    os_.seekp(107);
    os_.write(reinterpret_cast<const char*>(&n32), 4);
    os_.write(reinterpret_cast<const char*>(&n32), 4);
    os_.seekp(179);
    const double bb[6] = {xmx_, xmn_, ymx_, ymn_, zmx_, zmn_};
    for (int i = 0; i < 6; i++) os_.write(reinterpret_cast<const char*>(&bb[i]), 8);
    os_.close();
    return true;
  }
  std::size_t size() const { return n_; }

 private:
  static constexpr double kSc = 0.001;
  std::ofstream os_;
  Eigen::Vector2d uo_{0, 0};
  double xo_ = 0, yo_ = 0, zo_ = 0;
  double xmn_ = 1e18, xmx_ = -1e18, ymn_ = 1e18, ymx_ = -1e18, zmn_ = 1e18, zmx_ = -1e18;
  std::size_t n_ = 0;
  std::vector<char> buf_;
};

}  // namespace ialign
