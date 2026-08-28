// -----------------------------------------------------------------------------
// 极简 PNG 写出 (只支持 8bit RGB)。
//
// 为什么自己写: 每一对 submap 匹配都要出一张图, 数量在 500~2000 之间。
// 若走"C++ 存 PCD -> Python 渲染"的老路, 每对三份点云 x 24 万点 = 约 12MB,
// 上千对就是 10GB 以上的中间文件, 纯属浪费。直接在 C++ 里栅格化 + 写 PNG,
// 一张图几百 KB, 也不需要额外依赖 —— zlib 本来就在依赖链里。
// -----------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <zlib.h>

namespace ialign {

namespace detail {

inline void be32(std::vector<std::uint8_t>& v, std::uint32_t x) {
  v.push_back(static_cast<std::uint8_t>((x >> 24) & 0xff));
  v.push_back(static_cast<std::uint8_t>((x >> 16) & 0xff));
  v.push_back(static_cast<std::uint8_t>((x >> 8) & 0xff));
  v.push_back(static_cast<std::uint8_t>(x & 0xff));
}

/// @brief 一个 PNG chunk = 长度(4,BE) + 类型(4) + 数据 + CRC32(类型+数据)(4,BE)
inline void chunk(std::vector<std::uint8_t>& out, const char type[5], const std::vector<std::uint8_t>& data) {
  be32(out, static_cast<std::uint32_t>(data.size()));
  const std::size_t crc_begin = out.size();
  out.insert(out.end(), type, type + 4);
  out.insert(out.end(), data.begin(), data.end());
  const std::uint32_t crc =
    static_cast<std::uint32_t>(::crc32(0L, out.data() + crc_begin, static_cast<uInt>(4 + data.size())));
  be32(out, crc);
}

}  // namespace detail

/// @brief 写 8bit RGB PNG。rgb 长度必须是 w*h*3, 行优先, 左上角为第一个像素。
inline bool writePng(const std::string& path, int w, int h, const std::vector<std::uint8_t>& rgb) {
  if (w <= 0 || h <= 0 || rgb.size() != static_cast<std::size_t>(w) * h * 3) return false;

  // 每条扫描线前面要加一个 filter 字节 (0 = None)
  std::vector<std::uint8_t> raw;
  raw.reserve(static_cast<std::size_t>(h) * (1 + static_cast<std::size_t>(w) * 3));
  for (int y = 0; y < h; y++) {
    raw.push_back(0);
    const std::uint8_t* row = rgb.data() + static_cast<std::size_t>(y) * w * 3;
    raw.insert(raw.end(), row, row + static_cast<std::size_t>(w) * 3);
  }

  uLongf comp_cap = ::compressBound(static_cast<uLong>(raw.size()));
  std::vector<std::uint8_t> comp(comp_cap);
  if (::compress2(comp.data(), &comp_cap, raw.data(), static_cast<uLong>(raw.size()), 6) != Z_OK) return false;
  comp.resize(comp_cap);

  std::vector<std::uint8_t> png = {0x89, 'P', 'N', 'G', 0x0d, 0x0a, 0x1a, 0x0a};
  {
    std::vector<std::uint8_t> ihdr;
    detail::be32(ihdr, static_cast<std::uint32_t>(w));
    detail::be32(ihdr, static_cast<std::uint32_t>(h));
    ihdr.push_back(8);  // bit depth
    ihdr.push_back(2);  // color type 2 = truecolor RGB
    ihdr.push_back(0);  // compression
    ihdr.push_back(0);  // filter
    ihdr.push_back(0);  // interlace
    detail::chunk(png, "IHDR", ihdr);
  }
  detail::chunk(png, "IDAT", comp);
  detail::chunk(png, "IEND", {});

  FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return false;
  const bool ok = std::fwrite(png.data(), 1, png.size(), f) == png.size();
  std::fclose(f);
  return ok;
}



// -----------------------------------------------------------------------------
// 5x7 位图字体 + 画字
//
// 为什么要自带字体: 图是自己栅格化的, 没有任何字体库可用, 而"哪个颜色是哪片点云"必须写在图上 ——
// 光靠文件名和外部文档, 看图的人一定会记错。只覆盖 ASCII 大写/数字/常用符号(小写自动转大写),
// 中文没法用位图字体简单实现, 所以标注一律用英文。
// -----------------------------------------------------------------------------
namespace font5x7 {

/// @brief 每个字形 5 字节, 每字节一列, bit0 = 顶行
inline const std::uint8_t* glyph(char c) {
  static const std::uint8_t kUnknown[5] = {0x7f, 0x41, 0x41, 0x41, 0x7f};
  if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
  switch (c) {
    case ' ': { static const std::uint8_t g[5] = {0,0,0,0,0}; return g; }
    case '0': { static const std::uint8_t g[5] = {0x3E,0x51,0x49,0x45,0x3E}; return g; }
    case '1': { static const std::uint8_t g[5] = {0x00,0x42,0x7F,0x40,0x00}; return g; }
    case '2': { static const std::uint8_t g[5] = {0x42,0x61,0x51,0x49,0x46}; return g; }
    case '3': { static const std::uint8_t g[5] = {0x21,0x41,0x45,0x4B,0x31}; return g; }
    case '4': { static const std::uint8_t g[5] = {0x18,0x14,0x12,0x7F,0x10}; return g; }
    case '5': { static const std::uint8_t g[5] = {0x27,0x45,0x45,0x45,0x39}; return g; }
    case '6': { static const std::uint8_t g[5] = {0x3C,0x4A,0x49,0x49,0x30}; return g; }
    case '7': { static const std::uint8_t g[5] = {0x01,0x71,0x09,0x05,0x03}; return g; }
    case '8': { static const std::uint8_t g[5] = {0x36,0x49,0x49,0x49,0x36}; return g; }
    case '9': { static const std::uint8_t g[5] = {0x06,0x49,0x49,0x29,0x1E}; return g; }
    case 'A': { static const std::uint8_t g[5] = {0x7E,0x11,0x11,0x11,0x7E}; return g; }
    case 'B': { static const std::uint8_t g[5] = {0x7F,0x49,0x49,0x49,0x36}; return g; }
    case 'C': { static const std::uint8_t g[5] = {0x3E,0x41,0x41,0x41,0x22}; return g; }
    case 'D': { static const std::uint8_t g[5] = {0x7F,0x41,0x41,0x22,0x1C}; return g; }
    case 'E': { static const std::uint8_t g[5] = {0x7F,0x49,0x49,0x49,0x41}; return g; }
    case 'F': { static const std::uint8_t g[5] = {0x7F,0x09,0x09,0x01,0x01}; return g; }
    case 'G': { static const std::uint8_t g[5] = {0x3E,0x41,0x41,0x51,0x32}; return g; }
    case 'H': { static const std::uint8_t g[5] = {0x7F,0x08,0x08,0x08,0x7F}; return g; }
    case 'I': { static const std::uint8_t g[5] = {0x00,0x41,0x7F,0x41,0x00}; return g; }
    case 'J': { static const std::uint8_t g[5] = {0x20,0x40,0x41,0x3F,0x01}; return g; }
    case 'K': { static const std::uint8_t g[5] = {0x7F,0x08,0x14,0x22,0x41}; return g; }
    case 'L': { static const std::uint8_t g[5] = {0x7F,0x40,0x40,0x40,0x40}; return g; }
    case 'M': { static const std::uint8_t g[5] = {0x7F,0x02,0x04,0x02,0x7F}; return g; }
    case 'N': { static const std::uint8_t g[5] = {0x7F,0x04,0x08,0x10,0x7F}; return g; }
    case 'O': { static const std::uint8_t g[5] = {0x3E,0x41,0x41,0x41,0x3E}; return g; }
    case 'P': { static const std::uint8_t g[5] = {0x7F,0x09,0x09,0x09,0x06}; return g; }
    case 'Q': { static const std::uint8_t g[5] = {0x3E,0x41,0x51,0x21,0x5E}; return g; }
    case 'R': { static const std::uint8_t g[5] = {0x7F,0x09,0x19,0x29,0x46}; return g; }
    case 'S': { static const std::uint8_t g[5] = {0x46,0x49,0x49,0x49,0x31}; return g; }
    case 'T': { static const std::uint8_t g[5] = {0x01,0x01,0x7F,0x01,0x01}; return g; }
    case 'U': { static const std::uint8_t g[5] = {0x3F,0x40,0x40,0x40,0x3F}; return g; }
    case 'V': { static const std::uint8_t g[5] = {0x1F,0x20,0x40,0x20,0x1F}; return g; }
    case 'W': { static const std::uint8_t g[5] = {0x7F,0x20,0x18,0x20,0x7F}; return g; }
    case 'X': { static const std::uint8_t g[5] = {0x63,0x14,0x08,0x14,0x63}; return g; }
    case 'Y': { static const std::uint8_t g[5] = {0x03,0x04,0x78,0x04,0x03}; return g; }
    case 'Z': { static const std::uint8_t g[5] = {0x61,0x51,0x49,0x45,0x43}; return g; }
    case '.': { static const std::uint8_t g[5] = {0x00,0x60,0x60,0x00,0x00}; return g; }
    case ',': { static const std::uint8_t g[5] = {0x00,0x50,0x30,0x00,0x00}; return g; }
    case '-': { static const std::uint8_t g[5] = {0x08,0x08,0x08,0x08,0x08}; return g; }
    case '_': { static const std::uint8_t g[5] = {0x40,0x40,0x40,0x40,0x40}; return g; }
    case '/': { static const std::uint8_t g[5] = {0x20,0x10,0x08,0x04,0x02}; return g; }
    case ':': { static const std::uint8_t g[5] = {0x00,0x36,0x36,0x00,0x00}; return g; }
    case '(': { static const std::uint8_t g[5] = {0x00,0x1C,0x22,0x41,0x00}; return g; }
    case ')': { static const std::uint8_t g[5] = {0x00,0x41,0x22,0x1C,0x00}; return g; }
    case '=': { static const std::uint8_t g[5] = {0x14,0x14,0x14,0x14,0x14}; return g; }
    case '+': { static const std::uint8_t g[5] = {0x08,0x08,0x3E,0x08,0x08}; return g; }
    case '%': { static const std::uint8_t g[5] = {0x23,0x13,0x08,0x64,0x62}; return g; }
    case '#': { static const std::uint8_t g[5] = {0x14,0x7F,0x14,0x7F,0x14}; return g; }
    case '>': { static const std::uint8_t g[5] = {0x00,0x41,0x22,0x14,0x08}; return g; }
    case '<': { static const std::uint8_t g[5] = {0x08,0x14,0x22,0x41,0x00}; return g; }
    default: return kUnknown;
  }
}

}  // namespace font5x7

/// @brief 在 RGB 缓冲上画一行文字。scale = 像素放大倍数; 返回文字宽度(像素)
inline int drawText(
  std::vector<std::uint8_t>& img,
  int W,
  int H,
  int x,
  int y,
  const std::string& text,
  std::uint8_t r,
  std::uint8_t g,
  std::uint8_t b,
  int scale = 2) {
  int cx = x;
  for (const char ch : text) {
    const std::uint8_t* gl = font5x7::glyph(ch);
    for (int col = 0; col < 5; col++) {
      for (int row = 0; row < 7; row++) {
        if (!((gl[col] >> row) & 1)) continue;
        for (int sy = 0; sy < scale; sy++) {
          for (int sx = 0; sx < scale; sx++) {
            const int px = cx + col * scale + sx;
            const int py = y + row * scale + sy;
            if (px < 0 || px >= W || py < 0 || py >= H) continue;
            const std::size_t di = (static_cast<std::size_t>(py) * W + px) * 3;
            img[di + 0] = r;
            img[di + 1] = g;
            img[di + 2] = b;
          }
        }
      }
    }
    cx += 6 * scale;  // 5 列字形 + 1 列间隔
  }
  return cx - x;
}

/// @brief 画一个实心矩形 (给图例做色块)
inline void fillRect(
  std::vector<std::uint8_t>& img, int W, int H, int x, int y, int w, int h,
  std::uint8_t r, std::uint8_t g, std::uint8_t b) {
  for (int py = y; py < y + h; py++) {
    for (int px = x; px < x + w; px++) {
      if (px < 0 || px >= W || py < 0 || py >= H) continue;
      const std::size_t di = (static_cast<std::size_t>(py) * W + px) * 3;
      img[di + 0] = r;
      img[di + 1] = g;
      img[di + 2] = b;
    }
  }
}

}  // namespace ialign
