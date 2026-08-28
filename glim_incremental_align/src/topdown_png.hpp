// -----------------------------------------------------------------------------
// 俯视图对比渲染: 把"一对点云的匹配前/后"画成一张 PNG
//
// 两个程序共用 (incremental_align 的 submap<->submap, incremental_g2o 的 frame<->老图),
// 所以抽成头文件。判读方式和配色理由见下面 renderTopDownPair 的注释。
// -----------------------------------------------------------------------------
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtsam_points/types/point_cloud.hpp>

#include "png_io.hpp"

namespace ialign {

// -----------------------------------------------------------------------------
// 把一对 submap 的匹配结果画成俯视图 PNG
//
// 配色是刻意选的: target 走**红**通道, source 走**绿**通道, 重合处自然成**黄**。
// 这比"两种颜色画散点"清楚得多 —— 散点图里前景会盖住背景, 分不清是对齐还是被遮挡。
//
// 必须做膨胀: 点云是离散点, 不膨胀就要求两片云命中**同一个像素**才算重合,
// 即使完全对齐也几乎不出现黄色, 图会一片红绿, 无法判读。
// 膨胀 tol 米之后 "黄色" 的含义变成**在 tol 以内吻合**, 正好对应我们关心的精度尺度。
//
// 四格布局 (2 行 x 2 列):
//     全部点     匹配前 | 匹配后
//     地面以上   匹配前 | 匹配后
// 第二行才是判读重点: 俯视图里地面是一大片填满的色块, 横向错开半米也看不出来(平面连续);
// 真正约束水平对齐的是路缘/护栏/杆状物这类**竖直结构**, 在俯视图里是细线, 错位立刻可见。
// -----------------------------------------------------------------------------
struct LoopImageParams {
  double res = 0.15;    // m/像素
  double tol = 0.20;    // "黄色"的含义: 两片云在这个距离内吻合
  double z_above = 0.5;    // 地面以上多少米算竖直结构
  double ground_cell = 8.0;// 局部地面高度图的网格边长 (m)
  int max_px = 1400;       // 单格边长上限, 超过则自动降分辨率
};

/// @brief 点云 -> 二值占据栅格 (俯视), 再按 tol 膨胀
/// @brief 局部地面高度图: 按 cell 米的网格取该格内 z 的最低值。
///
/// !! 不能用"整片点云的最小 z + 0.5m"当地面阈值 !!
/// submap 跨 50m 以上, 路面本身有坡度和起伏, 全局最小 z 往往比大部分位置的实际路面低得多,
/// 于是阈值仍在路面之下, 一个地面点都滤不掉 —— 实测四格图上下两行完全一样, 就是这个 bug。
/// 按局部网格估地面才对得上"地面以上"的语义。
struct GroundMap {
  double cell = 8.0;
  double x0 = 0, y0 = 0;
  int gw = 0, gh = 0;
  std::vector<float> zmin;

  double at(double x, double y) const {
    if (zmin.empty()) return -1e18;
    int ix = static_cast<int>((x - x0) / cell);
    int iy = static_cast<int>((y - y0) / cell);
    ix = std::max(0, std::min(gw - 1, ix));
    iy = std::max(0, std::min(gh - 1, iy));
    const float v = zmin[static_cast<std::size_t>(iy) * gw + ix];
    return v > -1e17f ? v : -1e18;
  }
};

static GroundMap buildGroundMap(
  const std::vector<std::pair<const gtsam_points::PointCloud*, Eigen::Isometry3d>>& clouds,
  double x0,
  double y0,
  double x1,
  double y1,
  double cell) {
  GroundMap g;
  g.cell = cell;
  g.x0 = x0;
  g.y0 = y0;
  g.gw = std::max(1, static_cast<int>((x1 - x0) / cell) + 1);
  g.gh = std::max(1, static_cast<int>((y1 - y0) / cell) + 1);
  g.zmin.assign(static_cast<std::size_t>(g.gw) * g.gh, -1e18f);
  for (const auto& [pc, T] : clouds) {
    for (std::size_t i = 0; i < pc->size(); i++) {
      const Eigen::Vector4d p = (T) * pc->points[i];
      int ix = static_cast<int>((p.x() - x0) / cell);
      int iy = static_cast<int>((p.y() - y0) / cell);
      if (ix < 0 || ix >= g.gw || iy < 0 || iy >= g.gh) continue;
      float& z = g.zmin[static_cast<std::size_t>(iy) * g.gw + ix];
      if (z < -1e17f || p.z() < z) z = static_cast<float>(p.z());
    }
  }
  // 空格用邻域填, 免得边缘出现"地面高度 = -inf"导致整格被判成地面以上
  for (int it = 0; it < 2; it++) {
    std::vector<float> cp = g.zmin;
    for (int y = 0; y < g.gh; y++) {
      for (int x = 0; x < g.gw; x++) {
        const std::size_t k = static_cast<std::size_t>(y) * g.gw + x;
        if (cp[k] > -1e17f) continue;
        double sum = 0.0;
        int n = 0;
        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            const int yy = y + dy, xx = x + dx;
            if (yy < 0 || yy >= g.gh || xx < 0 || xx >= g.gw) continue;
            const float v = cp[static_cast<std::size_t>(yy) * g.gw + xx];
            if (v > -1e17f) { sum += v; n++; }
          }
        }
        if (n) g.zmin[k] = static_cast<float>(sum / n);
      }
    }
  }
  return g;
}

static void rasterTopDown(
  const gtsam_points::PointCloud& pc,
  const Eigen::Isometry3d& T,
  const GroundMap& gm,
  double z_above,
  bool above_only,
  double x0,
  double y0,
  int w,
  int h,
  double res,
  int dil,
  std::vector<std::uint8_t>& grid) {
  grid.assign(static_cast<std::size_t>(w) * h, 0);
  for (std::size_t i = 0; i < pc.size(); i++) {
    const Eigen::Vector4d p = T * pc.points[i];
    if (above_only && p.z() <= gm.at(p.x(), p.y()) + z_above) continue;
    const int ix = static_cast<int>((p.x() - x0) / res);
    const int iy = static_cast<int>((p.y() - y0) / res);
    if (ix < 0 || ix >= w || iy < 0 || iy >= h) continue;
    grid[static_cast<std::size_t>(iy) * w + ix] = 1;
  }
  if (dil <= 0) return;
  // 圆形结构元膨胀, 分两趟(先 x 再 y)会得到方形, 这里直接用圆盘保证"tol 米内"的语义
  std::vector<std::uint8_t> src = grid;
  const int r2 = dil * dil;
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      if (!src[static_cast<std::size_t>(y) * w + x]) continue;
      for (int dy = -dil; dy <= dil; dy++) {
        const int yy = y + dy;
        if (yy < 0 || yy >= h) continue;
        for (int dx = -dil; dx <= dil; dx++) {
          if (dx * dx + dy * dy > r2) continue;
          const int xx = x + dx;
          if (xx < 0 || xx >= w) continue;
          grid[static_cast<std::size_t>(yy) * w + xx] = 1;
        }
      }
    }
  }
}

/// @brief 渲染一对匹配, 返回是否成功。A=target(红), B=source(绿)
static bool renderTopDownPair(
  const gtsam_points::PointCloud& A,
  const gtsam_points::PointCloud& B,
  const Eigen::Isometry3d& T_before,
  const Eigen::Isometry3d& T_after,
  const LoopImageParams& ip,
  const std::string& label_a,   // 例 "A=S0-042 (TARGET)"
  const std::string& label_b,   // 例 "B=S2-017 (SOURCE)"
  const std::string& label_metrics,
  const std::string& out_path) {
  if (A.size() == 0 || B.size() == 0) return false;

  // 取三者并集的包围盒 (A 固定在原点系, B 有两个位姿)
  Eigen::Vector2d mn(1e18, 1e18), mx(-1e18, -1e18);
  const auto acc = [&](const gtsam_points::PointCloud& pc, const Eigen::Isometry3d& T) {
    for (std::size_t i = 0; i < pc.size(); i++) {
      const Eigen::Vector4d p = T * pc.points[i];
      mn = mn.cwiseMin(p.head<2>());
      mx = mx.cwiseMax(p.head<2>());
    }
  };
  acc(A, Eigen::Isometry3d::Identity());
  acc(B, T_before);
  acc(B, T_after);
  mn -= Eigen::Vector2d(1.0, 1.0);
  mx += Eigen::Vector2d(1.0, 1.0);

  double res = ip.res;
  int w = std::max(8, static_cast<int>((mx.x() - mn.x()) / res));
  int h = std::max(8, static_cast<int>((mx.y() - mn.y()) / res));
  while (std::max(w, h) > ip.max_px) {  // 超大 submap 自动降分辨率, 免得出几万像素的图
    res *= 1.3;
    w = std::max(8, static_cast<int>((mx.x() - mn.x()) / res));
    h = std::max(8, static_cast<int>((mx.y() - mn.y()) / res));
  }
  const int dil = std::max(1, static_cast<int>(std::lround(ip.tol / res)));
  // 局部地面高度图 (用 A + B 的并集建, 保证两边判"地面以上"用的是同一个基准)
  const GroundMap gmap = buildGroundMap(
    {{&A, Eigen::Isometry3d::Identity()}, {&B, T_after}}, mn.x(), mn.y(), mx.x(), mx.y(), ip.ground_cell);

  // 2x2 拼图 + 顶部标题栏。格与格之间留分隔线。
  // 标题栏必须有: 光靠文件名说明"哪个颜色是哪片点云", 看图的人一定会记错。
  constexpr int kGap = 4;
  const int kScale = 2;                    // 字号放大倍数
  const int kLine = 7 * kScale + 5;        // 单行文字占的高度
  const int kHead = kLine * 3 + 6;         // 标题栏 3 行
  const int W = w * 2 + kGap;
  const int H = kHead + h * 2 + kGap;
  std::vector<std::uint8_t> img(static_cast<std::size_t>(W) * H * 3, 20);  // 深灰底

  std::vector<std::uint8_t> ga, gb;
  for (int row = 0; row < 2; row++) {
    const bool above = row == 1;
    rasterTopDown(A, Eigen::Isometry3d::Identity(), gmap, ip.z_above, above, mn.x(), mn.y(), w, h, res, dil, ga);
    for (int col = 0; col < 2; col++) {
      rasterTopDown(
        B, col == 0 ? T_before : T_after, gmap, ip.z_above, above, mn.x(), mn.y(), w, h, res, dil, gb);
      const int ox = col * (w + kGap);
      const int oy = kHead + row * (h + kGap);
      for (int y = 0; y < h; y++) {
        // PNG 行优先、左上为原点; 点云 y 向上, 所以这里翻一下, 出图才是常见的"上北下南"观感
        const int py = oy + (h - 1 - y);
        for (int x = 0; x < w; x++) {
          const std::size_t si = static_cast<std::size_t>(y) * w + x;
          const std::size_t di = (static_cast<std::size_t>(py) * W + (ox + x)) * 3;
          img[di + 0] = ga[si] ? 235 : 12;
          img[di + 1] = gb[si] ? 235 : 12;
          img[di + 2] = 12;
        }
      }
      // 每格左上角标注"哪一行/哪一列", 免得四格看起来一样时分不清
      char tag[96];
      std::snprintf(
        tag, sizeof(tag), "%s  %s", above ? "ABOVE GROUND" : "ALL POINTS", col == 0 ? "BEFORE (INS)" : "AFTER (REG)");
      drawText(img, W, H, ox + 6, oy + 6, tag, 255, 255, 255, kScale);
    }
  }

  // ---- 标题栏: 色块图例 + 两片点云的身份 + 关键指标 ----
  {
    const int sw = 7 * kScale;  // 色块边长
    int x = 6, y = 4;
    fillRect(img, W, H, x, y, sw, sw, 235, 0, 0);
    x += sw + 5;
    x += drawText(img, W, H, x, y, label_a, 255, 160, 160, kScale) + 6 * kScale;
    fillRect(img, W, H, x, y, sw, sw, 0, 235, 0);
    x += sw + 5;
    x += drawText(img, W, H, x, y, label_b, 160, 255, 160, kScale) + 6 * kScale;
    fillRect(img, W, H, x, y, sw, sw, 235, 235, 0);
    x += sw + 5;
    char tol[64];
    std::snprintf(tol, sizeof(tol), "OVERLAP WITHIN %.0FCM", ip.tol * 100);
    drawText(img, W, H, x, y, tol, 255, 255, 160, kScale);

    y += kLine;
    drawText(img, W, H, 6, y, label_metrics, 200, 200, 255, kScale);

    y += kLine;
    drawText(
      img, W, H, 6, y,
      "JUDGE BY THE LOWER ROW: ROAD SURFACE HIDES LATERAL SHIFT IN TOP VIEW",
      170, 170, 170, kScale);
  }
  return writePng(out_path, W, H, img);
}

}  // namespace ialign
