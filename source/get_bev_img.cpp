/**
 * @file   get_bev_img.cpp
 * @brief  由 7 路环视相机 + 内外参生成 BEV(鸟瞰) 图
 *
 * 数据组织方式与 03_GetRgbAndNonuniformSample.cpp 一致:
 *   work_dir/calib/intrinsics/camN_Intrinsic.yaml      相机内参 (K, D, ImageWidth/Height)
 *   work_dir/calib/extrinsics/N_camN_2_vehicle_*.yaml  camN -> vehicle 外参 (wxyz 四元数 + xyz 平移)
 *   work_dir/images/camN/<frame>_camN.jpg              同一时刻各路图像 (frame 为公共时间戳前缀)
 *
 * 原理: IPM(逆透视变换)。假设车体系下 z = ground_z 为地平面, 对 BEV 栅格中的每个像素
 * 反算其车体系三维坐标 (X, Y, ground_z), 用 T_cam_vehicle 变换到相机系, 再用 K/D 投影到
 * 原图, 得到 remap 查找表。7 路相机各自得到一张 BEV, 按权重(入射角余弦 + 图像边缘羽化)
 * 加权融合成一张 360° 环视 BEV。
 *
 * 查找表与权重只跟标定有关, 与帧无关 —— 只构建一次, 之后每帧仅做 remap + 加权求和。
 *
 * 参数见 rviz_cfg/bev_config.yaml, 输出到 work_dir/bev/<frame>.jpg
 *   rosrun hba get_bev_img [bev_config.yaml] [work_dir]
 */

#include <atomic>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "sensor.hpp"

namespace fs = std::filesystem;

// ============================================================================
// BEV 参数 (可由 config.yaml 的 bev: 段覆盖)
// ============================================================================
struct BevParams
{
    // BEV 覆盖范围, 车体系: x 向前, y 向左, 单位 m
    double x_min = -20.0;
    double x_max = 40.0;
    double y_min = -20.0;
    double y_max = 20.0;
    double resolution = 0.05; // m / pixel

    double ground_z = 0.0; // 车体系下地平面高度

    // 投影有效性 / 融合权重
    double max_norm_radius = 1.2; // 归一化像平面半径上限, 抑制畸变模型外推
    double feather_px = 80.0;     // 图像边缘羽化宽度(像素)
    double cos_power = 3.0;       // 入射角余弦的幂次, 越大越偏向正对的相机

    // 自车区域(相机看不到, 全是车身/畸变) —— 涂黑并画车体框
    bool blank_ego = true;
    double ego_x_min = -1.5;
    double ego_x_max = 3.5;
    double ego_y_min = -1.1;
    double ego_y_max = 1.1;

    // 叠加信息
    bool draw_overlay = true;
    double grid_step = 5.0; // 距离栅格线间隔 m

    // 自车静态遮罩 masks/static/<cam>/mask.png (黑=车身, 白=有效), 用于剔除车体拖影
    bool use_static_mask = true;
    std::string mask_root = "masks/static";
    int mask_erode_px = 3;  // 在遮罩自身分辨率下收缩有效区, 避免边界渗色
    int mask_blur_px = 5;   // 遮罩边缘软化, 使权重平滑过渡

    std::vector<std::string> cameras{"cam1", "cam2", "cam3", "cam4", "cam5", "cam6", "cam7"};
    std::string vehicle_frame = "vehicle";

    int jpeg_quality = 92;
    bool save_per_cam = false; // 调试: 额外保存每路相机单独的 BEV
    int max_frames = 0;        // 0 = 全部
    int num_threads = 0;       // 0 = 自动
};

// 每路相机的 BEV 查找表 + 融合权重
struct CamBevLut
{
    std::string name;
    cv::Mat map_x;  // CV_32FC1, BEV 尺寸, 原图列坐标
    cv::Mat map_y;  // CV_32FC1
    cv::Mat weight; // CV_32FC1, [0,1]
    int img_w = 0;
    int img_h = 0;
    bool valid = false;
};

// ============================================================================
// 工具函数
// ============================================================================
static void create_dir_if_not_exists(const std::string &dir)
{
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec)
        std::cerr << "[BEV] Failed to create directory " << dir << ": " << ec.message() << std::endl;
}

static bool is_image_ext(const std::string &ext)
{
    std::string e = ext;
    for (auto &c : e)
        c = static_cast<char>(::tolower(c));
    return e == ".jpg" || e == ".jpeg" || e == ".png" || e == ".bmp";
}

/**
 * @brief 扫描 images/<camera> 目录, 建立 frame_id -> 图像路径 的索引
 *
 * 文件名形如 "0_1766197757.000_cam1.jpg", frame_id 为去掉结尾 "_camN" 后的 "0_1766197757.000";
 * 若没有该后缀 (形如 "<frame>.jpg") 则直接用 stem 作为 frame_id。
 */
static std::unordered_map<std::string, std::string>
index_camera_images(const std::string &cam_dir, const std::string &camera_name)
{
    std::unordered_map<std::string, std::string> index;
    std::error_code ec;
    if (!fs::is_directory(cam_dir, ec))
    {
        std::cerr << "[BEV] Not a directory: " << cam_dir << std::endl;
        return index;
    }
    const std::string suffix = "_" + camera_name;
    for (const auto &entry : fs::directory_iterator(cam_dir, ec))
    {
        if (!entry.is_regular_file())
            continue;
        if (!is_image_ext(entry.path().extension().string()))
            continue;
        std::string stem = entry.path().stem().string();
        if (stem.size() > suffix.size() &&
            stem.compare(stem.size() - suffix.size(), suffix.size(), suffix) == 0)
        {
            stem = stem.substr(0, stem.size() - suffix.size());
        }
        index.emplace(stem, entry.path().string());
    }
    return index;
}

/// 按 frame_id 末尾 '_' 之后的时间戳排序, 解析失败时退化为字典序
static void sort_frames_by_timestamp(std::vector<std::string> &frames)
{
    struct FrameInfo
    {
        std::string name;
        double timestamp = 0.0;
        bool has_timestamp = false;
    };
    std::vector<FrameInfo> infos;
    infos.reserve(frames.size());
    for (const auto &f : frames)
    {
        FrameInfo info{f, 0.0, false};
        const auto pos = f.find_last_of('_');
        if (pos != std::string::npos && pos + 1 < f.size())
        {
            try
            {
                info.timestamp = std::stod(f.substr(pos + 1));
                info.has_timestamp = true;
            }
            catch (const std::exception &)
            {
            }
        }
        infos.push_back(info);
    }
    std::sort(infos.begin(), infos.end(), [](const FrameInfo &a, const FrameInfo &b) {
        if (a.has_timestamp && b.has_timestamp)
            return a.timestamp < b.timestamp;
        if (a.has_timestamp != b.has_timestamp)
            return a.has_timestamp;
        return a.name < b.name;
    });
    frames.clear();
    frames.reserve(infos.size());
    for (const auto &i : infos)
        frames.push_back(i.name);
}

// BEV 像素中心 <-> 车体系坐标
static inline double bev_row_to_x(const BevParams &p, int row)
{
    return p.x_max - (row + 0.5) * p.resolution;
}
static inline double bev_col_to_y(const BevParams &p, int col)
{
    return p.y_max - (col + 0.5) * p.resolution;
}
static inline int bev_x_to_row(const BevParams &p, double x)
{
    return static_cast<int>(std::lround((p.x_max - x) / p.resolution - 0.5));
}
static inline int bev_y_to_col(const BevParams &p, double y)
{
    return static_cast<int>(std::lround((p.y_max - y) / p.resolution - 0.5));
}

/**
 * @brief 载入并预处理某路相机的自车静态遮罩 (白=有效, 黑=车身)
 *
 * 遮罩分辨率可以低于原图, 采样时按比例缩放。先腐蚀收缩有效区, 再高斯模糊做软边,
 * 返回 CV_32FC1 的 [0,1] 权重图; 文件不存在时返回空 Mat 表示"不使用遮罩"。
 */
static cv::Mat load_static_mask(const std::string &path, const BevParams &p)
{
    cv::Mat mask = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (mask.empty())
        return cv::Mat();

    if (p.mask_erode_px > 0)
    {
        const int k = 2 * p.mask_erode_px + 1;
        cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k}));
    }
    if (p.mask_blur_px > 0)
    {
        const int k = 2 * p.mask_blur_px + 1;
        cv::GaussianBlur(mask, mask, {k, k}, 0);
    }
    cv::Mat maskf;
    mask.convertTo(maskf, CV_32FC1, 1.0 / 255.0);
    return maskf;
}

// ============================================================================
// 构建单路相机的 BEV 查找表
// ============================================================================
static CamBevLut build_cam_lut(const std::string &camera_name,
                               const BevParams &p,
                               const CameraIntrinsic &cam_intr,
                               ExtrinsicManager &tf_cache,
                               const cv::Mat &static_mask,
                               int bev_rows, int bev_cols)
{
    CamBevLut lut;
    lut.name = camera_name;

    cv::Mat K, D;
    int img_w = 0, img_h = 0;
    if (!cam_intr.getIntrinsic(camera_name, K, D, &img_w, &img_h))
    {
        std::cerr << "[BEV] Missing intrinsics for " << camera_name << std::endl;
        return lut;
    }
    if (img_w <= 0 || img_h <= 0)
    {
        std::cerr << "[BEV] Intrinsics of " << camera_name << " has no ImageWidth/ImageHeight" << std::endl;
        return lut;
    }

    // T_vehicle_cam (外参文件即 camN -> vehicle), 这里需要 vehicle -> cam
    Eigen::Matrix4d T_cam_vehicle;
    if (!tf_cache.lookupTransform(camera_name, p.vehicle_frame, T_cam_vehicle))
    {
        std::cerr << "[BEV] Missing transform: " << camera_name << " <- " << p.vehicle_frame << std::endl;
        return lut;
    }

    const Eigen::Matrix3d R = T_cam_vehicle.block<3, 3>(0, 0);
    const Eigen::Vector3d t = T_cam_vehicle.block<3, 1>(0, 3);

    // 1) 收集地平面上落在相机前方的 BEV 像素
    std::vector<int> flat_idx;
    std::vector<cv::Point3f> pts_cam;
    std::vector<float> cos_incidence;
    flat_idx.reserve(static_cast<size_t>(bev_rows) * bev_cols / 4);
    pts_cam.reserve(flat_idx.capacity());
    cos_incidence.reserve(flat_idx.capacity());

    for (int r = 0; r < bev_rows; ++r)
    {
        const double X = bev_row_to_x(p, r);
        for (int c = 0; c < bev_cols; ++c)
        {
            const double Y = bev_col_to_y(p, c);
            const Eigen::Vector3d pc = R * Eigen::Vector3d(X, Y, p.ground_z) + t;
            if (pc.z() <= 0.1)
                continue; // 相机后方
            const double xn = pc.x() / pc.z();
            const double yn = pc.y() / pc.z();
            if (std::hypot(xn, yn) > p.max_norm_radius)
                continue; // 远超 FOV, 畸变模型不可信
            flat_idx.push_back(r * bev_cols + c);
            pts_cam.emplace_back(static_cast<float>(pc.x()), static_cast<float>(pc.y()),
                                 static_cast<float>(pc.z()));
            cos_incidence.push_back(static_cast<float>(pc.z() / pc.norm()));
        }
    }

    lut.map_x = cv::Mat(bev_rows, bev_cols, CV_32FC1, cv::Scalar(-1.f));
    lut.map_y = cv::Mat(bev_rows, bev_cols, CV_32FC1, cv::Scalar(-1.f));
    lut.weight = cv::Mat::zeros(bev_rows, bev_cols, CV_32FC1);
    lut.img_w = img_w;
    lut.img_h = img_h;

    if (pts_cam.empty())
    {
        std::cerr << "[BEV] " << camera_name << " covers no BEV cell." << std::endl;
        return lut;
    }

    // 2) 一次性投影 (带畸变, 直接采样原图, 无需先 undistort)
    const cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    const cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    const cv::Mat dist = D.empty() ? cv::Mat::zeros(1, 5, CV_64F) : D.reshape(1, 1).clone();
    std::vector<cv::Point2f> pts_img;
    cv::projectPoints(pts_cam, rvec, tvec, K, dist, pts_img);

    // 3) 写入查找表与权重
    const float feather = static_cast<float>(std::max(1.0, p.feather_px));
    const bool has_mask = !static_mask.empty();
    const float mask_sx = has_mask ? static_cast<float>(static_mask.cols) / img_w : 1.f;
    const float mask_sy = has_mask ? static_cast<float>(static_mask.rows) / img_h : 1.f;
    auto *mx = lut.map_x.ptr<float>();
    auto *my = lut.map_y.ptr<float>();
    auto *mw = lut.weight.ptr<float>();
    size_t n_used = 0;
    for (size_t i = 0; i < pts_img.size(); ++i)
    {
        const float u = pts_img[i].x;
        const float v = pts_img[i].y;
        if (!std::isfinite(u) || !std::isfinite(v))
            continue;
        if (u < 0.f || v < 0.f || u > img_w - 1.f || v > img_h - 1.f)
            continue;

        // 边缘羽化: 越靠近画面边界权重越低, 使相邻相机的接缝平滑过渡
        const float d_edge = std::min(std::min(u, v), std::min(img_w - 1.f - u, img_h - 1.f - v));
        const float w_edge = std::min(1.0f, d_edge / feather);
        if (w_edge <= 0.f)
            continue;

        float w = static_cast<float>(std::pow(cos_incidence[i], p.cos_power)) * w_edge;

        // 自车遮罩: 落在车身上的像素权重置零, 避免车头/车尾被拉成拖影
        if (has_mask)
        {
            const int mu = std::clamp(static_cast<int>(u * mask_sx), 0, static_mask.cols - 1);
            const int mv = std::clamp(static_cast<int>(v * mask_sy), 0, static_mask.rows - 1);
            w *= static_mask.at<float>(mv, mu);
        }
        if (w <= 1e-6f)
            continue;

        const int idx = flat_idx[i];
        mx[idx] = u;
        my[idx] = v;
        mw[idx] = w;
        ++n_used;
    }

    lut.valid = n_used > 0;
    std::cout << "[BEV] LUT " << camera_name << ": " << n_used << " / "
              << (static_cast<size_t>(bev_rows) * bev_cols) << " BEV cells covered" << std::endl;
    return lut;
}

// ============================================================================
// 叠加信息: 距离栅格 + 自车框 + 朝向
// ============================================================================
static void draw_overlay(cv::Mat &bev, const BevParams &p)
{
    const cv::Scalar grid_color(70, 70, 70);
    const cv::Scalar axis_color(0, 200, 255);
    const cv::Scalar ego_color(0, 255, 255);

    if (p.grid_step > 0.0)
    {
        for (double x = std::ceil(p.x_min / p.grid_step) * p.grid_step; x <= p.x_max; x += p.grid_step)
        {
            const int r = bev_x_to_row(p, x);
            if (r < 0 || r >= bev.rows)
                continue;
            cv::line(bev, {0, r}, {bev.cols - 1, r}, std::abs(x) < 1e-6 ? axis_color : grid_color, 1);
            cv::putText(bev, cv::format("%.0f", x), {4, r - 3}, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        cv::Scalar(200, 200, 200), 1);
        }
        for (double y = std::ceil(p.y_min / p.grid_step) * p.grid_step; y <= p.y_max; y += p.grid_step)
        {
            const int c = bev_y_to_col(p, y);
            if (c < 0 || c >= bev.cols)
                continue;
            cv::line(bev, {c, 0}, {c, bev.rows - 1}, std::abs(y) < 1e-6 ? axis_color : grid_color, 1);
        }
    }

    // 自车框 (车体系 x 向前 -> 图像向上)
    const int r0 = bev_x_to_row(p, p.ego_x_max);
    const int r1 = bev_x_to_row(p, p.ego_x_min);
    const int c0 = bev_y_to_col(p, p.ego_y_max);
    const int c1 = bev_y_to_col(p, p.ego_y_min);
    cv::Rect ego(cv::Point(c0, r0), cv::Point(c1, r1));
    ego &= cv::Rect(0, 0, bev.cols, bev.rows);
    if (ego.area() > 0)
    {
        cv::rectangle(bev, ego, ego_color, 2);
        // 车头方向箭头
        const cv::Point head(ego.x + ego.width / 2, ego.y + 6);
        const cv::Point tail(ego.x + ego.width / 2, ego.y + ego.height / 2);
        cv::arrowedLine(bev, tail, head, ego_color, 2, cv::LINE_AA, 0, 0.3);
    }
}

// ============================================================================
// 单帧: 7 路图像 -> 融合 BEV
// ============================================================================
static bool make_bev_frame(const std::string &frame,
                           const BevParams &p,
                           const std::vector<CamBevLut> &luts,
                           const std::vector<std::unordered_map<std::string, std::string>> &cam_index,
                           const cv::Mat &weight_sum, // CV_32FC1, 预先累加
                           const std::string &out_dir,
                           const std::string &per_cam_dir,
                           cv::Mat &bev_out)
{
    const int rows = weight_sum.rows;
    const int cols = weight_sum.cols;
    cv::Mat acc = cv::Mat::zeros(rows, cols, CV_32FC3);

    int used_cams = 0;
    for (size_t i = 0; i < luts.size(); ++i)
    {
        const CamBevLut &lut = luts[i];
        if (!lut.valid)
            continue;
        auto it = cam_index[i].find(frame);
        if (it == cam_index[i].end())
        {
            std::cerr << "[BEV] frame " << frame << ": missing image for " << lut.name << std::endl;
            continue;
        }
        cv::Mat image = cv::imread(it->second, cv::IMREAD_COLOR);
        if (image.empty())
        {
            std::cerr << "[BEV] Failed to read: " << it->second << std::endl;
            continue;
        }
        if (image.cols != lut.img_w || image.rows != lut.img_h)
            cv::resize(image, image, cv::Size(lut.img_w, lut.img_h));

        cv::Mat warped;
        cv::remap(image, warped, lut.map_x, lut.map_y, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        cv::Mat warped_f;
        warped.convertTo(warped_f, CV_32FC3);

        // 逐通道乘权重后累加
        cv::Mat w3;
        cv::merge(std::vector<cv::Mat>{lut.weight, lut.weight, lut.weight}, w3);
        cv::accumulateProduct(warped_f, w3, acc);
        ++used_cams;

        if (p.save_per_cam)
            cv::imwrite(per_cam_dir + "/" + frame + "_" + lut.name + ".jpg", warped,
                        {cv::IMWRITE_JPEG_QUALITY, p.jpeg_quality});
    }

    if (used_cams == 0)
    {
        std::cerr << "[BEV] frame " << frame << ": no usable camera." << std::endl;
        return false;
    }

    // 加权平均 (weight_sum 为 0 的像素保持黑色)
    cv::Mat denom;
    cv::max(weight_sum, 1e-6f, denom);
    cv::Mat d3;
    cv::merge(std::vector<cv::Mat>{denom, denom, denom}, d3);
    cv::Mat bev_f;
    cv::divide(acc, d3, bev_f);
    bev_f.setTo(cv::Scalar(0, 0, 0), weight_sum <= 1e-6f);
    bev_f.convertTo(bev_out, CV_8UC3);

    if (p.blank_ego)
    {
        const int r0 = bev_x_to_row(p, p.ego_x_max);
        const int r1 = bev_x_to_row(p, p.ego_x_min);
        const int c0 = bev_y_to_col(p, p.ego_y_max);
        const int c1 = bev_y_to_col(p, p.ego_y_min);
        cv::Rect ego(cv::Point(c0, r0), cv::Point(c1, r1));
        ego &= cv::Rect(0, 0, bev_out.cols, bev_out.rows);
        if (ego.area() > 0)
            bev_out(ego).setTo(cv::Scalar(40, 40, 40));
    }

    if (p.draw_overlay)
    {
        draw_overlay(bev_out, p);
        cv::putText(bev_out, frame, {8, bev_out.rows - 10}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    const std::string out_path = out_dir + "/" + frame + ".jpg";
    if (!cv::imwrite(out_path, bev_out, {cv::IMWRITE_JPEG_QUALITY, p.jpeg_quality}))
    {
        std::cerr << "[BEV] Failed to write " << out_path << std::endl;
        return false;
    }
    return true;
}

// ============================================================================
static void load_bev_params(const YAML::Node &config, BevParams &p)
{
    if (!config["bev"])
        return;
    const YAML::Node b = config["bev"];
    auto get = [&](const char *key, auto &field) {
        using T = std::decay_t<decltype(field)>;
        if (b[key])
            field = b[key].template as<T>();
    };
    get("x_min", p.x_min);
    get("x_max", p.x_max);
    get("y_min", p.y_min);
    get("y_max", p.y_max);
    get("resolution", p.resolution);
    get("ground_z", p.ground_z);
    get("max_norm_radius", p.max_norm_radius);
    get("feather_px", p.feather_px);
    get("cos_power", p.cos_power);
    get("blank_ego", p.blank_ego);
    get("ego_x_min", p.ego_x_min);
    get("ego_x_max", p.ego_x_max);
    get("ego_y_min", p.ego_y_min);
    get("ego_y_max", p.ego_y_max);
    get("draw_overlay", p.draw_overlay);
    get("grid_step", p.grid_step);
    get("vehicle_frame", p.vehicle_frame);
    get("use_static_mask", p.use_static_mask);
    get("mask_root", p.mask_root);
    get("mask_erode_px", p.mask_erode_px);
    get("mask_blur_px", p.mask_blur_px);
    get("jpeg_quality", p.jpeg_quality);
    get("save_per_cam", p.save_per_cam);
    get("max_frames", p.max_frames);
    get("num_threads", p.num_threads);
    if (b["cameras"] && b["cameras"].IsSequence())
        p.cameras = b["cameras"].as<std::vector<std::string>>();
}

int main(int argc, char **argv)
{
    // 用法: get_bev_img [bev_config.yaml] [work_dir]
    std::string config_file = "/home/tyjt/Desktop/ros_ws/src/HBA/rviz_cfg/bev_config.yaml";
    if (argc > 1)
        config_file = argv[1];
    std::cout << "[BEV] config: " << config_file << std::endl;

    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_file);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[BEV] Failed to load config file: " << config_file << ", error: " << e.what() << std::endl;
        return 1;
    }

    // work_dir: 优先本文件顶层 work_dir, 兼容 config.yaml 的 paths.work_dir, 最后由命令行覆盖
    std::string work_dir;
    if (config["work_dir"])
        work_dir = config["work_dir"].as<std::string>();
    else if (config["paths"] && config["paths"]["work_dir"])
        work_dir = config["paths"]["work_dir"].as<std::string>();
    if (argc > 2)
        work_dir = argv[2];
    if (work_dir.empty())
    {
        std::cerr << "[BEV] work_dir is not set (config key `work_dir` or argv[2])." << std::endl;
        return 1;
    }
    while (work_dir.size() > 1 && work_dir.back() == '/')
        work_dir.pop_back();
    std::cout << "[BEV] work_dir: " << work_dir << std::endl;

    BevParams params;
    load_bev_params(config, params);

    const std::string extrinsics_folder = work_dir + "/calib/extrinsics/";
    const std::string cam_folder = work_dir + "/calib/new_intrinsics/";
    const std::string image_root = work_dir + "/images";
    const std::string output_root = work_dir + "/bev";
    const std::string per_cam_dir = work_dir + "/bev_per_cam";

    for (const auto &d : {extrinsics_folder, cam_folder, image_root})
    {
        if (!fs::is_directory(d))
        {
            std::cerr << "[BEV] Missing directory: " << d << std::endl;
            return 1;
        }
    }
    create_dir_if_not_exists(output_root);
    if (params.save_per_cam)
        create_dir_if_not_exists(per_cam_dir);

    CameraIntrinsic cam_intr;
    cam_intr.loadFolder(cam_folder);

    ExtrinsicManager tf_cache;
    tf_cache.loadFolder(extrinsics_folder);

    const int bev_rows = static_cast<int>(std::lround((params.x_max - params.x_min) / params.resolution));
    const int bev_cols = static_cast<int>(std::lround((params.y_max - params.y_min) / params.resolution));
    if (bev_rows <= 0 || bev_cols <= 0)
    {
        std::cerr << "[BEV] Invalid BEV size: " << bev_rows << " x " << bev_cols << std::endl;
        return 1;
    }
    std::cout << "[BEV] BEV image: " << bev_cols << " x " << bev_rows << " px, "
              << params.resolution << " m/px, x[" << params.x_min << ", " << params.x_max
              << "] y[" << params.y_min << ", " << params.y_max << "], ground_z=" << params.ground_z
              << std::endl;

    // ---- 1) 构建每路相机的查找表, 并索引图像文件 ----
    std::vector<CamBevLut> luts;
    std::vector<std::unordered_map<std::string, std::string>> cam_index;
    luts.reserve(params.cameras.size());
    cam_index.reserve(params.cameras.size());
    for (const auto &cam : params.cameras)
    {
        cv::Mat static_mask;
        if (params.use_static_mask)
        {
            const std::string mask_path = work_dir + "/" + params.mask_root + "/" + cam + "/mask.png";
            static_mask = load_static_mask(mask_path, params);
            if (static_mask.empty())
                std::cerr << "[BEV] No static mask for " << cam << " (" << mask_path
                          << "), self-body may smear into BEV." << std::endl;
        }
        luts.push_back(build_cam_lut(cam, params, cam_intr, tf_cache, static_mask, bev_rows, bev_cols));
        cam_index.push_back(index_camera_images(image_root + "/" + cam, cam));
        std::cout << "[BEV] " << cam << ": " << cam_index.back().size() << " images indexed" << std::endl;
    }

    cv::Mat weight_sum = cv::Mat::zeros(bev_rows, bev_cols, CV_32FC1);
    int valid_cams = 0;
    for (const auto &lut : luts)
    {
        if (!lut.valid)
            continue;
        weight_sum += lut.weight;
        ++valid_cams;
    }
    if (valid_cams == 0)
    {
        std::cerr << "[BEV] No valid camera LUT, abort." << std::endl;
        return 1;
    }
    {
        const double covered = cv::countNonZero(weight_sum > 1e-6f);
        std::cout << "[BEV] " << valid_cams << " cameras usable, BEV coverage: "
                  << (100.0 * covered / (bev_rows * bev_cols)) << " %" << std::endl;
    }

    // ---- 2) 收集所有帧 (以第一路有图像的相机为准) ----
    std::vector<std::string> frames;
    for (size_t i = 0; i < cam_index.size(); ++i)
    {
        if (cam_index[i].empty())
            continue;
        for (const auto &kv : cam_index[i])
            frames.push_back(kv.first);
        std::cout << "[BEV] Frame list taken from " << params.cameras[i] << std::endl;
        break;
    }
    if (frames.empty())
    {
        std::cerr << "[BEV] No images found under " << image_root << std::endl;
        return 1;
    }
    sort_frames_by_timestamp(frames);
    if (params.max_frames > 0 && static_cast<int>(frames.size()) > params.max_frames)
        frames.resize(params.max_frames);
    const int n_frames = static_cast<int>(frames.size());
    std::cout << "[BEV] " << n_frames << " frames to process." << std::endl;

    // ---- 3) 多线程逐帧生成 ----
    int n_threads = params.num_threads > 0
                        ? params.num_threads
                        : static_cast<int>(std::thread::hardware_concurrency()) - 2;
    n_threads = std::max(1, std::min(n_threads, n_frames));

    std::atomic<int> task_idx{0};
    std::atomic<int> ok_count{0};
    std::mutex log_mutex;

    auto worker = [&]() {
        cv::Mat bev;
        while (true)
        {
            const int fi = task_idx.fetch_add(1);
            if (fi >= n_frames)
                return;
            if (make_bev_frame(frames[fi], params, luts, cam_index, weight_sum,
                               output_root, per_cam_dir, bev))
            {
                const int done = ok_count.fetch_add(1) + 1;
                if (done % 20 == 0)
                {
                    std::lock_guard<std::mutex> lk(log_mutex);
                    std::cout << "[BEV] " << done << " / " << n_frames << " done." << std::endl;
                }
            }
        }
    };

    std::cout << "[BEV] Generating with " << n_threads << " threads..." << std::endl;
    std::vector<std::thread> threads;
    threads.reserve(n_threads);
    for (int i = 0; i < n_threads; ++i)
        threads.emplace_back(worker);
    for (auto &t : threads)
        t.join();

    std::cout << "\n========================================" << std::endl;
    std::cout << "[BEV] Done. " << ok_count.load() << " / " << n_frames << " BEV images written to "
              << output_root << std::endl;
    return ok_count.load() > 0 ? 0 : 1;
}
