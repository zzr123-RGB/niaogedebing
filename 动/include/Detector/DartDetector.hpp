#ifndef DART_DETECTOR_HPP_
#define DART_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <deque>
#include <chrono>


namespace DartDetect
{


class DartDetector
{
public:
    explicit DartDetector(float ref_x = 720.f);

    void processFrame(const cv::Mat& frame);
    bool hasLanded() const { return landed_; }
    cv::Point2f landingPoint() const { return landing_point_; }

    // 基准像素点（标定得到），用于计算落点偏差
    float ref_x_;

private:
    int frame_count_ = 0;
    std::chrono::steady_clock::time_point prev_frame_time_;  // 上一帧时刻
    float current_fps_ = 0.f;                                  // 当前帧率

    // —— 背景帧差 ——
    static constexpr int motion_threshold_ = 28;     //30  // 帧差灰度阈值
    static constexpr double bg_update_alpha_ = 0.01;   // 背景慢速更新系数（仅静态像素）
    cv::Mat background_gray_f32_;  // 背景（CV_32F 浮点累加器：第一帧初始化，
                                   // 之后只在非运动像素慢速更新）

    // —— 拖影 ——
    static constexpr int motion_trail_frames_ = 5;          // 拖影历史帧数（掩码 OR 桥接断裂）
    static constexpr float trail_disable_motion_ratio_ = 0.30f; // 运动占比超此值关闭拖影
    std::deque<cv::Mat> motion_mask_history_;  // 拖影用历史运动掩码
    std::deque<cv::Mat> trail_frame_history_;  // 拖影用历史 BGR 帧（图像 max）

    // —— 质心位移稳定法落点判定（帧间白色区域质心移动距离）——
    // 飞行时白色区域质心每帧都在移 → 帧间距离大；落地后质心固定 → 距离≈0
    bool last_centroid_valid_ = false;           // 上一帧是否算出了有效质心
    cv::Point2f last_centroid_;                  // 上一帧运动区域质心
    bool was_flying_ = false;                    // 曾出现过大位移（证明"它在飞"）
    bool landed_ = false;                        // 落点闩锁（只认第一次稳定）
    cv::Point2f landing_point_{-1.f, -1.f};     // 落点（闩锁瞬间的运动区域质心）
    cv::Rect landing_rect_;                      // 落点时白色区域的外接矩形
    int land_lost_count_ = 0;                    // 落点后掩码清空的连续帧数（复位用）

    static constexpr float fly_move_dist_ = 15.f;      // 判"在飞"：质心帧间移动距离下限(像素)
    static constexpr float land_still_dist_ = 5.f;     // 判"停下"：质心帧间移动距离上限(像素)
    static constexpr int min_motion_pixels_ = 300;     // 掩码至少含这么多运动像素（有目标在）
    static constexpr float land_max_motion_ratio_ = 0.05f; // 运动占比超此值=曝光跳变，不判落点
    static constexpr int land_reset_lost_frames_ = 15;     // 掩码连续清空这么多帧→复位等下一发
};


} // namespace DartDetect

#endif // DART_DETECTOR_HPP_
