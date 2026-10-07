#include "../../include/Detector/DartDetector.hpp"


namespace DartDetect
{


DartDetector::DartDetector(float ref_x)
    : ref_x_(ref_x)
{
}

void DartDetector::processFrame(const cv::Mat& frame)
{
    frame_count_++;

    
   auto start_time = std::chrono::steady_clock::now();

   
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Mat diff;   
    cv::Mat motion_mask; 
    if (background_gray_f32_.empty()) {
        // 第一帧：存为背景（转浮点累加器）；没有可作差的前帧，差值/掩码全黑
        gray.convertTo(background_gray_f32_, CV_32F);
        diff = cv::Mat::zeros(gray.size(), CV_8UC1);
        motion_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
    } else {
        // 3) 后续每帧都与背景作差（背景先转回 8bit 再逐像素绝对差）
        cv::Mat bg_u8;
        cv::convertScaleAbs(background_gray_f32_, bg_u8);
        cv::absdiff(gray, bg_u8, diff);

        // 4) 对差值二值化：>阈值 → 255(运动区域)，否则 → 0(非运动区域)
        cv::threshold(diff, motion_mask,
                      motion_threshold_, 255, cv::THRESH_BINARY);
    }

    
    int motion_pixels = 0;
    float motion_ratio = 0.f;
    if (!motion_mask.empty()) {
        motion_pixels = cv::countNonZero(motion_mask);
        motion_ratio = static_cast<float>(motion_pixels)
                       / static_cast<float>(motion_mask.total());
    }

    
    cv::Mat trail_mask = motion_mask.clone();
    const bool suppress_trail = motion_ratio >= trail_disable_motion_ratio_;

    if (motion_trail_frames_ > 0 && !suppress_trail) {
        // 把当前帧的掩码和 BGR 画面入队
        motion_mask_history_.push_back(motion_mask.clone());
        trail_frame_history_.push_back(frame.clone());


        
        cv::putText(frame,
                    "FPS:" + std::to_string(static_cast<int>(current_fps_)),
                    cv::Point(frame.cols - 180, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

        const size_t max_history = static_cast<size_t>(motion_trail_frames_ + 1);
        while (motion_mask_history_.size() > max_history)
            motion_mask_history_.pop_front();
        while (trail_frame_history_.size() > max_history)
            trail_frame_history_.pop_front();

        const size_t n = motion_mask_history_.size();
        if (n > 1 && n == trail_frame_history_.size()) {
           
            for (size_t i = 0; i + 1 < n; ++i)
                cv::bitwise_or(trail_mask, motion_mask_history_[i], trail_mask);

            
            cv::Mat trail_img = frame.clone();
            for (size_t i = 0; i + 1 < n; ++i)
                cv::max(trail_img, trail_frame_history_[i], trail_img);
            trail_img.copyTo(frame, trail_mask);
        }
    } else {
        motion_mask_history_.clear();
        trail_frame_history_.clear();
    }
    if (frame_count_ > 1) {
        cv::Mat static_mask;
        cv::bitwise_not(motion_mask, static_mask);
        cv::accumulateWeighted(gray, background_gray_f32_,
                               bg_update_alpha_, static_mask);
    }
    cv::Point2f centroid;
    bool centroid_valid = false;
    if (motion_pixels > 0) {
        cv::Moments m = cv::moments(motion_mask, true);  
        if (m.m00 > 1e-6) {
            centroid = cv::Point2f(
                static_cast<float>(m.m10 / m.m00),
                static_cast<float>(m.m01 / m.m00));
            centroid_valid = true;
        }
    }
    float move_dist = -1.f;
    if (centroid_valid && last_centroid_valid_)
        move_dist = static_cast<float>(
            cv::norm(centroid - last_centroid_));

    if (!landed_) {
        if (move_dist > fly_move_dist_)
            was_flying_ = true;
        if (was_flying_
            && move_dist >= 0.f
            && move_dist < land_still_dist_
            && motion_pixels > min_motion_pixels_
            && motion_ratio < land_max_motion_ratio_)
        {
            landed_ = true;
           
            landing_point_ = centroid;
            landing_rect_ = cv::boundingRect(motion_mask);
            std::cout << "[Dart] LANDED at (" << landing_point_.x
                      << ", " << landing_point_.y << ")"
                      << "  err(ref_x - x)="
                      << (ref_x_ - landing_point_.x) << std::endl;
        }
    } else {
        // 已判定落点：掩码连续清空（镖被拿走）若干帧 → 复位，等下一发
        if (motion_pixels == 0) {
            if (++land_lost_count_ >= land_reset_lost_frames_) {
                landed_ = false;
                was_flying_ = false;
                last_centroid_valid_ = false;
                landing_point_ = cv::Point2f(-1.f, -1.f);
                landing_rect_ = cv::Rect();
                land_lost_count_ = 0;
                std::cout << "[Dart] landing reset" << std::endl;
            }
        } else {
            land_lost_count_ = 0;
        }
    }
    if (centroid_valid) {
        last_centroid_ = centroid;
        last_centroid_valid_ = true;
    } else {
        last_centroid_valid_ = false;
    }
    if (landed_ && landing_point_.x >= 0) {
        cv::rectangle(frame, landing_rect_, cv::Scalar(0, 0, 255), 2);
        cv::drawMarker(frame, landing_point_, cv::Scalar(0, 0, 255),
                       cv::MARKER_CROSS, 20, 2);
        cv::putText(frame,
                    "LANDING x=" + std::to_string((int)landing_point_.x),
                    cv::Point(landing_rect_.x + landing_rect_.width + 5,
                              (int)landing_point_.y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        cv::line(frame, landing_point_, cv::Point2f(ref_x_, landing_point_.y),
                 cv::Scalar(255, 0, 255), 2);
        cv::putText(frame,
                    "err=" + std::to_string((int)(ref_x_ - landing_point_.x)),
                    cv::Point((int)((landing_point_.x + ref_x_) / 2) - 20,
                              (int)landing_point_.y - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);
    }
    cv::line(frame,
             cv::Point((int)ref_x_, 0),
             cv::Point((int)ref_x_, frame.rows),
             cv::Scalar(0, 255, 0), 2);
    cv::putText(frame, "ref=" + std::to_string((int)ref_x_),
                cv::Point((int)ref_x_ + 6, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);


                std::cout << "[Dart] frame=" << frame_count_
              << " motion_ratio=" << motion_ratio << std::endl;
    cv::putText(frame,
                "motion:" + std::to_string(
                    static_cast<int>(motion_ratio * 100.f)) + "%",
                cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
    auto end_time = std::chrono::steady_clock::now();
    double cost_ms = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
    current_fps_ = (cost_ms > 0.001) ? static_cast<float>(1000.0 / cost_ms) : 0.f;
    cv::imshow("dart_camera_raw", frame);
    cv::imshow("dart_motion_mask", motion_mask);

    // 比例同时打印到终端和画面（百分比）
    // std::cout << "[Dart] frame=" << frame_count_
    //           << " motion_ratio=" << motion_ratio << std::endl;
    // cv::putText(frame,
    //             "motion:" + std::to_string(
    //                 static_cast<int>(motion_ratio * 100.f)) + "%",
    //             cv::Point(20, 40),
    //             cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

    // 轮廓 / ID / 滤波等后续步骤按需再添加
}

} // namespace DartDetect
