#ifndef GUIDE_LIGHT_DETECTOR__LIGHT_CLASS_HPP_
#define GUIDE_LIGHT_DETECTOR__LIGHT_CLASS_HPP_

#include "../Filter/OneEuroFilter.hpp"

#include <opencv2/opencv.hpp>

inline cv::Scalar lower_, upper_;

namespace GuideLight
{

struct TrackedBlob {

    cv::Point2f center;

    float max_r;

    double circularity;

    int id;

    int last_seen; // 帧序号，用于判断是否消失

};


class GuideLight
{
public:
    GuideLight(cv::Scalar lower, cv::Scalar upper);
        // : lower_(lower), upper_(upper), next_id_(0), frame_count_(0) {}

    std::vector<TrackedBlob> processFrame(const cv::Mat& frame);

    void drawBlob(cv::Mat& frame, const TrackedBlob& blob);

    void drawBlobs(cv::Mat& frame, const std::vector<TrackedBlob>& blobs);

    cv::Point2f Filter(std::vector<TrackedBlob>& current);

    double dt_ = 0.013;
private:

    void assignIds(std::vector<TrackedBlob>& current);

    double calcu_circularity(const std::vector<cv::Point> &contour);

    void cleanupOldBlobs(std::vector<TrackedBlob>& current);

    // void assignIds(std::vector<TrackedBlob>& current) {

    //     // 简单最近邻匹配（匈牙利算法更优，但此处简化）

    //     std::vector<bool> matched_prev(prev_blobs_.size(), false);

    //     std::vector<bool> matched_curr(current.size(), false);


    //     // 计算距离矩阵并贪心匹配

    //     for (size_t i = 0; i < current.size(); ++i) {
    //         float best_dist = 50.0f; // 最大匹配距离
    //         int best_j = -1;
    //         for (size_t j = 0; j < prev_blobs_.size(); ++j) {
    //             if (matched_prev[j]) continue;
    //             float dx = current[i].center.x - prev_blobs_[j].center.x;
    //             float dy = current[i].center.y - prev_blobs_[j].center.y;
    //             float dist = std::sqrt(dx*dx + dy*dy);
    //             if (dist < best_dist) {
    //                 best_dist = dist;
    //                 best_j = j;
    //             }
    //         }
    //         if (best_j != -1) {
    //             current[i].id = prev_blobs_[best_j].id;
    //             matched_prev[best_j] = true;
    //             matched_curr[i] = true;
    //         }

    //     }

    //     // 为未匹配的 blob 分配新 ID
    //     for (size_t i = 0; i < current.size(); ++i) {
    //         if (!matched_curr[i]) {
    //             current[i].id = next_id_++;
    //         }
    //     }
    // }

    // inline cv::Scalar lower_, upper_;

    int next_id_ = 0;

    int frame_count_ = 0;

    std::vector<TrackedBlob> prev_blobs_;
    std::vector<TrackedBlob> blobs_prev_lost_;
    int last_tracked_id;

    static constexpr float min_area_ = 188.0f;   // 最小面积阈值
    static constexpr float max_area_ = 2000.0f;   // 最大面积阈值

    static constexpr int max_missing_frames_ = 20; // 最多容忍消失10帧

    std::shared_ptr<OneEuroFilter> EuroFilter;
    bool init_Filter = false;

    const cv::Mat MORPH_ELLIPSEElement4 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
    const cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));

};








}

#endif // GUIDE_LIGHT_DETECTOR__LIGHT_CLASS_HPP_