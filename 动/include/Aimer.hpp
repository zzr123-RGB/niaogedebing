#ifndef DARTS_AIMER_HPP_
#define DARTS_AIMER_HPP_

#include "Camera/Camera.hpp"
#include "Camera/HikCamera.hpp"
#include "Detector/Guide_light.hpp"
#include "Detector/DartDetector.hpp"
#include "Filter/OneEuroFilter.hpp"
#include "Serial/serial.hpp"
#include "TImeLogger/Time_logger.hpp"



// inline cv::Scalar lower_;
// inline cv::Scalar upper_;
class Aimer
{
public:
    Aimer();

    enum class TARGET { OutPost, Base };

    ~Aimer() = default;

private:

    bool Aimer_thread;

    cv::Mat image_;


    double dt_ = 0.0133333;
 
    // —— 引导灯链路（上相机：迈德）——
    std::shared_ptr<MVCameraNode> MVCamera_;
    std::shared_ptr<GuideLight::GuideLight> GuideLight_;

    // —— 飞镖镖头链路（下相机：海康）——
    std::shared_ptr<HikCameraNode> HikCamera_;
    std::shared_ptr<DartDetect::DartDetector> DartDetect_;
    cv::Mat dart_image_;
    float dart_ref_x_ = 720.f;   // 飞镖链路基准像素（标定后填入）

    // —— 落点闭环修正（上位机融合：首次接触"预测点"偏差叠加到引导灯帧 c_x）——
    float dart_correction_ = 0.f;       // 锁定的预测落点偏差 Δx，随下一次接触更新
    float dart_correction_gain_ = 1.f;  // 方向/比例系数（上机标定，方向反了改 -1）
    // 首次接触判定（NIS 突变 + 飞行速度 + 减速方向）在 DartDetector 内完成，
    // 落点为位置+速度外推的预测点，通过 isImpacted() / impactPosition() 查询，
    // 弹跳帧不更新锁定点

    // —— 共用串口（两条链路都通过它向下位机发 DateStruct）——
    std::shared_ptr<SerialNode> Serial_;

    const cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));


    bool target_init_ = false;
    std::chrono::_V2::system_clock::time_point last_tracked_time_;
    TARGET last_target_;
    TARGET target_;

    double lost_time_thr_;
    double lost_time_;

    // int s_min = 40;
    // int s_max = 255;
    // int v_min = 40;
    // int v_max = 255;

    // void on_trackbar(int, void*);

};



#endif // DARTS_AIMER_HPP_