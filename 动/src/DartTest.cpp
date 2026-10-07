// 飞镖镖头检测独立测试程序
// 只跑下相机（海康）+ DartDetector，不依赖 Aimer 主流程
// 用途：单独调通镖头识别、调参、验证通道相减+运动筛选效果

#include "../include/Camera/HikCamera.hpp"
#include "../include/Detector/DartDetector.hpp"
#include "../include/Serial/serial.hpp"

#include <iostream>
#include <iomanip>


int main(int argc, char* argv[])
{
    
    int camera_index = 0;
    if (argc > 1) {
        camera_index = std::atoi(argv[1]);
    }

    // 基准像素（标定后填入实际值，这里先用画面中心占位）
    float ref_x = 720.f;
    if (argc > 2) {
        ref_x = std::atof(argv[2]);
    }

    std::cout << "=== Dart Tip Detection Test ===" << std::endl;
    std::cout << "Hik camera index : " << camera_index << std::endl;
    std::cout << "Reference x pixel: " << ref_x << std::endl;


    auto HikCam = std::make_shared<HikCameraNode>(camera_index);
    auto DartDetect_ = std::make_shared<DartDetect::DartDetector>(ref_x);
    auto Serial_ = std::make_shared<SerialNode>();

    bool running = true;

    while (running) {
        // 退出：按 q 或 ESC
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            running = false;
            break;
        }

        // 3. 取一帧
        if (HikCam->type == 0 || !HikCam->image_got_completed || !HikCam->image_data_.data()) {
            continue;
        }

        std::unique_lock<std::mutex> lock(HikCam->capture_mutex_);
        cv::Mat frame = cv::Mat(
            HikCam->h, HikCam->w, HikCam->type,
            HikCam->image_data_.data()
        ).clone();
        HikCam->image_got_completed = false;
        lock.unlock();

        // 4. 检测（落点判定与画面标注都在 processFrame 内完成）
        DartDetect_->processFrame(frame);
        DateStruct data;
        data.Flag = 0xFE;
        data.c_x = ref_x;
        data.lv_x = 0.f;
        data.dart_x = 0.f;
        data.dart_state = 0;
        data.target_id = 0;
        data.End = 0xFF;

        if (DartDetect_->hasLanded()) {
            cv::Point2f land = DartDetect_->landingPoint();
            float error = ref_x - land.x;
            data.dart_x = land.x;
            data.dart_state = 2;
            std::cout << std::fixed << std::setprecision(1)
                      << "land_x=" << land.x
                      << "  error(ref_x - land_x)=" << error
                      << std::endl;
        }

        // 发送给下位机
        try {
            Serial_->sendData(data);
        } catch (const std::exception& e) {
            std::cerr << "Serial send error: " << e.what() << std::endl;
        }
    }

    cv::destroyAllWindows();
    std::cout << "Dart test stopped." << std::endl;
    return 0;
}
