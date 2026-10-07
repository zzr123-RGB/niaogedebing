// #include "CameraApi.h" // 相机SDK的API头文件
// #include <cstdint>
// #include "opencv2/opencv.hpp"
// // #include "opencv2/core.hpp"       // 替换为新的头文件
// // #include "opencv2/highgui.hpp"    // 替换为新的头文件
// // #include "opencv2/imgproc.hpp"    // 添加图像处理模块的头文件
// #include <stdio.h>
// #include <iostream>
// #include <vector>
// #include <ctime>
// #include <string>

// using namespace cv;
// using namespace std;

// unsigned char *g_pRgbBufferLeft;     // 左相机处理后数据缓存区
// int main() {
//     int iCameraCounts = 1;
//     int iStatus = -1;
//     tSdkCameraDevInfo tCameraEnumList[1]; // 相机枚举信息
//     int hCameraLeft, hCameraRight;
//     tSdkCameraCapbility tCapabilityLeft, tCapabilityRight; // 左右相机的设备描述信息
//     tSdkFrameHead sFrameInfoLeft, sFrameInfoRight;
//     BYTE* pbyBufferLeft, *pbyBufferRight;
//     int iDisplayFrames = 10000;
//     int channelLeft = 3, channelRight = 3;

//     CameraSdkInit(1);

//     // 枚举设备，并建立设备列表
//     iStatus = CameraEnumerateDevice(tCameraEnumList, &iCameraCounts);
//     printf("state = %d\n", iStatus);
//     printf("count = %d\n", iCameraCounts);

//     if (iStatus != CAMERA_STATUS_SUCCESS) {
//         printf("Failed to enumerate devices. Status code: %d\n", iStatus);
//         CameraUnInit(0); // 释放SDK资源
//         return -1;
//     }

//     if (iCameraCounts < 1) {
//         printf("Not enough cameras connected. Need at least 1 cameras.\n");
//         CameraUnInit(0); // 释放SDK资源
//         return -1;
//     }

//     // 左相机初始化
//     iStatus = CameraInit(&tCameraEnumList[0], -1, -1, &hCameraLeft);
//     printf("Left camera init state = %d\n", iStatus);
//     if (iStatus != CAMERA_STATUS_SUCCESS) {
//         printf("Failed to initialize left camera. Status code: %d\n", iStatus);
//         CameraUnInit(0); // 释放SDK资源
//         return -1;
//     }

//     // 获取左相机的特性描述结构体
//     CameraGetCapability(hCameraLeft, &tCapabilityLeft);
//     g_pRgbBufferLeft = (unsigned char*)malloc(tCapabilityLeft.sResolutionRange.iHeightMax * tCapabilityLeft.sResolutionRange.iWidthMax * channelLeft);

//     // 让SDK进入工作模式，开始接收来自相机发送的图像数据
//     CameraPlay(hCameraLeft);

//     CameraSetIspOutFormat(hCameraLeft, CAMERA_MEDIA_TYPE_BGR8);

//     Mat L;

//     // 设置相机参数
//     CameraSetAeState(hCameraLeft, false); // 关闭自动曝光
//     CameraSetExposureTime(hCameraLeft, 10000);// 设置曝光时间
//     CameraSetWbMode(hCameraLeft, FALSE); // 关闭自动白平衡

//     int redGain = 133;   // 红色增益，表示2倍
//     int greenGain = 112; // 绿色增益，表示2.5倍
//     int blueGain = 106;  // 蓝色增益，表示1.8倍

//     // 设置左相机的白平衡增益
//     CameraSdkStatus statusLeft = CameraSetGain(hCameraLeft, redGain, greenGain, blueGain);

//     // 计数器，用于生成唯一的文件名
//     int frameCounter = 0;
//     while (1) {
//         // 获取左相机图像
//         if (CameraGetImageBuffer(hCameraLeft, &sFrameInfoLeft, &pbyBufferLeft, 1000) == CAMERA_STATUS_SUCCESS) {
//             CameraImageProcess(hCameraLeft, pbyBufferLeft, g_pRgbBufferLeft, &sFrameInfoLeft);

//             cv::Mat matImageLeft(
//                 Size(sFrameInfoLeft.iWidth, sFrameInfoLeft.iHeight),
//                 sFrameInfoLeft.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
//                 g_pRgbBufferLeft
//             );
//             L = matImageLeft;

//             CameraReleaseImageBuffer(hCameraLeft, pbyBufferLeft);
//         }

//         // 调整图像大小
//         cv::resize(L, L, Size(), 1, 1);  // 将图像等比例缩小为原来的一半


//         // 显示图像
//         cv::namedWindow("Left Camera", cv::WINDOW_NORMAL);
//         imshow("Left Camera", L);
//         cv::waitKey(1);
//     }

//     // 释放资源
//     CameraUnInit(hCameraLeft);

//     free(g_pRgbBufferLeft);

//     return 0;
// }


// #include "CameraApi.h" // 相机SDK的API头文件

// #include <iostream>
// #include <thread>

// #include "opencv2/opencv.hpp"

#include "../../include/Camera/Camera.hpp"
// #include "../../include/TImeLogger/Time_logger.hpp"


MVCameraNode::MVCameraNode()
{
    std::cout << Time->now() << "Starting MVCameraNode!" << std::endl;
    i = 0;


    flip_image_ = false;

    try
    {
        init();
    }
    catch(const std::exception& e)
    {
        reopen();
    }

    camera_thread_ = true;
    capture_thread_ = std::thread{[this]() -> void {
    //   RCLCPP_INFO(this->get_logger(), "Publishing image!");
    // cv::namedWindow("image_msg_", cv::WINDOW_NORMAL);

        while (camera_thread_) {
            // image_got_completed = false;
            // std::unique_lock<std::mutex> lock(capture_mutex_);
            int status = CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 1000);
            if (status == CAMERA_STATUS_SUCCESS || status == -18) {
            CameraImageProcess(h_camera_, pby_buffer_, image_data_.data(), &s_frame_info_);
            if (flip_image_) {
                CameraFlipFrameBuffer(image_data_.data(), &s_frame_info_, 3);
            }
            // std::cout << Time->now() << 
            //     "image_data_.size(): " 
            //     << image_data_.size() 
            //     << std::endl;
            image_got_completed = true;

            // image_msg_.rows = s_frame_info_.iHeight;
            // image_msg_.cols = s_frame_info_.iWidth;
            // image_msg_.step = s_frame_info_.iWidth * 3;
            if(first_init)
            {
                int h = s_frame_info_.iHeight;
                int w = s_frame_info_.iWidth;
                int channels = (s_frame_info_.uiMediaType == CAMERA_MEDIA_TYPE_MONO8) ? 1 : 3;
                int type = (channels == 1) ? CV_8UC1 : CV_8UC3;
                this->h = h;
                this->w = w;
                this->type = type;
            }

            image_msg_ = cv::Mat(
                this->h,
                this->w,
                this->type,
                image_data_.data()
            );

            // std::cout << image_msg_.size() << std::endl;


            // cv::imshow("image_msg_", image_msg_);
            videoWriter.write(image_msg_);
            // cv::waitKey(1);

            std::cout << Time->now() << ++i << std::endl;

            // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
            // 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，
            // 直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
            CameraReleaseImageBuffer(h_camera_, pby_buffer_);
            fail_conut_ = 0;
            } else {
                std::cout << Time->now() << "Failed to get image buffer, status = " << status << std::endl;
                fail_conut_++;
            }
            if (fail_conut_ > 5) {
                std::cout << Time->now() << "Failed to get image buffer, goto loop!" << std::endl;
                reopen();
            // rclcpp::shutdown();
            }
        }
    }};

}

MVCameraNode::~MVCameraNode()
{
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    camera_thread_ = false;

    CameraUnInit(h_camera_);

    videoWriter.release(); // 释放视频写入对象

    std::cout << Time->now() << "Camera node destroyed!" << std::endl;
}

void MVCameraNode::init()
{
    bool init = false;
    while(!init)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        fail_conut_ = 0;
        CameraSdkInit(1);

        // 枚举设备，并建立设备列表
        int i_camera_counts = 1;
        int i_status = -1;
        tSdkCameraDevInfo t_camera_enum_list;
        i_status = CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts);
        std::cout << Time->now() << "Enumerate state = " << i_status << std::endl;
        std::cout << Time->now() << "Found camera count = " << i_camera_counts << std::endl;

        // 没有连接设备
        if (i_camera_counts == 0) {
            std::cout << Time->now() << "No camera found!" << std::endl;
            // goto loop;
            // return;
        }
        else{
            init = true;
        }

        // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
        i_status = CameraInit(&t_camera_enum_list, -1, -1, &h_camera_);

        // 内录初始化
        frame_size = cv::Size(t_capability_.sResolutionRange.iWidthMax, t_capability_.sResolutionRange.iHeightMax);
        frame_size = cv::Size(2448, 2048);
        videoWriter = cv::VideoWriter(output_file, codec, fps, frame_size);

        // 初始化失败
        std::cout << Time->now() << "Init state = " << i_status << std::endl;
        if (i_status != CAMERA_STATUS_SUCCESS && i_status != -18 && !videoWriter.isOpened()) {
            std::cout << Time->now() << "Init failed!" << std::endl;
            videoWriter.release(); // 释放视频写入对象
            // goto loop;
            // return;
        }
        else
        {
            init = true;
        }
        std::string output_file = "output_video25.avi"; // 视频文件的保存路径和名称
        int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // MJPG编码器
        double fps = 60.0; // 帧率，根据实际情况调整

        // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
        CameraGetCapability(h_camera_, &t_capability_);

        // 直接使用vector的内存作为相机输出buffer
        image_data_.reserve(
            t_capability_.sResolutionRange.iHeightMax * t_capability_.sResolutionRange.iWidthMax * 3);

        // 设置手动曝光
        CameraSetAeState(h_camera_, false);
        // NoiseFilter
        CameraSetNoiseFilter(h_camera_, 0);  

        double exposure_time = 3000;
        CameraSetExposureTime(h_camera_, exposure_time);
        std::cout << Time->now() << "Exposure time = " << exposure_time << std::endl;

        int analog_gain = 7;
        CameraSetAnalogGain(h_camera_, analog_gain);
        std::cout << Time->now() << "Analog gain = " << analog_gain << std::endl;

        CameraSetWbMode(h_camera_, FALSE);
        // CameraGetGain(h_camera_, &r_gain_, &g_gain_, &b_gain_);

        b_gain_ = 88;
        g_gain_ = 122;
        r_gain_ = 112;
        CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
        std::cout << Time->now() << "RGB Gain: R = " << r_gain_ << std::endl;
        std::cout << Time->now() << "RGB Gain: G = " << g_gain_ << std::endl;
        std::cout << Time->now() << "RGB Gain: B = " << b_gain_ << std::endl;

        int saturation = 80;
        CameraSetSaturation(h_camera_, saturation);
        std::cout << Time->now() << "Saturation = " << saturation << std::endl;

        int gamma = 150;
        CameraSetGamma(h_camera_, gamma);
        std::cout << Time->now() << "Gamma = " << gamma << std::endl;
        // Declare camera parameters
        // if(first_init)
        // {
        //   declareParameters();
        //   first_init = false;
        // }
        // 让SDK进入工作模式，开始接收来自相机发送的图像
        // 数据。如果当前相机是触发模式，则需要接收到
        // 触发帧以后才会更新图像。
        CameraPlay(h_camera_);

        // CameraSetFrameSpeed(h_camera_, t_capability_.iFrameSpeedDesc);
        CameraSetFrameSpeed(h_camera_, t_capability_.iFrameSpeedDesc - 1);
        // CameraSetFrameSpeed(h_camera_, 0);
        std::cout << Time->now() << "t_capability_.iFrameSpeedDesc - 1: " << t_capability_.iFrameSpeedDesc - 1 << std::endl;

        CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_BGR8);
        // CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_BAYGR8);

        // Create camera publisher
    }

}

void MVCameraNode::reopen() {
    std::cout << Time->now() << "Attempting to reopen camera" << std::endl;

    while (1) {
        try {
            if (fail_conut_ > 5) {
                init();
            }
            std::cout << Time->now() << "Successfully reopened camera" << std::endl;
            break;
        }
        catch (const std::exception& ex) {
            std::cout << Time->now() << "Error while reopening camera: " << ex.what() << std::endl;

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}



// int main(int argc, char const *argv[])
// {
//   MVCameraNode MVCamera;
//   return 0;
// }