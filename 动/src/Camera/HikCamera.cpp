#include "../../include/Camera/HikCamera.hpp"


HikCameraNode::HikCameraNode(int camera_index)
    : camera_index_(camera_index), handle_(nullptr)
{
    std::cout << Time->now() << "Starting HikCameraNode (index=" << camera_index_ << ")!" << std::endl;

    output_file = "output_hik_" + std::to_string(camera_index_) + ".avi";
    codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    fps = 60.0;

    try
    {
        init();
    }
    catch (const std::exception& e)
    {
        reopen();
    }

    camera_thread_ = true;
    capture_thread_ = std::thread{[this]() -> void {
        while (camera_thread_) {
            MV_FRAME_OUT st_out_frame;
            memset(&st_out_frame, 0, sizeof(MV_FRAME_OUT));

            // 取一帧，超时1000ms
            int ret = MV_CC_GetImageBuffer(handle_, &st_out_frame, 1000);
            if (ret == MV_OK) {
                unsigned int n_width  = st_out_frame.stFrameInfo.nWidth;
                unsigned int n_height = st_out_frame.stFrameInfo.nHeight;
                MvGvspPixelType n_pixel = st_out_frame.stFrameInfo.enPixelType;

                // 判断像素格式，统一转成 BGR8 写入 image_data_
                unsigned char* p_bgr = nullptr;
                unsigned int n_bgr_size = n_width * n_height * 3;

                if (n_pixel == PixelType_Gvsp_BGR8_Packed) {
                    // 相机直接输出 BGR8，直接拷贝
                    image_data_.assign(
                        (unsigned char*)st_out_frame.pBufAddr,
                        (unsigned char*)st_out_frame.pBufAddr + n_bgr_size);
                } else {
                    // 其他格式（Bayer/Mono等），用SDK转换为 BGR8_Packed
                    if (image_data_.size() < n_bgr_size) {
                        image_data_.resize(n_bgr_size);
                    }
                    MV_CC_PIXEL_CONVERT_PARAM st_convert;
                    memset(&st_convert, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
                    st_convert.nWidth = n_width;
                    st_convert.nHeight = n_height;
                    st_convert.pSrcData = st_out_frame.pBufAddr;
                    st_convert.nSrcDataLen = st_out_frame.stFrameInfo.nFrameLen;
                    st_convert.enSrcPixelType = n_pixel;
                    st_convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
                    st_convert.pDstBuffer = image_data_.data();
                    st_convert.nDstBufferSize = n_bgr_size;

                    int conv_ret = MV_CC_ConvertPixelType(handle_, &st_convert);
                    if (conv_ret != MV_OK) {
                        std::cout << Time->now() << "Convert pixel type failed, ret=" << conv_ret << std::endl;
                        MV_CC_FreeImageBuffer(handle_, &st_out_frame);
                        fail_count_++;
                        if (fail_count_ > 5) reopen();
                        continue;
                    }
                }

                image_got_completed = true;

                if (first_init) {
                    h = n_height;
                    w = n_width;
                    type = CV_8UC3;  // 已统一为 BGR8
                    first_init = false;

                    frame_size = cv::Size(w, h);
                    videoWriter.open(output_file, codec, fps, frame_size);
                }

                image_msg_ = cv::Mat(h, w, type, image_data_.data());

                if (videoWriter.isOpened()) {
                    videoWriter.write(image_msg_);
                }

                MV_CC_FreeImageBuffer(handle_, &st_out_frame);
                fail_count_ = 0;
            } else {
                std::cout << Time->now() << "Failed to get image buffer, ret = " << ret << std::endl;
                fail_count_++;
            }

            if (fail_count_ > 5) {
                std::cout << Time->now() << "Get image failed too many times, reopening..." << std::endl;
                reopen();
            }
        }
    }};
}

HikCameraNode::~HikCameraNode()
{
    camera_thread_ = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    if (handle_ != nullptr) {
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }

    if (videoWriter.isOpened()) {
        videoWriter.release();
    }

    std::cout << Time->now() << "HikCameraNode (index=" << camera_index_ << ") destroyed!" << std::endl;
}

void HikCameraNode::init()
{
    bool init_ok = false;
    while (!init_ok) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        fail_count_ = 0;

        // 1. 枚举设备
        MV_CC_DEVICE_INFO_LIST st_device_list;
        memset(&st_device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &st_device_list);
        std::cout << Time->now() << "Hik enumerate ret = " << ret
                  << ", found " << st_device_list.nDeviceNum << " device(s)" << std::endl;

        if (st_device_list.nDeviceNum == 0) {
            std::cout << Time->now() << "No Hik camera found!" << std::endl;
            continue;
        }
        if ((unsigned int)camera_index_ >= st_device_list.nDeviceNum) {
            std::cout << Time->now() << "Camera index " << camera_index_
                      << " out of range (total " << st_device_list.nDeviceNum << ")" << std::endl;
            continue;
        }

        // 2. 创建句柄
        ret = MV_CC_CreateHandle(&handle_, st_device_list.pDeviceInfo[camera_index_]);
        if (ret != MV_OK) {
            std::cout << Time->now() << "Create handle failed, ret = " << ret << std::endl;
            continue;
        }

        // 3. 打开设备
        ret = MV_CC_OpenDevice(handle_);
        if (ret != MV_OK) {
            std::cout << Time->now() << "Open device failed, ret = " << ret << std::endl;
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            continue;
        }

        // 4. 配置参数（与 MVCameraNode 对齐：手动曝光3000us、增益7、RGB增益112/122/88）
        // 触发模式：连续采集
        MV_CC_SetEnumValue(handle_, "TriggerMode", 0);

        // 像素格式：优先 BGR8_Packed，采集线程里也做了格式兜底
        MV_CC_SetEnumValue(handle_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);

        // 关闭自动曝光，设置手动曝光
        MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
        double exposure_time = 3000.0;
        MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_time);
        std::cout << Time->now() << "Exposure time = " << exposure_time << std::endl;

        // 关闭自动增益，设置模拟增益
        MV_CC_SetEnumValue(handle_, "GainAuto", 0);
        double analog_gain = 7.0;
        MV_CC_SetFloatValue(handle_, "Gain", analog_gain);
        std::cout << Time->now() << "Gain = " << analog_gain << std::endl;

        // RGB 增益（白平衡）—— 海康用 BalanceRatio 接口
        MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", 0);
        MV_CC_SetFloatValue(handle_, "BalanceRatioSelector", 0.0);  // 0=Red
        MV_CC_SetFloatValue(handle_, "BalanceRatio", 112.0);
        MV_CC_SetFloatValue(handle_, "BalanceRatioSelector", 1.0);  // 1=Green
        MV_CC_SetFloatValue(handle_, "BalanceRatio", 122.0);
        MV_CC_SetFloatValue(handle_, "BalanceRatioSelector", 2.0);  // 2=Blue
        MV_CC_SetFloatValue(handle_, "BalanceRatio", 88.0);
        std::cout << Time->now() << "RGB Gain: R=112, G=122, B=88" << std::endl;

        // 饱和度、Gamma
        MV_CC_SetIntValue(handle_, "Saturation", 80);
        MV_CC_SetIntValue(handle_, "Gamma", 150);

        // 5. 开始取流
        ret = MV_CC_StartGrabbing(handle_);
        if (ret != MV_OK) {
            std::cout << Time->now() << "Start grabbing failed, ret = " << ret << std::endl;
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            continue;
        }

        init_ok = true;
        std::cout << Time->now() << "HikCamera (index=" << camera_index_ << ") init OK" << std::endl;
    }
}

void HikCameraNode::reopen()
{
    std::cout << Time->now() << "Attempting to reopen Hik camera (index=" << camera_index_ << ")" << std::endl;

    // 先释放现有资源
    if (handle_ != nullptr) {
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }

    while (true) {
        try {
            if (fail_count_ > 5) {
                init();
            }
            std::cout << Time->now() << "Successfully reopened Hik camera" << std::endl;
            break;
        }
        catch (const std::exception& ex) {
            std::cout << Time->now() << "Error while reopening Hik camera: " << ex.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

std::vector<uchar> HikCameraNode::get_cameraImage()
{
    std::lock_guard<std::mutex> lock(capture_mutex_);
    return image_data_;
}
