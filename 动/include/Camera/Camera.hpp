#ifndef GUIDE_LIGHT_DETECTOR__CAMERA_HPP_
#define GUIDE_LIGHT_DETECTOR__CAMERA_HPP_


#include "CameraApi.h" // 相机SDK的API头文件

#include "../TImeLogger/Time_logger.hpp"

#include <iostream>
#include <thread>
#include <atomic>

#include "opencv2/opencv.hpp"


class MVCameraNode
{
public:
  int i;
  explicit MVCameraNode();
  ~MVCameraNode();
  std::vector<uchar> get_cameraImage();

  int h = 0;
  int w = 0;
  int type = 0;
  std::vector<uchar> image_data_;
  std::mutex capture_mutex_;
  std::atomic<bool> camera_thread_{false};

  bool image_got_completed = false;

private:
  void init();

  void reopen();

  int h_camera_;
  uint8_t * pby_buffer_;
  tSdkCameraCapbility t_capability_;  // 设备描述信息
  tSdkFrameHead s_frame_info_;        // 图像帧头信息

  cv::Mat image_msg_;

  std::string output_file = "output_video25.avi"; // 视频文件的保存路径和名称
  int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // MJPG编码器
  double fps = 60.0; // 帧率，根据实际情况调整
  cv::Size frame_size;
  cv::VideoWriter videoWriter;

  // image_transport::CameraPublisher camera_pub_;

  // RGB Gain
  int r_gain_, g_gain_, b_gain_;

  bool flip_image_;

  std::string camera_name_;
  // std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  // sensor_msgs::msg::CameraInfo camera_info_msg_;

  bool first_init = true;
  int fail_conut_ = 0;

  std::thread capture_thread_;


  std::shared_ptr<LoggerTimestamp> Time;

};



#endif // GUIDE_LIGHT_DETECTOR__CAMERA_HPP_