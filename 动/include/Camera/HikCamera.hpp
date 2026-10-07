#ifndef DARTS_VISION__HIK_CAMERA_HPP_
#define DARTS_VISION__HIK_CAMERA_HPP_


#include "MvCameraControl.h"  

#include "../TImeLogger/Time_logger.hpp"

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>

#include "opencv2/opencv.hpp"



class HikCameraNode
{
public:
  explicit HikCameraNode(int camera_index = 0);
  ~HikCameraNode();

  
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

  int camera_index_;  // 相机索引（用于多相机区分）
  void* handle_;      // 海康相机句柄（MV_CC_CreateHandle 输出）

  cv::Mat image_msg_;

  // 录像相关（与 MVCameraNode 对齐）
  std::string output_file;
  int codec;
  double fps;
  cv::Size frame_size;
  cv::VideoWriter videoWriter;

  bool first_init = true;
  int fail_count_ = 0;

  std::thread capture_thread_;
  std::shared_ptr<LoggerTimestamp> Time;
};


#endif // DARTS_VISION__HIK_CAMERA_HPP_
