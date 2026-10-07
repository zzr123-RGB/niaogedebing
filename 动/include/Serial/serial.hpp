#ifndef DARTS_SERIAL_HPP_
#define DARTS_SERIAL_HPP_

#include <iostream>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <dirent.h>  // 用于目录遍历


// 合并帧：引导灯 + 飞镖镖头数据一帧发送（16 字节 packed）
struct DateStruct {
    uint8_t Flag;        // 0xFE 帧头
    float c_x;           // 引导灯瞄准基准 = 1224 + 落点修正量
    float lv_x;          // 引导灯质心 x（丢失帧 = 0）
    float dart_x;        // 预测落点 x（飞行中位置+速度外推；接触后锁定首触点；无效=0）
    uint8_t dart_state;  // 0=无目标 1=飞行中(实时预测) 2=首次接触预测点已锁定（弹跳帧不更新）
    uint8_t target_id;   // 0=前哨站 1=基地（丢失时按交替逻辑）
    uint8_t End;         // 0xFF 帧尾
} __attribute__((packed));

class SerialNode
{
public:
    SerialNode();
    ~SerialNode();

    void sendData(const DateStruct &data);
private:
    // 自动查找串口设备
    std::string findSerialDevice(const std::string& prefix);
    
    // 波特率转换函数
    speed_t getBaudRateConstant(int baud_rate);
    
    // 打开并配置串口
    bool openSerialPort();
    
    // 串口重连函数
    bool reopenPort();
    
    // 串口线程函数
    // void serialThread();
    
    // 发送DateStruct数据
    
    int serial_fd_{-1};
    std::string device_prefix_;  // 设备前缀，而非完整路径
    int baud_rate_;
    std::string parity_;
    int stop_bits_;
    std::string flow_control_;
    int max_reconnect_attempts_;  // 最大重连次数
    
    std::atomic<bool> run_thread_{false};
    
    // 接收缓冲区及锁
    size_t recv_idx_{0};
};

#endif // DARTS_SERIAL_HPP_