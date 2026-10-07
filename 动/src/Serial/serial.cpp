#include "../../include/Serial/serial.hpp"


SerialNode::SerialNode()
{
    // 获取参数
    device_prefix_ = "ttyUSB";
    baud_rate_ = 115200;
    parity_ = "none";
    stop_bits_ = 1;
    flow_control_ = "none";
    max_reconnect_attempts_ = 0;
    
    // 启动串口线程和发布线程
    run_thread_ = true;

    // 初始打开串口，失败则进入重连逻辑
    if (!openSerialPort()) {
        if (!reopenPort()) {
            std::cout << "串口初始化失败，线程退出" << std::endl;
            return;
        }
    }
    
    std::cout << "SerialNode初始化完成" << std::endl;
}
    
SerialNode::~SerialNode()
{
    // 资源清理
    run_thread_ = false;
    // if (serial_thread_.joinable()) {
    //     serial_thread_.join();
    // }
    // if (publish_thread_.joinable()) {
    //     publish_thread_.join();
    // }
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        std::cout << "串口已关闭" << std::endl;
    }
}

// 自动查找串口设备
std::string SerialNode::findSerialDevice(const std::string& prefix) {
    DIR* dir = nullptr;
    struct dirent* dir_entry = nullptr;
    const char* dev_dir = "/dev/";
    
    // 打开/dev目录
    dir = opendir(dev_dir);
    if (dir == nullptr) {
        printf("无法打开/dev目录: %s\n", strerror(errno));
        return "";
    }
    
    // 遍历目录查找匹配前缀的设备
    while ((dir_entry = readdir(dir)) != nullptr) {
        if (std::strstr(dir_entry->d_name, prefix.c_str()) != nullptr) {
            std::string device_path = std::string(dev_dir) + dir_entry->d_name;
            closedir(dir);
            std::cout << "找到匹配的串口设备: " << device_path.c_str() << std::endl;
            return device_path;
        }
    }
    
    // 未找到设备
    closedir(dir);
    std::cout << "未找到前缀为" << prefix.c_str() << "的串口设备" << std::endl;
    return "";
}
    
    // 波特率转换函数
speed_t SerialNode::getBaudRateConstant(int baud_rate)
{
    switch (baud_rate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        default: 
            printf("不支持的波特率: %d，使用默认115200\n", baud_rate);
            return B115200;
    }
}
    
    // 打开并配置串口
bool SerialNode::openSerialPort()
{
    // 先关闭已打开的端口
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
    
    // 查找串口设备
    std::string device_path = findSerialDevice(device_prefix_);
    if (device_path.empty()) {
        return false;
    }
    
    // 打开串口设备（读写、不成为控制终端、非阻塞）
    serial_fd_ = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ == -1) {
        printf("无法打开串口设备 %s: %s\n", 
                    device_path.c_str(), strerror(errno));
        return false;
    }
    
    // 获取当前串口配置
    struct termios options;
    if (tcgetattr(serial_fd_, &options) != 0) {
        std::cout << "无法获取串口配置: " << strerror(errno) << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    // 设置波特率
    speed_t baud = getBaudRateConstant(baud_rate_);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    
    // 设置数据位为8位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // 设置奇偶校验
    if (parity_ == "even") {
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
    } else if (parity_ == "odd") {
        options.c_cflag |= PARENB | PARODD;
    } else {
        options.c_cflag &= ~PARENB;
    }
    
    // 设置停止位
    if (stop_bits_ == 2) {
        options.c_cflag |= CSTOPB;
    } else {
        options.c_cflag &= ~CSTOPB;
    }
    
    // 设置流控制
    if (flow_control_ == "hardware") {
        options.c_cflag |= CRTSCTS;
    } else {
        options.c_cflag &= ~CRTSCTS;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
    }
    
    // 设置为原始模式（禁用 canonical 模式和回显）
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    
    // 设置超时和最小接收字节数（0.1秒超时，无最小字节要求）
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;
    
    // 应用配置（立即生效）
    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
        std::cout << "无法设置串口配置: " << strerror(errno) << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    printf("串口已成功配置 - 端口: %s, 波特率: %d\n",
                device_path.c_str(), baud_rate_);
    return true;
}
    
// 串口重连函数
bool SerialNode::reopenPort() {
    int attempt = 0;
    while (run_thread_) {
        // 检查是否达到最大重试次数
        if (max_reconnect_attempts_ > 0 && attempt >= max_reconnect_attempts_) {
            printf("达到最大重连次数 %d，停止重试\n", max_reconnect_attempts_);
            return false;
        }
        
        printf("尝试重连串口 (第 %d 次)...\n", attempt + 1);
        if (openSerialPort()) {
            return true;
        }
        
        // 重连失败，等待1秒后重试
        std::this_thread::sleep_for(std::chrono::seconds(1));
        attempt++;
    }  
    return false;
}
    
// 发送DateStruct数据
void SerialNode::sendData(const DateStruct &data)
{
    // // 初始打开串口，失败则进入重连逻辑
    // if (!openSerialPort()) {
    //     if (!reopenPort()) {
    //         std::cout << "串口初始化失败，线程退出" << std::endl;
    //         return;
    //     }
    // }

    std::cout << "send_cx: " << data.c_x << "  "
        << "send_lv_x: " << data.lv_x << "  "
        << "send_dart_x: " << data.dart_x << "  "
        << "send_dart_state: " << (int)data.dart_state << "  "
        << "send_target_id: " << (int)data.target_id << std::endl;
    if (run_thread_) 
    {
        try 
        {
            // 检查串口状态，必要时重新打开
            if (serial_fd_ < 0) {
                std::cout << "串口连接丢失，尝试重连..." << std::endl;
                if (!reopenPort()) {
                    std::cout << "重连失败，线程退出" << std::endl;
                    return;
                }
            }
            
            // 读取并解析GyroDate数据
            // parseGyroData();
            
            // 发送DateStruct数据
            // sendData();
            // 转换为二进制数据
            const uint8_t* binary_data = reinterpret_cast<const uint8_t*>(&data);
            uint data_len = sizeof(DateStruct);
            
            // 发送数据
            uint bytes_written = write(serial_fd_, binary_data, data_len);
            if (bytes_written < 0) {
                std::cout << "串口发送失败: " << strerror(errno) << std::endl;
                // 标记串口异常，触发重连
                close(serial_fd_);
                serial_fd_ = -1;
            } 
            else if (bytes_written != data_len) {
                printf("发送不完整: 预期%d字节,实际%d字节",
                            data_len, bytes_written);
            } 
            else {
                // RCLCPP_INFO(get_logger(), "发送DateStruct成功(%d字节): ID=%d, yaw=%.2f, pitch=%.2f",
                //            data_len, data.number_ID, data.yaw, data.pitch);
            }
            
            // 短暂休眠
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } catch (const std::exception& e) {
            std::cout << "串口线程异常: " << e.what() << std::endl;
            if (serial_fd_ >= 0) {
                close(serial_fd_);
                serial_fd_ = -1;
            }
        }
    }
        
    // }
}
