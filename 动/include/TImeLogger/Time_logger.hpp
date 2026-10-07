#ifndef TIME_LOGGER_HPP_
#define TIME_LOGGER_HPP_

#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>


class LoggerTimestamp {
public:

    std::string now() {

        auto now = std::chrono::system_clock::now();

        auto time_t_now = std::chrono::system_clock::to_time_t(now);

        

        // 转换为 UTC 时间

        std::tm utc_tm = *std::gmtime(&time_t_now);


        // 提取纳秒部分（用于毫秒级精度）

        auto duration = now.time_since_epoch();

        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() % 1000;


        // 使用 ostringstream 格式化输出

        std::ostringstream oss;

        oss << std::put_time(&utc_tm, "%Y-%m-%d %H:%M:%S") 

            << "." << std::setw(3) << std::setfill('0') << milliseconds 

            << ": ";


        return oss.str();

    }
};

#endif //TIME_LOGGER_HPP_