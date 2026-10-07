cmake_minimum_required(VERSION 3.10)
project(Darts_Aimer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE RELEASE)

set(CMAKE_CXX_FLAGS_RELEASE "-O0")

# 找到OpenCV库
set(OpenCV_DIR "/usr/include/opencv4/") 
find_package(OpenCV 4.6.0 REQUIRED)

set(Include_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# 海康威视 MVS SDK 路径（Linux 默认安装在 /opt/MVS）
set(HIK_SDK_INCLUDE_DIR "/opt/MVS/include")
set(HIK_SDK_LIB_DIR "/opt/MVS/lib/64")

# 包含头文件目录
include_directories(
    ${Include_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${HIK_SDK_INCLUDE_DIR})

link_directories(${HIK_SDK_LIB_DIR})

# 添加可执行文件
add_executable(Darts_Aimer 
    src/Aimer.cpp
    src/Camera/Camera.cpp
    src/Camera/HikCamera.cpp
    src/Detector/Guide_light.cpp
    src/Detector/DartDetector.cpp
    src/Filter/OneEuroFilter.cpp
    src/Serial/serial.cpp
)

# 链接 OpenCV + 迈德威视 MVSDK + 海康 MvCameraControl
target_link_libraries(Darts_Aimer ${OpenCV_LIBS} MVSDK MvCameraControl)

# 飞镖镖头检测独立测试程序（海康相机 + DartDetector，不依赖 Aimer/Serial）
add_executable(DartTest
    src/DartTest.cpp
    src/Camera/HikCamera.cpp
    src/Detector/DartDetector.cpp
    src/Filter/OneEuroFilter.cpp
    src/Serial/serial.cpp
)
target_link_libraries(DartTest ${OpenCV_LIBS} MvCameraControl)

message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Found include dir: ${Include_DIR}")

# 如果相机SDK有特定的库需要链接，可以在这里添加
# 例如，如果相机SDK的库名为"CameraSDK"，则添加以下行：
# target_link_libraries(CameraOpenCVDemo CameraSDK)

# 如果相机SDK的头文件和库不在标准路径下，你可能需要设置额外的变量来指定它们的位置
# 例如：
# set(SDK_INCLUDE_DIR "/path/to/sdk/include")
# set(SDK_LIBRARY_DIR "/path/to/sdk/lib")
# include_directories(${SDK_INCLUDE_DIR})
# link_directories(${SDK_LIBRARY_DIR})

# 注意：根据你的相机SDK的具体安装路径和库名称，你可能需要修改上述路径和库名称。
