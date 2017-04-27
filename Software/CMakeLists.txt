project(robot)
cmake_minimum_required(VERSION 3.5)
set (OpenCV_DIR /home/alex/opencv-3.1.0/build)

# source code
set(SRCS
  common/ARobot.cpp
  common/IMUData.cpp
  common/LightData.cpp
  common/TempData.cpp
  common/processThread.cpp
  common/RangeData.cpp
  common/main.cpp
  common/SerialPort.cpp
  common/Thread.cpp
  common/UartRx.cpp
  common/UartTx.cpp
  imu/MadgwickAHRS.cpp
  imu/MahonyAHRS.c
)

#include path
include_directories(
  common
  imu
)

# need pthread
find_package(Threads)

# opencv
find_package(OpenCV REQUIRED)

# boost
find_package(Boost COMPONENTS system filesystem REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIR}
  #${PROJECT_SOURCE_DIR}/eigen_dir
  #/usr/local/include/eigen_dir
)

link_directories(${Boost_LIBRARY_DIR})

# executable
add_executable(${PROJECT_NAME} ${SRCS})

# link pthread library
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBS}
  ${CMAKE_THREAD_LIBS_INIT} 
  ${Boost_LIBRARIES}
)
