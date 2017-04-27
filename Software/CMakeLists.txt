project(robot)
cmake_minimum_required(VERSION 3.5)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
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
  navigate/cell.cpp
  navigate/floormap.cpp
  navigate/greedy_dijkstra.cpp
  navigate/linefitting.cpp
  navigate/mazemap_gen.cpp
  navigate/mazemap.cpp
  navigate/navigateMap.cpp
  navigate/navigate2D.cpp
  navigate/navigate_simul.cpp
  navigate/textDetect.cpp
  xml/tinystr.cpp
  xml/tinyxml.cpp
  xml/tinyxmlparser.cpp
  xml/tinyxmlerror.cpp
)

#include path
include_directories(
  common
  imu
  navigate
  xml
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
