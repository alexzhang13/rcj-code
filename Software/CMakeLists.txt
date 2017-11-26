project(robot)
cmake_minimum_required(VERSION 3.5)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
set (OpenCV_DIR /home/alex/opencv-3.1.0/build)
set (raspicam_DIR /home/alex/raspicam-0.1.6/build)
# source code
set(SRCS
  build/common/_src/ARobot.cpp
  build/common/_src/IMUData.cpp
  build/common/_src/LightData.cpp
  build/common/_src/TempData.cpp
  build/common/_src/processThread.cpp
  build/common/_src/navThread.cpp
  build/common/_src/RangeData.cpp
  build/common/_src/main.cpp
  build/common/_src/SerialPort.cpp
  build/common/_src/Thread.cpp
  build/common/_src/UartRx.cpp
  build/common/_src/UartTx.cpp
  build/common/picamera.cpp
  imu/MadgwickAHRS.cpp
  imu/MahonyAHRS.c
  knn/kNNFilter.cpp
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
  build/common/_headers
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

find_package(raspicam REQUIRED)

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
  ${raspicam_LIBS}
)
