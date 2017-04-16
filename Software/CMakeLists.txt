project(robot)

# source code
set(SRCS
  common/main.cpp
  common/SerialPort.cpp
  common/Thread.cpp
  common/UartRx.cpp
  common/UartTx.cpp
  common/IMUData.cpp
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

# executable
add_executable(${PROJECT_NAME} ${SRCS})

# link pthread library
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} )
