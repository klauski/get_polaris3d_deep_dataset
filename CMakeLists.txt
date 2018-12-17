# Copyright (c) 2018, Polaris3D. All rights reserved.

cmake_minimum_required (VERSION 2.6)

project(io_node)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" "${CMAKE_MODULE_PATH}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_definitions(
    -std=c++14
    -O2
    -DASIO_STANDALONE
# stack backtrace
    -g -rdynamic
)

set(IoNode_SOURCES
  src/ControlEmitter.cpp
  src/ControlUdp.cpp
  src/icm20602.c
  src/ImuGrabber.cpp
  src/IoNode.cpp
  src/messageRobotics.pb.cc
  src/Mocap2Mav/mavlink_interface/serial_port.cpp
  src/Mocap2Mav/mavlink_interface/autopilot_interface.cpp
  src/Mocap2Mav/mocap_interface/mocap_interface.cpp
  src/Mocap2Mav/mocap2mav.cpp
  src/myRoboClaw/software/driver/serial.cpp
  src/myRoboClaw/software/driver/roboclaw.cpp
)

set(KeyboardControlNode_SOURCES
  src/KeyboardControlNode.cpp
  src/messageRobotics.pb.cc
)

set(RpgControlNode_SOURCES
  src/RpgControlNode.cpp
  src/messageRobotics.pb.cc
)

set(OnlineImuCalibNode_SOURCES
  src/icm20602.c
  src/OnlineImuCalibNode.cpp
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/../vsys/inc
# only for imu calibration
  ${CMAKE_CURRENT_SOURCE_DIR}/../vlidaralgo/inc
)

add_executable(${PROJECT_NAME} ${IoNode_SOURCES})
target_link_libraries(${PROJECT_NAME}
	pthread
	glog
	/usr/local/lib/libprotobuf.a
	${CMAKE_CURRENT_SOURCE_DIR}/../vsys/lib/libvsys.a
	/usr/local/lib/libdlib.a
# stack backtrace
  dl
)

add_executable(KeyboardControlNode ${KeyboardControlNode_SOURCES})
target_link_libraries(KeyboardControlNode
  # protobuf require pthread
  pthread
  glog
  /usr/local/lib/libprotobuf.a
  ${CMAKE_CURRENT_SOURCE_DIR}/../vsys/lib/libvsys.a
#  /usr/local/lib/libdlib.a
)

add_executable(RpgControlNode ${RpgControlNode_SOURCES})
target_link_libraries(RpgControlNode
  # protobuf require pthread
  pthread
  glog
  /usr/local/lib/libprotobuf.a
  ${CMAKE_CURRENT_SOURCE_DIR}/../vsys/lib/libvsys.a
#  /usr/local/lib/libdlib.a
)



add_executable(OnlineImuCalibNode ${OnlineImuCalibNode_SOURCES})
target_link_libraries(OnlineImuCalibNode
  ${CMAKE_CURRENT_SOURCE_DIR}/../vlidaralgo/lib/libvlidaralgo.a
  ${CMAKE_CURRENT_SOURCE_DIR}/../vsys/lib/libvsys.a
  ceres
  cxsparse
  cholmod
  blas
  lapack
  opencv_world
  glog
  pthread
  gomp
)

install(TARGETS ${PROJECT_NAME} DESTINATION ../bin)

include(CPack)
