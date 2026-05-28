cmake_minimum_required(VERSION 3.12)
project(h1_2_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0")
endif()

# unitree_sdk2 is installed at /opt/unitree_robotics on this machine
# (upstream CMakeLists assumes /usr/local — override here)
set(UNITREE_SDK2_DIR "/opt/unitree_robotics")

include_directories(
  /usr/include/eigen3
  ${UNITREE_SDK2_DIR}/include
  ${UNITREE_SDK2_DIR}/include/ddscxx
  /opt/ros/humble/include/iceoryx/v2.0.5
  include
  ${ONNXRUNTIME_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_directories(
  ${UNITREE_SDK2_DIR}/lib
)


link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread
  libboost_program_options.a libyaml-cpp.a fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(h1_2_ctrl main.cpp)