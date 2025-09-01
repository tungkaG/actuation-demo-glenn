cmake_minimum_required(VERSION 3.10)
project(dds_board_input_subscriber)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)

find_package(CycloneDDS REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(subscriber
  subscriber.cpp
  msg/board_input_data.c
  msg/cartesian_sample.c
  msg/curvilinear_sample.c
  msg/board_output_data.c
)

target_include_directories(subscriber PRIVATE
  ${CycloneDDS_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/msg
)

target_link_libraries(subscriber
  CycloneDDS::ddsc
  Eigen3::Eigen
)
