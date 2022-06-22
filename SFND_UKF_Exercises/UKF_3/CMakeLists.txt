cmake_minimum_required(VERSION 3.16)
project(UKF_3)

set(CMAKE_CXX_STANDARD 14)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(UKF_3 ukf.cpp main.cpp)

target_link_libraries (UKF_3 Eigen3::Eigen)