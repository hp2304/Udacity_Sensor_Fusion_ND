cmake_minimum_required(VERSION 3.16)
project(UKF_2)

set(CMAKE_CXX_STANDARD 14)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(UKF_2 ukf.cpp main.cpp)

target_link_libraries (UKF_2 Eigen3::Eigen)