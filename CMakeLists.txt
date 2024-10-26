cmake_minimum_required(VERSION 3.5.0)
project(liaun VERSION 0.1.0 LANGUAGES C CXX)
include_directories(sophus)
include_directories("/usr/include/eigen3")
add_executable(liaun liqun.cpp sophus/so3.cpp sophus/se3.cpp)


