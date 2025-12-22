cmake_minimum_required(VERSION 3.16)

project(s3_lib_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(example src/example.cpp)

target_include_directories(example
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
