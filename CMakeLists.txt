cmake_minimum_required(VERSION 3.28)
project(voltage_control)

set(CMAKE_CXX_STANDARD 17)

add_executable(voltage_control
        voltage_control.cpp
        cJSON.c)
# 添加这一行：将配置文件复制到输出目录
#configure_file(${CMAKE_SOURCE_DIR}/config.json ${CMAKE_CURRENT_BINARY_DIR}/config.json COPYONLY)