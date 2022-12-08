cmake_minimum_required(VERSION 3.10.2)
project(UnitreeMotorSDK_M80106)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

include_directories("include/")
link_directories(
    lib/
)

#example
add_executable(motorctrl example/main.cpp)
add_executable(motor_test src/motor_test.cpp)
add_executable(changeID example/changeID.cpp)
target_link_libraries(motorctrl libunitreeMotorSDK_Linux64.so)
target_link_libraries(motor_test libunitreeMotorSDK_Linux64.so pthread rt)
target_link_libraries(changeID libunitreeMotorSDK_Linux64.so)

set(CMAKE_CXX_FLAGS "-fpermissive -std=c++0x")