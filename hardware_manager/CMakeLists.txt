cmake_minimum_required(VERSION 3.5)
project(reddog_hardware)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/can
    /usr/include/eigen3
)

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/xspublic
    ${CMAKE_SOURCE_DIR}/xspublic/xscontroller/include
    ${CMAKE_SOURCE_DIR}/xspublic/xscommon/include
    ${CMAKE_SOURCE_DIR}/xspublic/xstypes/include
)

add_custom_target(xspublic_build ALL
    COMMAND ${CMAKE_MAKE_PROGRAM} -C ${CMAKE_SOURCE_DIR}/xspublic
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/xspublic
    COMMENT "Building xspublic libraries with Makefile"
)

add_custom_target(clean-all
    COMMAND ${CMAKE_MAKE_PROGRAM} -C ${CMAKE_SOURCE_DIR}/xspublic clean
    COMMENT "Cleaning xspublic..."
)

# 設定 lib 搜尋路徑
link_directories(${PROJECT_SOURCE_DIR}/can)

link_directories(
    ${CMAKE_SOURCE_DIR}/xspublic/xscontroller
    ${CMAKE_SOURCE_DIR}/xspublic/xscommon
    ${CMAKE_SOURCE_DIR}/xspublic/xstypes
)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 主節點編譯
add_executable(can_node_motor_imu
    src/can_node_motor_imu.cpp   
    src/usb2can_motor_imu.cpp
    src/callback_handler.cpp
)

target_include_directories(can_node_motor_imu PRIVATE include)
target_link_libraries(can_node_motor_imu
    pthread
    rt
    usb_can
    unitree_sdk2
    yaml-cpp
    xscontroller 
    xscommon 
    xstypes
    dl
)
add_dependencies(can_node_motor_imu xspublic_build)

add_executable(example_mti_reddog
    src/example_mti_reddog.cpp
    src/callback_handler.cpp
)

add_dependencies(example_mti_reddog xspublic_build)
target_link_libraries(example_mti_reddog
    xscontroller xscommon xstypes pthread rt dl
)



