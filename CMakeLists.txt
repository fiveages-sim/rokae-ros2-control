cmake_minimum_required(VERSION 3.8)
project(rokae_ros2_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
pkg_check_modules(OROCOS_KDL REQUIRED orocos-kdl)

# Create library
add_library(${PROJECT_NAME} SHARED
        src/rokae_hardware.cpp
        src/jodell_gripper.cpp
        src/ag2f90d_gripper.cpp
        src/linkerhand.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE src
        PRIVATE external/librokae/include
        PRIVATE ${EIGEN3_INCLUDE_DIRS}
        PRIVATE ${OROCOS_KDL_INCLUDE_DIRS}
)

# Detect architecture and set library path
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ROKAE_LIB_DIR "aarch64")
else()
    set(ROKAE_LIB_DIR "x86_64")
endif()

# Link librokae libraries (v0.7.1.ar_9)
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${hardware_interface_TARGETS}
        ${rclcpp_TARGETS}
        ${rclcpp_lifecycle_TARGETS}
        ${pluginlib_TARGETS}
        ${CMAKE_CURRENT_SOURCE_DIR}/external/librokae/lib/Linux/${ROKAE_LIB_DIR}/libxCoreSDK.a
        ${CMAKE_CURRENT_SOURCE_DIR}/external/librokae/lib/Linux/${ROKAE_LIB_DIR}/libxMateModel.a
        ${EIGEN3_LIBRARIES}
        ${OROCOS_KDL_LIBRARIES}
        pthread
        rt
)

pluginlib_export_plugin_description_file(hardware_interface rokae_ros2_control.xml)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(linkerhand_protocol_test test/linkerhand_protocol_test.cpp)
    target_include_directories(linkerhand_protocol_test PRIVATE include)
    ament_add_gtest(linkerhand_interfaces_test test/linkerhand_interfaces_test.cpp)
    target_include_directories(linkerhand_interfaces_test PRIVATE external/librokae/include ${EIGEN3_INCLUDE_DIRS})
    target_link_libraries(linkerhand_interfaces_test ${PROJECT_NAME})
    ament_add_gtest(rokae_fault_test test/rokae_fault_test.cpp)
    target_include_directories(rokae_fault_test PRIVATE external/librokae/include ${EIGEN3_INCLUDE_DIRS})
    target_link_libraries(rokae_fault_test ${PROJECT_NAME})
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_dependencies(hardware_interface rclcpp rclcpp_lifecycle pluginlib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
