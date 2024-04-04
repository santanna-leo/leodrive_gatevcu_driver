cmake_minimum_required(VERSION 3.8)
project(leodrive_gatevcu_joy)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif ()

include_directories(
        include
)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(vcu_joy_handler SHARED
        src/vcu_joy_handler.cpp
        src/button.cpp
        src/press_detector.cpp
)

rclcpp_components_register_node(vcu_joy_handler
        PLUGIN "leodrive_gatevcu_joy::VcuJoyHandler"
        EXECUTABLE vcu_joy_handler_node
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package()
