cmake_minimum_required(VERSION 3.8)
project(leodrive_gatevcu_adapter)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif ()

include_directories(
        include
)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(can_interface SHARED
        src/can_interface/drivedb.c
        src/can_interface/drivedb-fmon.c
        src/can_interface/drivedb-binutil.c
)

ament_auto_add_library(vcu_sender SHARED
        src/vcu_sender.cpp
)

target_link_libraries(vcu_sender can_interface)

rclcpp_components_register_node(vcu_sender
        PLUGIN "leodrive_gatevcu_adapter::VcuSender"
        EXECUTABLE vcu_sender_node
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package(
        #        INSTALL_TO_SHARE
        #        launch
        #        config
)
