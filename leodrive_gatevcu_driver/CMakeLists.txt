cmake_minimum_required(VERSION 3.8)
project(leodrive_gatevcu_driver)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        config
)
