cmake_minimum_required(VERSION 3.8)
project(rm_ros2_common)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CONTROLLER_INCLUDE_DEPENDS
        rclcpp
        rclcpp_lifecycle
        tf2_ros
        tf2_geometry_msgs
        realtime_tools
        rm_ros2_msgs
        controller_manager
)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
foreach (Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()
file(GLOB DECISION "include/rm_ros2_common/decision/*")
file(GLOB FILTERS "include/rm_ros2_common/filters/*")
file(GLOB TOOLS "include/rm_ros2_common/tools/*")
list(REMOVE_ITEM DECISION "include/rm_ros2_common/decision/bullet_solver/*")
list(REMOVE_ITEM FILTER "include/rm_ros2_common/filters/complementary_filter.hpp")
add_library(${PROJECT_NAME} SHARED
        ${DECISION}
        ${FILTERS}
        ${TOOLS}
        src/decision/bullet_solver/bullet_solver.cpp
        src/filters/complementary_filter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${CONTROLLER_INCLUDE_DEPENDS})

# INSTALL
install(
        DIRECTORY include/
        DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        ${CONTROLLER_INCLUDE_DEPENDS}
        Eigen3
)
ament_package()
