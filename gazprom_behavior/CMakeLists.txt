cmake_minimum_required(VERSION 3.8)
project(gazprom_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(THIS_PACKAGE_INCLUDE_DEPENDS
    behaviortree_cpp
    rclcpp
    std_srvs
    )

find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
  endforeach ()    

include_directories(include)  


set(behavior_tree_src
    src/behavior_tree_factory.cpp
    src/behavior_tree.cpp)

set(device_src 
    src/device/robot.cpp 
    src/device/dashboard.cpp)

set(lists_src 
    src/behavior_tree_node/action/robot_action/get_io.cpp 
    src/behavior_tree_node/action/robot_action/servo_move.cpp 
    src/behavior_tree_node/action/robot_action/set_io.cpp
    src/behavior_tree_node/action/robot_action/stop_robot.cpp

    src/behavior_tree_node/action/error_handler.cpp
    src/behavior_tree_node/action/get_number_cycle_paint.cpp
    src/behavior_tree_node/action/paint_counter_increment.cpp
    src/behavior_tree_node/action/set_mode.cpp

    src/behavior_tree_node/is/is_mark_selected.cpp
    src/behavior_tree_node/is/is_mode.cpp

    src/behavior_tree_node/check/check_connection_robot.cpp


     )


add_library(behavior_tree ${behavior_tree_src})
target_include_directories(behavior_tree PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(behavior_tree ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(device ${device_src})
target_include_directories(device PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(device ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(lists ${lists_src})
target_include_directories(lists PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(lists ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
    TARGETS device behavior_tree lists
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION include)

add_executable(main src/main.cpp)
target_include_directories(main PUBLIC PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(main ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(main  lists device behavior_tree)
install(TARGETS main DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY xml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
  PATTERN "_*" EXCLUDE)

ament_package()
