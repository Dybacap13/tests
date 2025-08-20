cmake_minimum_required(VERSION 3.8)
project(aubo_sdk_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions( -DROS_BUILD )

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(ENV{CPM_SOURCE_CACHE} $ENV{HOME}/.cache/CPM)

# 下载 aubo_sdk 发布包(二进制方式引入)
    if(NOT aubo_sdk_POPULATED)
        include(FetchContent)
		if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
			# 使用 ARM64 版本 SDK 的 URL
			set(SDK_URL "https://download.aubo-robotics.cn/sdk/aubo_sdk-0.24.1-rc.3-Linux_aarch64+318754d.zip")
		else()
			# 使用 x86_64 版本 SDK 的 URL
			# set(SDK_URL "https://download.aubo-robotics.cn/sdk/aubo_sdk-0.22.1-Linux_x86_64+2ed0c14.zip")
			set(SDK_URL "https://download.aubo-robotics.cn/sdk/aubo_sdk-0.24.1-rc.3-Linux_x86_64+318754d.zip")
		endif()
		
        FetchContent_Declare(
            aubo_sdk
			URL ${SDK_URL}
        )
        FetchContent_Populate(aubo_sdk)
    endif()
set(aubo_sdk_DIR ${aubo_sdk_SOURCE_DIR}/lib/cmake/aubo_sdk)
message("aubo_sdk_DIR" ${aubo_sdk_SOURCE_DIR}/lib/cmake/aubo_sdk)

message("aubo_sdk_SOURCE_DIR ${aubo_sdk_SOURCE_DIR}")




set(THIS_PACKAGE_INCLUDE_DEPENDS
    pluginlib
    rclcpp
    rclcpp_lifecycle
    rclcpp_components
    realtime_tools
    aubo_msgs
    aubo_sdk_client
    aubo_sdk
    aubo_dashboard_msgs
    moveit_msgs
    )
find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
  endforeach ()    

include_directories(include)  



add_library(${PROJECT_NAME} SHARED src/aubo_sdk_control.cpp)
target_link_libraries(${PROJECT_NAME}   aubo_sdk::aubo_sdk)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION
                                                 "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})


# rclcpp_components_register_node(${PROJECT_NAME}
#                                 PLUGIN "aubo_sdk_control::AuboSdkControl"
#                                 EXECUTABLE aubo_sdk_control_
#                               )


# install
install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
install(EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# export
ament_export_include_directories(include)
ament_export_libraries( ${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
