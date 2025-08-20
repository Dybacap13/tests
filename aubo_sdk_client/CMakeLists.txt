cmake_minimum_required(VERSION 3.8)
project(aubo_sdk_client)


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

find_package(aubo_sdk REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aubo_dashboard_msgs REQUIRED)
find_package(aubo_msgs REQUIRED)

find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  controller_manager_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  tf2_geometry_msgs
  aubo_msgs
  aubo_dashboard_msgs
)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)


include_directories(
  include
  
  ${Boost_INCLUDE_DIRS}
  ${aubo_sdk_SOURCE_DIR}/include
  
)


add_library(${PROJECT_NAME} 
  SHARED
  src/aubo_sdk_client.cpp
   src/aubo_imitator.cpp
)
target_link_libraries(${PROJECT_NAME}   aubo_sdk::aubo_sdk)

ament_target_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(
  ${PROJECT_NAME} 
  PRIVATE
  include
)
install(
  TARGETS ${PROJECT_NAME} 
  DESTINATION lib
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME} 
)
install(DIRECTORY include/
  DESTINATION include
)
install(PROGRAMS ${aubo_sdk_SOURCE_DIR}/lib/libaubo_sdkd.so
  DESTINATION lib
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_package()



