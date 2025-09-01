cmake_minimum_required(VERSION 3.8)
project(ixcom_driver_lc)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ixcom_interfaces REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "../interfaces/srv/ExtAidXcomMsg.srv"
# )

set(src
        src/modules/utility.cpp
        src/modules/transform.cpp
        src/modules/imu.cpp
        src/modules/navsatstatus.cpp
        src/modules/nav_gnss.cpp
        src/modules/nav_ins.cpp
        src/modules/time_reference.cpp
        src/modules/magnetic_field.cpp
        src/modules/odometry.cpp
        src/modules/pose_ecef.cpp
        src/modules/twist.cpp
        src/modules/srv_extaid.cpp
)

set(ixcom_src
        include/libxcom/src/ixcom.cpp
        include/libxcom/src/crc16.cpp
        include/libxcom/src/xcom_parser.cpp
        include/libxcom/src/tcp_client.cpp
        include/libxcom/app/xcomhandler.cpp
)

include_directories(
        include/libxcom/include/
)

add_executable(ixcom_driver_lc
        ${ixcom_src}
        ${src}
        src/ixcom_driver_lifecycle_node.cpp
)
# target_link_libraries(ixcom_driver_lc
#         ${CMAKE_CURRENT_LIST_DIR}/include/libxcom/bin/libxcom/libxcom.a
# )

# target_include_directories(ixcom_driver_lc PUBLIC
#     ${PROJECT_SOURCE_DIR}/include/libxcom/include
# )

set_property(TARGET ixcom_driver_lc PROPERTY CXX_STANDARD 11)

target_include_directories(ixcom_driver_lc PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ixcom_driver_lc PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ixcom_driver_lc
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  ixcom_interfaces
)

install(TARGETS ixcom_driver_lc
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  params launch
  DESTINATION share/${PROJECT_NAME}/
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}_common_test
#                     test/common_test.cpp
#                     ${ixcom_src}
#                     ${src}
#   )
#   target_include_directories(${PROJECT_NAME}_common_test PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
#   )

#   target_link_libraries(${PROJECT_NAME}_common_test)

#   ament_target_dependencies(
#     ${PROJECT_NAME}_common_test
#     rclcpp
#     rclcpp_lifecycle
#     lifecycle_msgs
#     sensor_msgs
#     geometry_msgs
#     nav_msgs
#     tf2
#     tf2_geometry_msgs
#     ixcom_interfaces
#   )
# endif()



# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}_common_test test/common_test.cpp)
#   target_include_directories(${PROJECT_NAME}_common_test PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#   )
#   # target_link_libraries(${PROJECT_NAME}_common_test ament_target_dependencies)

# endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
