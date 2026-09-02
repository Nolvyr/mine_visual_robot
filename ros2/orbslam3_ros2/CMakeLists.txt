cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
# set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")

set(
  CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules
)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR
   CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MapOperation.msg" "msg/MapBatch.msg" "msg/TxCompletion.msg")
rosidl_get_typesupport_target(comm_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_export_dependencies(rosidl_default_runtime)
find_package(sensor_msgs REQUIRED)

# SLAM输出消息
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(OpenCV 4.0 QUIET)
find_package(Threads REQUIRED)

include(CTest)

# Transport-independent V3 encoders for the future serial gateway.
add_library(comm_packet_encoder STATIC
  src/comm/packet_encoder.cpp
  src/comm/map_packet_encoder.cpp
)
target_include_directories(comm_packet_encoder PUBLIC include)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)

link_directories(
  include
)

add_executable(
  mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)

ament_target_dependencies(
  mono
  rclcpp
  sensor_msgs
  cv_bridge
  ORB_SLAM3
  Pangolin
)

add_executable(
  rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)

ament_target_dependencies(
  rgbd
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  ORB_SLAM3
  Pangolin
)

add_executable(
  stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
  src/comm/binary_protocol.cpp
  src/comm/comm_map.cpp
  src/comm/comm_map_commit.cpp
  src/comm/comm_worker.cpp
)

ament_target_dependencies(
  stereo
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  cv_bridge
  message_filters
  ORB_SLAM3
  Pangolin
  OpenCV
)
target_link_libraries(stereo Threads::Threads)
target_sources(stereo PRIVATE src/comm/map_producer.cpp)
target_link_libraries(stereo "${comm_typesupport}")

add_executable(serial_gateway_node src/comm/serial_gateway_node.cpp src/comm/uart_transport.cpp)
ament_target_dependencies(serial_gateway_node rclcpp)
target_link_libraries(serial_gateway_node comm_packet_encoder "${comm_typesupport}")

if(BUILD_TESTING)
  add_executable(uart_transport_test test/uart_transport_test.cpp src/comm/uart_transport.cpp)
  target_link_libraries(uart_transport_test comm_packet_encoder util)
  add_test(NAME uart_transport_test COMMAND uart_transport_test)
  set_tests_properties(uart_transport_test PROPERTIES TIMEOUT 10)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(gateway_producer_test test/gateway_producer_test.cpp
    src/comm/map_producer.cpp src/comm/comm_map_commit.cpp)
  ament_target_dependencies(gateway_producer_test rclcpp)
  target_link_libraries(gateway_producer_test "${comm_typesupport}" Threads::Threads)
  add_test(NAME gateway_ros_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/run_gateway_test.py
      $<TARGET_FILE:serial_gateway_node> $<TARGET_FILE:gateway_producer_test>)
  set_tests_properties(gateway_ros_integration PROPERTIES TIMEOUT 50 RUN_SERIAL TRUE)
  foreach(mode uart unavailable)
    add_test(NAME gateway_${mode}_integration
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/run_gateway_test.py
        $<TARGET_FILE:serial_gateway_node> $<TARGET_FILE:gateway_producer_test> ${mode})
    set_tests_properties(gateway_${mode}_integration PROPERTIES TIMEOUT 50 RUN_SERIAL TRUE)
  endforeach()

  add_executable(packet_encoder_test test/packet_encoder_test.cpp)
  target_link_libraries(packet_encoder_test comm_packet_encoder)
  add_test(NAME packet_encoder_test COMMAND packet_encoder_test)

  add_executable(
    binary_protocol_golden_test
    test/binary_protocol_golden_test.cpp
    src/comm/binary_protocol.cpp
  )
  target_include_directories(
    binary_protocol_golden_test
    PRIVATE include
  )
  add_test(
    NAME binary_protocol_golden_test
    COMMAND binary_protocol_golden_test
  )

  add_executable(
    comm_worker_test
    test/comm_worker_test.cpp
    src/comm/comm_worker.cpp
  )
  target_include_directories(
    comm_worker_test
    PRIVATE include
  )
  target_link_libraries(comm_worker_test Threads::Threads)
  add_test(
    NAME comm_worker_test
    COMMAND comm_worker_test
  )

  add_executable(
    comm_map_commit_test
    test/comm_map_commit_test.cpp
    src/comm/comm_map_commit.cpp
    src/comm/comm_worker.cpp
  )
  target_include_directories(
    comm_map_commit_test
    PRIVATE include
  )
  target_link_libraries(comm_map_commit_test Threads::Threads)
  add_test(
    NAME comm_map_commit_test
    COMMAND comm_map_commit_test
  )
endif()

add_executable(
  stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)

ament_target_dependencies(
  stereo-inertial
  rclcpp
  sensor_msgs
  cv_bridge
  ORB_SLAM3
  Pangolin
  OpenCV
)

install(
  TARGETS
  serial_gateway_node
  mono
  rgbd
  stereo
  stereo-inertial
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
# install(
#   DIRECTORY launch config vocabulary
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
