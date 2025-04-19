cmake_minimum_required(VERSION 3.8)
project(hd_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

link_directories(
  /usr/local/lib
)


add_executable(inverse3_cpp_node src/inverse3_cpp.cpp)
target_link_libraries(inverse3_cpp_node
  Haply.HardwareAPI
)
target_include_directories(inverse3_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(inverse3_cpp_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(inverse3_cpp_node rclcpp std_msgs sensor_msgs geometry_msgs)

install(TARGETS 
  inverse3_cpp_node
  DESTINATION lib/${PROJECT_NAME})



ament_package()
