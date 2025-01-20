cmake_minimum_required(VERSION 3.8)
project(dwa_ros)


find_package(OpenCV REQUIRED )
find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(Eigen3 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(EXTERNAL_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}  ${Boost_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(dwa_ros src/dwa.cpp src/utils.cpp)
target_include_directories(dwa_ros PUBLIC include/dwa_ros ${EXTERNAL_INCLUDE_DIRS})
target_link_libraries(dwa_ros ${EXTERNAL_LIBS})
ament_target_dependencies(dwa_ros rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs)
install(TARGETS
  dwa_ros
  DESTINATION lib/${PROJECT_NAME})

ament_package()
