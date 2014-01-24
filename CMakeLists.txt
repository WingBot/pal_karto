cmake_minimum_required(VERSION 2.8.3)
project(pal_karto)

set(ROS_DEPS tf sensor_msgs visualization_msgs roscpp rosconsole nav_msgs message_filters sba open_karto dynamic_reconfigure message_generation pal_map_utils)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPS})
find_package(Eigen REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

add_service_files(
  FILES
  UseScanMatching.srv
)
generate_messages()

generate_dynamic_reconfigure_options(cfg/Karto.cfg)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_executable(slam_karto src/slam_karto.cpp src/spa_solver.cpp)
add_dependencies(slam_karto ${PROJECT_NAME}_gencfg)
target_link_libraries(slam_karto ${catkin_LIBRARIES})

catkin_package()
