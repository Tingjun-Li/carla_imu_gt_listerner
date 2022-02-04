cmake_minimum_required(VERSION 2.8.3)
project(carla_path)

SET(CMAKE_MODULE_PATH ${${PROJECT_NAME}_SOURCE_DIR}/cmake)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  tf
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_filters
  carla_msgs
)


## Declare a catkin package
catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}
)

## Build talker and listener
include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR})

find_package(Boost 1.58 REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
message("Boost_INCLUDE_DIR: " ${Boost_INCLUDE_DIRS})

add_executable(carla_listenser_node "${PROJECT_SOURCE_DIR}/imu_gt_listener.cpp")
target_link_libraries(carla_listenser_node ${catkin_LIBRARIES} 
                                          ${Boost_INCLUDE_DIRS}
                                          ${PROJECT_SOURCE_DIR})

# %EndTag(FULLTEXT)%