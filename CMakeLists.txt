cmake_minimum_required(VERSION 2.8.3)
project(ros_parameters)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    image_transport
    cv_bridge
    dynamic_reconfigure
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/CameraNode.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    image_transport
    cv_bridge
    dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(camera_node_0 src/camera_node_0.cpp)
target_link_libraries(camera_node_0 ${catkin_LIBRARIES})

add_executable(camera_node_1 src/camera_node_1.cpp)
target_link_libraries(camera_node_1 ${catkin_LIBRARIES})

add_executable(camera_node_2 src/camera_node_2.cpp)
add_dependencies(camera_node_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_node_2 ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS camera_node_0 camera_node_1 camera_node_2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config data
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
