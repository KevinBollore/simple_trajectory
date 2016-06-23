cmake_minimum_required(VERSION 2.8.3)
project(simple_trajectory)

find_package(catkin REQUIRED COMPONENTS
			 roscpp
			 std_msgs genmsg
			 tf
             tf_conversions
             genmsg
             moveit_ros_planning_interface
             visualization_msgs
             geometry_msgs
             fanuc_grinding_post_processor)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/simple_trajectory.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
