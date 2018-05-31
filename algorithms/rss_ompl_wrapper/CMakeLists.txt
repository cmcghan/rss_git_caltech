cmake_minimum_required(VERSION 2.8.3)
project(rss_ompl_wrapper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
  actionlib_msgs
  actionlib
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  int32list.msg
  float64list.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_action_files(
  DIRECTORY action
  FILES rssomplaction.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p3dx_gazebo
  CATKIN_DEPENDS rospy actionlib
  DEPENDS Boost #system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(p3dx_gazebo
#   src/${PROJECT_NAME}/p3dx_gazebo.cpp
# )

## Declare a cpp executable
# add_executable(p3dx_gazebo_node src/p3dx_gazebo_node.cpp)

# for .cpp files, not .py?
#add_executable(rss_ompl_server scripts/rss_ompl.py)
#target_link_libraries(rss_ompl_server ${catkin_LIBRARIES})
#add_dependencies(rss_ompl_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(p3dx_gazebo_node p3dx_gazebo_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(p3dx_gazebo_node
#   ${catkin_LIBRARIES}
# )

## Install scripts and executables
install(PROGRAMS
  scripts/rss_ompl.py
  scripts/rss_ompl_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# for .cpp files? example: https://github.com/ros/common_tutorials/blob/hydro-devel/actionlib_tutorials/CMakeLists.txt
#install(TARGETS averaging_server averaging_client fibonacci_server fibonacci_client
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
