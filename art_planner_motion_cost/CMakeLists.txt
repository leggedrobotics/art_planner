cmake_minimum_required(VERSION 3.0.2)
project(art_planner_motion_cost)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

# Set up python modules.
catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
   FILES
   costQuery.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy
    std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
                      scripts/cost_query_server.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

