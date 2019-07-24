cmake_minimum_required(VERSION 2.8.3)
project(turtle_controller)

add_definitions(--std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  message_generation
)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(DIRECTORY action FILES goToGoal.action)
add_service_files(DIRECTORY srv FILES goal.srv)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS 
    roscpp
    std_msgs
    geometry_msgs
    actionlib_msgs
    message_runtime
#  DEPENDS 
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
   src/Algorithm.cpp
)


## Declare C++ executable
add_executable(${PROJECT_NAME}
  src/turtle_controller_node.cpp
  src/TurtleController.cpp
)

add_executable(${PROJECT_NAME}_interface
  src/interface_node.cpp
  src/Interface.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_interface
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_interface
  ${PROJECT_NAME}_core
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)
