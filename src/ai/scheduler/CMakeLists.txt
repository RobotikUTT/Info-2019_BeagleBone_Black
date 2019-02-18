cmake_minimum_required(VERSION 2.8.3)
project(scheduler)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS c++ dependencies
  roscpp

  # Required messages
  ai_msgs
  can_msgs

  # Other packages
  action_manager
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp can_msgs ai_msgs action_manager
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


# Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/scheduler_node.cpp src/ActionsParser.cpp src/ActionFilePath.cpp src/ActionTools.cpp)

# Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link the executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)


# Unit tests

## == NOTICE ==
## The following make scheduler broken when a dependency is built
## but not this package

## How to reproduce problem :
## - build all packages
## - change 1 line in action_manager package
## - rebuild action_manager
## - run scheduler node
## -> blank output and no behavior

## Possibly broken since last tests addition (tests_scheduler)

# # ==       ==

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED COMPONENTS
#     std_msgs
#   )

#   # Tests for action parsing
#   add_rostest_gtest(tests_action_parsing
#     launch/parsing.test
#     src/test/parsing_test.cpp src/ActionsParser.cpp src/ActionFilePath.cpp
#   )
#   target_link_libraries(tests_action_parsing ${catkin_LIBRARIES})

#   # Tests for action performing
# #  add_rostest_gtest(tests_performing
# #    launch/performing.test
# #    src/test/performing_test.cpp
# #  )
# #  target_link_libraries(tests_performing ${catkin_LIBRARIES})

#   # Executable for scheduler tests
#   add_executable(${PROJECT_NAME}_test_node src/test/test_node.cpp)
#   set_target_properties(${PROJECT_NAME}_test_node PROPERTIES OUTPUT_NAME node PREFIX "")
#   target_link_libraries(${PROJECT_NAME}_test_node ${catkin_LIBRARIES})

#   # Tests for scheduler
#   add_rostest_gtest(tests_scheduler
#     launch/scheduler.test
#     src/test/scheduler_test.cpp
#   )
#   target_link_libraries(tests_scheduler ${catkin_LIBRARIES})
# endif()