cmake_minimum_required(VERSION 2.8.3)
project(action_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS c++ dependencies
  roscpp
  rospy

  # Messages packages
  ai_msgs
  interface_msgs

  # Other packages
  args_lib
  actionlib
  node_watcher
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy ai_msgs interface_msgs actionlib node_watcher args_lib
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/ActionPerformer.cpp
)

# Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # C++ test performers nodes
  foreach(T
    test_fail_performer
    test_move_performer
    test_sleep_performer
  )
    add_executable(${T} test/actions_performers/${T}.cpp src/ActionPerformer.cpp)
    add_dependencies(${T} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${T}
      ${catkin_LIBRARIES}
      ${PROJECT_NAME}
    )
  endforeach()

  # Tests to perform
  foreach(T
      test/test-actions-cpp.launch
      test/test-actions-py.launch
      test/test-parsing.launch)
     add_rostest(${T})
  endforeach()
endif()