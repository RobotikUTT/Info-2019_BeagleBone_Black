cmake_minimum_required(VERSION 2.8.3)
project(pathfinder)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    node_watcher
    genmsg
    geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
    FILES
    FindPath.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS message_runtime roslib
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


file(
    GLOB_RECURSE
    source_files_pathfinder
    src/pathfinder/*
)
add_executable(node ${source_files_pathfinder} src/pathfinder_node.cpp)

target_link_libraries(node
    ${catkin_LIBRARIES}
    ${SFML_LIBRARIES} ${SFML_DEPENDENCIES}
)

add_dependencies(node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${catkin_EXPORTED_TARGETS}
)

