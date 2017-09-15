cmake_minimum_required(VERSION 2.8.3)
project(robotiq_85_rs485_action_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        message_generation
        robotiq_85_msgs
        control_msgs
        roscpp
        rospy
)


catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS actionlib actionlib_msgs  roscpp robotiq_85_msgs
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${robotiq_85_rs485_action_server_INCLUDE_DIRS}
)
# server node
add_executable(robotiq_85_rs485_action_server_node
        src/robotiq_85_rs485_action_server_node.cpp
        src/robotiq_85_rs_485_action_server.cpp
        include/robotiq_85_rs485_action_server/robotiq_85_rs485_action_server.h
        )

add_dependencies(robotiq_85_rs485_action_server_node
        robotiq_85_msgs_generate_messages_cpp
)

target_link_libraries(robotiq_85_rs485_action_server_node
        ${catkin_LIBRARIES}
        )

# test node
add_executable(robotiq_85_rs485_action_server_client_test
        src/robotiq_85_rs485_action_server_client_test.cpp
        )
add_dependencies(robotiq_85_rs485_action_server_client_test
        robotiq_85_msgs_generate_messages_cpp
        )
target_link_libraries(robotiq_85_rs485_action_server_client_test
        ${catkin_LIBRARIES}
        )