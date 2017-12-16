cmake_minimum_required(VERSION 2.8.3)
project(robotiq_85_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs std_srvs)

add_message_files(DIRECTORY msg FILES 
                                GripperCmd.msg 
                                GripperStat.msg)

add_service_files(
    FILES
    Gripper.srv
)


generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

                      
