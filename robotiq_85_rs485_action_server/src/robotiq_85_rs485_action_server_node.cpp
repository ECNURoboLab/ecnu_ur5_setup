#include "robotiq_85_rs485_action_server/robotiq_85_rs485_action_server.h"

int main(int argc, char** argv){
    // this can be renamed with ROS launch interface
    ros::init(argc,argv,"gripper_action_server");

    // private Node Handle for retrieving parameter arguments to  the server
    ros::NodeHandle private_nh("~");

    std::string gripper_name;
    private_nh.param<std::string>("gripper_name",gripper_name,"gripper");

    ROS_INFO("Initializing Robotiq action server for gripper: %s", gripper_name.c_str());

    // The name fo the gripper -> this server communicates over name/inputs and name/outputs
    robotiq_action_server::RS485GripperActionServer gripper(gripper_name);
    ROS_INFO("Robotiq action-server spinning for gripper: %s", gripper_name.c_str());
    ros::spin();
}
