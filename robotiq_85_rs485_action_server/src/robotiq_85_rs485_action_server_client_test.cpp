#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_robotiq_client");

    ros::NodeHandle pnh("~");

    std::string gripper_name;
    pnh.param<std::string>("gripper_name", gripper_name, "gripper_action");

    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac(gripper_name, true);

    ROS_INFO("Waiting for the action server start ...");
    ac.waitForServer(); // wait for infinite time

    ROS_INFO("Action server got ..., sending goal ");
    // prepare the goal msgs

    control_msgs::GripperCommandGoal goal;
    goal.command.position = 0.0;
    goal.command.max_effort = 100;
    ac.sendGoal(goal);

//    bool finish_before_timeout = ac.waitForResult(ros::Duration(30.0));
//
//    sleep(10);
//    goal.command.position = 0.8;
//    goal.command.max_effort = 100;
//    ac.sendGoal(goal);

//    if(finish_before_timeout)
//    {
//        actionlib::SimpleClientGoalState state = ac.getState();
//        ROS_INFO("Action finished: %s", state.toString().c_str());
//    } else
//    {
//        ROS_INFO("fucking we failed ....");
//
//    }
    while (!ros::isShuttingDown())
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action state: %s", state.toString().c_str());
        sleep(1);
    }
    return 0;

}
