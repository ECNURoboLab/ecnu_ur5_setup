//
// Created by sun on 17-9-7.
//

#ifndef ROBOTIQ_85_RS485_ACTION_SERVER_ROBOTIQ_85_RS485_ACTION_SERVER_H
#define ROBOTIQ_85_RS485_ACTION_SERVER_ROBOTIQ_85_RS485_ACTION_SERVER_H

//STL
#include <string>
//ROS standard
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
// Repo specific includes
#include <robotiq_85_msgs/GripperCmd.h>
#include <robotiq_85_msgs/GripperStat.h>


namespace robotiq_action_server
{
    typedef robotiq_85_msgs::GripperStat GripperInput;
    typedef robotiq_85_msgs::GripperCmd GripperOutput;

    typedef control_msgs::GripperCommandGoal GripperCommandGoal;
    typedef control_msgs::GripperCommandFeedback GripperCommandFeedback;
    typedef control_msgs::GripperCommandResult GripperCommandResult;

    class RS485GripperActionServer
    {
    public:
        RS485GripperActionServer(const std::string& name);

        // These functions are meant to be called by simple action server
        void goalCB();
        void preemptCB();
        void analysisCB(const GripperInput::ConstPtr& msg);

    private:
//        void issueActivation();

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_;

        ros::Subscriber state_sub_; // subscribe the gripper input msg
        ros::Publisher goal_pub_; // publish to grippers output topic

        GripperOutput goal_reg_state_;
        GripperInput current_reg_state_;

        std::string action_name_;
    };

}



#endif //ROBOTIQ_85_RS485_ACTION_SERVER_ROBOTIQ_85_RS485_ACTION_SERVER_H
