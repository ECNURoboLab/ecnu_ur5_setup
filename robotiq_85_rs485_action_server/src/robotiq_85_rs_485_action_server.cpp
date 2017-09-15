/**
 * ActionServer interface to the control/msgs/GripperCommand action
 * for a Robotiq 85 rs485 gripper
 */

#include <robotiq_85_rs485_action_server/robotiq_85_rs485_action_server.h>

// to keep the fully qualified names manageable
// Anonymous namespaces are file local -> sort of like global static objects
namespace
{
    using namespace robotiq_action_server;

    /*  This struct is declared for the sole purpose of being used as an exception internally
        to keep the code clean (i.e. no output params). It is caught by the action_server and
        should not propogate outwards. If you use these functions yourself, beware.
    */
    struct BadArgumentsError {};


    GripperOutput goalToRegisterState(const GripperCommandGoal& goal)
    {
        GripperOutput result;

        // the position will reflect to the robotiq_85_left_knuckle_joint so
        // the value will change from 0 to 0.8.
        // 0 means the gripper will fully open and 0.8 means the fully close
        if (goal.command.position > 0.8 || goal.command.position < 0.0)
        {
            ROS_WARN("Goal gripper rad size is out of range(%f to %f): %f m",
                     0.0, 0.8, goal.command.position);
            throw BadArgumentsError();
        }

        if (goal.command.max_effort < 5.0 || goal.command.max_effort > 220.0)
        {
            ROS_WARN("Goal gripper effort out of range (%f to %f N): %f N",
                     5.0, 220.0, goal.command.max_effort);
            throw BadArgumentsError();
        }


        result.position = static_cast<float_t >(0.085*(1 - goal.command.position/0.8));
        result.force    = static_cast<float_t >(goal.command.max_effort);
        result.speed    = 0.015;



        ROS_INFO("Setting goal position register to %f", result.position);

        return result;
    }

    /*  This function is templatized because both GripperCommandResult and GripperCommandFeedback consist
        of the same fields yet have different types. Templates here act as a "duck typing" mechanism to avoid
        code duplication.
    */
    template<typename T>
    T registerStateToResultT(const GripperInput& input)
    {
        T result;

        result.position = input.position;
        result.effort = input.current*10;
        result.stalled = !input.is_ready;
        result.reached_goal = fabs(input.requested_position - input.position) < 0.002 ;
        return result;
    }

    // Inline api-transformers to avoid confusion when reading the action_server source
    inline
    GripperCommandResult registerStateToResult(const GripperInput& input)
    {
        return registerStateToResultT<GripperCommandResult>(input);
    }

    inline
    GripperCommandFeedback registerStateToFeedback(const GripperInput& input)
    {
        return registerStateToResultT<GripperCommandFeedback>(input);
    }

} // end of anon namespace


namespace robotiq_action_server
{



    RS485GripperActionServer::RS485GripperActionServer(const std::string &name)
            : nh_()
            , as_(nh_, name, false)
            , action_name_(name)
    {
        as_.registerGoalCallback(boost::bind(&RS485GripperActionServer::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&RS485GripperActionServer::preemptCB, this));

        state_sub_ = nh_.subscribe("input", 1, &RS485GripperActionServer::analysisCB, this);
        goal_pub_ = nh_.advertise<GripperOutput>("output", 1);
        as_.start();
    }

    void RS485GripperActionServer::goalCB()
    {
        // Check to see if the gripper is in an active state where it can take goals
        if (!current_reg_state_.is_ready)
        {
            ROS_WARN("%s could not accept goal because the gripper is not yet active", action_name_.c_str());
            return;
        }

        GripperCommandGoal current_goal (*(as_.acceptNewGoal()));

        if (as_.isPreemptRequested())
        {
            as_.setPreempted();
        }

        try
        {
            if (current_goal.command.max_effort == 0.0){
                ROS_INFO("requested to move gripper with max_effort of 0.0. Defaulting to 70.0");
                current_goal.command.max_effort = 70.0;
            }
            goal_reg_state_ = goalToRegisterState(current_goal);
            goal_pub_.publish(goal_reg_state_);
        }
        catch (BadArgumentsError& e)
        {
            ROS_INFO("%s No goal issued to gripper", action_name_.c_str());
        }
    }

    void RS485GripperActionServer::preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
    }

    void RS485GripperActionServer::analysisCB(const GripperInput::ConstPtr& msg)
    {
        current_reg_state_ = *msg;
        // Check to see if the gripper is in its activated state
        if (!current_reg_state_.is_ready)
        {
            return;
        }
        if (!as_.isActive()) return;


        // Check for errors
        if (current_reg_state_.fault_status)
        {
            ROS_WARN("%s faulted with code: %x", action_name_.c_str(), current_reg_state_.fault_status);
            as_.setAborted(registerStateToResult(current_reg_state_));
        }
        else if ((current_reg_state_.obj_detected || fabs(goal_reg_state_.position - current_reg_state_.position) < 0.002) && !current_reg_state_.is_moving)
        {
            // when the gripper detected a object or get the goal position the action success
            // we should notice no using the feedback goal position, because it did't update at the first callback
            ROS_INFO("%s succeeded", action_name_.c_str());
            as_.setSucceeded(registerStateToResult(current_reg_state_));
        }
        else
        {
            // Publish feedback
            as_.publishFeedback(registerStateToFeedback(current_reg_state_));
        }
    }


} // end robotiq_action_server namespace