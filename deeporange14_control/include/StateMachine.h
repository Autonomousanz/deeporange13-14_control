/* A high level state machine to interact with Raptor to control brakes torque command */

#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

#include <string.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <deeporange13_msgs/RaptorState.h>
#include <deeporange13_msgs/RosState.h>
#include <deeporange13_msgs/TorqueValues.h>
#include <deeporange13_control/DeeporangeStateEnums.h>

namespace deeporange14
{
    class StateMachine{
        public:
        StateMachine(ros::NodeHandle &node, ros::NodeHandle &priv_nh){};
        ~StateMachine(){};

        private:
        void publishROSState(const ros::TimerEvent& event);
        void getRaptorMsg(const deeporange13_msgs::RaptorState::ConstPtr& msg);
        void updateROSStateMsg();

        void getCommandedTwist(const geometry_msgs::Twist::ConstPtr& msg);
        void StackFault(const deeporange13_msgs::StackFault::ConstPtr& msg);
        void RaptorHBFail(const deeporange13_msgs::RaptorHBFail::ConstPtr& msg);

        ros::Timer timer_;
        // Publishers
        ros::Publisher pub_Torq;
        ros::Publisher pub_BrakeEnable;
        ros::Publisher pub_rosState;
        ros::Publisher pub_estop; // to be confirmed

        // Subscribers
        ros::Subscriber sub_raptor;
        ros::Subscriber sub_cmdVel;
        ros::Subscriber sub_stackfault;
        ros::Subscriber raptor_heartbeat;

        // Init the msg variables
        
        deeporange14_msgs::RaptorState raptorMsg;
        deeporange14_msgs::RosState rosSupMsg;
        deeporange14_msgs::TorqueValues torqueMsg;
        deeporange14_msgs::StackFaults faultMsg;
        deeporange14_msgs::RaptorHeartbeat raptorHBMsg;
        geometry_msgs::Twist commandedTwist;
        nav_msgs::Odometry odometryMsg;
        
        // Setting up internal semaphores:
        bool is_raptorMsg_old;
        int raptorMsgCounter;
        bool is_Phx_NavigationActive;        

    };

}

#endif
