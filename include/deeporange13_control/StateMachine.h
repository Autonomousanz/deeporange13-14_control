#include <string.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <deeporange13_control/deeporange14_state_enums.h>

// Custom Neeed to be determined

#include <deeporange13_msgs/RaptorState.h>
#include <deeporange13_msgs/RosState.h>
#include <deeporange13_msgs/TrackVelocity.h>
#include <deeporange13_msgs/RosHealth.h>

// 

namespace deeporange14{
    class StateMachine{
        public:
        StateMachine(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
        ~StateMachine();

        private:

        void UpdateROSState(const ros::TimerEvent& event);
        void getState();
        void getRaptorMsg(const deeporange13_msgs::RaptorState::ConstPtr& msg);
        void ROSHealthCallback(const deeporange13_msgs::RosHealth::ConstPtr& msg);
        void getCommandedTwist(const geometry_msgs::Twist::ConstPtr& msg);
        ros::Timer timer;

        // Publishers
        ros::Publisher pub_trackVel;
        ros::Publisher pub_rosState;
        ros::Publisher pub_estop;
        // Subscribers
        ros::Subscriber sub_raptor;
        ros::Subscriber sub_cmdVel;
        ros::Subscriber sub_rosHealth;
        // Init the msg variables
        nav_msgs::Odometry odometryMsg;
        deeporange13_msgs::RaptorState raptorMsg;
        deeporange13_msgs::RosState rosSupMsg;
        deeporange13_msgs::TrackVelocity trackVelMsg;
        geometry_msgs::Twist commandedTwist;

        // Setting up internal semaphores:
        bool is_raptorMsg_old;
        int raptorMsgCounter;
        bool is_Phx_NavigationActive;
        std::string topicns = '/deeporange14'

    };

}