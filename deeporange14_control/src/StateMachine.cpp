#include <deeporange13_control/StateMachine.h>

namespace deeporange14 {

    StateMachine::StateMachine(ros::NodeHandle &node, ros::NodeHandle &priv_nh){
        // Instantiate sub/pubs
        topic_ns = "/deeporange14"
        sub_raptor = node.subscribe(std::string(topic_ns + "/raptor_state"), 10, &StateMachine::getRaptorMsg, this, ros::TransportHints().tcpNoDelay(true));
        sub_cmdVel = node.subscribe(std::string(topic_ns + "/cmd_vel"), 10, &StateMachine::getCommandedTwist, this, ros::TransportHints().tcpNoDelay(true));
        sub_stackStatus = node.subscribe(std::string(topic_ns +"/stack_status"), 10, &StateMachine::getStackstatus, this, ros::TransportHints().tcpNoDelay(true));
        sub_cmdTrq = node.subscribe(std::string(topic_ns +"/cmd_torq"), 10, &StateMachine::getCommandedTorques, this, ros::TransportHints().tcpNoDelay(true));


        pub_trackVel = node.advertise<can_msgs::Frame>("can_tx", 10);
        pub_estop    = node.advertise<std_msgs::Bool>("fort_estop", 10);
        pub_rosState = node.advertise<deeporange14_msgs::RosState>(std::string(topic_ns +"/ros_state"), 10);
        pub_rtkState = node.advertise<deeporange14_msgs::RosState>(std::string(topic_ns +"/ros_state"), 10);
        // Set up Timer - with calback to publish ROS state all the time that the node is running
        timer = node.createTimer(ros::Duration(1.0 / 20.0), &StateMachine::publishROSState, this);

        /* Initiate ROS State with a DEFAULT state to be safe. This state will be published till the ...
        ROS supervisor intentionally changes it.Default Node is On and it is running continuously in linux service
        */
        rosSupMsg.ros_state = AU0_DEFAULT;
        rosSupMsg.system_state= SS1_DEFAULT;
        rosSupMsg.fault_code = 0;
        rosSupMsg.stack_faults = false ;
        rosSupMsg.raptorHeartfail = false;
        rosSupMsg.counter = 1;
        rosSupMsg.brake_enable = true;
        rosSupMsg.logging_enable = true;

        
    }
    StateMachine::~StateMachine(){}

    void StateMachine::publishROSState(const ros::TimerEvent& event)
    {
        /* Always continue to publish ROS state  */

        StateMachine::updateROSStateMsg();
        pub_rosState.publish(rosSupMsg);
    }
    void StateMachine::updateROSStateMsg(){

        if (rosSupMsg.ros_state == AU1_STARTUP){

            ros::Time first_time_counter = ros::Time::now();

            if (!isHandshakefailed()){
                // we are in Startup Raptor Handshake established
                rosSupMsg.ros_state = AU2_IDLE;
            }else{
                // keep checking for 3 secs 
                // if still not after 3 sec give Error
                while(abs(first_time_counter.toSec())- ros::Time::now().toSec()<= 5){ // allow 5 seconds to {
                    
                    rosSupMsg.counter = counter++;
                    if (!isHandshakefailed()){
                        // we are in Startup Raptor Handshake established
                        rosSupMsg.ros_state = AU2_IDLE;
                    }    
                }

            }
            
        }
        else if (rosSupMsg.ros_state == AU2_IDLE){
            // %set Brakes Enable true % set torques = 0 check ss and ros modecheck isHanshakeFailed()
            rosSupMsg.brake_enable = true;
            torqueMsg.left_torque = 0;
            torqueMsg.right_torque = 0;
            if (isHandshakefailed()){
                rosSupMsg.ros_state = AU1_STARTUP;
            }

            if (raptorMsg.system_state == SS8_NOMINALOP && raptorMsg.dbw_mode == 3 ){
                rosSupMsg.ros_state = AU3_WAIT_EXECUTION;
            }

        }
        else if (rosSupMsg.ros_state == AU3_WAIT_EXECUTION){
            if (isHandshakefailed()){
                rosSupMsg.ros_state = AU1_STARTUP;
            }
            if (isStackFault()){
                rosSupMsg.ros_state = AU2_IDLE;
                //  ****************change dbw mode ***********************
            }
            rosSupMsg.brake_enable = true;
            torqueMsg.left_torque = 0;
            torqueMsg.right_torque = 0;
            // 
             // is execution button pressed
                //  update state if true
            //  
            if (executed_Nav){
                rosSupMsg.ros_state = AU4_EXEC_IMINENT;
            }
        }
        else if (rosSupMsg.ros_state == AU4_EXEC_IMINENT){
            if (isHandshakefailed()){
                rosSupMsg.ros_state = AU1_STARTUP;
            }
            if (isStackFault()){
                rosSupMsg.ros_state = AU2_IDLE;
                // ****************change dbw mode ***********************
            }
            // //   publish brake command and torque commands
            rosSupMsg.brake_enable = true;
            torqueMsg.left_torque = 0;
            torqueMsg.right_torque = 0;
            // 
            // is Global plan ready
                //  update state if true 
            //                         
        }

        else if (rosSupMsg.ros_state == AU5_DISENGAGE_BRAKE){
            if (isHandshakefailed()){
                rosSupMsg.ros_state = AU1_STARTUP;
            }
            if (isStackFault()){
                rosSupMsg.ros_state = AU2_IDLE;
                //  ****************change dbw mode ***********************
            }
            rosSupMsg.brake_enable = false;
            // raptor acknowledged brake disabled
            //  update state if true
                        
        }    
        else if (rosSupMsg.ros_state == AU6_COMMAND_TORQUES){
            if (isHandshakefailed()){
                rosSupMsg.ros_state = AU1_STARTUP;
            }
            if (isStackFault()){
                rosSupMsg.ros_state = AU2_IDLE;
                 //  ****************change dbw mode ***********************
            }
            // is mission completed check from navigation manager
                // command status == 6
                // Update Ros state

                        
        }    
        else if (rosSupMsg.ros_state == AU98_SAFE_STOP){
            if (isHandshakefailed()){
                rosSupMsg.ros_state = AU1_STARTUP;
            } 
            if (isStackFault()){
                rosSupMsg.ros_state = AU2_IDLE;
                //  ****************change dbw mode ***********************
            }    
            rosSupMsg.brake_enable = true;
                //   publish brake command
        } 
        else if (rosSupMsg.ros_state == AU254_HARD_STOP){
            //  change dbw mode 
        }  
    }
    bool StateMachine::isHandshakeFailed(){
        // checking system_state & ros_state shows default values

    }
    bool StateMachine::isStackFault(){
        // Stack Crashed
        std::string topicName = topic_ns + "/cmd_vel"
        bool topicExists = ros::master::topicExists(topicName);
        if (topicExists){
            return true;
        }else{
            ROS_WARN("ERROR : Stack Crashed !, ROS topic '%s' does not exist ",topicName.c_str());
            return false;
        }

        // Mission Cancelled or StopROS buttons executed 
        if (mission_cancelled or stop_ROS){
            return true;
        }

        // ROS mode/ dbw mode not equal to 3
        if (raptorMsg.dbw_mode != 3){
            ROS_WARN("ERROR : Out of DBW Mode");
            return true;
        }
        
        
    }
    void StateMachine::getCommandedTwist(const geometry_msgs::Twist::ConstPtr& msg){
        commandedTwist.linear.x = msg->linear.x;
        commandedTwist.angular.z = msg->angular.z;
    }
 
    void StateMachine::getCommandedTorques(const deeporange14_msgs::TorqueValues::ConstPtr& msg){   // not used as of now
        torqueMsg.left_torque = msg->left_torque;
        torqueMsg.right_torque = msg->right_torque;    
    }
    void StateMachine::getRaptorMsg(const deeporange14_msgs::RaptorState::ConstPtr& msg){
        raptorMsg_.system_state = msg->system_state;
        rosSupMsg_.raptor_state = msg->system_state;
       
    }
    void StateMachine::getStackStatus(const deeporange14_msgs::RaptorState::ConstPtr& msg){
        mission_cancelled = msg->mission_cancelled;
        mission_completed = msg->mission_completed;
        stop_ROS = msg->stop_ROS;
        executed_nav = msg->executed_nav;
        global_plan_ready = msg->global_plan_ready
       
    }
    
}
