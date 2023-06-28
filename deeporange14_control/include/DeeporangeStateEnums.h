/*********************************************************************
Refer to DO13 Raptor DBW state machine for info on states
 *********************************************************************/

#ifndef _DEEPORANGE_STATE_ENUMS_H
#define _DEEPORANGE_STATE_ENUMS_H
#include <stdint.h>

namespace deeporange14
{

#undef BUILD_ASSERT

enum {
    // ROS States
    AU0_DEFAULT              = 0,
    AU1_STARTUP              = 1,
    AU2_IDLE                 = 2,
    AU3_WAIT_EXECUTION       = 3,
    AU4_EXEC_IMINENT         = 4,
    AU5_DISENGAGE_BRAKE      = 5,
    AU6_COMMAND_TORQUES      = 6,
    AU98_SAFE_STOP           = 98,  // GO BACK TO WAIT EXECUTION
    AU254_HARD_STOP          = 254, //GO BACK TO IDLE

    
    // System States
    SS1_DEFAULT               = 1,
    SS2_WAKE                  = 2,
    SS6_READY                 = 6,
    SS8_NOMINALOP             = 8,
    SS99_SAFESTOP             = 99,
    SS98_SAFESTOP_EXECUTED    = 98,
    SS31_SHUTDOWN_REQUESTED   = 31,
    SS32_SHUTDOWN             = 32,

    //FAULT States
    FAULT1_STACK_CRASH        = 1,  // GO BACK TO WAIT EXECUTION
    FAULT2_MISSION_CANCELLED  = 2,  // GO BACK TO WAIT EXECUTION
    FAULT3_ROS_MODE_DISABLED  = 3,  // GO BACK TO WAIT EXECUTION
    FAULT254_RAPTOR_HEARTFAIL = 8 //GO BACK TO IDLE

  };
} //deeporange14

#endif


