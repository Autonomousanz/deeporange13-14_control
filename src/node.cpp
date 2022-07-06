/*********************************************************************
Written for use in Deep Orange 13 Drive-by-wire package
Makes use of New Eagle package: can_dbc_parser (https://github.com/NewEagleRaptor/raptor-dbw-ros/tree/master/can_dbc_parser)

 *********************************************************************/

#include <ros/ros.h>
#include <deeporange13_control/DeepOrangeDbwCan.h>
#include <deeporange13_control/VehicleModel.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "deeporange_dbw");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create CAN dbw object
  deeporange_dbw_ros::DeepOrangeDbwCan n_can(node, priv_nh);

  // create Model object
  deeporange_dbw_ros::VehicleModel n_dynamics(node, priv_nh);

  // create the dbwSupervisor Object


  // handle callbacks until shut down
  ros::spin();

  return 0;
}
