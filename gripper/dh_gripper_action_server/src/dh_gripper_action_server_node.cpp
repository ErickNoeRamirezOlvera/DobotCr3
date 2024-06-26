#include <dh_gripper_action_server/dh_gripper_action_server.h>

int main(int argc, char** argv)
{
  // Can be renamed with standard ROS-node launch interface
  ros::init(argc, argv, "dh_gripper_action_server");
  
  // Private Note Handle for retrieving parameter arguments to the server
  ros::NodeHandle private_nh("~");

  std::string action_server_name;
  private_nh.param<std::string>("action_server_name", action_server_name, "gripper_controller/gripper_cmd");

  // Fill out DH-Gripper Params
  dh_gripper_action_server::DHGripperParams cparams;
  
  private_nh.param<double>("min_angle", cparams.min_angle_, 0.0);
  private_nh.param<double>("max_angle", cparams.max_angle_, 0.93);
  private_nh.param<double>("min_effort", cparams.min_effort_, 20);
  private_nh.param<double>("max_effort", cparams.max_effort_, 100);
  private_nh.param<double>("default_effort", cparams.default_effort_, 100);
  private_nh.param<double>("speed", cparams.speed_, 100);
  private_nh.param<std::string>("control_topic", cparams.control_topic_, "gripper/ctrl");
  private_nh.param<std::string>("state_topic", cparams.state_topic_, "gripper/states");
  private_nh.param<std::string>("joint_states_topic", cparams.joint_states_topic_, "joint_states");
  private_nh.param<std::string>("joint_name", cparams.joint_name_, "left_outer_knuckle_joint");

  ROS_INFO("Initializing DH Gripper action server: %s", action_server_name.c_str());

  // The name of the gripper -> this server communicates over name/inputs and name/outputs
  dh_gripper_action_server::DHGripperActionServer gripper (action_server_name, cparams);

  ROS_INFO("action-server spinning for DH gripper: %s", action_server_name.c_str());
  ros::spin();
}
