#include <ros/ros.h>
#include <boost/bind.hpp>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <eigen_conversions/eigen_msg.h>

#include <robot_state_publisher/robot_state_publisher.h>
#include <tf_conversions/tf_eigen.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

std::string linkgroup = "right_arm";

ros::NodeHandle *node_handle = NULL;
moveit::planning_interface::MoveGroup *group = NULL;
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface = NULL;
robot_state::RobotStatePtr kinematic_state;
robot_state::JointModelGroup* iiwa_modelgroup;
robot_model::RobotModelPtr kinematic_model;
std_msgs::Float64MultiArray msg;

void publish_joint_values(ros::Publisher kinematics_pub){

 ros::Rate loop_rate(10);
  while (ros::ok())
   {
     kinematics_pub.publish(msg);
     ros::spinOnce();
     loop_rate.sleep();
   }
}

void calculate_joint_angles(std_msgs::Float64MultiArray array){

	moveit_msgs::DisplayTrajectory display_trajectory;
	geometry_msgs::Pose target_position1;

	target_position1.position.x = array.data[0];
	target_position1.position.y = array.data[1];
	target_position1.position.z = array.data[2];

	// Compute IK
	Eigen::Affine3d target_eig_pose1;
	tf::poseMsgToEigen(target_position1, target_eig_pose1);
	kinematic_state->setToIKSolverFrame(target_eig_pose1, group->getPlanningFrame()); // Convert to robotmodel frame if needed.
	bool found_ik = kinematic_state->setFromIK(iiwa_modelgroup, target_eig_pose1, 10, 0.1);
	if(!found_ik){
	    ROS_INFO("Not Found");
		return;
	}

	// Retrieve joint values map
	std::map<std::string, double> goal_joint_values;
	std::vector<double> iiwa_joint_values;
	kinematic_state->copyJointGroupPositions(iiwa_modelgroup, iiwa_joint_values);
	const std::vector<std::string> &iiwa_joint_names = iiwa_modelgroup->getJointModelNames();

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = msg.data.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "Joint Angles";
    msg.data.clear();
	for(std::size_t i = 1; i < iiwa_joint_names.size(); ++i) {
		ROS_INFO("Joint %s: %f", iiwa_joint_names[i].c_str(), iiwa_joint_values[i]);
		msg.data.push_back(iiwa_joint_values[i]);
		goal_joint_values[iiwa_joint_names[i]] = iiwa_joint_values[i];
	}

	group->setJointValueTarget(goal_joint_values);

	robot_state::RobotState start_state(*group->getCurrentState());
	group->setStartState(start_state);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group->plan(my_plan);
	group->execute(my_plan);

    ros::Publisher kinematics_pub= node_handle->advertise<std_msgs::Float64MultiArray>("joint_values", 1000);
    publish_joint_values(kinematics_pub);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "kinematics");
	ros::NodeHandle n;
	node_handle = &n;

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model = robot_model_loader.getModel();
	kinematic_state.reset(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	iiwa_modelgroup = kinematic_model->getJointModelGroup(linkgroup);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::PlanningSceneInterface psi;
	planning_scene_interface = &psi;
	moveit::planning_interface::MoveGroup g(linkgroup);
	group = &g;

    ros::Subscriber sub = node_handle->subscribe("inverse_kinematics", 1000, calculate_joint_angles);
    ros::spin();
	ros::shutdown();
	return 0;
}

