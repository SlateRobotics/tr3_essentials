#include "ros/ros.h"
#include "tr3_msgs/InverseIK.h"
#include <Eigen/Geometry>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

bool inverse_ik(tr3_msgs::InverseIK::Request &req, tr3_msgs::InverseIK::Response &res)
{
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
	
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("tr3_arm");

	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

	const geometry_msgs::Pose &m = req.pose;
	const Eigen::Isometry3d& end_effector_state = Eigen::Translation3d(m.position.x, m.position.y, m.position.z)
		* Eigen::Quaterniond(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z);
	
	std::vector<double> joint_values;
	
	double timeout = 5.0;
	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);
	if (found_ik)
	{
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for (std::size_t i = 0; i < joint_names.size(); ++i)
		{
			res.state.name.push_back(joint_names[i]);
			res.state.position.push_back(joint_values[i]);
			res.state.effort.push_back(0);
			res.state.velocity.push_back(0);
		}
		
		ROS_INFO_STREAM("Found Solution: \n" << res.state << "\n");
	}
	else
	{
		ROS_INFO("Could not find IK solution");
	}

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "inverse_ik_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("inverse_ik", inverse_ik);
	ROS_INFO("Ready.");
	ros::spin();

	return 0;
}
