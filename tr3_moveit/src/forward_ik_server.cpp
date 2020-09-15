#include "ros/ros.h"
#include "tr3_msgs/ForwardIK.h"
#include <Eigen/Geometry>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

bool forward_ik(tr3_msgs::ForwardIK::Request &req, tr3_msgs::ForwardIK::Response &res)
{
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("tr3_arm");

	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	for (std::size_t i = 0; i < joint_names.size(); ++i)
	{
		for (std::size_t j = 0; j < req.state.name.size(); j++)
		{
			if (joint_names[i] == req.state.name[j])
			{
				joint_values[i] = req.state.position[j];
			}
		}
	}

	kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
	kinematic_state->enforceBounds();

	const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link_g0_fixed");

	res.pose.position.x = end_effector_state.translation().x();
	res.pose.position.y = end_effector_state.translation().y();
	res.pose.position.z = end_effector_state.translation().z();

	Eigen::Quaterniond q = (Eigen::Quaterniond)end_effector_state.rotation();
	res.pose.orientation.x = q.x();
	res.pose.orientation.y = q.y();
	res.pose.orientation.z = q.z();
	res.pose.orientation.w = q.w();

  ROS_INFO_STREAM("Found pose: \n" << res.pose << "\n");

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "forward_ik_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("forward_ik", forward_ik);
	ROS_INFO("Ready.");
	ros::spin();

	return 0;
}
