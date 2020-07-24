#include <sstream>
#include <math.h>
#include <tr3_hardware_interface/tr3_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <tr3cpp/tr3.h>
#include <tr3cpp/joint.h>

#define M_TAU (M_PI * 2)

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace tr3_hardware_interface
{
	TR3HardwareInterface::TR3HardwareInterface(ros::NodeHandle& nh) \
		: nh_(nh)
	{
		init();
		controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

		nh_.param("/tr3/hardware_interface/loop_hz", loop_hz_, 0.1);
		ROS_DEBUG_STREAM_NAMED("constructor","Using loop freqency of " << loop_hz_ << " hz");
		ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
		non_realtime_loop_ = nh_.createTimer(update_freq, &TR3HardwareInterface::update, this);

		ROS_INFO_NAMED("hardware_interface", "Loaded tr3_hardware_interface.");
	}

	TR3HardwareInterface::~TR3HardwareInterface()
	{
	}

	void TR3HardwareInterface::init()
	{
		//joint_mode_ = 3; // ONLY EFFORT FOR NOW
		// Get joint names
		nh_.getParam("/tr3/hardware_interface/joints", joint_names_);
		if (joint_names_.size() == 0)
		{
		  ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
		}
		num_joints_ = joint_names_.size();

		// Resize vectors
		joint_position_.resize(num_joints_);
		joint_velocity_.resize(num_joints_);
		joint_effort_.resize(num_joints_);
		joint_position_command_.resize(num_joints_);
		joint_velocity_command_.resize(num_joints_);
		joint_effort_command_.resize(num_joints_);


		// Initialize controller
		for (int i = 0; i < num_joints_; ++i)
		{
			ROS_INFO_STREAM("Loading controller for joint " << joint_names_[i]);
			tr3cpp::Joint joint = tr3.getJoint(joint_names_[i]);

		  ROS_DEBUG_STREAM_NAMED("constructor","Loading joint name: " << joint.name);

		  // Create joint state interface
			JointStateHandle jointStateHandle(joint.name, &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
		  joint_state_interface_.registerHandle(jointStateHandle);

		  // Create position joint interface
			JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
			JointLimits limits;
 	   	SoftJointLimits softLimits;
			if (getJointLimits(joint.name, nh_, limits) == false) {
				ROS_INFO_STREAM("Could not find joint limits for " << joint.name);
			} else {
				PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
				positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
			}
		  position_joint_interface_.registerHandle(jointPositionHandle);

		  // Create velocity joint interface
			//JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
		  //effort_joint_interface_.registerHandle(jointVelocityHandle);

		  // Create effort joint interface
			JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
		  effort_joint_interface_.registerHandle(jointEffortHandle);

		}

		registerInterface(&joint_state_interface_);
		registerInterface(&position_joint_interface_);
		//registerInterface(&velocity_joint_interface_);
		registerInterface(&effort_joint_interface_);
		registerInterface(&positionJointSoftLimitsInterface);
	}

	void TR3HardwareInterface::update(const ros::TimerEvent& e)
	{
		_logInfo = "\n";
		_logInfo += "Joint Position Command:\n";
		for (int i = 0; i < num_joints_; i++)
		{
			std::ostringstream jointPositionStr;
			jointPositionStr << joint_position_command_[i];
			_logInfo += "  " + joint_names_[i] + ": " + jointPositionStr.str() + "\n";
		}

		tr3.step();

		elapsed_time_ = ros::Duration(e.current_real - e.last_real);

		read();
		controller_manager_->update(ros::Time::now(), elapsed_time_);
		write(elapsed_time_);

		//ROS_INFO_STREAM(_logInfo);
	}

	void TR3HardwareInterface::read()
	{
		_logInfo += "Joint State:\n";
		for (int i = 0; i < num_joints_; i++)
		{
			tr3cpp::Joint joint = tr3.getJoint(joint_names_[i]);

			joint_position_[i] = joint.getPosition();

			if (joint_position_[i] > M_PI) {
				joint_position_[i] = joint_position_[i] - M_TAU;
			}

			if (joint_position_[i] > M_PI) {
				joint_position_[i] = joint_position_[i] - M_TAU;
			}

			std::ostringstream jointPositionStr;
			jointPositionStr << joint_position_[i];
			_logInfo += "  " + joint.name + ": " + jointPositionStr.str() + "\n";
			tr3.setJoint(joint);
		}
	}

	void TR3HardwareInterface::write(ros::Duration elapsed_time)
	{
		positionJointSoftLimitsInterface.enforceLimits(elapsed_time);

		_logInfo += "Joint Effort Command:\n";
		for (int i = 0; i < num_joints_; i++)
		{
			tr3cpp::Joint joint = tr3.getJoint(joint_names_[i]);

			if (joint.name == "g0") {
				//std::cout << joint_names_[i] << ", " << joint_position_command_[i] << "\n";
			} else {
				double effort = joint_effort_command_[i];
				int duration = 150;
				//std::cout << joint_names_[i] << ", " << effort << "\n";
				joint.actuate(effort, duration);

				std::ostringstream jointEffortStr;
				jointEffortStr << joint_effort_command_[i];
				_logInfo += "  " + joint.name + ": " + jointEffortStr.str() + "\n";
			}
		}
	}
}

