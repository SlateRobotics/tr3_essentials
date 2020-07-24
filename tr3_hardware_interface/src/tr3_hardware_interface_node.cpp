#include <ros/callback_queue.h>
#include <tr3_hardware_interface/tr3_hardware_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tr3_hardware_interface");
	ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
	nh.setCallbackQueue(&ros_queue);
  tr3_hardware_interface::TR3HardwareInterface tr3(nh);

	ros::MultiThreadedSpinner spinner(0);
	spinner.spin(&ros_queue);

  return 0;
}
