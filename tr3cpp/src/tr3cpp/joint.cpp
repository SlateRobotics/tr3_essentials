#include <stdlib.h>
#include <math.h>
#include <stdexcept>
#include "ros/ros.h"
#include <tr3cpp/packet.h>
#include <tr3cpp/joint.h>
#include <tr3cpp/msgs.h>

#define PI 3.14159265359
#define TAU 6.28318530718

namespace tr3cpp
{
	Joint::Joint() { }
	Joint::~Joint() { }

	double Joint::getPosition()
	{
		return pos;
	}

	void Joint::setPosition(double pos, double speed = 100)
	{
		int x = pos / TAU * 65535;
	
		Packet packet = Packet();
		packet.jointName = name;
		packet.cmd = CMD_SET_POS;
		packet.addParam(std::floor(x % 256));
		packet.addParam(std::floor(x / 256));
		packet.addParam(std::floor(speed / 100.0 * 255.0));
		
		_msgs->add(packet);
	}

	void Joint::setMode(int mode)
	{
		// send msg over tcp
	}

	void Joint::actuate(double effort, int duration = 500)
	{
		int offsetBinary = 128;
		int x = std::floor(effort);

		if (x > 100)
		{
			x = 100;
		}
		else if (x < -100)
		{
			x = -100;
		}
		
		Packet packet = Packet();
		packet.jointName = name;
		packet.cmd = CMD_ROTATE;
		packet.addParam(x + offsetBinary);
		packet.addParam(std::floor(duration % 256));
		packet.addParam(std::floor(duration / 256));
		
		_msgs->add(packet);

		_previousEffort = effort;
	}

	double Joint::getPreviousEffort() {
		return this->_previousEffort;
	}
}
