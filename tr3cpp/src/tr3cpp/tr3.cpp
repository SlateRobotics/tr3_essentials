#include "ros/ros.h"
#include <stdexcept>
#include <tr3cpp/tr3.h>
#include <tr3cpp/joint.h>

namespace tr3cpp
{
	TR3::TR3()
	{
		joints[0].name = "b0";
		joints[1].name = "b1";
		joints[2].name = "a0";
		joints[3].name = "a1";
		joints[4].name = "a2";
		joints[5].name = "a3";
		joints[6].name = "a4";
		joints[7].name = "g0";
		joints[8].name = "h0";
		joints[9].name = "h1";

		int numJoints = sizeof(joints) / sizeof(joints[0]);
		for (int i = 0; i < numJoints; i++)
		{
			joints[i]._msgs = &_msgs;
		}

		waitForState();
		step();
	}

	TR3::~TR3()
	{

	}

	void TR3::waitForState()
	{
		std::string state = "";
		while (state == "")
		{
			_msgs.step();
			state = _msgs.getState();
		}
	}

	Joint TR3::getJoint(std::string jointName)
	{
		int numJoints = sizeof(joints) / sizeof(joints[0]);
		for (int i = 0; i < numJoints; i++)
		{
			if (joints[i].name == jointName)
			{
				return joints[i];
			}
		}

		throw std::runtime_error("Could not find joint with name " + jointName);
	}

	void TR3::step()
	{
		_msgs.step();

		std::string state = _msgs.getState();
		int numJoints = sizeof(joints) / sizeof(joints[0]);

		for (int i = 0; i < numJoints; i++)
		{
			unsigned p1 = state.find(joints[i].name) + 3;
			unsigned p2 = p1 + state.substr(p1, state.length() - p1).find(";");
			std::string s = state.substr(p1, p2-p1);

			// if joint name is in state ( state.find() returns -1 )
			if (p1 > 2) {
				if (s != "" && s != "ns")
				{
					try
					{
						joints[i].pos = std::stod(s);
					}
					catch (int e)
					{

					}
				}
			}
		}
		
	}

	void TR3::setJoint(Joint joint)
	{
		bool foundJoint = false;

		int numJoints = sizeof(joints) / sizeof(joints[0]);
		for (int i = 0; i < numJoints; i++)
		{
			if (joints[i].name == joint.name)
			{
				foundJoint = true;
				joints[i] = joint;
			}
		}

		if (foundJoint == false)
		{
			throw std::runtime_error("Could not find joint with name " + joint.name);
		}
	}
}
