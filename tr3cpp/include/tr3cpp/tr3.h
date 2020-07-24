#ifndef TR3CPP__TR3_H
#define TR3CPP__TR3_H

#include <sstream>
#include <tr3cpp/joint.h>
#include <tr3cpp/msgs.h>

namespace tr3cpp
{
	class TR3
	{
		private:
			Msgs _msgs;

		public:
			TR3();
			~TR3();

			Joint joints[10];
			Joint getJoint(std::string jointName);

			void step();
			void waitForState();
			void setJoint(tr3cpp::Joint joint);
	};
}

#endif
