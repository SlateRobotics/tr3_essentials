
#ifndef TR3CPP__MSG_H
#define TR3CPP__MSG_H

#include "packet.h"

namespace tr3cpp
{
	class Msg
	{
		private:
			Packet packet;

		public:
			uint64_t expires;
			uint64_t _expirePeriod = 500;

			Msg() { }

			Msg(Packet p)
			{
				packet = p;
			}

			~Msg() { }

			int id()
			{
				return packet.msgId;
			}

			void step()
			{
			}
	};
}

#endif
