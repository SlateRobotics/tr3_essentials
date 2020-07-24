
#ifndef TR3CPP__PACKET_H
#define TR3CPP__PACKET_H

namespace tr3cpp
{
	class Packet
	{
		private:
			uint8_t paramsCount = 0;
			uint8_t params[16];

			char string[32];

		public:
			int msgId = 0;
			std::string jointName;
			uint8_t cmd;

			Packet() { }
			~Packet() { }

			void addParam(int p)
			{
				params[paramsCount++] = p;
			}

			std::string toString()
			{
				int length = 4 + paramsCount;

				std::string p;
				p = p + jointName + ":";
				p = p + std::to_string(msgId) + ",0,";
				p = p + std::to_string(length) + ",";
				p = p + std::to_string(cmd) + ",";

				for (int i = 0; i < paramsCount; i++) {
					p = p + std::to_string(params[i]) + ",";
				}

				p = p + ";";

				return p;
			}
	};
}

#endif
