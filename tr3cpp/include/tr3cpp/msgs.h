#ifndef TR3CPP__MSGS_H
#define TR3CPP__MSGS_H

#define CMD_SET_MODE 0x10
#define CMD_SET_POS 0x11
#define CMD_RESET_POS 0x12
#define CMD_ROTATE 0x13
#define CMD_RETURN_STATUS 0x14
#define CMD_SET_FREQUENCY 0x15

#define MODE_SERVO 0x10
#define MODE_BACKDRIVE 0x11
#define MODE_ROTATE 0x12

#include <tr3cpp/packet.h>
#include "Socket.h"
#include "ProtocolSimple.h"
#include <iostream>
#include <chrono>

namespace Sock = ThorsAnvil::Socket;

namespace tr3cpp
{
	class Msgs
	{
		private:
			int _msgId = 0;

			std::string req;
			std::string state;
			Sock::ServerSocket server = Sock::ServerSocket(12345);

			int incrementMsgId ()
			{
				if (_msgId < 255)
				{
					_msgId++;
				} else {
					_msgId = 0;
				}
				return _msgId;
			}

		public:
			Msgs() { }
			~Msgs() { }

			std::string getState()
			{
				return state;
			}

			void step()
			{
					Sock::DataSocket accept = server.accept();
					Sock::ProtocolSimple acceptSimple(accept);

					// send commands
					acceptSimple.sendMessage("", req);
					req = "nc;";

					// recieve states
					std::string res;
					res.reserve(256);
					acceptSimple.recvMessage(res);
					state = res.substr(0, res.find(";;")) + ";";
			}

			void add(Packet p)
			{
				p.msgId = incrementMsgId();
				req = req + p.toString();
			}
	};
}

#endif
