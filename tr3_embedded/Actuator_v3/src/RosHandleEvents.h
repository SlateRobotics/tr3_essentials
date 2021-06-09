#ifndef ROS_HANDLE_EVENTS_H
#define ROS_HANDLE_EVENTS_H

#include "RosHandle.h"
#include "RosHandleBase.h"

// Allows for specification of behavior under certain events
namespace RosHandleEvents {
    Controller* controller;

    void setup (Controller* c) {
        controller = c;
    }

    void handle_ConnectionFailure () {
        controller->cmd_stop();
    }

    void handle_ConnectionRecovery () {
        controller->cmd_release();
        controller->flag_send_commands = true;
    }
}

#endif