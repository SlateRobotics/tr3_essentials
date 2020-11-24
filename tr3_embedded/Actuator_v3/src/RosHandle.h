#ifndef ROS_HANDLE_H
#define ROS_HANDLE_H

#include "Config.h"
#include "Controller.h"

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tr3_msgs/ActuatorState.h>
#include <tr3_msgs/ActuatorPositionCommand.h>
#include <ESP32Hardware.h>

#include "Controller.h"
#include "RosHandleBase.h"
#include "Timer.h"

namespace RosHandle {
  ros::NodeHandle_<ESP32Hardware> nh;
  Controller *controller;

  int failures = 0;
  const int max_failures = 5;

  Timer configTimer(0.5); // hz
  Timer stateTimer(150); // hz

  tr3_msgs::ActuatorState state;
  std_msgs::Float32MultiArray limits;
  std_msgs::Float32MultiArray pid_pos;
  std_msgs::Float32MultiArray pid_vel;
  std_msgs::Float32MultiArray pid_trq;

  ros::Publisher pub_state(RT_STATE, &state);
  ros::Publisher pub_limit(RT_LIMIT, &limits);
  ros::Publisher pub_pid_pos(RT_PID_POS, &pid_pos);
  ros::Publisher pub_pid_vel(RT_PID_VEL, &pid_vel);
  ros::Publisher pub_pid_trq(RT_PID_TRQ, &pid_trq);

  void ros_callback_control_position (const tr3_msgs::ActuatorPositionCommand &msg) {
    controller->cmd_setPosition(msg.position, msg.duration);
  }

  void ros_callback_control_velocity (const std_msgs::Float64 &msg) {
    controller->cmd_setVelocity(msg.data);
  }

  void ros_callback_control_torque (const std_msgs::Float64 &msg) {
    controller->cmd_setTorque(msg.data);
  }

  void ros_callback_control_voltage (const std_msgs::Float64 &msg) {
    controller->cmd_setVoltage(msg.data);
  }
  
  void ros_callback_mode (const std_msgs::UInt8 &msg) {
    controller->cmd_setMode(msg.data);
  }
  
  void ros_callback_reset (const std_msgs::Bool &msg) {
    if (msg.data == true) {
      controller->cmd_reset();
    }
  }
  
  void ros_callback_reset_pos (const std_msgs::Bool &msg) {
    if (msg.data == true) {
      controller->cmd_resetPosition();
    }
  }
  
  void ros_callback_reset_trq (const std_msgs::Bool &msg) {
    if (msg.data == true) {
      controller->cmd_resetTorque();
    }
  }
  
  void ros_callback_flip (const std_msgs::Bool &msg) {
    if (msg.data == true) {
      controller->cmd_flipMotorPins();
    }
  }
  
  void ros_callback_stop (const std_msgs::Bool &msg) {
    if (msg.data == true) {
      controller->cmd_stop();
    } else {
      controller->cmd_release();
    }
  }
  
  void ros_callback_shutdown (const std_msgs::Bool &msg) {
    if (msg.data == true) {
      controller->cmd_shutdown();
    }
  }

  void ros_callback_limit_pos_min (const std_msgs::Float64 &msg) {
    controller->cmd_setLimitPositionMin(msg.data);
  }

  void ros_callback_limit_pos_max (const std_msgs::Float64 &msg) {
    controller->cmd_setLimitPositionMax(msg.data);
  }

  void ros_callback_limit_vel_min (const std_msgs::Float64 &msg) {
    controller->cmd_setLimitVelocityMin(msg.data);
  }

  void ros_callback_limit_vel_max (const std_msgs::Float64 &msg) {
    controller->cmd_setLimitVelocityMax(msg.data);
  }

  void ros_callback_limit_trq_min (const std_msgs::Float64 &msg) {
    controller->cmd_setLimitTorqueMin(msg.data);
  }

  void ros_callback_limit_trq_max (const std_msgs::Float64 &msg) {
    controller->cmd_setLimitTorqueMax(msg.data);
  }
  
  void ros_callback_pid_pos_set (const std_msgs::Float32MultiArray &msg) {
    controller->cmd_updatePidPos(msg.data[0], msg.data[1], msg.data[2]);
  }
  
  void ros_callback_pid_vel_set (const std_msgs::Float32MultiArray &msg) {
    controller->cmd_updatePidVel(msg.data[0], msg.data[1], msg.data[2]);
  }
  
  void ros_callback_pid_trq_set (const std_msgs::Float32MultiArray &msg) {
    controller->cmd_updatePidTrq(msg.data[0], msg.data[1], msg.data[2]);
  }

  ros::Subscriber<std_msgs::UInt8> sub_mode(RT_MODE, &ros_callback_mode);
  ros::Subscriber<std_msgs::Bool> sub_reset(RT_RESET, &ros_callback_reset);
  ros::Subscriber<std_msgs::Bool> sub_reset_pos(RT_RESET_POS, &ros_callback_reset_pos);
  ros::Subscriber<std_msgs::Bool> sub_reset_trq(RT_RESET_TRQ, &ros_callback_reset_trq);
  ros::Subscriber<std_msgs::Bool> sub_flip(RT_FLIP, &ros_callback_flip);
  ros::Subscriber<std_msgs::Bool> sub_stop(RT_STOP, &ros_callback_stop);
  ros::Subscriber<std_msgs::Bool> sub_shutdown(RT_SHUTDOWN, &ros_callback_shutdown);
  ros::Subscriber<std_msgs::Float32MultiArray> sub_pid_pos_set(RT_PID_POS_SET, &ros_callback_pid_pos_set);
  ros::Subscriber<std_msgs::Float32MultiArray> sub_pid_vel_set(RT_PID_VEL_SET, &ros_callback_pid_vel_set);
  ros::Subscriber<std_msgs::Float32MultiArray> sub_pid_trq_set(RT_PID_TRQ_SET, &ros_callback_pid_trq_set);
  ros::Subscriber<std_msgs::Float64> sub_limit_pos_min(RT_LIMIT_POS_MIN, &ros_callback_limit_pos_min);
  ros::Subscriber<std_msgs::Float64> sub_limit_pos_max(RT_LIMIT_POS_MAX, &ros_callback_limit_pos_max);
  ros::Subscriber<std_msgs::Float64> sub_limit_vel_min(RT_LIMIT_VEL_MIN, &ros_callback_limit_vel_min);
  ros::Subscriber<std_msgs::Float64> sub_limit_vel_max(RT_LIMIT_VEL_MAX, &ros_callback_limit_vel_max);
  ros::Subscriber<std_msgs::Float64> sub_limit_trq_min(RT_LIMIT_TRQ_MIN, &ros_callback_limit_trq_min);
  ros::Subscriber<std_msgs::Float64> sub_limit_trq_max(RT_LIMIT_TRQ_MAX, &ros_callback_limit_trq_max);
  ros::Subscriber<tr3_msgs::ActuatorPositionCommand> sub_control_position(RT_CONTROL_POSITION, &ros_callback_control_position);
  ros::Subscriber<std_msgs::Float64> sub_control_velocity(RT_CONTROL_VELOCITY, &ros_callback_control_velocity);
  ros::Subscriber<std_msgs::Float64> sub_control_torque(RT_CONTROL_TORQUE, &ros_callback_control_torque);
  ros::Subscriber<std_msgs::Float64> sub_control_voltage(RT_CONTROL_VOLTAGE, &ros_callback_control_voltage);

  void setup(Controller* c) {
    RosHandleEvents::setup(c);
    controller = c;

    // subscribers
    nh.subscribe(sub_mode);
    nh.subscribe(sub_reset);
    nh.subscribe(sub_reset_pos);
    nh.subscribe(sub_reset_trq);
    nh.subscribe(sub_flip);
    nh.subscribe(sub_stop);
    nh.subscribe(sub_shutdown);
    nh.subscribe(sub_pid_pos_set);
    nh.subscribe(sub_pid_vel_set);
    nh.subscribe(sub_pid_trq_set);
    nh.subscribe(sub_limit_pos_min);
    nh.subscribe(sub_limit_pos_max);
    nh.subscribe(sub_limit_vel_min);
    nh.subscribe(sub_limit_vel_max);
    nh.subscribe(sub_limit_trq_min);
    nh.subscribe(sub_limit_trq_max);
    nh.subscribe(sub_control_position);
    nh.subscribe(sub_control_velocity);
    nh.subscribe(sub_control_torque);
    nh.subscribe(sub_control_voltage);

    // publishers
    nh.advertise(pub_state);
    nh.advertise(pub_limit);
    nh.advertise(pub_pid_pos);
    nh.advertise(pub_pid_vel);
    nh.advertise(pub_pid_trq);

    RosHandleBase::setup(&nh);
  }

  void step() {
    RosHandleBase::step();

    if (nh.connected()) {
      if (configTimer.ready()) {
        controller->setLimits(&limits);
        controller->setPidPosTunings(&pid_pos);
        controller->setPidVelTunings(&pid_vel);
        controller->setPidTrqTunings(&pid_trq);
        
        RosHandle::pub_limit.publish(&RosHandle::limits);
        RosHandle::pub_pid_pos.publish(&RosHandle::pid_pos);
        RosHandle::pub_pid_vel.publish(&RosHandle::pid_vel);
        RosHandle::pub_pid_trq.publish(&RosHandle::pid_trq);
      }

      if (stateTimer.ready()) {
        controller->setActuatorState(&RosHandle::state);
        RosHandle::pub_state.publish(&RosHandle::state);
      }

      int result = nh.spinOnce();
      if (result != ros::SPIN_OK) {
        RosHandleBase::connectRecovery();
      }
    } else {
      RosHandleBase::connectRecovery();
    }
  }
}

#endif