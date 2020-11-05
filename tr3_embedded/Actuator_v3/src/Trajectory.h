#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "ControllerState.h"

class Trajectory {
  private:
    ControllerState state*;
    double targetVelocity*;
    double targetAcceleration*;

    double limitVelocity = 0.7;
    double limitAcceleration = 0.15;

    double targetPos;
    long targetDuration;
    long trajectoryStart;
    double trajectoryPosStart;
    long lastStep;

  public:
    Trajectory (ControllerState s*, double vel*, double acc*) {
      state = s;
      targetVelocity = vel;
      targetAcceleration = acc;
    }

    void setLimits(double vel, double acc) {
      limitVelocity = vel;
      limitAcceleration = acc;
    }

    void begin (double pos, long duration) {
      targetPos = pos;
      targetDuration = duration;
      trajectoryPosStart = state->position;
      trajectoryStart = millis();
      lastStep = millis();
    }

    void step () {
      double delta = (millis() - lastStep) / 1000.0;
      lastStep = millis();
      
      long dur_elapsed = millis() - trajectoryStart;
      long dur_remain = targetDuration - dur_elapsed;
      float dist_total = targetPos - trajectoryPosStart;
      float dist_remain = targetPos - state->position;
      
      float break_dist = (state.velocity * state.velocity) / (acceleration * 2.0);
      
      float vel_req = limtVelocity;
      if (dur_remain > 200) {
        vel_req = dist_remain / (float)dur_remain * 1000.0;
      }

      if (dist_remain <= break_dist) { // decel
        *targetVelocity = targetVelocity - limitAcceleration * delta;
        *targetAcceleration = -limitAcceleration;
      } else if (targetVelocity < vel_req) { // accel
        *targetVelocity = targetVelocity + limitAcceleration * delta;
        *targetAcceleration = limitAcceleration;
      } else { // maintain
        *targetVelocity = vel_req;
        *targetAcceleration = 0.0;
      }
    }
    
};

#endif
