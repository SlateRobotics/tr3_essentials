#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "ControllerState.h"

class Trajectory {
  private:
    ControllerState* state;
    double targetPos;
    double targetVelocity;

    double limitVelocity = 0.95;
    double limitAcceleration = 0.75;

    long targetDuration;
    long trajectoryStart;
    double trajectoryPosStart;
    double trajectoryPosGoal;
    long lastStep;

    // geometric properties of velocity profile (isosceles trapezoid)
    float trapezoidArea; // position delta (radians)
    float trapezoidBase; // duration (seconds)
    float trapezoidTop; // duration w/ no accel (seconds)
    float trapezoidHeight; // max velocity (rad / sec)

    // riangle from extending trapezoid sides until they meet (isosceles triangle)
    float triangleA_Base;
    float triangleA_Theta;
    float triangleA_Height;
    float triangleA_Area;

    // triangle formed on top of trapezoid to form triangle B (isosceles triangle)
    float triangleB_Base;
    float triangleB_Theta;
    float triangleB_Height;
    float triangleB_Area;
    float triangleB_Side;

    // right-triangle representing initial accel section of trapezoid
    float triangleC_Base;
    float triangleC_Height;
    float triangleC_Theta;

    void computeProfile () {
      trapezoidArea = trajectoryPosGoal - trajectoryPosStart;
      trapezoidBase = targetDuration / 1000.0;

      triangleA_Base = trapezoidBase;
      triangleA_Theta = atan(limitAcceleration) * sign();
      triangleA_Height = tan(triangleA_Theta) * triangleA_Base / 2.0;
      triangleA_Area = triangleA_Base * triangleA_Height / 2.0;

      triangleB_Area = triangleA_Area - trapezoidArea;
      triangleB_Theta = triangleA_Theta;

      // x will be < 0 if it is impossible to get to position
      // in allotted time given acceleration limits
      float x = (2.0 * triangleB_Area) / sin(2.0 * triangleB_Theta);
      if (x >= 0) {
        triangleB_Side = sqrt(x);
        triangleB_Height = triangleB_Side * sin(triangleB_Theta);
        triangleB_Base = 2.0 * triangleB_Side * cos(triangleB_Theta);

        trapezoidTop = triangleB_Base;
        trapezoidHeight = triangleA_Height - triangleB_Height;
      } else {
        trapezoidTop = 0;
        trapezoidHeight = triangleA_Height;
      }

      triangleC_Height = trapezoidHeight;
      triangleC_Theta = triangleA_Theta;
      triangleC_Base = triangleC_Height / tan(triangleC_Theta);

      //trapezoidHeight = constrain(trapezoidHeight, -limitVelocity, limitVelocity);
    }

  public:
    Trajectory (ControllerState* s) {
      state = s;
    }

    void setLimits(double vel, double acc) {
      limitVelocity = vel;
      limitAcceleration = acc;
    }

    void begin (double pos, long duration) {
      targetPos = state->position;
      targetDuration = duration;
      trajectoryPosStart = state->position;
      trajectoryPosGoal = pos;

      computeProfile();

      trajectoryStart = millis();
      lastStep = millis();
    }

    bool complete () {
      return (millis() > trajectoryStart + targetDuration + 100);
    }

    double getTargetPosition () {
      return targetPos;
    }

    double getTargetVelocity () {
      return targetVelocity;
    }

    float sign () {
      float s = 1.0;
      if (trajectoryPosGoal - trajectoryPosStart < 0) {
        s = -1.0;
      }
      return s;
    }

    void step () {
      double delta = (millis() - lastStep) / 1000.0;
      lastStep = millis();

      long dur_elapsed = millis() - trajectoryStart;
      long dur_remain = targetDuration - dur_elapsed;
      int i = 0;

      if (dur_remain <= 0) {
        i = 3;
        targetPos = trajectoryPosGoal;
        targetVelocity = 0;
      } else if (dur_elapsed > (triangleC_Base + trapezoidTop) * 1000.0) { // decel
        i = 2;
        targetVelocity = targetVelocity - limitAcceleration * delta * sign();
        targetPos = targetPos + targetVelocity * delta;
      } else if (dur_elapsed < triangleC_Base * 1000.0) { // accel
        i = 0;
        targetVelocity = targetVelocity + limitAcceleration * delta * sign();
        targetPos = targetPos + targetVelocity * delta;
      } else { // maintain
        i = 1;
        targetVelocity = trapezoidHeight;
        targetPos = targetPos + targetVelocity * delta;
      }

      if (dur_remain >= 0) {
        Serial.print(millis());
        Serial.print(", ");
        Serial.print(dur_elapsed);
        Serial.print(", ");
        Serial.print(dur_remain);
        Serial.print("::");
        Serial.print(i);
        Serial.print(", ");
        Serial.print(targetPos);
        Serial.print(", ");
        Serial.print(targetVelocity);
        Serial.println();
      }
    }
    
};

#endif
