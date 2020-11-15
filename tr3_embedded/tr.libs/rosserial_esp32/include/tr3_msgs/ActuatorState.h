#ifndef _ROS_tr3_msgs_ActuatorState_h
#define _ROS_tr3_msgs_ActuatorState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tr3_msgs
{

  class ActuatorState : public ros::Msg
  {
    public:
      typedef const char* _id_type;
      _id_type id;
      typedef uint8_t _mode_type;
      _mode_type mode;
      typedef bool _stop_type;
      _stop_type stop;
      typedef double _position_type;
      _position_type position;
      typedef int64_t _rotations_type;
      _rotations_type rotations;
      typedef double _effort_type;
      _effort_type effort;
      typedef double _velocity_type;
      _velocity_type velocity;
      typedef double _torque_type;
      _torque_type torque;
      typedef double _temperature_type;
      _temperature_type temperature;
      uint32_t imu_accel_length;
      typedef double _imu_accel_type;
      _imu_accel_type st_imu_accel;
      _imu_accel_type * imu_accel;
      uint32_t imu_gyro_length;
      typedef double _imu_gyro_type;
      _imu_gyro_type st_imu_gyro;
      _imu_gyro_type * imu_gyro;
      uint32_t imu_mag_length;
      typedef double _imu_mag_type;
      _imu_mag_type st_imu_mag;
      _imu_mag_type * imu_mag;

    ActuatorState():
      id(""),
      mode(0),
      stop(0),
      position(0),
      rotations(0),
      effort(0),
      velocity(0),
      torque(0),
      temperature(0),
      imu_accel_length(0), imu_accel(NULL),
      imu_gyro_length(0), imu_gyro(NULL),
      imu_mag_length(0), imu_mag(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      union {
        bool real;
        uint8_t base;
      } u_stop;
      u_stop.real = this->stop;
      *(outbuffer + offset + 0) = (u_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stop);
      union {
        double real;
        uint64_t base;
      } u_position;
      u_position.real = this->position;
      *(outbuffer + offset + 0) = (u_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_position.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_position.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_position.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_position.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->position);
      union {
        int64_t real;
        uint64_t base;
      } u_rotations;
      u_rotations.real = this->rotations;
      *(outbuffer + offset + 0) = (u_rotations.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rotations.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rotations.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rotations.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rotations.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rotations.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rotations.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rotations.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rotations);
      union {
        double real;
        uint64_t base;
      } u_effort;
      u_effort.real = this->effort;
      *(outbuffer + offset + 0) = (u_effort.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_effort.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_effort.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_effort.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_effort.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_effort.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_effort.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_effort.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->effort);
      union {
        double real;
        uint64_t base;
      } u_velocity;
      u_velocity.real = this->velocity;
      *(outbuffer + offset + 0) = (u_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocity.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocity.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocity.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocity.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocity);
      union {
        double real;
        uint64_t base;
      } u_torque;
      u_torque.real = this->torque;
      *(outbuffer + offset + 0) = (u_torque.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torque.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torque.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torque.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_torque.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_torque.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_torque.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_torque.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->torque);
      union {
        double real;
        uint64_t base;
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_temperature.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_temperature.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_temperature.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_temperature.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->temperature);
      *(outbuffer + offset + 0) = (this->imu_accel_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->imu_accel_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->imu_accel_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->imu_accel_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->imu_accel_length);
      for( uint32_t i = 0; i < imu_accel_length; i++){
      union {
        double real;
        uint64_t base;
      } u_imu_acceli;
      u_imu_acceli.real = this->imu_accel[i];
      *(outbuffer + offset + 0) = (u_imu_acceli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_imu_acceli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_imu_acceli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_imu_acceli.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_imu_acceli.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_imu_acceli.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_imu_acceli.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_imu_acceli.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->imu_accel[i]);
      }
      *(outbuffer + offset + 0) = (this->imu_gyro_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->imu_gyro_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->imu_gyro_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->imu_gyro_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->imu_gyro_length);
      for( uint32_t i = 0; i < imu_gyro_length; i++){
      union {
        double real;
        uint64_t base;
      } u_imu_gyroi;
      u_imu_gyroi.real = this->imu_gyro[i];
      *(outbuffer + offset + 0) = (u_imu_gyroi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_imu_gyroi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_imu_gyroi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_imu_gyroi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_imu_gyroi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_imu_gyroi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_imu_gyroi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_imu_gyroi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->imu_gyro[i]);
      }
      *(outbuffer + offset + 0) = (this->imu_mag_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->imu_mag_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->imu_mag_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->imu_mag_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->imu_mag_length);
      for( uint32_t i = 0; i < imu_mag_length; i++){
      union {
        double real;
        uint64_t base;
      } u_imu_magi;
      u_imu_magi.real = this->imu_mag[i];
      *(outbuffer + offset + 0) = (u_imu_magi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_imu_magi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_imu_magi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_imu_magi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_imu_magi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_imu_magi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_imu_magi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_imu_magi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->imu_mag[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
      union {
        bool real;
        uint8_t base;
      } u_stop;
      u_stop.base = 0;
      u_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->stop = u_stop.real;
      offset += sizeof(this->stop);
      union {
        double real;
        uint64_t base;
      } u_position;
      u_position.base = 0;
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->position = u_position.real;
      offset += sizeof(this->position);
      union {
        int64_t real;
        uint64_t base;
      } u_rotations;
      u_rotations.base = 0;
      u_rotations.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rotations.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rotations.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rotations.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rotations.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rotations.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rotations.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rotations.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rotations = u_rotations.real;
      offset += sizeof(this->rotations);
      union {
        double real;
        uint64_t base;
      } u_effort;
      u_effort.base = 0;
      u_effort.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_effort.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_effort.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_effort.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_effort.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_effort.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_effort.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_effort.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->effort = u_effort.real;
      offset += sizeof(this->effort);
      union {
        double real;
        uint64_t base;
      } u_velocity;
      u_velocity.base = 0;
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_velocity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->velocity = u_velocity.real;
      offset += sizeof(this->velocity);
      union {
        double real;
        uint64_t base;
      } u_torque;
      u_torque.base = 0;
      u_torque.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torque.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torque.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torque.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_torque.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_torque.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_torque.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_torque.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->torque = u_torque.real;
      offset += sizeof(this->torque);
      union {
        double real;
        uint64_t base;
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_temperature.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_temperature.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_temperature.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_temperature.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
      uint32_t imu_accel_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      imu_accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      imu_accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      imu_accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->imu_accel_length);
      if(imu_accel_lengthT > imu_accel_length)
        this->imu_accel = (double*)realloc(this->imu_accel, imu_accel_lengthT * sizeof(double));
      imu_accel_length = imu_accel_lengthT;
      for( uint32_t i = 0; i < imu_accel_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_imu_accel;
      u_st_imu_accel.base = 0;
      u_st_imu_accel.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_imu_accel.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_imu_accel.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_imu_accel.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_imu_accel.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_imu_accel.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_imu_accel.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_imu_accel.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_imu_accel = u_st_imu_accel.real;
      offset += sizeof(this->st_imu_accel);
        memcpy( &(this->imu_accel[i]), &(this->st_imu_accel), sizeof(double));
      }
      uint32_t imu_gyro_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      imu_gyro_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      imu_gyro_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      imu_gyro_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->imu_gyro_length);
      if(imu_gyro_lengthT > imu_gyro_length)
        this->imu_gyro = (double*)realloc(this->imu_gyro, imu_gyro_lengthT * sizeof(double));
      imu_gyro_length = imu_gyro_lengthT;
      for( uint32_t i = 0; i < imu_gyro_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_imu_gyro;
      u_st_imu_gyro.base = 0;
      u_st_imu_gyro.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_imu_gyro.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_imu_gyro.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_imu_gyro.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_imu_gyro.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_imu_gyro.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_imu_gyro.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_imu_gyro.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_imu_gyro = u_st_imu_gyro.real;
      offset += sizeof(this->st_imu_gyro);
        memcpy( &(this->imu_gyro[i]), &(this->st_imu_gyro), sizeof(double));
      }
      uint32_t imu_mag_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      imu_mag_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      imu_mag_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      imu_mag_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->imu_mag_length);
      if(imu_mag_lengthT > imu_mag_length)
        this->imu_mag = (double*)realloc(this->imu_mag, imu_mag_lengthT * sizeof(double));
      imu_mag_length = imu_mag_lengthT;
      for( uint32_t i = 0; i < imu_mag_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_imu_mag;
      u_st_imu_mag.base = 0;
      u_st_imu_mag.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_imu_mag.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_imu_mag.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_imu_mag.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_imu_mag.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_imu_mag.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_imu_mag.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_imu_mag.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_imu_mag = u_st_imu_mag.real;
      offset += sizeof(this->st_imu_mag);
        memcpy( &(this->imu_mag[i]), &(this->st_imu_mag), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "tr3_msgs/ActuatorState"; };
    const char * getMD5(){ return "b833ec3033cb140fdd405da76de49b57"; };

  };

}
#endif
