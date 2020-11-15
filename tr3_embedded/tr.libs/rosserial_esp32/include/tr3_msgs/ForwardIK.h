#ifndef _ROS_SERVICE_ForwardIK_h
#define _ROS_SERVICE_ForwardIK_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"

namespace tr3_msgs
{

static const char FORWARDIK[] = "tr3_msgs/ForwardIK";

  class ForwardIKRequest : public ros::Msg
  {
    public:
      typedef sensor_msgs::JointState _state_type;
      _state_type state;

    ForwardIKRequest():
      state()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->state.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->state.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return FORWARDIK; };
    const char * getMD5(){ return "b869e8f6f1d03107da0fd57ef24c9c1d"; };

  };

  class ForwardIKResponse : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;

    ForwardIKResponse():
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return FORWARDIK; };
    const char * getMD5(){ return "f192399f711a48924df9a394d37edd67"; };

  };

  class ForwardIK {
    public:
    typedef ForwardIKRequest Request;
    typedef ForwardIKResponse Response;
  };

}
#endif
