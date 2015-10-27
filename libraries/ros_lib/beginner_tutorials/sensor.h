#ifndef _ROS_beginner_tutorials_sensor_h
#define _ROS_beginner_tutorials_sensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace beginner_tutorials
{

  class sensor : public ros::Msg
  {
    public:
      uint16_t a;
      float b;

    sensor():
      a(0),
      b(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->a >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->a >> (8 * 1)) & 0xFF;
      offset += sizeof(this->a);
      union {
        float real;
        uint32_t base;
      } u_b;
      u_b.real = this->b;
      *(outbuffer + offset + 0) = (u_b.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_b.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_b.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_b.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->b);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->a =  ((uint16_t) (*(inbuffer + offset)));
      this->a |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->a);
      union {
        float real;
        uint32_t base;
      } u_b;
      u_b.base = 0;
      u_b.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_b.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_b.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_b.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->b = u_b.real;
      offset += sizeof(this->b);
     return offset;
    }

    const char * getType(){ return "beginner_tutorials/sensor"; };
    const char * getMD5(){ return "4a12171405d3d51976e01fe860b171e5"; };

  };

}
#endif