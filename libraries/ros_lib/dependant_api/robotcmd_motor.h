#ifndef _ROS_dependant_api_robotcmd_motor_h
#define _ROS_dependant_api_robotcmd_motor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dependant_api
{

  class robotcmd_motor : public ros::Msg
  {
    public:
      uint32_t id;
      int32_t hor_angle;
      int32_t ver_angle;

    robotcmd_motor():
      id(0),
      hor_angle(0),
      ver_angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_hor_angle;
      u_hor_angle.real = this->hor_angle;
      *(outbuffer + offset + 0) = (u_hor_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hor_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_hor_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_hor_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->hor_angle);
      union {
        int32_t real;
        uint32_t base;
      } u_ver_angle;
      u_ver_angle.real = this->ver_angle;
      *(outbuffer + offset + 0) = (u_ver_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ver_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ver_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ver_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ver_angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_hor_angle;
      u_hor_angle.base = 0;
      u_hor_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hor_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_hor_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_hor_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->hor_angle = u_hor_angle.real;
      offset += sizeof(this->hor_angle);
      union {
        int32_t real;
        uint32_t base;
      } u_ver_angle;
      u_ver_angle.base = 0;
      u_ver_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ver_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ver_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ver_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ver_angle = u_ver_angle.real;
      offset += sizeof(this->ver_angle);
     return offset;
    }

    const char * getType(){ return "dependant_api/robotcmd_motor"; };
    const char * getMD5(){ return "327619fa8eaac79f7f1f593db62f4c94"; };

  };

}
#endif