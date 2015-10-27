#ifndef _ROS_dependant_api_arduino_motor_h
#define _ROS_dependant_api_arduino_motor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dependant_api
{

  class arduino_motor : public ros::Msg
  {
    public:
      uint32_t id;
      int32_t ret_hor;
      int32_t ret_ver;

    arduino_motor():
      id(0),
      ret_hor(0),
      ret_ver(0)
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
      } u_ret_hor;
      u_ret_hor.real = this->ret_hor;
      *(outbuffer + offset + 0) = (u_ret_hor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ret_hor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ret_hor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ret_hor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ret_hor);
      union {
        int32_t real;
        uint32_t base;
      } u_ret_ver;
      u_ret_ver.real = this->ret_ver;
      *(outbuffer + offset + 0) = (u_ret_ver.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ret_ver.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ret_ver.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ret_ver.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ret_ver);
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
      } u_ret_hor;
      u_ret_hor.base = 0;
      u_ret_hor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ret_hor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ret_hor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ret_hor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ret_hor = u_ret_hor.real;
      offset += sizeof(this->ret_hor);
      union {
        int32_t real;
        uint32_t base;
      } u_ret_ver;
      u_ret_ver.base = 0;
      u_ret_ver.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ret_ver.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ret_ver.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ret_ver.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ret_ver = u_ret_ver.real;
      offset += sizeof(this->ret_ver);
     return offset;
    }

    const char * getType(){ return "dependant_api/arduino_motor"; };
    const char * getMD5(){ return "30137f396af89ca96caaaece5de56522"; };

  };

}
#endif