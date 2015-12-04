#ifndef _ROS_dependant_api_motor_angle_h
#define _ROS_dependant_api_motor_angle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dependant_api
{

  class motor_angle : public ros::Msg
  {
    public:
      uint32_t id;
      int32_t rec_hor;
      int32_t rec_ver;

    motor_angle():
      id(0),
      rec_hor(0),
      rec_ver(0)
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
      } u_rec_hor;
      u_rec_hor.real = this->rec_hor;
      *(outbuffer + offset + 0) = (u_rec_hor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rec_hor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rec_hor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rec_hor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rec_hor);
      union {
        int32_t real;
        uint32_t base;
      } u_rec_ver;
      u_rec_ver.real = this->rec_ver;
      *(outbuffer + offset + 0) = (u_rec_ver.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rec_ver.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rec_ver.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rec_ver.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rec_ver);
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
      } u_rec_hor;
      u_rec_hor.base = 0;
      u_rec_hor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rec_hor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rec_hor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rec_hor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rec_hor = u_rec_hor.real;
      offset += sizeof(this->rec_hor);
      union {
        int32_t real;
        uint32_t base;
      } u_rec_ver;
      u_rec_ver.base = 0;
      u_rec_ver.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rec_ver.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rec_ver.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rec_ver.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rec_ver = u_rec_ver.real;
      offset += sizeof(this->rec_ver);
     return offset;
    }

    const char * getType(){ return "dependant_api/motor_angle"; };
    const char * getMD5(){ return "806f09a3b25be001a66718d38cec3e50"; };

  };

}
#endif