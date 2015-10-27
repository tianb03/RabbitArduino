#ifndef _ROS_msg_robotcmd_motor_h
#define _ROS_msg_robotcmd_motor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msg
{

  class robotcmd_motor : public ros::Msg
  {
    public:
      uint32_t id;
      uint32_t angle;

    robotcmd_motor():
      id(0),
      angle(0)
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
      *(outbuffer + offset + 0) = (this->angle >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->angle >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->angle >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->angle >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle);
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
      this->angle =  ((uint32_t) (*(inbuffer + offset)));
      this->angle |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->angle |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->angle |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->angle);
     return offset;
    }

    const char * getType(){ return "msg/robotcmd_motor"; };
    const char * getMD5(){ return "646b4c0b5ec6b78cc972f724eceed756"; };

  };

}
#endif