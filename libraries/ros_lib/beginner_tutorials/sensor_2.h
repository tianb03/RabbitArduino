#ifndef _ROS_beginner_tutorials_sensor_2_h
#define _ROS_beginner_tutorials_sensor_2_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace beginner_tutorials
{

  class sensor_2 : public ros::Msg
  {
    public:
      uint16_t AD;
      uint16_t man;
      uint16_t humi;
      uint16_t temp;

    sensor_2():
      AD(0),
      man(0),
      humi(0),
      temp(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->AD >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->AD >> (8 * 1)) & 0xFF;
      offset += sizeof(this->AD);
      *(outbuffer + offset + 0) = (this->man >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->man >> (8 * 1)) & 0xFF;
      offset += sizeof(this->man);
      *(outbuffer + offset + 0) = (this->humi >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->humi >> (8 * 1)) & 0xFF;
      offset += sizeof(this->humi);
      *(outbuffer + offset + 0) = (this->temp >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->temp >> (8 * 1)) & 0xFF;
      offset += sizeof(this->temp);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->AD =  ((uint16_t) (*(inbuffer + offset)));
      this->AD |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->AD);
      this->man =  ((uint16_t) (*(inbuffer + offset)));
      this->man |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->man);
      this->humi =  ((uint16_t) (*(inbuffer + offset)));
      this->humi |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->humi);
      this->temp =  ((uint16_t) (*(inbuffer + offset)));
      this->temp |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->temp);
     return offset;
    }

    const char * getType(){ return "beginner_tutorials/sensor_2"; };
    const char * getMD5(){ return "d2d3c0b816dfd755adea91972b855827"; };

  };

}
#endif