#ifndef _ROS_my_msg_all_sensor_h
#define _ROS_my_msg_all_sensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace my_msg
{

  class all_sensor : public ros::Msg
  {
    public:
      uint16_t yan_wu;
      uint16_t test_man;
      uint16_t huminity;
      uint16_t temperature;
      uint16_t light;
      uint16_t microphone;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->yan_wu >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->yan_wu >> (8 * 1)) & 0xFF;
      offset += sizeof(this->yan_wu);
      *(outbuffer + offset + 0) = (this->test_man >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->test_man >> (8 * 1)) & 0xFF;
      offset += sizeof(this->test_man);
      *(outbuffer + offset + 0) = (this->huminity >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->huminity >> (8 * 1)) & 0xFF;
      offset += sizeof(this->huminity);
      *(outbuffer + offset + 0) = (this->temperature >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->temperature >> (8 * 1)) & 0xFF;
      offset += sizeof(this->temperature);
      *(outbuffer + offset + 0) = (this->light >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->light >> (8 * 1)) & 0xFF;
      offset += sizeof(this->light);
      *(outbuffer + offset + 0) = (this->microphone >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->microphone >> (8 * 1)) & 0xFF;
      offset += sizeof(this->microphone);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->yan_wu =  ((uint16_t) (*(inbuffer + offset)));
      this->yan_wu |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->yan_wu);
      this->test_man =  ((uint16_t) (*(inbuffer + offset)));
      this->test_man |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->test_man);
      this->huminity =  ((uint16_t) (*(inbuffer + offset)));
      this->huminity |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->huminity);
      this->temperature =  ((uint16_t) (*(inbuffer + offset)));
      this->temperature |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->temperature);
      this->light =  ((uint16_t) (*(inbuffer + offset)));
      this->light |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->light);
      this->microphone =  ((uint16_t) (*(inbuffer + offset)));
      this->microphone |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->microphone);
     return offset;
    }

    const char * getType(){ return "my_msg/all_sensor"; };
    const char * getMD5(){ return "2c8964ff699a99327e96f4ecb66b6f6c"; };

  };

}
#endif