#ifndef _ROS_dependant_api_UhfRfid_h
#define _ROS_dependant_api_UhfRfid_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace dependant_api
{

  class UhfRfid : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* data;
      int32_t count;
      int32_t pos_x;
      int32_t pos_y;
      int32_t RSSI;

    UhfRfid():
      header(),
      data(""),
      count(0),
      pos_x(0),
      pos_y(0),
      RSSI(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_data = strlen(this->data);
      memcpy(outbuffer + offset, &length_data, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      union {
        int32_t real;
        uint32_t base;
      } u_count;
      u_count.real = this->count;
      *(outbuffer + offset + 0) = (u_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_count.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_count.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_count.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->count);
      union {
        int32_t real;
        uint32_t base;
      } u_pos_x;
      u_pos_x.real = this->pos_x;
      *(outbuffer + offset + 0) = (u_pos_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_x);
      union {
        int32_t real;
        uint32_t base;
      } u_pos_y;
      u_pos_y.real = this->pos_y;
      *(outbuffer + offset + 0) = (u_pos_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_y);
      union {
        int32_t real;
        uint32_t base;
      } u_RSSI;
      u_RSSI.real = this->RSSI;
      *(outbuffer + offset + 0) = (u_RSSI.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_RSSI.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_RSSI.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_RSSI.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->RSSI);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_data;
      memcpy(&length_data, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
      union {
        int32_t real;
        uint32_t base;
      } u_count;
      u_count.base = 0;
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->count = u_count.real;
      offset += sizeof(this->count);
      union {
        int32_t real;
        uint32_t base;
      } u_pos_x;
      u_pos_x.base = 0;
      u_pos_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos_x = u_pos_x.real;
      offset += sizeof(this->pos_x);
      union {
        int32_t real;
        uint32_t base;
      } u_pos_y;
      u_pos_y.base = 0;
      u_pos_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos_y = u_pos_y.real;
      offset += sizeof(this->pos_y);
      union {
        int32_t real;
        uint32_t base;
      } u_RSSI;
      u_RSSI.base = 0;
      u_RSSI.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_RSSI.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_RSSI.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_RSSI.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->RSSI = u_RSSI.real;
      offset += sizeof(this->RSSI);
     return offset;
    }

    const char * getType(){ return "dependant_api/UhfRfid"; };
    const char * getMD5(){ return "6faba43563df619b753b65f5c2d3e33e"; };

  };

}
#endif