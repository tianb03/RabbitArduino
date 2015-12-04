#ifndef _ROS_dependant_api_rcmd_move_h
#define _ROS_dependant_api_rcmd_move_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dependant_api
{

  class rcmd_move : public ros::Msg
  {
    public:
      uint32_t id;
      float position_x;
      float position_y;
      float position_z;
      const char* tag_epc;

    rcmd_move():
      id(0),
      position_x(0),
      position_y(0),
      position_z(0),
      tag_epc("")
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
        float real;
        uint32_t base;
      } u_position_x;
      u_position_x.real = this->position_x;
      *(outbuffer + offset + 0) = (u_position_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_x);
      union {
        float real;
        uint32_t base;
      } u_position_y;
      u_position_y.real = this->position_y;
      *(outbuffer + offset + 0) = (u_position_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_y);
      union {
        float real;
        uint32_t base;
      } u_position_z;
      u_position_z.real = this->position_z;
      *(outbuffer + offset + 0) = (u_position_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_z);
      uint32_t length_tag_epc = strlen(this->tag_epc);
      memcpy(outbuffer + offset, &length_tag_epc, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->tag_epc, length_tag_epc);
      offset += length_tag_epc;
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
        float real;
        uint32_t base;
      } u_position_x;
      u_position_x.base = 0;
      u_position_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_x = u_position_x.real;
      offset += sizeof(this->position_x);
      union {
        float real;
        uint32_t base;
      } u_position_y;
      u_position_y.base = 0;
      u_position_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_y = u_position_y.real;
      offset += sizeof(this->position_y);
      union {
        float real;
        uint32_t base;
      } u_position_z;
      u_position_z.base = 0;
      u_position_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_z = u_position_z.real;
      offset += sizeof(this->position_z);
      uint32_t length_tag_epc;
      memcpy(&length_tag_epc, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_tag_epc; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_tag_epc-1]=0;
      this->tag_epc = (char *)(inbuffer + offset-1);
      offset += length_tag_epc;
     return offset;
    }

    const char * getType(){ return "dependant_api/rcmd_move"; };
    const char * getMD5(){ return "43980cb9d46cea7b728cb7431ecf5f5c"; };

  };

}
#endif