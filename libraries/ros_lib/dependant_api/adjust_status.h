#ifndef _ROS_dependant_api_adjust_status_h
#define _ROS_dependant_api_adjust_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dependant_api
{

  class adjust_status : public ros::Msg
  {
    public:
      uint32_t id;
      uint32_t status;
      int32_t reason;

    adjust_status():
      id(0),
      status(0),
      reason(0)
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
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->status >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->status >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->status >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status);
      union {
        int32_t real;
        uint32_t base;
      } u_reason;
      u_reason.real = this->reason;
      *(outbuffer + offset + 0) = (u_reason.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reason.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reason.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reason.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reason);
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
      this->status =  ((uint32_t) (*(inbuffer + offset)));
      this->status |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->status |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->status |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->status);
      union {
        int32_t real;
        uint32_t base;
      } u_reason;
      u_reason.base = 0;
      u_reason.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reason.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reason.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reason.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reason = u_reason.real;
      offset += sizeof(this->reason);
     return offset;
    }

    const char * getType(){ return "dependant_api/adjust_status"; };
    const char * getMD5(){ return "d3610c7eca3b32c285432d2b6c1814f1"; };

  };

}
#endif