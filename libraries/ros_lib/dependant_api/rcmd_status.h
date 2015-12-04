#ifndef _ROS_dependant_api_rcmd_status_h
#define _ROS_dependant_api_rcmd_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dependant_api
{

  class rcmd_status : public ros::Msg
  {
    public:
      uint32_t id;
      int32_t status;
      int32_t reason;

    rcmd_status():
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
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_status.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_status.base >> (8 * 3)) & 0xFF;
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
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->status = u_status.real;
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

    const char * getType(){ return "dependant_api/rcmd_status"; };
    const char * getMD5(){ return "53b7146d5c23a87948bf244de404535c"; };

  };

}
#endif