#ifndef _ROS_dependant_api_rcmd_adjust_h
#define _ROS_dependant_api_rcmd_adjust_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dependant_api
{

  class rcmd_adjust : public ros::Msg
  {
    public:
      uint32_t id;
      int32_t xposition;

    rcmd_adjust():
      id(0),
      xposition(0)
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
      } u_xposition;
      u_xposition.real = this->xposition;
      *(outbuffer + offset + 0) = (u_xposition.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xposition.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xposition.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xposition.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xposition);
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
      } u_xposition;
      u_xposition.base = 0;
      u_xposition.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xposition.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xposition.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xposition.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xposition = u_xposition.real;
      offset += sizeof(this->xposition);
     return offset;
    }

    const char * getType(){ return "dependant_api/rcmd_adjust"; };
    const char * getMD5(){ return "7f3f706a1b94b3f3433cb61966c92cdb"; };

  };

}
#endif