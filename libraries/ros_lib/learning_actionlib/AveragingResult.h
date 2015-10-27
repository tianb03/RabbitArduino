#ifndef _ROS_learning_actionlib_AveragingResult_h
#define _ROS_learning_actionlib_AveragingResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace learning_actionlib
{

  class AveragingResult : public ros::Msg
  {
    public:
      float mevean;
      float std_d;

    AveragingResult():
      mevean(0),
      std_d(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_mevean;
      u_mevean.real = this->mevean;
      *(outbuffer + offset + 0) = (u_mevean.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mevean.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mevean.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mevean.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mevean);
      union {
        float real;
        uint32_t base;
      } u_std_d;
      u_std_d.real = this->std_d;
      *(outbuffer + offset + 0) = (u_std_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_std_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_std_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_std_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->std_d);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_mevean;
      u_mevean.base = 0;
      u_mevean.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mevean.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mevean.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mevean.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mevean = u_mevean.real;
      offset += sizeof(this->mevean);
      union {
        float real;
        uint32_t base;
      } u_std_d;
      u_std_d.base = 0;
      u_std_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_std_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_std_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_std_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->std_d = u_std_d.real;
      offset += sizeof(this->std_d);
     return offset;
    }

    const char * getType(){ return "learning_actionlib/AveragingResult"; };
    const char * getMD5(){ return "339db30b54370a608664fd135d36b0c6"; };

  };

}
#endif