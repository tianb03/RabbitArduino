#ifndef _ROS_dependant_api_robot_status_h
#define _ROS_dependant_api_robot_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dependant_api
{

  class robot_status : public ros::Msg
  {
    public:
      uint32_t status;
      uint32_t powerlevel;
      uint32_t uptime;
      uint32_t timeleft;
      uint32_t chargetime;
      float position_x;
      float position_y;
      float position_z;
      const char* tag_epc;
      int32_t rec_hor;
      int32_t rec_ver;

    robot_status():
      status(0),
      powerlevel(0),
      uptime(0),
      timeleft(0),
      chargetime(0),
      position_x(0),
      position_y(0),
      position_z(0),
      tag_epc(""),
      rec_hor(0),
      rec_ver(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->status >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->status >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->status >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status);
      *(outbuffer + offset + 0) = (this->powerlevel >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->powerlevel >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->powerlevel >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->powerlevel >> (8 * 3)) & 0xFF;
      offset += sizeof(this->powerlevel);
      *(outbuffer + offset + 0) = (this->uptime >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uptime >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->uptime >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->uptime >> (8 * 3)) & 0xFF;
      offset += sizeof(this->uptime);
      *(outbuffer + offset + 0) = (this->timeleft >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeleft >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeleft >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeleft >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeleft);
      *(outbuffer + offset + 0) = (this->chargetime >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->chargetime >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->chargetime >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->chargetime >> (8 * 3)) & 0xFF;
      offset += sizeof(this->chargetime);
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
      this->status =  ((uint32_t) (*(inbuffer + offset)));
      this->status |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->status |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->status |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->status);
      this->powerlevel =  ((uint32_t) (*(inbuffer + offset)));
      this->powerlevel |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->powerlevel |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->powerlevel |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->powerlevel);
      this->uptime =  ((uint32_t) (*(inbuffer + offset)));
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->uptime);
      this->timeleft =  ((uint32_t) (*(inbuffer + offset)));
      this->timeleft |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeleft |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeleft |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timeleft);
      this->chargetime =  ((uint32_t) (*(inbuffer + offset)));
      this->chargetime |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->chargetime |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->chargetime |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->chargetime);
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

    const char * getType(){ return "dependant_api/robot_status"; };
    const char * getMD5(){ return "bce5a7c1bd2abf39daf7771f597f0139"; };

  };

}
#endif