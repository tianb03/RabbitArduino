#ifndef _ROS_serve_logitech_all_sensor_h
#define _ROS_serve_logitech_all_sensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace serve_logitech
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
      float a_f;
      float b_f;
      float c_f;
      uint16_t a_ui;
      uint16_t b_ui;
      uint16_t c_ui;

    all_sensor():
      yan_wu(0),
      test_man(0),
      huminity(0),
      temperature(0),
      light(0),
      microphone(0),
      a_f(0),
      b_f(0),
      c_f(0),
      a_ui(0),
      b_ui(0),
      c_ui(0)
    {
    }

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
      union {
        float real;
        uint32_t base;
      } u_a_f;
      u_a_f.real = this->a_f;
      *(outbuffer + offset + 0) = (u_a_f.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_a_f.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_a_f.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_a_f.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->a_f);
      union {
        float real;
        uint32_t base;
      } u_b_f;
      u_b_f.real = this->b_f;
      *(outbuffer + offset + 0) = (u_b_f.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_b_f.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_b_f.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_b_f.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->b_f);
      union {
        float real;
        uint32_t base;
      } u_c_f;
      u_c_f.real = this->c_f;
      *(outbuffer + offset + 0) = (u_c_f.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c_f.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c_f.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c_f.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->c_f);
      *(outbuffer + offset + 0) = (this->a_ui >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->a_ui >> (8 * 1)) & 0xFF;
      offset += sizeof(this->a_ui);
      *(outbuffer + offset + 0) = (this->b_ui >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->b_ui >> (8 * 1)) & 0xFF;
      offset += sizeof(this->b_ui);
      *(outbuffer + offset + 0) = (this->c_ui >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->c_ui >> (8 * 1)) & 0xFF;
      offset += sizeof(this->c_ui);
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
      union {
        float real;
        uint32_t base;
      } u_a_f;
      u_a_f.base = 0;
      u_a_f.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_a_f.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_a_f.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_a_f.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->a_f = u_a_f.real;
      offset += sizeof(this->a_f);
      union {
        float real;
        uint32_t base;
      } u_b_f;
      u_b_f.base = 0;
      u_b_f.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_b_f.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_b_f.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_b_f.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->b_f = u_b_f.real;
      offset += sizeof(this->b_f);
      union {
        float real;
        uint32_t base;
      } u_c_f;
      u_c_f.base = 0;
      u_c_f.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c_f.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c_f.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c_f.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->c_f = u_c_f.real;
      offset += sizeof(this->c_f);
      this->a_ui =  ((uint16_t) (*(inbuffer + offset)));
      this->a_ui |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->a_ui);
      this->b_ui =  ((uint16_t) (*(inbuffer + offset)));
      this->b_ui |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->b_ui);
      this->c_ui =  ((uint16_t) (*(inbuffer + offset)));
      this->c_ui |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->c_ui);
     return offset;
    }

    const char * getType(){ return "serve_logitech/all_sensor"; };
    const char * getMD5(){ return "7f3d82c757f934d13f587f51434e2daa"; };

  };

}
#endif