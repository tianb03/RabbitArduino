#ifndef _ROS_dependant_api_all_sensor_h
#define _ROS_dependant_api_all_sensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dependant_api
{

  class all_sensor : public ros::Msg
  {
    public:
      int32_t smoke;
      int32_t human;
      int32_t humidity;
      int32_t temperature;
      int32_t illumination;
      int32_t noise;

    all_sensor():
      smoke(0),
      human(0),
      humidity(0),
      temperature(0),
      illumination(0),
      noise(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_smoke;
      u_smoke.real = this->smoke;
      *(outbuffer + offset + 0) = (u_smoke.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_smoke.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_smoke.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_smoke.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->smoke);
      union {
        int32_t real;
        uint32_t base;
      } u_human;
      u_human.real = this->human;
      *(outbuffer + offset + 0) = (u_human.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_human.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_human.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_human.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->human);
      union {
        int32_t real;
        uint32_t base;
      } u_humidity;
      u_humidity.real = this->humidity;
      *(outbuffer + offset + 0) = (u_humidity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_humidity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_humidity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_humidity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->humidity);
      union {
        int32_t real;
        uint32_t base;
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature);
      union {
        int32_t real;
        uint32_t base;
      } u_illumination;
      u_illumination.real = this->illumination;
      *(outbuffer + offset + 0) = (u_illumination.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_illumination.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_illumination.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_illumination.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->illumination);
      union {
        int32_t real;
        uint32_t base;
      } u_noise;
      u_noise.real = this->noise;
      *(outbuffer + offset + 0) = (u_noise.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_noise.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_noise.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_noise.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->noise);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_smoke;
      u_smoke.base = 0;
      u_smoke.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_smoke.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_smoke.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_smoke.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->smoke = u_smoke.real;
      offset += sizeof(this->smoke);
      union {
        int32_t real;
        uint32_t base;
      } u_human;
      u_human.base = 0;
      u_human.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_human.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_human.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_human.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->human = u_human.real;
      offset += sizeof(this->human);
      union {
        int32_t real;
        uint32_t base;
      } u_humidity;
      u_humidity.base = 0;
      u_humidity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_humidity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_humidity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_humidity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->humidity = u_humidity.real;
      offset += sizeof(this->humidity);
      union {
        int32_t real;
        uint32_t base;
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
      union {
        int32_t real;
        uint32_t base;
      } u_illumination;
      u_illumination.base = 0;
      u_illumination.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_illumination.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_illumination.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_illumination.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->illumination = u_illumination.real;
      offset += sizeof(this->illumination);
      union {
        int32_t real;
        uint32_t base;
      } u_noise;
      u_noise.base = 0;
      u_noise.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_noise.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_noise.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_noise.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->noise = u_noise.real;
      offset += sizeof(this->noise);
     return offset;
    }

    const char * getType(){ return "dependant_api/all_sensor"; };
    const char * getMD5(){ return "d7926eb2af15ec27a2a8537f37c49de6"; };

  };

}
#endif