#ifndef _ROS_vacumm_hardware_WheelCmd_h
#define _ROS_vacumm_hardware_WheelCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace vacumm_hardware
{

  class WheelCmd : public ros::Msg
  {
    public:
      float vel[2];
      int32_t pos[2];

    WheelCmd():
      vel(),
      pos()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 2; i++){
      union {
        float real;
        uint32_t base;
      } u_veli;
      u_veli.real = this->vel[i];
      *(outbuffer + offset + 0) = (u_veli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_veli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_veli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_veli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_posi;
      u_posi.real = this->pos[i];
      *(outbuffer + offset + 0) = (u_posi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_posi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_posi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_posi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 2; i++){
      union {
        float real;
        uint32_t base;
      } u_veli;
      u_veli.base = 0;
      u_veli.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_veli.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_veli.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_veli.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel[i] = u_veli.real;
      offset += sizeof(this->vel[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_posi;
      u_posi.base = 0;
      u_posi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_posi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_posi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_posi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos[i] = u_posi.real;
      offset += sizeof(this->pos[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "vacumm_hardware/WheelCmd"; };
    virtual const char * getMD5() override { return "35524fa304999ef0b8123445f7d24472"; };

  };

}
#endif
