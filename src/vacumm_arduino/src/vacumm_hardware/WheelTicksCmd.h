#ifndef _ROS_vacumm_hardware_WheelTicksCmd_h
#define _ROS_vacumm_hardware_WheelTicksCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace vacumm_hardware
{

  class WheelTicksCmd : public ros::Msg
  {
    public:
      typedef int32_t _leftTick_type;
      _leftTick_type leftTick;
      typedef int32_t _rightTick_type;
      _rightTick_type rightTick;

    WheelTicksCmd():
      leftTick(0),
      rightTick(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_leftTick;
      u_leftTick.real = this->leftTick;
      *(outbuffer + offset + 0) = (u_leftTick.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftTick.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leftTick.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leftTick.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->leftTick);
      union {
        int32_t real;
        uint32_t base;
      } u_rightTick;
      u_rightTick.real = this->rightTick;
      *(outbuffer + offset + 0) = (u_rightTick.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightTick.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rightTick.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rightTick.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rightTick);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_leftTick;
      u_leftTick.base = 0;
      u_leftTick.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftTick.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leftTick.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leftTick.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->leftTick = u_leftTick.real;
      offset += sizeof(this->leftTick);
      union {
        int32_t real;
        uint32_t base;
      } u_rightTick;
      u_rightTick.base = 0;
      u_rightTick.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightTick.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rightTick.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rightTick.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rightTick = u_rightTick.real;
      offset += sizeof(this->rightTick);
     return offset;
    }

    virtual const char * getType() override { return "vacumm_hardware/WheelTicksCmd"; };
    virtual const char * getMD5() override { return "73ab5b178d9496d0426ad919577467f8"; };

  };

}
#endif
