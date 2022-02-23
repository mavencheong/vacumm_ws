#ifndef _ROS_vacumm_hardware_VacummDiag_h
#define _ROS_vacumm_hardware_VacummDiag_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace vacumm_hardware
{

  class VacummDiag : public ros::Msg
  {
    public:
      typedef double _left_setpoint_type;
      _left_setpoint_type left_setpoint;
      typedef double _right_setpoint_type;
      _right_setpoint_type right_setpoint;
      typedef double _left_input_type;
      _left_input_type left_input;
      typedef double _right_input_type;
      _right_input_type right_input;
      typedef double _left_output_type;
      _left_output_type left_output;
      typedef double _right_output_type;
      _right_output_type right_output;
      typedef double _left_pulse_type;
      _left_pulse_type left_pulse;
      typedef double _right_pulse_type;
      _right_pulse_type right_pulse;

    VacummDiag():
      left_setpoint(0),
      right_setpoint(0),
      left_input(0),
      right_input(0),
      left_output(0),
      right_output(0),
      left_pulse(0),
      right_pulse(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_left_setpoint;
      u_left_setpoint.real = this->left_setpoint;
      *(outbuffer + offset + 0) = (u_left_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_setpoint.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_left_setpoint.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_left_setpoint.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_left_setpoint.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_left_setpoint.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->left_setpoint);
      union {
        double real;
        uint64_t base;
      } u_right_setpoint;
      u_right_setpoint.real = this->right_setpoint;
      *(outbuffer + offset + 0) = (u_right_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_setpoint.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_right_setpoint.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_right_setpoint.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_right_setpoint.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_right_setpoint.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->right_setpoint);
      union {
        double real;
        uint64_t base;
      } u_left_input;
      u_left_input.real = this->left_input;
      *(outbuffer + offset + 0) = (u_left_input.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_input.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_input.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_input.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_left_input.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_left_input.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_left_input.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_left_input.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->left_input);
      union {
        double real;
        uint64_t base;
      } u_right_input;
      u_right_input.real = this->right_input;
      *(outbuffer + offset + 0) = (u_right_input.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_input.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_input.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_input.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_right_input.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_right_input.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_right_input.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_right_input.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->right_input);
      union {
        double real;
        uint64_t base;
      } u_left_output;
      u_left_output.real = this->left_output;
      *(outbuffer + offset + 0) = (u_left_output.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_output.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_output.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_output.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_left_output.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_left_output.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_left_output.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_left_output.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->left_output);
      union {
        double real;
        uint64_t base;
      } u_right_output;
      u_right_output.real = this->right_output;
      *(outbuffer + offset + 0) = (u_right_output.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_output.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_output.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_output.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_right_output.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_right_output.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_right_output.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_right_output.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->right_output);
      union {
        double real;
        uint64_t base;
      } u_left_pulse;
      u_left_pulse.real = this->left_pulse;
      *(outbuffer + offset + 0) = (u_left_pulse.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_pulse.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_pulse.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_pulse.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_left_pulse.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_left_pulse.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_left_pulse.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_left_pulse.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->left_pulse);
      union {
        double real;
        uint64_t base;
      } u_right_pulse;
      u_right_pulse.real = this->right_pulse;
      *(outbuffer + offset + 0) = (u_right_pulse.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_pulse.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_pulse.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_pulse.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_right_pulse.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_right_pulse.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_right_pulse.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_right_pulse.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->right_pulse);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_left_setpoint;
      u_left_setpoint.base = 0;
      u_left_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_left_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_left_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_left_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_left_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->left_setpoint = u_left_setpoint.real;
      offset += sizeof(this->left_setpoint);
      union {
        double real;
        uint64_t base;
      } u_right_setpoint;
      u_right_setpoint.base = 0;
      u_right_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_right_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_right_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_right_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_right_setpoint.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->right_setpoint = u_right_setpoint.real;
      offset += sizeof(this->right_setpoint);
      union {
        double real;
        uint64_t base;
      } u_left_input;
      u_left_input.base = 0;
      u_left_input.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_input.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_input.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_input.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_left_input.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_left_input.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_left_input.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_left_input.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->left_input = u_left_input.real;
      offset += sizeof(this->left_input);
      union {
        double real;
        uint64_t base;
      } u_right_input;
      u_right_input.base = 0;
      u_right_input.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_input.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_input.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_input.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_right_input.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_right_input.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_right_input.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_right_input.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->right_input = u_right_input.real;
      offset += sizeof(this->right_input);
      union {
        double real;
        uint64_t base;
      } u_left_output;
      u_left_output.base = 0;
      u_left_output.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_output.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_output.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_output.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_left_output.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_left_output.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_left_output.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_left_output.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->left_output = u_left_output.real;
      offset += sizeof(this->left_output);
      union {
        double real;
        uint64_t base;
      } u_right_output;
      u_right_output.base = 0;
      u_right_output.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_output.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_output.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_output.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_right_output.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_right_output.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_right_output.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_right_output.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->right_output = u_right_output.real;
      offset += sizeof(this->right_output);
      union {
        double real;
        uint64_t base;
      } u_left_pulse;
      u_left_pulse.base = 0;
      u_left_pulse.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_pulse.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_pulse.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_pulse.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_left_pulse.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_left_pulse.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_left_pulse.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_left_pulse.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->left_pulse = u_left_pulse.real;
      offset += sizeof(this->left_pulse);
      union {
        double real;
        uint64_t base;
      } u_right_pulse;
      u_right_pulse.base = 0;
      u_right_pulse.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_pulse.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_pulse.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_pulse.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_right_pulse.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_right_pulse.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_right_pulse.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_right_pulse.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->right_pulse = u_right_pulse.real;
      offset += sizeof(this->right_pulse);
     return offset;
    }

    virtual const char * getType() override { return "vacumm_hardware/VacummDiag"; };
    virtual const char * getMD5() override { return "aadd4daafd910044057f442f70383aa6"; };

  };

}
#endif
