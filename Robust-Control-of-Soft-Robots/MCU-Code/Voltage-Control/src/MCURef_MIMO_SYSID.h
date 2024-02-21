#ifndef _ROS_sttr_phase2_MCURef_MIMO_SYSID_h
#define _ROS_sttr_phase2_MCURef_MIMO_SYSID_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sttr_phase2
{

  class MCURef_MIMO_SYSID : public ros::Msg
  {
    public:
      typedef int16_t _ch1_ref_type;
      _ch1_ref_type ch1_ref;
      typedef int16_t _ch2_ref_type;
      _ch2_ref_type ch2_ref;
      typedef bool _kill_ref_type;
      _kill_ref_type kill_ref;
      typedef float _KP_type;
      _KP_type KP;
      typedef float _KI_type;
      _KI_type KI;
      typedef float _KD_type;
      _KD_type KD;

    MCURef_MIMO_SYSID():
      ch1_ref(0),
      ch2_ref(0),
      kill_ref(0),
      KP(0),
      KI(0),
      KD(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_ch1_ref;
      u_ch1_ref.real = this->ch1_ref;
      *(outbuffer + offset + 0) = (u_ch1_ref.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ch1_ref.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ch1_ref);
      union {
        int16_t real;
        uint16_t base;
      } u_ch2_ref;
      u_ch2_ref.real = this->ch2_ref;
      *(outbuffer + offset + 0) = (u_ch2_ref.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ch2_ref.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ch2_ref);
      union {
        bool real;
        uint8_t base;
      } u_kill_ref;
      u_kill_ref.real = this->kill_ref;
      *(outbuffer + offset + 0) = (u_kill_ref.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->kill_ref);
      union {
        float real;
        uint32_t base;
      } u_KP;
      u_KP.real = this->KP;
      *(outbuffer + offset + 0) = (u_KP.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_KP.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_KP.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_KP.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->KP);
      union {
        float real;
        uint32_t base;
      } u_KI;
      u_KI.real = this->KI;
      *(outbuffer + offset + 0) = (u_KI.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_KI.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_KI.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_KI.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->KI);
      union {
        float real;
        uint32_t base;
      } u_KD;
      u_KD.real = this->KD;
      *(outbuffer + offset + 0) = (u_KD.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_KD.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_KD.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_KD.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->KD);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_ch1_ref;
      u_ch1_ref.base = 0;
      u_ch1_ref.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ch1_ref.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ch1_ref = u_ch1_ref.real;
      offset += sizeof(this->ch1_ref);
      union {
        int16_t real;
        uint16_t base;
      } u_ch2_ref;
      u_ch2_ref.base = 0;
      u_ch2_ref.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ch2_ref.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ch2_ref = u_ch2_ref.real;
      offset += sizeof(this->ch2_ref);
      union {
        bool real;
        uint8_t base;
      } u_kill_ref;
      u_kill_ref.base = 0;
      u_kill_ref.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->kill_ref = u_kill_ref.real;
      offset += sizeof(this->kill_ref);
      union {
        float real;
        uint32_t base;
      } u_KP;
      u_KP.base = 0;
      u_KP.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_KP.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_KP.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_KP.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->KP = u_KP.real;
      offset += sizeof(this->KP);
      union {
        float real;
        uint32_t base;
      } u_KI;
      u_KI.base = 0;
      u_KI.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_KI.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_KI.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_KI.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->KI = u_KI.real;
      offset += sizeof(this->KI);
      union {
        float real;
        uint32_t base;
      } u_KD;
      u_KD.base = 0;
      u_KD.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_KD.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_KD.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_KD.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->KD = u_KD.real;
      offset += sizeof(this->KD);
     return offset;
    }

    virtual const char * getType() override { return "sttr_phase2/MCURef_MIMO_SYSID"; };
    virtual const char * getMD5() override { return "827e5e68b5ed5c98ca2ad185e28db3ba"; };

  };

}
#endif
