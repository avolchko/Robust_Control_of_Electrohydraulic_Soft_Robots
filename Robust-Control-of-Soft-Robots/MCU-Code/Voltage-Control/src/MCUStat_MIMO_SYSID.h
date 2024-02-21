#ifndef _ROS_sttr_phase2_MCUStat_MIMO_SYSID_h
#define _ROS_sttr_phase2_MCUStat_MIMO_SYSID_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sttr_phase2
{

  class MCUStat_MIMO_SYSID : public ros::Msg
  {
    public:
      typedef int16_t _chg1_type;
      _chg1_type chg1;
      typedef int16_t _chg2_type;
      _chg2_type chg2;
      typedef int16_t _dis1_type;
      _dis1_type dis1;
      typedef int16_t _dis2_type;
      _dis2_type dis2;
      typedef int16_t _volt1_type;
      _volt1_type volt1;
      typedef int16_t _volt2_type;
      _volt2_type volt2;
      typedef bool _pol1_type;
      _pol1_type pol1;
      typedef bool _pol2_type;
      _pol2_type pol2;
      typedef int16_t _hv_type;
      _hv_type hv;
      typedef int16_t _cap1_type;
      _cap1_type cap1;
      typedef int16_t _cap2_type;
      _cap2_type cap2;

    MCUStat_MIMO_SYSID():
      chg1(0),
      chg2(0),
      dis1(0),
      dis2(0),
      volt1(0),
      volt2(0),
      pol1(0),
      pol2(0),
      hv(0),
      cap1(0),
      cap2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_chg1;
      u_chg1.real = this->chg1;
      *(outbuffer + offset + 0) = (u_chg1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_chg1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->chg1);
      union {
        int16_t real;
        uint16_t base;
      } u_chg2;
      u_chg2.real = this->chg2;
      *(outbuffer + offset + 0) = (u_chg2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_chg2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->chg2);
      union {
        int16_t real;
        uint16_t base;
      } u_dis1;
      u_dis1.real = this->dis1;
      *(outbuffer + offset + 0) = (u_dis1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dis1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dis1);
      union {
        int16_t real;
        uint16_t base;
      } u_dis2;
      u_dis2.real = this->dis2;
      *(outbuffer + offset + 0) = (u_dis2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dis2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dis2);
      union {
        int16_t real;
        uint16_t base;
      } u_volt1;
      u_volt1.real = this->volt1;
      *(outbuffer + offset + 0) = (u_volt1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_volt1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->volt1);
      union {
        int16_t real;
        uint16_t base;
      } u_volt2;
      u_volt2.real = this->volt2;
      *(outbuffer + offset + 0) = (u_volt2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_volt2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->volt2);
      union {
        bool real;
        uint8_t base;
      } u_pol1;
      u_pol1.real = this->pol1;
      *(outbuffer + offset + 0) = (u_pol1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pol1);
      union {
        bool real;
        uint8_t base;
      } u_pol2;
      u_pol2.real = this->pol2;
      *(outbuffer + offset + 0) = (u_pol2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pol2);
      union {
        int16_t real;
        uint16_t base;
      } u_hv;
      u_hv.real = this->hv;
      *(outbuffer + offset + 0) = (u_hv.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hv.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->hv);
      union {
        int16_t real;
        uint16_t base;
      } u_cap1;
      u_cap1.real = this->cap1;
      *(outbuffer + offset + 0) = (u_cap1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cap1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cap1);
      union {
        int16_t real;
        uint16_t base;
      } u_cap2;
      u_cap2.real = this->cap2;
      *(outbuffer + offset + 0) = (u_cap2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cap2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cap2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_chg1;
      u_chg1.base = 0;
      u_chg1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_chg1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->chg1 = u_chg1.real;
      offset += sizeof(this->chg1);
      union {
        int16_t real;
        uint16_t base;
      } u_chg2;
      u_chg2.base = 0;
      u_chg2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_chg2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->chg2 = u_chg2.real;
      offset += sizeof(this->chg2);
      union {
        int16_t real;
        uint16_t base;
      } u_dis1;
      u_dis1.base = 0;
      u_dis1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dis1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dis1 = u_dis1.real;
      offset += sizeof(this->dis1);
      union {
        int16_t real;
        uint16_t base;
      } u_dis2;
      u_dis2.base = 0;
      u_dis2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dis2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dis2 = u_dis2.real;
      offset += sizeof(this->dis2);
      union {
        int16_t real;
        uint16_t base;
      } u_volt1;
      u_volt1.base = 0;
      u_volt1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_volt1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->volt1 = u_volt1.real;
      offset += sizeof(this->volt1);
      union {
        int16_t real;
        uint16_t base;
      } u_volt2;
      u_volt2.base = 0;
      u_volt2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_volt2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->volt2 = u_volt2.real;
      offset += sizeof(this->volt2);
      union {
        bool real;
        uint8_t base;
      } u_pol1;
      u_pol1.base = 0;
      u_pol1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pol1 = u_pol1.real;
      offset += sizeof(this->pol1);
      union {
        bool real;
        uint8_t base;
      } u_pol2;
      u_pol2.base = 0;
      u_pol2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pol2 = u_pol2.real;
      offset += sizeof(this->pol2);
      union {
        int16_t real;
        uint16_t base;
      } u_hv;
      u_hv.base = 0;
      u_hv.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hv.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->hv = u_hv.real;
      offset += sizeof(this->hv);
      union {
        int16_t real;
        uint16_t base;
      } u_cap1;
      u_cap1.base = 0;
      u_cap1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cap1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cap1 = u_cap1.real;
      offset += sizeof(this->cap1);
      union {
        int16_t real;
        uint16_t base;
      } u_cap2;
      u_cap2.base = 0;
      u_cap2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cap2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cap2 = u_cap2.real;
      offset += sizeof(this->cap2);
     return offset;
    }

    virtual const char * getType() override { return "sttr_phase2/MCUStat_MIMO_SYSID"; };
    virtual const char * getMD5() override { return "ce1f3ff76df94a7d15db59ffaf2f3adf"; };

  };

}
#endif
