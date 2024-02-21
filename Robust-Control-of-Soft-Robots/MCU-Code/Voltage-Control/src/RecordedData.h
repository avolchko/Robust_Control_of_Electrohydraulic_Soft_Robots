#ifndef _ROS_sttr_phase2_RecordedData_h
#define _ROS_sttr_phase2_RecordedData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sttr_phase2
{

  class RecordedData : public ros::Msg
  {
    public:
      typedef uint16_t _volt_type;
      _volt_type volt;
      typedef uint16_t _voltref_type;
      _voltref_type voltref;
      typedef uint16_t _charge_type;
      _charge_type charge;
      typedef uint16_t _discharge_type;
      _discharge_type discharge;
      typedef uint32_t _timestamp_type;
      _timestamp_type timestamp;

    RecordedData():
      volt(0),
      voltref(0),
      charge(0),
      discharge(0),
      timestamp(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->volt >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->volt >> (8 * 1)) & 0xFF;
      offset += sizeof(this->volt);
      *(outbuffer + offset + 0) = (this->voltref >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->voltref >> (8 * 1)) & 0xFF;
      offset += sizeof(this->voltref);
      *(outbuffer + offset + 0) = (this->charge >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->charge >> (8 * 1)) & 0xFF;
      offset += sizeof(this->charge);
      *(outbuffer + offset + 0) = (this->discharge >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->discharge >> (8 * 1)) & 0xFF;
      offset += sizeof(this->discharge);
      *(outbuffer + offset + 0) = (this->timestamp >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->volt =  ((uint16_t) (*(inbuffer + offset)));
      this->volt |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->volt);
      this->voltref =  ((uint16_t) (*(inbuffer + offset)));
      this->voltref |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->voltref);
      this->charge =  ((uint16_t) (*(inbuffer + offset)));
      this->charge |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->charge);
      this->discharge =  ((uint16_t) (*(inbuffer + offset)));
      this->discharge |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->discharge);
      this->timestamp =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp);
     return offset;
    }

    virtual const char * getType() override { return "sttr_phase2/RecordedData"; };
    virtual const char * getMD5() override { return "4f9c60c33d639263a5bbc2ff6daf0096"; };

  };

}
#endif
