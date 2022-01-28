#ifndef _ROS_inchworm_hw_interface_PID_h
#define _ROS_inchworm_hw_interface_PID_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace inchworm_hw_interface
{

  class PID : public ros::Msg
  {
    public:
      typedef float _p_type;
      _p_type p;
      typedef float _i_type;
      _i_type i;
      typedef float _d_type;
      _d_type d;

    PID():
      p(0),
      i(0),
      d(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->p);
      offset += serializeAvrFloat64(outbuffer + offset, this->i);
      offset += serializeAvrFloat64(outbuffer + offset, this->d);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->p));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->i));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->d));
     return offset;
    }

    virtual const char * getType() override { return "inchworm_hw_interface/PID"; };
    virtual const char * getMD5() override { return "4d7f5db5580abe953fdf6985b0b4717c"; };

  };

}
#endif
