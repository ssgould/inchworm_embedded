#ifndef _ROS_inchworm_hw_interface_MagnetState_h
#define _ROS_inchworm_hw_interface_MagnetState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace inchworm_hw_interface
{

  class MagnetState : public ros::Msg
  {
    public:
      typedef bool _magnet1_type;
      _magnet1_type magnet1;
      typedef bool _magnet2_type;
      _magnet2_type magnet2;

    MagnetState():
      magnet1(0),
      magnet2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_magnet1;
      u_magnet1.real = this->magnet1;
      *(outbuffer + offset + 0) = (u_magnet1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->magnet1);
      union {
        bool real;
        uint8_t base;
      } u_magnet2;
      u_magnet2.real = this->magnet2;
      *(outbuffer + offset + 0) = (u_magnet2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->magnet2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_magnet1;
      u_magnet1.base = 0;
      u_magnet1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->magnet1 = u_magnet1.real;
      offset += sizeof(this->magnet1);
      union {
        bool real;
        uint8_t base;
      } u_magnet2;
      u_magnet2.base = 0;
      u_magnet2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->magnet2 = u_magnet2.real;
      offset += sizeof(this->magnet2);
     return offset;
    }

    virtual const char * getType() override { return "inchworm_hw_interface/MagnetState"; };
    virtual const char * getMD5() override { return "66dfe172ae943ec286c9a497c1549bb9"; };

  };

}
#endif
