#ifndef _ROS_assembly_msgs_Mate_h
#define _ROS_assembly_msgs_Mate_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace assembly_msgs
{

  class Mate : public ros::Msg
  {
    public:
      typedef const char* _description_type;
      _description_type description;
      typedef float _linear_error_type;
      _linear_error_type linear_error;
      typedef float _angular_error_type;
      _angular_error_type angular_error;

    Mate():
      description(""),
      linear_error(0),
      angular_error(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      offset += serializeAvrFloat64(outbuffer + offset, this->linear_error);
      offset += serializeAvrFloat64(outbuffer + offset, this->angular_error);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->linear_error));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angular_error));
     return offset;
    }

    virtual const char * getType() override { return "assembly_msgs/Mate"; };
    virtual const char * getMD5() override { return "2d406f8407eb6700160ebd8b30dc3a3f"; };

  };

}
#endif
