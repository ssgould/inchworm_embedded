#ifndef _ROS_inchworm_hw_interface_PIDConsts_h
#define _ROS_inchworm_hw_interface_PIDConsts_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "inchworm_hw_interface/PID.h"

namespace inchworm_hw_interface
{

  class PIDConsts : public ros::Msg
  {
    public:
      uint32_t forward_length;
      typedef inchworm_hw_interface::PID _forward_type;
      _forward_type st_forward;
      _forward_type * forward;
      uint32_t backward_length;
      typedef inchworm_hw_interface::PID _backward_type;
      _backward_type st_backward;
      _backward_type * backward;

    PIDConsts():
      forward_length(0), st_forward(), forward(nullptr),
      backward_length(0), st_backward(), backward(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->forward_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->forward_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->forward_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->forward_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->forward_length);
      for( uint32_t i = 0; i < forward_length; i++){
      offset += this->forward[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->backward_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->backward_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->backward_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->backward_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->backward_length);
      for( uint32_t i = 0; i < backward_length; i++){
      offset += this->backward[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t forward_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      forward_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      forward_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      forward_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->forward_length);
      if(forward_lengthT > forward_length)
        this->forward = (inchworm_hw_interface::PID*)realloc(this->forward, forward_lengthT * sizeof(inchworm_hw_interface::PID));
      forward_length = forward_lengthT;
      for( uint32_t i = 0; i < forward_length; i++){
      offset += this->st_forward.deserialize(inbuffer + offset);
        memcpy( &(this->forward[i]), &(this->st_forward), sizeof(inchworm_hw_interface::PID));
      }
      uint32_t backward_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      backward_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      backward_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      backward_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->backward_length);
      if(backward_lengthT > backward_length)
        this->backward = (inchworm_hw_interface::PID*)realloc(this->backward, backward_lengthT * sizeof(inchworm_hw_interface::PID));
      backward_length = backward_lengthT;
      for( uint32_t i = 0; i < backward_length; i++){
      offset += this->st_backward.deserialize(inbuffer + offset);
        memcpy( &(this->backward[i]), &(this->st_backward), sizeof(inchworm_hw_interface::PID));
      }
     return offset;
    }

    virtual const char * getType() override { return "inchworm_hw_interface/PIDConsts"; };
    virtual const char * getMD5() override { return "83e592dff442c5cad4781f77477b90cf"; };

  };

}
#endif
