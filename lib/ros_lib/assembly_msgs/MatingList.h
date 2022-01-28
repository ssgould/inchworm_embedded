#ifndef _ROS_assembly_msgs_MatingList_h
#define _ROS_assembly_msgs_MatingList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "assembly_msgs/Mate.h"

namespace assembly_msgs
{

  class MatingList : public ros::Msg
  {
    public:
      uint32_t mates_length;
      typedef assembly_msgs::Mate _mates_type;
      _mates_type st_mates;
      _mates_type * mates;

    MatingList():
      mates_length(0), st_mates(), mates(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->mates_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mates_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mates_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mates_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mates_length);
      for( uint32_t i = 0; i < mates_length; i++){
      offset += this->mates[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t mates_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      mates_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      mates_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      mates_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->mates_length);
      if(mates_lengthT > mates_length)
        this->mates = (assembly_msgs::Mate*)realloc(this->mates, mates_lengthT * sizeof(assembly_msgs::Mate));
      mates_length = mates_lengthT;
      for( uint32_t i = 0; i < mates_length; i++){
      offset += this->st_mates.deserialize(inbuffer + offset);
        memcpy( &(this->mates[i]), &(this->st_mates), sizeof(assembly_msgs::Mate));
      }
     return offset;
    }

    virtual const char * getType() override { return "assembly_msgs/MatingList"; };
    virtual const char * getMD5() override { return "da1261a8d505f219dcc958c1ea29635f"; };

  };

}
#endif
