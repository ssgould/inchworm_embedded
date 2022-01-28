#ifndef _ROS_SERVICE_SetMateSuppression_h
#define _ROS_SERVICE_SetMateSuppression_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace assembly_msgs
{

static const char SETMATESUPPRESSION[] = "assembly_msgs/SetMateSuppression";

  class SetMateSuppressionRequest : public ros::Msg
  {
    public:
      uint32_t scoped_link_length;
      typedef char* _scoped_link_type;
      _scoped_link_type st_scoped_link;
      _scoped_link_type * scoped_link;
      typedef bool _suppress_type;
      _suppress_type suppress;

    SetMateSuppressionRequest():
      scoped_link_length(0), st_scoped_link(), scoped_link(nullptr),
      suppress(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->scoped_link_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->scoped_link_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->scoped_link_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->scoped_link_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scoped_link_length);
      for( uint32_t i = 0; i < scoped_link_length; i++){
      uint32_t length_scoped_linki = strlen(this->scoped_link[i]);
      varToArr(outbuffer + offset, length_scoped_linki);
      offset += 4;
      memcpy(outbuffer + offset, this->scoped_link[i], length_scoped_linki);
      offset += length_scoped_linki;
      }
      union {
        bool real;
        uint8_t base;
      } u_suppress;
      u_suppress.real = this->suppress;
      *(outbuffer + offset + 0) = (u_suppress.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->suppress);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t scoped_link_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      scoped_link_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      scoped_link_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      scoped_link_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->scoped_link_length);
      if(scoped_link_lengthT > scoped_link_length)
        this->scoped_link = (char**)realloc(this->scoped_link, scoped_link_lengthT * sizeof(char*));
      scoped_link_length = scoped_link_lengthT;
      for( uint32_t i = 0; i < scoped_link_length; i++){
      uint32_t length_st_scoped_link;
      arrToVar(length_st_scoped_link, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_scoped_link; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_scoped_link-1]=0;
      this->st_scoped_link = (char *)(inbuffer + offset-1);
      offset += length_st_scoped_link;
        memcpy( &(this->scoped_link[i]), &(this->st_scoped_link), sizeof(char*));
      }
      union {
        bool real;
        uint8_t base;
      } u_suppress;
      u_suppress.base = 0;
      u_suppress.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->suppress = u_suppress.real;
      offset += sizeof(this->suppress);
     return offset;
    }

    virtual const char * getType() override { return SETMATESUPPRESSION; };
    virtual const char * getMD5() override { return "5dfd80d5f4bd3cdcf9e33005fc163d40"; };

  };

  class SetMateSuppressionResponse : public ros::Msg
  {
    public:
      typedef bool _suppressed_type;
      _suppressed_type suppressed;

    SetMateSuppressionResponse():
      suppressed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_suppressed;
      u_suppressed.real = this->suppressed;
      *(outbuffer + offset + 0) = (u_suppressed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->suppressed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_suppressed;
      u_suppressed.base = 0;
      u_suppressed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->suppressed = u_suppressed.real;
      offset += sizeof(this->suppressed);
     return offset;
    }

    virtual const char * getType() override { return SETMATESUPPRESSION; };
    virtual const char * getMD5() override { return "6219af3a562749deb64ee5a012a2bbfa"; };

  };

  class SetMateSuppression {
    public:
    typedef SetMateSuppressionRequest Request;
    typedef SetMateSuppressionResponse Response;
  };

}
#endif
