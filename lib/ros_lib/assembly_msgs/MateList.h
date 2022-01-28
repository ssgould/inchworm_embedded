#ifndef _ROS_assembly_msgs_MateList_h
#define _ROS_assembly_msgs_MateList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Quaternion.h"

namespace assembly_msgs
{

  class MateList : public ros::Msg
  {
    public:
      uint32_t female_length;
      typedef char* _female_type;
      _female_type st_female;
      _female_type * female;
      uint32_t male_length;
      typedef char* _male_type;
      _male_type st_male;
      _male_type * male;
      uint32_t linear_error_length;
      typedef float _linear_error_type;
      _linear_error_type st_linear_error;
      _linear_error_type * linear_error;
      uint32_t angular_error_length;
      typedef float _angular_error_type;
      _angular_error_type st_angular_error;
      _angular_error_type * angular_error;
      uint32_t symmetry_length;
      typedef geometry_msgs::Quaternion _symmetry_type;
      _symmetry_type st_symmetry;
      _symmetry_type * symmetry;

    MateList():
      female_length(0), st_female(), female(nullptr),
      male_length(0), st_male(), male(nullptr),
      linear_error_length(0), st_linear_error(), linear_error(nullptr),
      angular_error_length(0), st_angular_error(), angular_error(nullptr),
      symmetry_length(0), st_symmetry(), symmetry(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->female_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->female_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->female_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->female_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->female_length);
      for( uint32_t i = 0; i < female_length; i++){
      uint32_t length_femalei = strlen(this->female[i]);
      varToArr(outbuffer + offset, length_femalei);
      offset += 4;
      memcpy(outbuffer + offset, this->female[i], length_femalei);
      offset += length_femalei;
      }
      *(outbuffer + offset + 0) = (this->male_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->male_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->male_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->male_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->male_length);
      for( uint32_t i = 0; i < male_length; i++){
      uint32_t length_malei = strlen(this->male[i]);
      varToArr(outbuffer + offset, length_malei);
      offset += 4;
      memcpy(outbuffer + offset, this->male[i], length_malei);
      offset += length_malei;
      }
      *(outbuffer + offset + 0) = (this->linear_error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->linear_error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->linear_error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->linear_error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear_error_length);
      for( uint32_t i = 0; i < linear_error_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->linear_error[i]);
      }
      *(outbuffer + offset + 0) = (this->angular_error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->angular_error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->angular_error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->angular_error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular_error_length);
      for( uint32_t i = 0; i < angular_error_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->angular_error[i]);
      }
      *(outbuffer + offset + 0) = (this->symmetry_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->symmetry_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->symmetry_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->symmetry_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->symmetry_length);
      for( uint32_t i = 0; i < symmetry_length; i++){
      offset += this->symmetry[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t female_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      female_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      female_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      female_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->female_length);
      if(female_lengthT > female_length)
        this->female = (char**)realloc(this->female, female_lengthT * sizeof(char*));
      female_length = female_lengthT;
      for( uint32_t i = 0; i < female_length; i++){
      uint32_t length_st_female;
      arrToVar(length_st_female, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_female; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_female-1]=0;
      this->st_female = (char *)(inbuffer + offset-1);
      offset += length_st_female;
        memcpy( &(this->female[i]), &(this->st_female), sizeof(char*));
      }
      uint32_t male_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      male_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      male_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      male_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->male_length);
      if(male_lengthT > male_length)
        this->male = (char**)realloc(this->male, male_lengthT * sizeof(char*));
      male_length = male_lengthT;
      for( uint32_t i = 0; i < male_length; i++){
      uint32_t length_st_male;
      arrToVar(length_st_male, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_male; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_male-1]=0;
      this->st_male = (char *)(inbuffer + offset-1);
      offset += length_st_male;
        memcpy( &(this->male[i]), &(this->st_male), sizeof(char*));
      }
      uint32_t linear_error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      linear_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      linear_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      linear_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->linear_error_length);
      if(linear_error_lengthT > linear_error_length)
        this->linear_error = (float*)realloc(this->linear_error, linear_error_lengthT * sizeof(float));
      linear_error_length = linear_error_lengthT;
      for( uint32_t i = 0; i < linear_error_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_linear_error));
        memcpy( &(this->linear_error[i]), &(this->st_linear_error), sizeof(float));
      }
      uint32_t angular_error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      angular_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      angular_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      angular_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->angular_error_length);
      if(angular_error_lengthT > angular_error_length)
        this->angular_error = (float*)realloc(this->angular_error, angular_error_lengthT * sizeof(float));
      angular_error_length = angular_error_lengthT;
      for( uint32_t i = 0; i < angular_error_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_angular_error));
        memcpy( &(this->angular_error[i]), &(this->st_angular_error), sizeof(float));
      }
      uint32_t symmetry_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      symmetry_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      symmetry_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      symmetry_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->symmetry_length);
      if(symmetry_lengthT > symmetry_length)
        this->symmetry = (geometry_msgs::Quaternion*)realloc(this->symmetry, symmetry_lengthT * sizeof(geometry_msgs::Quaternion));
      symmetry_length = symmetry_lengthT;
      for( uint32_t i = 0; i < symmetry_length; i++){
      offset += this->st_symmetry.deserialize(inbuffer + offset);
        memcpy( &(this->symmetry[i]), &(this->st_symmetry), sizeof(geometry_msgs::Quaternion));
      }
     return offset;
    }

    virtual const char * getType() override { return "assembly_msgs/MateList"; };
    virtual const char * getMD5() override { return "bc42bae49a4d13f299689563ea5cbbf5"; };

  };

}
#endif
