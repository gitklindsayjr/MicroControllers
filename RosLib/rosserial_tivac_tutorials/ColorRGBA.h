#ifndef _ROS_SERVICE_ColorRGBA_h
#define _ROS_SERVICE_ColorRGBA_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/ColorRGBA.h"

namespace rosserial_tivac_tutorials
{

static const char COLORRGBA[] = "rosserial_tivac_tutorials/ColorRGBA";

  class ColorRGBARequest : public ros::Msg
  {
    public:
      typedef std_msgs::ColorRGBA _color_type;
      _color_type color;

    ColorRGBARequest():
      color()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->color.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->color.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return COLORRGBA; };
    const char * getMD5(){ return "3e04b62b1b39cd97e873789f0bb130e7"; };

  };

  class ColorRGBAResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    ColorRGBAResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    const char * getType(){ return COLORRGBA; };
    const char * getMD5(){ return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class ColorRGBA {
    public:
    typedef ColorRGBARequest Request;
    typedef ColorRGBAResponse Response;
  };

}
#endif
