#ifndef _ROS_rosserial_tivac_tutorials_Buttons_h
#define _ROS_rosserial_tivac_tutorials_Buttons_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Bool.h"

namespace rosserial_tivac_tutorials
{

  class Buttons : public ros::Msg
  {
    public:
      typedef std_msgs::Bool _sw1_type;
      _sw1_type sw1;
      typedef std_msgs::Bool _sw2_type;
      _sw2_type sw2;

    Buttons():
      sw1(),
      sw2()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->sw1.serialize(outbuffer + offset);
      offset += this->sw2.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->sw1.deserialize(inbuffer + offset);
      offset += this->sw2.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "rosserial_tivac_tutorials/Buttons"; };
    const char * getMD5(){ return "a78ccaade8fa723d1ebeb7b099042085"; };

  };

}
#endif
