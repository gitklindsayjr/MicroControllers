/****************************************************************************
 *  Copyright (C) 2017 Ken Lindsay
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************
 *  DESCRIPTION
 * Serial interface command and interface protocall.  Commands and returned status
 * across a serial interface are tokenized using 2 character ASCII.  The command/start
 * strings are delimited by "<" less than and ">" greater than characters.
*/

#ifndef ROS_MSP432_IO_H_
#define ROS_MSP432_IO_H_

#include <inttypes.h>
#include <ros.h>
#include <std_msgs/String.h>
#include "cmd_status.h"

#define CHAR_STR_SIZE    256
class RosMSP432_Io
{
    public:
        RosMSP432_Io(void);
        bool handleCommand(unsigned command, ros::Publisher pub, std_msgs::String txMsg);
        unsigned parseMessage(void);
        bool     getIntParameter(int *parameter);
        bool     getFloatParameter(float *parameter);
        bool     getStringParameter(unsigned size, char *parameter);
        int      getError(void) { return messageError; }
        int      getInteger(void) { return integerValue; }
        int      getFloat(void)   { return floatValue; }
        void     toggleLed(void);
        void     setInteger(int integerValue) { this->integerValue = integerValue; }
        void     setFloat(int floatValue) { this->floatValue = floatValue; }
        int      setCharStr(int size, char *charStr);
        int      getCharStr(int size, char *charStr);
     private:
         int   messageError;
         int   integerValue;
         float floatValue;
         char  sprintfBuffer[256];
         char  charStr[CHAR_STR_SIZE];
     public:
         char *messageStrPtr;
     private:
};
#endif
