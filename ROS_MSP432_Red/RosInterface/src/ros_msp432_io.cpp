
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
 * Serial interface command and interface protocol.  Commands and returned status
 * across a serial interface are tokenized using 2 character ASCII.  The command/start
 * strings are delimited by "<" less than and ">" greater than characters.
 * */
#include <src/ros_msp432_io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ti/devices/msp432p4xx/driverlib/driverlib.h"

using namespace cmdStatus;

// Todo: This is the command handler example modify as you please to match you command and response set
RosMSP432_Io::RosMSP432_Io(void)
{
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
}
bool RosMSP432_Io::handleCommand(unsigned command, ros::Publisher pub, std_msgs::String txMsg)
{
    char  charParam[32];
    int   intParam;
    float floatParam;
    switch(command)
    {
        case RED_LED_TOGGLE_CMD:
            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);               // Toggle the pint
            intParam = (int)GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN0); // Get the pin state
            sprintf(sprintfBuffer, "%s%s,%d%s", START_DELIMITER, RED_LED_TOGGLE_MSG_STR, intParam, END_DELIMITER_NL);
            txMsg.data = sprintfBuffer; // Acknowledge with pin state response
            pub.publish(&txMsg);
            break;
        case RED_LED_ON_CMD:
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
            break;
        case RED_LED_OFF_CMD:
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
            break;
        case SET_STRING_CMD:
            if(getStringParameter(sizeof(charParam), charParam))
            {
                printf("String = %s\n", charParam);
                setCharStr(strlen(charParam), charParam);
            }
            txMsg.data = ACKNOWLEDGE_STR; // Use acknowledge as a response
            pub.publish(&txMsg);
            break;
        case GET_STRING_CMD:
            getCharStr(sizeof(charParam), charParam);
            sprintf(sprintfBuffer, "%s%s,%sX%s", START_DELIMITER, STRING_TYPE_MESSAGE_STR, charParam, END_DELIMITER_NL);
            txMsg.data = sprintfBuffer;
            pub.publish(&txMsg);
            break;
        case SET_INTEGER_CMD:
            if(getIntParameter(&intParam))
            {
                printf("Integer = %d\n", intParam);
                intParam++;
                setInteger(intParam);
            }
            txMsg.data = ACKNOWLEDGE_STR; // Use acknowledge as a response
            pub.publish(&txMsg);
            break;
        case GET_INTEGER_CMD:
            intParam = getInteger();
            sprintf(sprintfBuffer, "%s%s,%d%s", START_DELIMITER, INTEGER_TYPE_MESSAGE_STR, intParam, END_DELIMITER_NL);
            txMsg.data = sprintfBuffer;
            pub.publish(&txMsg);
            break;
        case SET_FLOAT_CMD:
            if(getFloatParameter(&floatParam))
            {
                printf("Float = %f\n", floatParam);
                floatParam+= 1.0;
                setFloat(floatParam);
            }
            txMsg.data = ACKNOWLEDGE_STR; // Use acknowledge as a response
            pub.publish(&txMsg);
            break;
        case GET_FLOAT_CMD:
            floatParam = getFloat();
            sprintf(sprintfBuffer, "%s%s,%f%s", START_DELIMITER, FLOAT_TYPE_MESSAGE_STR, floatParam, END_DELIMITER_NL);
            txMsg.data = sprintfBuffer;
            pub.publish(&txMsg);
            break;
        case QUIT_CMD:
            sprintf(sprintfBuffer, "%s%s%s", START_DELIMITER, QUIT_CMD_STR, END_DELIMITER_NL);
            txMsg.data = sprintfBuffer;
            pub.publish(&txMsg);
            break;
        default:
            return false;
    }
    return true;

}
// This function scan's the message for the start and end delimiters
unsigned RosMSP432_Io::parseMessage(void)
{
    char  rxChar;        // Temporary char storage
    char *tempPtr;
    unsigned command = NO_COMMAND;
    while((rxChar = *messageStrPtr++) == '<') // Start delimiter found ?
    {
        // Now we can parse the buffer
        tempPtr = messageStrPtr;
        while(true)
        {
            rxChar = *messageStrPtr++;
            if(rxChar == '<')
            {   // Another start delimiter ?
                messageError = MESSAGE_PARSE_DELIMIT;
             }
            if(rxChar=='>')
            {   // Command complete delimiter found ?
                command = *tempPtr++;
                command<<= 8;
                command|= *tempPtr++;
                if(*tempPtr != rxChar)
                    messageStrPtr = tempPtr; // Might have a parameter string
                return command;
            }
        } // End while loop
    }
    return command;
}
bool RosMSP432_Io::getIntParameter(int *parameter)
{
    if(*messageStrPtr == 0) // Do we have a parameter ?
        return false;       // No end of data
    if(*messageStrPtr++ != ',') // Do we have a comma ?
        return false;           // No, then no parameter
    char *tempPtr;
    tempPtr = strchr(messageStrPtr, '>'); // Do we have the end delimiter
    if(tempPtr == NULL) // No delimiter return
        return false;
    *tempPtr++= 0; // Terminate string
    *parameter = atoi(messageStrPtr);
    messageStrPtr = tempPtr;
    return true;
}
bool RosMSP432_Io::getFloatParameter(float *parameter)
{
    if(*messageStrPtr == 0) // Do we have a parameter ?
        return false;       // No end of data
    if(*messageStrPtr++ != ',') // Do we have a comma ?
        return false;           // No, then no parameter
    char *tempPtr;
    tempPtr = strchr(messageStrPtr, '>'); // Do we have the end delimiter
    if(tempPtr == NULL) // No delimiter return
        return false;
    *tempPtr++= 0; // Terminate string
    *parameter = atof(messageStrPtr);
    messageStrPtr = tempPtr;
    return true;
}
bool RosMSP432_Io::getStringParameter(unsigned n, char *parameter)
{
    if(*messageStrPtr == 0) // Do we have a parameter ?
        return false;       // No end of data
    if(*messageStrPtr++ != ',') // Do we have a comma ?
        return false;           // No, then no parameter
    char *tempPtr;
    tempPtr = strchr(messageStrPtr, '>'); // Do we have the end delimiter
    if(tempPtr == NULL) // No delimiter return
        return false;
    *tempPtr++= 0; // Terminate string
    strncpy(parameter, messageStrPtr, n);
    messageStrPtr = tempPtr;
    return true;
}
int RosMSP432_Io::setCharStr(int size, char *charStr)
{
    if(size > (CHAR_STR_SIZE - 1))
        size = CHAR_STR_SIZE - 1;
    strncpy(this->charStr, charStr, size);
    this->charStr[size] = 0; // Terminate
    return size;
}
int RosMSP432_Io::getCharStr(int size, char *charStr)
{
    int length = strlen(this->charStr);
    if(length > size)
        length = size;
    strncpy(charStr, this->charStr, length);
    charStr[length] = 0; // Terminate
    return length;
}




