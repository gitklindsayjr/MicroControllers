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
 * Ros subscriber publisher that test application that interfaces with a Qt node

 */
#include <ros.h>
#include <src/ros_msp432_io.h>
#include <std_msgs/String.h>
#include <stdio.h>

ros::NodeHandle  nh_; // Needs to be declared external, no room on the stack
std_msgs::String rxMsg;
std_msgs::String txMsg;
bool msgReceived = false;

// Declare our interface object
RosMSP432_Io msp;

// Callback for our subscriber
void messageCb(const std_msgs::String &msg)
{
     if(!msgReceived) // Process one message at a time
         rxMsg = msg; // don't save others
     msgReceived = true;
}

void main()
{
    unsigned command;
    nh_.initNode();

    // Create our publisher object to send messages to Qt's subscriber
    ros::Publisher serialIo_pub("msp432_pub", &txMsg);
    nh_.advertise(serialIo_pub);
    // Create our subscriber to handle msp432 messages from Qt's publisher
    ros::Subscriber<std_msgs::String> serialIo_sub("qt_pub", &messageCb);
    nh_.subscribe(serialIo_sub);

    while(true)
    {
        nh_.spinOnce();
        if(msgReceived)
        {
            msp.messageStrPtr = (char *)rxMsg.data; // Let interface point to our string data
            do
            {   // Do multiple times we may have more then one command
                command = msp.parseMessage();
                msp.handleCommand(command, serialIo_pub, txMsg);
            }while(command != NO_COMMAND);
            msgReceived = false;
        }
        // Todo: serialIo_pub.pulish(&txMsg); // You could send your own stuff here
        nh_.getHardware()->delay(100);
    }
}
