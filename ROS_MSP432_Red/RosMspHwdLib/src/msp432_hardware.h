/*
 * Copyright (c) 2017, Kenneth Lindsay
 * All rights reserved.
 * Author: Kenneth Lindsay
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *      following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *      following disclaimer in the documentation and/or other materials provided with the distribution.
 *   3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *      products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_MSP432_HARDWARE_LIB_H
#define ROS_MSP432_HARDWARE_LIB_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
extern "C"
{
	#include "clocks.h"
	#include "my_uart.h"
}

#define SYSTICK_HZ  1000 // Tick rate 100 Hz
#define SYSTICK_MS  1    // Convert to ms per tick cycle (1000ms/SYSTICK_HZ)

class Msp432CHardware
{
  public:
    Msp432CHardware() {}

    void init()
    {
    	/* Initializes EUSCI_A0 UART on USB/Debug port */
    	initializeUart(true); // Initializes clocks also
    }
    // read a byte from the serial port. return -1 for failure
    int read() { return _rxChar(); }

    // write data to the ROS connection
    void write(uint8_t* data, int length)
    {
     	for(int i = 0; i<length; i++)
     	    _txChar(data[i]);
    }
    // returns milliseconds since start of program
    uint32_t time(){ return SYSTICK_MS*tickCount; }
    // Hardware delay uses Timer32 interrupts
    void delay(uint32_t ms){ mySleepMs(ms); }
};
#endif  // ROS_MSP432_HARDWARE_LIB_H
