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

#ifndef MY_CLOCKS_H_
#define MY_CLOCKS_H_

#include "stdint.h"

#define CLOCK_FREQ              12000000     // Clock frequency 12 Mhz
#define SYS_TICK_FREQ_FREQ      1000         // System tick frequency Hz
#define SYS_TICK_PERIOD_MS      1            // System tick period milli-seconds
#define HEART_BEAT_COUNT        1000         // 1000ms/SYS_TICK_PERIOD_MS
#define SYS_TICK                12000        // Clock Divider, 12 Mhz/SYS_TICK_FREQ_FREQ
#define SLEEP_COUNT             46875        // Seconds to counts, 12000000/256
#define SLEEP_COUNT_MS          47           // Milliseconds to counts, 12000/256 = 46.875
#define SLEEP_COUNT_SCALED      750          // Because of round off shift this value to divide
#define SLEEP_COUNT_SHIFT       4

extern uint32_t tickCount;
extern void (*_periodicSvc)(void); // Future expansion to an array of pointers for multiple motors
void initializeClocks(void);
void initializePeriodicSvc(void (*svcPtr)());
void mySleep(float sleepTime);
void mySleepMs(uint32_t ms);
void waitForInterrupt(void);
void _sleepTimer(void);

#endif /* MY_CLOCKS_H_ */
