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

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "clocks.h"

uint32_t    tickCount = 0;
static bool _timesUp = false;
void (*_periodicSvc)(void); // Future expansion to an array of pointers for multiple motors
void initializePeriodicSvc(void (*svcPtr)())
{
    _periodicSvc = svcPtr;
}
void initializeClocks(void)
 {
    static bool _initialized = false;
    if(_initialized)
        return;
    /* Stop WDT  */
    MAP_WDT_A_holdTimer();

    /* Initialize MCLK to run from DCO with divider = 1, clock used for timer captures, and ADC */
    MAP_CS_initClockSignal(CS_MCLK,CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Initialize SMCLK to run from DCO with divider = 16, clock used for PWM */
    MAP_CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Setting DCO to 12MHz */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    /* Setting REFOCLK to 128Khz, then using it for ACLK, clock used for UART and SPI */
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Enable SysTick */
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(SYS_TICK);
    MAP_SysTick_enableInterrupt();
#ifdef LED_HEARTBEAT
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);  // LED Red Heartbeat
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    #ifdef LED_RX_BLINK
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);  // LED Green Rx
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    #endif
    #ifdef LED_TX_BLINK
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);  // LED Blue Tx
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
    #endif
#endif
    /* Enable 32 bit timer in one shot mode */
    MAP_Timer32_initModule(TIMER32_BASE, TIMER32_PRESCALER_256, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    MAP_Timer32_registerInterrupt(TIMER32_0_INTERRUPT, _sleepTimer);
    MAP_Interrupt_setPriority(TIMER32_0_INTERRUPT, 0x04);
    /* Enabling MASTER interrupts */
    tickCount = 0;
    Interrupt_enableMaster();
    _initialized = true;
}

/* SysTick ISR */
void SysTick_Handler(void)
{
#ifdef LED_HEARTBEAT
	static uint32_t ledCount = 0;
	ledCount++;
    if(ledCount > HEART_BEAT_COUNT) // 1 second
    {   // Toggle for a heartbeat
    	ledCount = 0;
    	P1->OUT^= GPIO_PIN0;  // Heartbeat toggle
    #ifdef LED_RX_BLINK
    	P2->OUT&= ~GPIO_PIN1;  // Rx off
    #endif
    #ifdef LED_TX_BLINK
    	P2->OUT&= ~GPIO_PIN2;  // Turn off Blue LED
    #endif
    }
#endif
    tickCount++;
    if(_periodicSvc != 0)
        _periodicSvc();
}

void _sleepTimer(void)
{
//	MAP_Timer32_clearInterruptFlag(TIMER32_0_BASE); // TIMER32_1
	_timesUp = true;
	TIMER32_1->INTCLR = 0; // Any write clears//
}

void mySleep(float sleepTime)
{   // Time is in seconds
	sleepTime*= SLEEP_COUNT;
	_timesUp = false;
	MAP_Timer32_setCount(TIMER32_BASE, (uint32_t)sleepTime);
	MAP_Timer32_enableInterrupt(TIMER32_0_BASE);
	MAP_Timer32_startTimer(TIMER32_0_BASE, 1);
	while(!_timesUp)
	{
		MAP_PCM_gotoLPM0();
	}
}

void mySleepMs(uint32_t ms)
{   // Time is in milliseconds
	ms*= SLEEP_COUNT_SCALED;
	ms>>= SLEEP_COUNT_SHIFT;
	_timesUp = false;
	MAP_Timer32_setCount(TIMER32_BASE, (uint32_t)ms);
	MAP_Timer32_enableInterrupt(TIMER32_0_BASE);
	MAP_Timer32_startTimer(TIMER32_0_BASE, 1);
	while(!_timesUp)
	{
		MAP_PCM_gotoLPM0();
	}
}
void waitForInterrupt(void)
{
    __WFI();
}

