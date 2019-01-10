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

#include <clocks.h>
#include <my_uart.h>

uint8_t   _uartRxBuffer[UART_BUFFER_SIZE];
volatile uint32_t  _uartRxHead = 0;
volatile uint32_t  _uartRxTail = 0;
uint8_t  _uartTxBuffer[UART_BUFFER_SIZE];
volatile uint32_t  _uartTxHead = 0;
volatile uint32_t  _uartTxTail = 0;

/* Setting a Baud Rate
* =================================
* For a given BRCLK clock source, the baud rate used determines the required division factor N:
*                    N = f BRCLK / baud rate
* The division factor N is often a non integer value, thus, at least one divider and one modulator
* stage is used to meet the factor as closely as possible.  If N is equal or greater than 16,
* it is recommended to use the over sampling baud-rate generation mode by setting UCOS16.
*
* NOTE:
*    Baud-rate settings quick set up
*    To calculate the correct settings for the baud-rate generation, perform these steps:
*
* 1. Calculate N = f BRCLK / baud rate
*    [if N > 16 continue with step 3, otherwise with step 2]
* 2. UCOS16 = 0, UCBRx = INT(N) [continue with step 4]
* 3. UCOS16 = 1, UCBRx = INT(N/16), UCBRFx = INT([(N/16) – INT(N/16)] × 16)
* 4. UCBRSx can be found by looking up the fractional part of N ( = N - INT(N) ) in table below.
* 5. If OS16 = 0 was chosen, a detailed error calculation is recommended to be performed
*    Table can be used as a lookup table for finding the correct UCBRSx modulation pattern for the
*    corresponding fractional part of N. The values there are optimized for transmitting.
*
* Table UCBRSx Settings for Fractional Portion of N = f BRCLK /Baud Rate
* ==============================================================================
* Fractional Portion of N  UCBRSx (1)   Fractional Portion of N   UCBRSx (1)
* -----------------------  ----------   -----------------------   ----------
*       0.0000               0x00             0.5002                0xAA
*       0.0529               0x01             0.5715                0x6B
*       0.0715               0x02             0.6003                0xAD
*       0.0835               0x04             0.6254                0xB5
*       0.1001               0x08             0.6432                0xB6
*       0.1252               0x10             0.6667                0xD6
*       0.1430               0x20             0.7001                0xB7
*       0.1670               0x11             0.7147                0xBB
*       0.2147               0x21             0.7503                0xDD
*       0.2224               0x22             0.7861                0xED
*       0.2503               0x44             0.8004                0xEE
*       0.3000               0x25             0.8333                0xBF
*       0.3335               0x49             0.8464                0xDF
*       0.3575               0x4A             0.8572                0xEF
*       0.3753               0x52             0.8751                0xF7
*       0.4003               0x92             0.9004                0xFB
*       0.4286               0x53             0.9170                0xFD
*       0.4378               0x55             0.#define CR_REQUIRED    // Checks for /r character echos a /n

*       9288                0xFE
*
* Note 1: The UCBRSx setting in one row is valid from the fractional portion given
*         in that row until the one in the next row
* ----------------------------------------------
* | Clock = 128000 Khz | 9,600 BAUD |  19,200  |  38,400  |  57,600  | 115,200 |
* |-----------------------------------------------------------------------------
* |   clockPreScaler   |     13     |     6    |     3    |    2     |    1    |
* |   firstModReg      |      0     |     0    |     0    |    0     |    0    |
* |   secondModReg     |     37     |   182    |   132    |   33     |    8    |
* |   overSampling     |      0     |     0    |     0    |    0     |    0    |
* ------------------------------------------------------------------------------
* | Clock = 750000 Khz | 9,600 BAUD |  19,200  |  38,400  |  57,600  | 115,200 |
* |------------------------------------------------------------------------------
* |   clockPreScaler   |      4     |     2    |    19     |    13   |    6    |
* |   firstModReg      |     14     |     7    |     0     |     0   |    0    |
* |   secondModReg     |      8     |     1    |   170     |     0   |  170    |
* |   overSampling     |      1     |     1    |     0     |     0   |    0    |
* ------------------------------------------------------------------------------
* ------------------------------------------------------------------------------
* | Clock = 12 Mhz     | 9,600 BAUD |  19,200  |  38,400  |  57,600  | 115,200 |
* |-----------------------------------------------------------------------------
* |   clockPreScaler   |     78     |    39    |   19     |    13   |    6     |
* |   firstModReg      |      2     |     1    |    8     |     0   |    8     |
* |   secondModReg     |      0     |     0    |   85     |    37   |   32     |
* |   overSampling     |      1     |     1    |    1     |     1   |    1     |
* ------------------------------------------------------------------------------
* Use calculator found at this link:
* http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
*/

const eUSCI_UART_Config uartConfig =
{   // Set 57600 BAUD rate
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
    13,                                      // UCBRx  = clockPrescaler
    0,                                       // UCBRFx = firstModReg, ignored if UCS016 = 0
    37,                                      // UCBRSx = secondModReg
    EUSCI_A_UART_NO_PARITY,                  // No Parity
    EUSCI_A_UART_LSB_FIRST,                  // MSB First
    EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
    EUSCI_A_UART_MODE,                       // UART mode
	EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // UCSOS16, overSampling 0 = disabled, 1 = enabled
};

/* EUSCI A0 UART ISR  */
void EUSCIA0_IRQHandler(void)
{
    register uint_fast16_t flags;
    flags = EUSCI_A0->IFG;
    if(flags & EUSCI_A_IFG_RXIFG)
    {
	#if defined (LED_RX_BLINK) && defined (LED_HEARTBEAT)
   		P2->OUT|= GPIO_PIN1;  // Turn on Green
	#endif
        // Place the character in the buffer
        _uartRxBuffer[_uartRxHead++] = (char)EUSCI_A0->RXBUF;
        _uartRxHead&= UART_BUFFER_MASK;
    }
    if((flags & EUSCI_A_IFG_TXIFG) && (EUSCI_A0->IE & EUSCI_A_IE_TXIE)) // Buffer is empty
    {
	#if defined (LED_TX_BLINK) && defined (LED_HEARTBEAT)
    	P2->OUT|= GPIO_PIN2;  // Turn on Blue LED
	#endif
        EUSCI_A0->TXBUF = _uartTxBuffer[_uartTxTail++]; // Clears interrupt flag
        _uartTxTail&= UART_BUFFER_MASK;
        if(_uartTxHead == _uartTxTail)
            EUSCI_A0->IE&= ~EUSCI_A_IE_TXIE;
   }
}
void _txChar(char c)
{
    unsigned nBytes = 0;
    unsigned nearFullFlag = FALSE;

    nBytes = _uartTxHead - _uartTxTail;
    nBytes&= UART_BUFFER_MASK;
	if((nBytes > (UART_BUFFER_SIZE - 64)) && (nBytes < UART_BUFFER_SIZE))
	    nearFullFlag = TRUE;
	while(nearFullFlag)
	{   // Wait here if buffer is near full
	    CPU_wfi();
	    nBytes = _uartTxHead - _uartTxTail;
	    nBytes&= UART_BUFFER_MASK;
	    if(nBytes < 64)
	        nearFullFlag = FALSE;
	}
	_uartTxBuffer[_uartTxHead++] = c;
	_uartTxHead&= UART_BUFFER_MASK;
    nBytes = _uartTxHead - _uartTxTail;
	nBytes&= UART_BUFFER_MASK;
	if(nBytes == 1) // Buffer not empty
        EUSCI_A0->IE|= EUSCI_A_IE_TXIE; // Enable interrupt
}
int _rxChar(void)
{
	int in;
	if((_uartRxHead - _uartRxTail) == 0)
		return -1;
	in = _uartRxBuffer[_uartRxTail++];
	_uartRxTail&= UART_BUFFER_MASK;
	return in;
}
int _rxReady(void)
{
    unsigned numBytes;
    numBytes = _uartRxHead - _uartRxTail;
    numBytes&= UART_BUFFER_MASK;
    return numBytes;
}
void _flush(void)
{
    _uartRxHead = _uartRxTail = 0;
}
int _peek(void)
{   /* get a character  returns -1 if nothing else returns the character */
    int in;
    in = _uartRxHead - _uartRxTail;
    in&= UART_BUFFER_MASK;
    if(in == 0) // Empty return
        return -1;
    return (int)_uartRxBuffer[_uartRxTail];
}

void initializeUart(bool initClocks)
{
	if(initClocks)
		initializeClocks();
    /* Setting REFOCLK to 128Khz, then using it for ACLK, clock used for UART and SPI */
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    /* Select P1.2 and P1.3 UART mode */
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    /* Configure UART */
	MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);
    /* Enable UART, EUSCI_A0_BASE */
	MAP_UART_enableModule(EUSCI_A0_BASE);
    /* Enable UART receive interrupt */
	MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    /* Enabling MASTER interrupts */
    _uartTxHead = _uartTxTail = 0;
    _uartRxHead = _uartRxTail = 0;
	MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
}

void _gotoLPM0(void) { MAP_PCM_gotoLPM0(); }
