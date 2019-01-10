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
/*********************************************************************************
*        MSP432P401 = EUSCI_A0                                  -------------
*         -------------------                                   |           |
*         |            P1.2 |---> EUCA0Rxd >---------> | >------| XDS100-ET |
*         |                 |                                   |   Debug   |--- > USB
*         |            P1.3 |---> EUCA0Txd >---------> | >------|   Probe   |
*         |                 |                                   |           |
*         |                 |                                   -------------
*         -------------------
*
*******************************************************************************/

#ifndef MY_UART_H_
#define MY_UART_H_

#include <stdint.h>
#include <stdbool.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define UART_BUFFER_SIZE         512 // This value must be a power of 2 ie: 128, 256, 512, 1024 etc.
#define UART_BUFFER_MASK         (UART_BUFFER_SIZE - 1)
#define EOF                      (-1)

#define TRUE                    1
#define FALSE                   0

#define MSP
#ifdef MSP
    extern uint8_t  _uartRxBuffer[UART_BUFFER_SIZE];
    extern volatile uint32_t  _uartRxHead;
    extern volatile uint32_t  _uartRxTail;
    extern uint8_t  _uartTxBuffer[UART_BUFFER_SIZE];
    extern volatile uint32_t  _uartTxHead;
    extern volatile uint32_t  _uartTxTail;
#endif

void initializeUart(bool initClocks);
int  _rxReady(void);
int  _lfReceived(void);
int  _crReceived(void);
void _txChar(char c);
int  _rxChar(void);
void _flush(void);
int  _peek(void);
#endif /* MY_UART_H_ */
