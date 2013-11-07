/*********************************************************************
 *
 *      PIC32MX UART Library Interface Example
 *
 *********************************************************************
 * FileName:        uart_basic.c
 * Dependencies:    plib.h
 *
 * Processor:       PIC32MX
 *
 * Complier:        MPLAB C32
 *                  MPLAB IDE
 * Company:         Microchip Technology Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the �Company�) for its PIC32MX Microcontroller is intended
 * and supplied to you, the Company�s customer, for use solely and
 * exclusively on Microchip Microcontroller products.
 * The software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *********************************************************************
 * The purpose of this example code is to demonstrate the PIC32MX
 * peripheral library macros and functions supporting the UART
 * module and its various features.
 *
 * Platform: Explorer-16 with PIC32MX PIM
 *
 * Features demonstrated:
 *    - UART perepheral library usage
 *
 * Description:
 *      This program uses the UART library to communicate through
 *      the serial port on the Explorer-16. With an RS232 cable
 *      attatched to the Explorer-16 and a pc, the program will
 *      output text and recieve input using a terminal program.
 *
 * Notes:
 *    - PIC32MX 2xx PIMS are unconnected to the Explorer-16 DB9
 *      connector. It must be soldered to the test points on top of
 *      the PIM for proper functionality. The README file contains
 *      a list of the connections that need to be made.
 ********************************************************************/
#include <plib.h>
#include <peripheral/ports.h>

// Configuration Bit settings
// SYSCLK = 80 MHz (8MHz Crystal / FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 80 MHz (SYSCLK / FPBDIV)
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1

#define SYS_FREQ (80000000L)

#define	GetPeripheralClock()		(SYS_FREQ/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(SYS_FREQ)

#define UART_MODULE_ID UART1 // PIM is connected to WF32 through UART1 module

// Function Prototypes
void SendDataBuffer(const char *buffer);
UINT32 ReadKeyboard(void);
UINT32 GetDataBuffer(char *buffer, UINT32 max_size);

// Constant Data
/*
const char mainMenu[] =
{
    "\r\nWelcome to PIC32 UART Peripheral Library Demo!\r\n"\
    "Here are the main menu choices\r\n"\
    "1. View Actual BAUD rate\r\n"\
    "2. Use AUTOBAUD\r\n"\
    "3. Set Line Control\r\n"\
    "4. ECHO\r\n"
    "\r\n\r\nPlease Choose a number\r\n"
};
*/

/*
const char lineMenu[] =
{
    "Line Control Menu\r\n"\
    "You may need to change your line control on your terminal to see data\r\n"\
    "1. 8-N-1\r\n"\
    "2. 8-E-1\r\n"\
    "3. 8-O-1\r\n"\
    "4. 8-N-2\r\n"\
    "5. 8-E-2\r\n"\
    "6. 8-O-2\r\n"\
    "\r\n\r\nPlease Choose a number\r\n"
};
*/

/*
const UINT32 lineControl[] =
{
    (UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1),
    (UART_DATA_SIZE_8_BITS | UART_PARITY_EVEN | UART_STOP_BITS_1),
    (UART_DATA_SIZE_8_BITS | UART_PARITY_ODD | UART_STOP_BITS_1),
    (UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2),
    (UART_DATA_SIZE_8_BITS | UART_PARITY_EVEN | UART_STOP_BITS_2),
    (UART_DATA_SIZE_8_BITS | UART_PARITY_ODD | UART_STOP_BITS_2)
};
*/

// *****************************************************************************
// void UARTTxBuffer(char *buffer, UINT32 size)
// *****************************************************************************
void SendDataBuffer(const char *buffer)
{
    char mander = buffer;

    while(mander != '\0')
    {
        while(!UARTTransmitterIsReady(UART_MODULE_ID));

        UARTSendDataByte(UART_MODULE_ID, mander);

        mander++;
    }

    while(!UARTTransmissionHasCompleted(UART_MODULE_ID));
}
// *****************************************************************************
// UINT32 GetDataBuffer(char *buffer, UINT32 max_size)
// *****************************************************************************
UINT32 GetDataBuffer(char *buffer, UINT32 max_size)
{
    UINT32 num_char;

    num_char = 0;

    while(num_char < max_size)
    {
        UINT8 character;

        while(!UARTReceivedDataIsAvailable(UART_MODULE_ID));

        character = UARTGetDataByte(UART_MODULE_ID);

        if(character == '\r')
            break;

        *buffer = character;

        buffer++;
        num_char++;
    }

    return num_char;
}
// *****************************************************************************
// UINT32 GetMenuChoice(void)
// *****************************************************************************
UINT32 ReadKeyboard(void)
{
    UINT8  menu_item;

    while(!UARTReceivedDataIsAvailable(UART_MODULE_ID));

    menu_item = UARTGetDataByte(UART_MODULE_ID);

    menu_item -= '0';

    SendDataBuffer("\r\n");

    return (UINT32)menu_item;
}

void UART_setup(void){
    UARTConfigure(UART_MODULE_ID, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART_MODULE_ID, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART_MODULE_ID, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART_MODULE_ID, GetPeripheralClock(), 9600);
    UARTEnable(UART_MODULE_ID, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
}