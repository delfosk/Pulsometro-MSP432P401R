/*
 * uart.c
 *
 *  Created on: Sep 24, 2017
 *      Author: kevinKuwata1
 */


#include "uart.h"

#include "math.h"

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#define BLUE_LED            (BIT2)



void UART_config(){



    //Set up pins.
    //Pin 2 is the RX           pin 3 is the TX
    //RX SETUP
    P1SEL0 |= BIT2;
    P1SEL1 &= ~BIT2;

    //TX SETUP
    P1SEL0 |= BIT3;
    P1SEL1 &= ~BIT3;


    // UART must be in reset mode to configure
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; //reset by setting to 1.


    EUSCI_A0->IFG = 0b0;
    EUSCI_A0->IE |= 0b01;//BIT1 | EUSCI_A_IFG_RXIFG; // set up interrupt enable for both Rx and Tx.

    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK; //frame parameter , enable interrupt on the RX
    //baud rate clock is 4 Mhz
    //what register is UCBR? it needs to be set as 26 is that the word one?
    EUSCI_A0->BRW = 26; //baud rate, so baud rate set at 26 with other settings will result in a rate of 9600

    // UCOS16 = 1,          UCbRx = 26;              UCBRF = 0 ;              UCBRSx = 0xB6
    EUSCI_A0->MCTLW|=  (EUSCI_A_MCTLW_OS16); //from table 22.3.13

    EUSCI_A0->MCTLW|= ((0xB600)) ;//| (EUSCI_A_MCTLW_OS16); //from table 22.3.13

    // CLEAR UCSWRST
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;

    //set up interrupt
    EUSCI_A0->IE |= UCRXIE ;//BIT1 | EUSCI_A_IFG_RXIFG; // set up interrupt enable for both Rx and Tx.

    NVIC_EnableIRQ(EUSCIA0_IRQn);

}


/*
 * Place Load data into Tx Buffer
 * */
void UART_putchar(uint8_t tx_data){
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A0->TXBUF = tx_data; //hoping 30 should be '0', for sure ascii
    // EUSCI_A_ifg
}

/*
 *  Iterates through the array, puts into the Tx Buffer
 * */
void UART_putchar_n(uint8_t * data, uint32_t length){
    //data is an array! so you can use pointer math to iterate through
    uint8_t index =0;

    for(index = 0; index <length ; index++){
        UART_putchar(data[index]);
    }
}




void reverse(char str[], int len)        //function to reverse string to get it readable and not backwards
{
    int start, end;
    char temp;
    for (start=0, end = len-1; start<end; start++, end--)
    {
        temp = *(str+start);
        *(str+start)= *(str+end);
        *(str+end) = temp;
    }
}

int intToStr(int value, char str[], int length)
{
    int i = 0;
    while (value)
    {
        str[i++] = (value%10) + '0';
        value = value/10;
    }
    while (i < length)
    {
        str[i++] = '0';
    }
    reverse(str,i);
    str[i] = '\0';
    return i;
}

extern char itoa (int value, char str[], int base)           //integer to string function
{
    int  i = 0;
    int sign;

    if ((sign = value) < 0){        //What to do if values is negative
        value = -value;         //make value positive so we can work with
    }

    do{
        str[i++] = value % base + '0';
    }while ((value /= base) > 0);

    if (sign < 0){
        str[i++] = '-';
    }
    str[i] = '\0';
    reverse(str,i);

}
