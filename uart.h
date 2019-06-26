/*
 * uart.h
 *
 *  Created on: Sep 24, 2017
 *      Author: kevinKuwata1
 */

#include <stdint.h>

#ifndef UART_H_
#define UART_H_

/*
 * Tune the clock for the UART, uses DCO set to be 4Mhz.
 * */
void configure_clocks();

/*
 * Configures UART A0.
 *  Clock:
 *  Baud:
 *  Settings: No parity, no address, LSb First, 8 Bit, 1 Start, 1 Stop
 *  Blocking
 * */
void UART_config();

/*
 * Place  data into Tx Buffer
 * */
void UART_putchar(uint8_t tx_data);

/*
 *  Iterates through the array, puts into the Tx Buffer
 * */
void UART_putchar_n(uint8_t * data, uint32_t length);



//Conversions
void reverse(char str[], int len);
int intToStr(int value, char str[], int length);

char itoa(int value, char str[], int base);          //Integer to ASCII (should return ascii)

char ftoa(float f, char str[], int point);         //Float to ASCII (should return ascii)

#endif /* UART_H_ */
