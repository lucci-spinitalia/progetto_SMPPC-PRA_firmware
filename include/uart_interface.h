/***************************************************
*
* FileName:        uart_interface.h
* Dependencies:    See INCLUDES section below
* Processor:       PIC18
* Compiler:        C18 3.43
* Company:         SpinItalia s.r.l.
* 
* All Rights Reserved.
*
* The information contained herein is confidential 
* property of SpinItalia s.r.l. The user, copying, transfer or 
* disclosure of such information is prohibited except
* by express written agreement with SpinItalia s.r.l.
*
* First written on 18/10/2012 by Luca Lucci.
*
* Module Description:
* Define prototype and const for uart_interface.c
*
****************************************************/

/**
 * \file
 * Header file del modulo uart_interface.
 */

#ifndef _UART_INTERFACE_H
#define _UART_INTERFACE_H

#ifndef USE_OR_MASKS
#undef USE_AND_MASKS
#define USE_OR_MASKS
#endif


/** I N C L U D E S ********************************/
#include <usart.h>
#include <string.h>

/** D E F I N E ************************************/
#define UART1_BUFFER_SIZE_TX 1
#define UART1_BUFFER_SIZE_RX 1
#define UART2_BUFFER_SIZE_TX 256
#define UART2_BUFFER_SIZE_RX 256

#ifdef USE_AND_MASKS
static unsigned char const UART1_BAUD_CONFIG = ((BAUD_16_BIT_RATE & BAUD_AUTO_OFF & BAUD_WAKEUP_OFF) & BAUD_IDLE_RX_PIN_STATE_HIGH & BAUD_IDLE_TX_PIN_STATE_HIGH);
static unsigned char const UART2_BAUD_CONFIG = ((BAUD_16_BIT_RATE & BAUD_AUTO_OFF & BAUD_WAKEUP_OFF) & BAUD_IDLE_RX_PIN_STATE_HIGH & BAUD_IDLE_TX_PIN_STATE_HIGH);
static unsigned char const UART1_CONFIG = (USART_TX_INT_OFF & USART_RX_INT_OFF & USART_BRGH_HIGH & USART_CONT_RX & USART_EIGHT_BIT & USART_ASYNCH_MODE);
static unsigned char const UART2_CONFIG = (USART_TX_INT_OFF & USART_RX_INT_OFF & USART_BRGH_HIGH & USART_CONT_RX & USART_EIGHT_BIT & USART_ASYNCH_MODE);
#else
static const unsigned char  UART1_BAUD_CONFIG = ((BAUD_16_BIT_RATE | BAUD_AUTO_OFF | BAUD_WAKEUP_OFF) | BAUD_IDLE_RX_PIN_STATE_HIGH | BAUD_IDLE_TX_PIN_STATE_HIGH);
static const unsigned char  UART2_BAUD_CONFIG = (BAUD_16_BIT_RATE | BAUD_AUTO_OFF | BAUD_WAKEUP_OFF | BAUD_IDLE_RX_PIN_STATE_HIGH | BAUD_IDLE_TX_PIN_STATE_HIGH);
static const unsigned char  UART1_CONFIG = (USART_TX_INT_OFF | USART_RX_INT_OFF | USART_BRGH_HIGH | USART_CONT_RX | USART_EIGHT_BIT | USART_ASYNCH_MODE);
static const unsigned char  UART2_CONFIG = (USART_TX_INT_OFF | USART_RX_INT_ON | USART_BRGH_HIGH | USART_CONT_RX | USART_EIGHT_BIT | USART_ASYNCH_MODE);

#endif

#define UART1_INTERRUPT_TX  (UART1_CONFIG&0x80)
#define UART1_INTERRUPT_RX (UART1_CONFIG&0x40)
#define UART2_INTERRUPT_TX (UART2_CONFIG & 0x80)
#define UART2_INTERRUPT_RX (UART2_CONFIG&0x40)

// RS485 enabel flag
#define RS485 1
#define RS232 0

// Error code
#define BUFFER_RX_OVERFLOW 0x11
#define FRAME_ERROR_CODE 0x12
#define BUFFER_RX_MICRO_OVERFLOW 0x13

/** M A C R O     **********************************/
#define uart1_open(baud_rate)	Open1USART(UART1_CONFIG, uart1_buad_rate_set(baud_rate, FOSC_HZ))
#define uart2_open(baud_rate)	Open2USART(UART2_CONFIG, uart2_buad_rate_set(baud_rate, FOSC_HZ))
#define uart1_close()			Close1USART()
#define uart2_close()			Close2USART()

/** S T R U C T S **********************************/
struct uart_status
{
  unsigned buffer_tx_full 			:1;	// tx buffer full
  unsigned buffer_tx_empty 			:1; // tx buffer empty
  unsigned buffer_rx_full			:1; // rx buffer full
  unsigned buffer_rx_empty			:1; // rx buffer empty
  unsigned buffer_rx_overflow		:1; // rx buffer overflow
  unsigned buffer_rx_micro_overflow		:1; // rx buffer overflow
  unsigned buffer_rx_error_frame	:1; // OERR or FERR error accured. User needs to
									// clear this error-bit (uart_rx_error) in
									// the fw
};

/** S H A R E D  V A R I A B L E S  ****************/
extern struct uart_status uart1_status;	// uart's status flag
extern struct uart_status uart2_status;	// uart's status flag
extern unsigned char uart1_buffer_tx[UART1_BUFFER_SIZE_TX];	//tx buffer
extern unsigned char uart1_buffer_rx[UART1_BUFFER_SIZE_RX];	//rx buffer
extern unsigned char uart2_buffer_tx[UART2_BUFFER_SIZE_TX];	//tx buffer
extern unsigned char uart2_buffer_rx[UART2_BUFFER_SIZE_RX];	//rx buffer


/** P R O T O T Y P E S ****************************/
unsigned char uart1_buffer_read(unsigned char *);	// returns a char from rx buffer
unsigned char uart1_buffer_rx_load(void);	// read a char from usart

// returns the number of characters in receive buffer
unsigned char uart1_get_rx_data_size(void);		

unsigned char uart1_buffer_send(void);			// send a byte from tx buff to usart
unsigned char uart1_buffer_tx_load(unsigned char);		// put a char in tx buf

// put a string in tx buffer
unsigned char uart1_buffer_tx_seq_load(unsigned char *data_write,
									   unsigned int length);

unsigned int uart1_get_tx_buffer_empty_space(void);	// returns size of empty 
														// section in tx buffer
// return BRG value to set the baud rate
unsigned int uart1_buad_rate_set(unsigned long baud_rate, long freq_Hz);

void uart1_tsr_poll(void); // Return TSR register's value
void uart1_isr(void);					// called from ISR from the main program
// initialization of uart module
void uart1_init(unsigned char rs485_enable);
// return error message
unsigned char uart1_error_handle();
unsigned char uart1_error_translate(unsigned char *error_message, unsigned char code, unsigned char *length);

unsigned char uart2_buffer_read(unsigned char *);	// returns a char from rx buffer
int uart2_buffer_read_filtered(char *data, char token);  // returns string from rx buffer when find token
int uart2_buffer_read_multifiltered(char *data, char *token, char token_number);
unsigned char uart2_buffer_rx_load(void);		// read a char from usart
unsigned char uart2_get_rx_data_size(void);		// returns the number of 
											    // characters in receive buffer
unsigned char uart2_buffer_send(void);	// send a byte from tx buff to usart
unsigned char uart2_buffer_tx_load(unsigned char data_write);		// put a char in tx buf
// put a string in tx buffer
unsigned char uart2_buffer_tx_seq_load(unsigned char *data_write,
									   unsigned int length);
unsigned int uart2_get_tx_buffer_empty_space(void);	// returns size of empty 
														// section in tx buffer

// return BRG value to set the baud rate
unsigned int uart2_buad_rate_set(unsigned long baud_rate, long freq_Hz);

void uart2_tsr_poll(void); // Return TSR register's value

int uart2_isr(void);					// called from ISR from the main program
// initialization of uart module
void uart2_init(unsigned char rs485_enable);
// return error message
unsigned char uart2_error_handle();
unsigned char uart2_error_translate(unsigned char *error_message, unsigned char code, unsigned char *length);

void uart1_flush_buffers(void);
void uart2_flush_buffers(void);

#endif

