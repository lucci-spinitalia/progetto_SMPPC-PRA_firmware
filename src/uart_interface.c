/***************************************************
*
* FileName:        uart_interface.c
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
* This module provide functions for rs232 interface
*
****************************************************/
/**
* \file
* Modulo per la gestione della seriale attraverso buffer circolari.
 * */

/** I N C L U D E S ********************************/
#include "../include/uart_interface.h"
#include "../system/io_cfg.h"	// here is defined the tx and rx pins
#include <stdio.h>

/** L O C A L  V A R I A B L E S  ******************/
struct uart_status uart1_status;	// uart's status flag
struct uart_status uart2_status;	// uart's status flag

#ifndef __XC8
#pragma udata UART_BUFF
#endif
unsigned char uart1_buffer_tx[UART1_BUFFER_SIZE_TX];	//tx buffer
unsigned char uart1_buffer_rx[UART1_BUFFER_SIZE_RX];	//rx buffer
unsigned char uart2_buffer_tx[UART2_BUFFER_SIZE_TX];	//tx buffer
unsigned char uart2_buffer_rx[UART2_BUFFER_SIZE_RX];	//rx buffer
#ifndef __XC8
#pragma udata
#endif

unsigned int uart1_buffer_tx_data_cnt;	// number of byte to transmit
unsigned int uart1_buffer_tx_wr_ptr;	// write position in tx buffer
unsigned int uart1_buffer_tx_rd_ptr;	// read position to place data from
											// buffer to TXREG
unsigned int uart1_buffer_rx_data_cnt;	// number of byte received
unsigned int uart1_buffer_rx_wr_ptr;	// write position in rx buffer

unsigned char uart1_rs485_enable;  // flag to enable rs485 buffer managment

unsigned int uart2_buffer_tx_data_cnt;	// number of byte to transmit
unsigned int uart2_buffer_tx_wr_ptr;	// write position in tx buffer
unsigned int uart2_buffer_tx_rd_ptr;	// read position to place data from
										// buffer to TXREG
unsigned char uart1_rs485_tx_count;

unsigned int uart2_buffer_rx_data_cnt;	// number of byte received
unsigned int uart2_buffer_rx_wr_ptr;	// write position in rx buffer
unsigned int uart2_buffer_rx_rd_ptr;	// read position by the application

unsigned char uart2_rs485_enable;  // flag to enable rs485 buffer managment
unsigned char uart2_rs485_tx_count;
/**************************************************
* Function name		: void uart1_init(unsigned char rs485_enable)
*   rs485_enable    : flag to enable rs485 buffer managment
* Created by		: Luca Lucci
* Date created		: 18/10/12
* Description		: This function initialize uart peripheral and it need to
*                         be called before using uart_put_char and uart_get_char
*                         function.
* Notes	                : The tx and rx pins must be se to OUTPUT and INPUT
*                         rispectively.
**************************************************/
void uart1_init(unsigned char rs485_enable)
{
  // Initialize the status variables and circular buffer variables
  uart1_status.buffer_tx_full = 0;
  uart1_status.buffer_tx_empty = 1;

  uart1_buffer_tx_data_cnt = 0;
  uart1_buffer_tx_wr_ptr = 0;
  uart1_buffer_tx_rd_ptr = 0;

  uart1_status.buffer_rx_full = 0;
  uart1_status.buffer_rx_empty = 1;
  uart1_status.buffer_rx_overflow = 0;
  uart1_status.buffer_rx_micro_overflow = 0;
  uart1_status.buffer_rx_error_frame = 0;
  uart1_buffer_rx_data_cnt = 0;
  uart1_buffer_rx_wr_ptr = 0;

  uart1_rs485_enable = rs485_enable;
  uart1_rs485_tx_count = 0;

  // Initialize baud rate control registers
  baud1USART(UART1_BAUD_CONFIG);

  // Initialize tx/rx interrupts
if(UART1_INTERRUPT_TX)
    PIE1bits.TX1IE = 1;
else
    PIE1bits.TX1IE = 0;

if(UART1_INTERRUPT_RX)
    PIE1bits.RC1IE = 1;
else
    PIE1bits.RC1IE = 0;
}

/**************************************************
* Function name		: void void uart2_init(unsigned char rs485_enable)
*   rs485_enable    : flag to enable rs485 buffer managment
*
* Created by		: Luca Lucci
* Date created		: 18/10/12
* Description		: This function initialize uart peripheral and it need to
*					  be called before using uart_put_char and uart_get_char
*					  function.
* Notes				: The tx and rx pins must be se to OUTPUT and INPUT rispectively
**************************************************/
void uart2_init(unsigned char rs485_enable)
{
  // Initialize the status variables and circular buffer variables
  uart2_status.buffer_tx_full = 0;
  uart2_status.buffer_tx_empty = 1;

  uart2_buffer_tx_data_cnt = 0;
  uart2_buffer_tx_wr_ptr = 0;
  uart2_buffer_tx_rd_ptr = 0;

  uart2_status.buffer_rx_full = 0;
  uart2_status.buffer_rx_empty = 1;
  uart2_status.buffer_rx_overflow = 0;
  uart2_status.buffer_rx_micro_overflow = 0;
  uart2_status.buffer_rx_error_frame = 0;
  uart2_buffer_rx_data_cnt = 0;
  uart2_buffer_rx_wr_ptr = 0;
  uart2_buffer_rx_rd_ptr = 0;

  uart2_rs485_enable = rs485_enable;
  uart2_rs485_tx_count = 0;

  // Initialize baud rate
  baud2USART(UART2_BAUD_CONFIG);

  // Initialize tx/rx interrupts
if(UART2_INTERRUPT_TX)
    PIE3bits.TX2IE = 1;
else
    PIE3bits.TX2IE = 0;
  
  if(UART2_INTERRUPT_RX)
    PIE3bits.RC2IE = 1;
  else
    PIE3bits.RC2IE = 0;
}

/**************************************************
* Function name		: unsigned char uart1_buffer_tx_load(unsigned char data_write)
*	return			: 0 = failure - the buffer is full
*					: 1 = success - buffer loaded
*	data_write		: unsigned char pointer to the data to load in the tx buffer
* Created by		: Luca Lucci
* Date created		: 18/10/12
* Description		: This function puts the data into tx buffer. It place the
*                	  argument data_write in transmit buffer and updates the data
*  					  count and write pointer variables.
* Notes				: The interrupt will be disable until the end 
**************************************************/
unsigned char uart1_buffer_tx_load(unsigned char data_write)
{
  // Check if the buffer is full; if not, add one byte of data
  if(uart1_status.buffer_tx_full)
    return 0;

  if(UART1_INTERRUPT_TX)
    PIE1bits.TX1IE = 0; // Disable tx interrupt to prevent data corruption

  uart1_buffer_tx[uart1_buffer_tx_wr_ptr] = data_write;
  uart1_status.buffer_tx_empty = 0;
  uart1_buffer_tx_data_cnt++; //update count

  if(uart1_buffer_tx_data_cnt == UART1_BUFFER_SIZE_TX)
    uart1_status.buffer_tx_full = 1;

  uart1_buffer_tx_wr_ptr++;	//point to the next location

  if(uart1_buffer_tx_wr_ptr == UART1_BUFFER_SIZE_TX)
    uart1_buffer_tx_wr_ptr = 0;

  if(UART1_INTERRUPT_TX)
    PIE1bits.TX1IE = 1; // Enable tx interrupt

  return 1;
}

/**************************************************
* Function name		: unsigned char uart2_buffer_tx_load(unsigned char data_write)
*	return			: 0 = failure - the buffer is full
*					: 1 = success - buffer loaded
*	data_write		: unsigned char pointer to the data to load in the tx buffer
* Created by		: Luca Lucci
* Date created		: 18/10/12
* Description		: This function puts the data into tx buffer. It place the
*                	  argument data in transmit buffer and updates the data
*  					  count and writa pointer variables.
* Notes				: The interrupt will be disable until the end. 
**************************************************/
unsigned char uart2_buffer_tx_load(unsigned char data_write)
{
  // check if the buffer is full; if not, add one byte of data
  if(uart2_status.buffer_tx_full)
    return 0;

  if(UART2_INTERRUPT_TX)
    PIE3bits.TX2IE = 0; // disable tx interrupt to prevent data corruption

  uart2_buffer_tx[uart2_buffer_tx_wr_ptr] = data_write;
  uart2_status.buffer_tx_empty = 0;
  uart2_buffer_tx_data_cnt++;

  if(uart2_buffer_tx_data_cnt == UART2_BUFFER_SIZE_TX)
    uart2_status.buffer_tx_full = 1;

  uart2_buffer_tx_wr_ptr++;

  if(uart2_buffer_tx_wr_ptr == UART2_BUFFER_SIZE_TX)
    uart2_buffer_tx_wr_ptr = 0;

  if(UART2_INTERRUPT_TX)
    PIE3bits.TX2IE = 1; // enable tx interrupt

  return 1;
}

/**************************************************
* Function name		: unsigned char uart1_buffer_tx_seq_load(unsigned char *data_write, unsigned char length)
*	return			: 0 = failure - the buffer is full
*					: 1 = success - buffer loaded
*	data_write		: pointer to the data to load in the tx buffer
*	length			: number of byte store in data_write
* Created by		: Luca Lucci
* Date created		: 01/11/12
* Description		: This function puts sequential the data into tx buffer. It
*					  place the argument data_write in transmit buffer and 
*					  updates the data count and write pointer variables.
* Notes				: The interrupt will be disable until the end 
**************************************************/
unsigned char uart1_buffer_tx_seq_load(unsigned char *data_write, unsigned int length)
{
  int i;	//for loop

  // check if there's enougth space
  if(uart1_get_tx_buffer_empty_space() < length)
    return 0;

  for(i = 0; i < length; i++)
  {
    if(!uart1_buffer_tx_load(data_write[i]))
	  return 0;
  }

  return 1;
}

/**************************************************
* Function name		: unsigned char uart2_buffer_tx_seq_load(unsigned char *data_write, unsigned char length)
*	return			: 0 = failure - the buffer is full
*					: 1 = success - buffer loaded
*	data_write		: pointer to the data to load in the tx buffer
*	length			: number of byte store in data_write
* Created by		: Luca Lucci
* Date created		: 01/11/12
* Description		: This function puts sequential the data into tx buffer. It
*					  place the argument data_write in transmit buffer and 
*					  updates the data count and write pointer variables.
* Notes				: The interrupt will be disable until the end. The max char
*					  that can be store is equal to the size of buffer free
**************************************************/
unsigned char uart2_buffer_tx_seq_load(unsigned char *data_write, unsigned int length)
{
  int i;	//for loop

  // check if there's enougth space
  if(uart2_get_tx_buffer_empty_space() < length)
    return 0;

  
  for(i = 0; i < length; i++)
  {
    if(!uart2_buffer_tx_load(data_write[i]))
	  return 0;
  }

  return 1;
}

/**************************************************
* Function name		: unsigned char uart1_buffer_send(void)
*  return			: 0 - bus not ready
*					  1 - message sent
*
* Created by		: Luca Lucci
* Date created		: 18/10/12
* Description		: This function sends the data from transmit buffer to 
*					  USART and updates the data count and read pointer 
*					  variables of transmit buffer
* Notes				: Here there's the control for the
*					  rs485 buffer, so I don't bother if I use interrupt or
*					  polling.
**************************************************/
unsigned char uart1_buffer_send(void)
{
  if(!uart1_status.buffer_tx_empty)
  {
    if(UART1_INTERRUPT_TX == 0)
    {
      /* TXxIF will be set regardless of the state of TXxIE; it cannot be cleared
	   in software.TXxIF is also not cleared immediately upon loading TXREGx, but
 	   becomes valid in the second instruction cycle followind the load 
	   istruction. Polling TXxIF immediately following a load of TXREGx will 
	   return invalid results. */
      if(!PIR1bits.TX1IF)
        return 0;

      Nop();
      Nop();
    }
    
    if(uart1_rs485_enable)
    {
      uart1_rs485_tx_count++;
      uart1_rs485_tx_enable = 1;
    }

    Write1USART(uart1_buffer_tx[uart1_buffer_tx_rd_ptr]);
    
    if(uart1_status.buffer_tx_full)
      uart1_status.buffer_tx_full = 0;

    uart1_buffer_tx_data_cnt--;

    if(uart1_buffer_tx_data_cnt == 0)
      uart1_status.buffer_tx_empty = 1;

    uart1_buffer_tx_rd_ptr++;

    if(uart1_buffer_tx_rd_ptr == UART1_BUFFER_SIZE_TX)
      uart1_buffer_tx_rd_ptr = 0;
  }
#if (UART1_INTERRUPT_TX)
  else
  {
    // disable tx interrupt because there's no data to send. This will be 
	// re-enabled as function uart_buffer_load is called
    PIE1bits.TX1IE = 0;
  }
#endif

  return 1;
}

/**************************************************
* Function name		: unsigned char uart2_buffer_send(void)
*  return			: 0 - bus not ready
*					  1 - message sent
*
* Created by		: Luca Lucci
* Date created		: 18/10/12
* Description		: This function sends the data from transmit buffer to 
*					  USART and updates the data count and read pointer 
*					  variables of transmit buffer
* Notes				: Here there's the control for the
*					  rs485 buffer, so I don't bother if I use interrupt or
*					  polling.
**************************************************/
unsigned char uart2_buffer_send(void)
{

  if( !uart2_status.buffer_tx_empty )
  {

    if(UART2_INTERRUPT_TX == 0)
    {
      /* TXxIF will be set regardless of the state of TXxIE; it cannot be cleared
	   in software.TXxIF is also not cleared immediately upon loading TXREGx, but
 	   becomes valid in the second instruction cycle followind the load 
	   istruction. Polling TXxIF immediately following a load of TXREGx will 
	   return invalid results. */
      if(!PIR3bits.TX2IF)
        return 0;

      Nop();
      Nop();
    }

    if(uart2_rs485_enable)
    {
      uart2_rs485_tx_count++;
#ifdef uart2_rs485_tx_enable
      uart2_rs485_tx_enable = 1;
#endif
    } 

    Write2USART(uart2_buffer_tx[uart2_buffer_tx_rd_ptr]);

    if( uart2_status.buffer_tx_full )
      uart2_status.buffer_tx_full = 0;

    uart2_buffer_tx_data_cnt--;

    if( uart2_buffer_tx_data_cnt == 0 )
      uart2_status.buffer_tx_empty = 1;

    uart2_buffer_tx_rd_ptr++;

    if( uart2_buffer_tx_rd_ptr == UART2_BUFFER_SIZE_TX)
      uart2_buffer_tx_rd_ptr = 0;
  }
#if (UART2_INTERRUPT_TX)
  else
  {
    // disable tx interrupt because there's no data to send. This will be 
	// re-enabled as function uart_buffer_load is called
    PIE3bits.TX2IE = 0;
  }
#endif

  return 1;
}

/**************************************************
* Function name		: unsigned char uart1_get_tx_buffer_empty_space(void)
*	return			: 0 = no space left
*					  unsigned char = number of free byte in tx buffer
* Created by		: Luca Lucci
* Date created		: 18/10/12
* Description		: This function returns the number of bytes of free space 
*					  left out in transmit buffer at the calling time. It 
*					  helps the user to further write data into trasmit buffer
*					  at once, rather than checking transmit buffer.
* Notes				: -
**************************************************/
unsigned int uart1_get_tx_buffer_empty_space(void)
{
  if(uart1_buffer_tx_data_cnt < UART1_BUFFER_SIZE_TX)
    return(UART1_BUFFER_SIZE_TX - uart1_buffer_tx_data_cnt);
  else
    return 0;
}

void uart1_flush_buffers(void)
{
  // Initialize the status variables and circular buffer variables
  uart1_status.buffer_tx_full = 0;
  uart1_status.buffer_tx_empty = 1;

  uart1_buffer_tx_data_cnt = 0;
  uart1_buffer_tx_wr_ptr = 0;
  uart1_buffer_tx_rd_ptr = 0;

  uart1_status.buffer_rx_full = 0;
  uart1_status.buffer_rx_empty = 1;
  uart1_status.buffer_rx_overflow = 0;
  uart1_status.buffer_rx_micro_overflow = 0;
  uart1_status.buffer_rx_error_frame = 0;
  uart1_buffer_rx_data_cnt = 0;
  uart1_buffer_rx_wr_ptr = 0;

  uart1_rs485_tx_count = 0;
}

/**************************************************
* Function name		: unsigned char uart2_get_tx_buffer_empty_space(void)
*	return			: 0 = no space left
*					  unsigned char = number of free byte in tx buffer
* Created by		: Luca Lucci
* Date created		: 18/10/12
* Description		: This function returns the number of bytes of free space 
*					  left out in transmit buffer at the calling time. It 
*					  helps the user to further write data into trasmit buffer
*					  at once, rather than checking transmit buffer.
* Notes				: -
**************************************************/
unsigned int uart2_get_tx_buffer_empty_space(void)
{
  if( uart2_buffer_tx_data_cnt < UART2_BUFFER_SIZE_TX )
    return( UART2_BUFFER_SIZE_TX - uart2_buffer_tx_data_cnt);
  else
    return 0;
}

void uart2_flush_buffers(void)
{
  // Initialize the status variables and circular buffer variables
  uart2_status.buffer_tx_full = 0;
  uart2_status.buffer_tx_empty = 1;

  uart2_buffer_tx_data_cnt = 0;
  uart2_buffer_tx_wr_ptr = 0;
  uart2_buffer_tx_rd_ptr = 0;

  uart2_status.buffer_rx_full = 0;
  uart2_status.buffer_rx_empty = 1;
  uart2_status.buffer_rx_overflow = 0;
  uart2_status.buffer_rx_micro_overflow = 0;
  uart2_status.buffer_rx_error_frame = 0;
  uart2_buffer_rx_data_cnt = 0;
  uart2_buffer_rx_wr_ptr = 0;
  uart2_buffer_rx_rd_ptr = 0;

  uart2_rs485_tx_count = 0;
}

/**************************************************
* Function name		: unsigned char uart1_buffer_rx_load(void)
*	return			: 0 = failure - frame error or overrun error from <usart.h>,
*					  	  rx buffer overflow, nothing to load
*					  1 = usart data loaded in rx buffer
* Created by		: Luca Lucci
* Date created		: 19/10/12
* Description		: This function load the data from the usart receiver to 
*					  the read buffer. It also check error states.
* Notes				: If interrupt has been disabled, this function polling 
*    				  RCIF bit. If it's clear no data will be storage in 
*					  receive buffer
**************************************************/
unsigned char uart1_buffer_rx_load(void)
{
  unsigned char chTemp;	// temporary variable for receiver

  if (UART1_INTERRUPT_RX == 0)
  {
    if(!PIR1bits.RC1IF)	// if there's no data return
      return 0;
  }

  // When overflow error accured, there's two word to read into usart buffers
  // Before claer and enable CREN I have to read these two words by polling
  // RCxIF
  if(RCSTA1bits.OERR)	// Overflow error
  {
    uart1_status.buffer_rx_micro_overflow = 1;

    chTemp = Read1USART();
    Nop();
    Nop();
    chTemp = Read1USART();

    // clear overflow condition
    RCSTA1bits.CREN = 0;
    RCSTA1bits.CREN = 1;
  }

  if(!uart1_status.buffer_rx_full)
  {
    if(RCSTA1bits.FERR)	// Frame error
    {
      uart1_status.buffer_rx_error_frame = 1;
      chTemp = Read1USART();
      return 0;
    }

    chTemp = Read1USART();

    // if I'm in rs485 mode and I sent something then I wont read it from
    // buffer (no loopback)
    if(uart1_rs485_enable && uart1_rs485_tx_count)
    {
      uart1_rs485_tx_count--;
      return 0;
    }

    //uart1_status.buffer_rx_overflow = 0;
    uart1_status.buffer_rx_empty = 0;
    uart1_buffer_rx[uart1_buffer_rx_wr_ptr] = chTemp;
    uart1_buffer_rx_data_cnt++;
      
    if(uart1_buffer_rx_data_cnt == UART1_BUFFER_SIZE_RX)
      uart1_status.buffer_rx_full = 1;

    uart1_buffer_rx_wr_ptr++;

    if(uart1_buffer_rx_wr_ptr == UART1_BUFFER_SIZE_RX)
      uart1_buffer_rx_wr_ptr = 0;
  }
  else
    return 0;

  return 1;
}

/**************************************************
* Function name		: unsigned char uart2_buffer_rx_load(void)
*	return	        : 0 = failure - frame error or overrun error from <usart.h>,
*			                rx buffer overflow,  nothing to load
*			  1 = usart data loaded in rx buffer
* Created by		: Luca Lucci
* Date created		: 19/10/12
* Description		: This function load the data from the usart receiver to 
*                         the read buffer. It also check error states.
* Notes	                : If interrupt has been disabled, this function polling
*                         RCIF bit. If it's clear no data will be storage in
*                         receive buffer
**************************************************/
unsigned char uart2_buffer_rx_load(void)
{
  unsigned char chTemp;	// temporary variable for receiver
  
  if((UART2_INTERRUPT_RX == 0) && (!PIR3bits.RC2IF)) // if there's no data return
    return 0;

  // When overflow error accured, there's two word to read into usart buffers
  // Before claer and enable CREN I have to read these two words by polling
  // RCxIF
  if(RCSTA2bits.OERR)	// Overflow error
  {
    uart2_status.buffer_rx_micro_overflow = 1;

    chTemp = Read2USART();
    Nop();
    Nop();
    chTemp = Read2USART();

    // clear overflow condition
    RCSTA2bits.CREN = 0;
    RCSTA2bits.CREN = 1;
  }

  if(!uart2_status.buffer_rx_full)
  {
    if(RCSTA2bits.FERR)	// Frame error
    {
      chTemp = Read2USART();
      uart2_status.buffer_rx_error_frame = 1;
      return 0;
    }

    chTemp = Read2USART();

    // if I'm in rs485 mode and I sent something then I wont read it from
    // buffer (no loopback)
    if(uart2_rs485_enable && uart2_rs485_tx_count)
    {
      uart2_rs485_tx_count--;
      return 0;
    }

    //uart2_status.buffer_rx_overflow = 0;
    uart2_status.buffer_rx_empty = 0;
    uart2_buffer_rx[uart2_buffer_rx_wr_ptr] = chTemp;
    uart2_buffer_rx_data_cnt++;
      
    if(uart2_buffer_rx_data_cnt == UART2_BUFFER_SIZE_RX)
      uart2_status.buffer_rx_full = 1;

    uart2_buffer_rx_wr_ptr++;

    if(uart2_buffer_rx_wr_ptr == UART2_BUFFER_SIZE_RX)
      uart2_buffer_rx_wr_ptr = 0;
  }
  else
  {
    uart2_status.buffer_rx_overflow = 1;
    return 0;
  }

  return 1;
}

/**************************************************
* Function name		: unsigned char uart1_buffer_read(unsigned char *data_read)
*	return			: 0 = buffer empty
*					  1 = data read correctly
*	data_read		: unsigned char pointer where store the data from rx buffer
* Created by		: Luca Lucci
* Date created		: 19/10/12
* Description		: This function reads the data from the receive buffer. It
*					  places the data in to argument and updates the data count
*					  and read pointer
* Notes				: In this function the interrupt will be disabled, so it 
*					  can possible keeps the access pointer values proper.
**************************************************/
unsigned char uart1_buffer_read(unsigned char *data_read)
{
  // read position by the application
  static unsigned int uart1_buffer_rx_rd_ptr = 0;	

  if(uart1_status.buffer_rx_empty)
    return 0;
  
  // critical code: diabling intrrupts here keeps the access pointer values 
  // proper
  if(UART1_INTERRUPT_RX)
    PIE1bits.RC1IE = 0;

  uart1_status.buffer_rx_full = 0;

  *data_read = uart1_buffer_rx[uart1_buffer_rx_rd_ptr];

  uart1_buffer_rx_data_cnt--;

  if(uart1_buffer_rx_data_cnt == 0)
    uart1_status.buffer_rx_empty = 1;

  uart1_buffer_rx_rd_ptr++;

  if(uart1_buffer_rx_rd_ptr == UART1_BUFFER_SIZE_RX)
    uart1_buffer_rx_rd_ptr = 0;

  if(UART1_INTERRUPT_RX)
    PIE1bits.RC1IE = 1;

  return 1;
}

/**************************************************
* Function name		: unsigned char uart2_buffer_read(unsigned char *data_read)
*	return          : 0 = buffer empty
*                         1 = data read correctly
*	data_read	: unsigned char pointer where store the data from rx buffer
* Created by		: Luca Lucci
* Date created		: 19/10/12
* Description		: This function reads the data from the receive buffer. It
*			  places the data in to argument and updates the data count
*			  and read pointer
* Notes			: In this function the interrupt will be disabled, so it 
*                         can possible keeps the access pointer values proper.
**************************************************/
unsigned char uart2_buffer_read(unsigned char *data_read)
{
  if(uart2_status.buffer_rx_empty)
    return 0;

  // critical code: diabling intrrupts here keeps the access pointer values 
  // proper
if(UART2_INTERRUPT_RX)
  PIE3bits.RC2IE = 0;

  uart2_status.buffer_rx_full = 0;

  *data_read = uart2_buffer_rx[uart2_buffer_rx_rd_ptr];

  uart2_buffer_rx_data_cnt--;

  if(uart2_buffer_rx_data_cnt == 0)
    uart2_status.buffer_rx_empty = 1;

  uart2_buffer_rx_rd_ptr++;

  if(uart2_buffer_rx_rd_ptr == UART2_BUFFER_SIZE_RX)
    uart2_buffer_rx_rd_ptr = 0;

if(UART2_INTERRUPT_RX)
  PIE3bits.RC2IE = 1;

  return 1;
}

//TODO: create uart1_buffer_read_filtered
/**************************************************
* Function name		: unsigned char uart2_buffer_read(char *data, char token)
*	return          : 0 = buffer empty
*                         # of bytes read = data read correctly
*	data            : unsigned char pointer where store the data from rx buffer
*       tocke           : end of frame character
* Created by		: Luca Lucci
* Date created		: 17/07/14
* Description		: This function search for tocken into rx buffer and
*                         store the frame into data.
* Notes			: In this function the interrupt will be disabled, so it
*                         can possible keeps the access pointer values proper.
*
*                         Warning! The null character can't be accepted due to
*                         strchr function that it will be parse it as end of
*                         string
**************************************************/
int uart2_buffer_read_filtered(char *data, char token)
{
  int length_to_write = 0;
  char uart2_buffer_rx_temp[UART2_BUFFER_SIZE_RX];
  char *token_ptr;
  int null_check_index = 0;

  if(uart2_status.buffer_rx_empty)
    return 0;

  // if it doesn't roll up then it copy message into temp buffer
  // else it copy the last part of the buffer and the first one until data
  // length
  if(uart2_buffer_rx_rd_ptr < uart2_buffer_rx_wr_ptr)
  {
    // it checks for null character into the string
    // and replace it with 0x01 character.
    for(null_check_index = uart2_buffer_rx_rd_ptr; null_check_index < (uart2_buffer_rx_wr_ptr + 1); null_check_index++)
    {
      if(uart2_buffer_rx[null_check_index] == '\0')
        uart2_buffer_rx[null_check_index] = 1;
    }

    length_to_write = uart2_buffer_rx_wr_ptr - uart2_buffer_rx_rd_ptr;
    memcpy(uart2_buffer_rx_temp, &uart2_buffer_rx[uart2_buffer_rx_rd_ptr], length_to_write);
    uart2_buffer_rx_temp[length_to_write] = '\0';
  }
  else
  {
    // it checks for null character into the string
    // and replace it with 0x01 character.
    for(null_check_index = uart2_buffer_rx_rd_ptr; null_check_index < UART2_BUFFER_SIZE_RX; null_check_index++)
    {
      if(uart2_buffer_rx[null_check_index] == '\0')
        uart2_buffer_rx[null_check_index] = 1;
    }

    for(null_check_index = 0; null_check_index < (uart2_buffer_rx_wr_ptr + 1); null_check_index++)
    {
      if(uart2_buffer_rx[null_check_index] == '\0')
        uart2_buffer_rx[null_check_index] = 1;
    }

    length_to_write = (UART2_BUFFER_SIZE_RX - uart2_buffer_rx_rd_ptr);
    memcpy(uart2_buffer_rx_temp, &uart2_buffer_rx[uart2_buffer_rx_rd_ptr], length_to_write);
    memcpy(&uart2_buffer_rx_temp[length_to_write], uart2_buffer_rx, uart2_buffer_rx_wr_ptr);

    length_to_write = length_to_write + uart2_buffer_rx_wr_ptr;
    uart2_buffer_rx_temp[length_to_write] = '\0';
  }

  // it search for token
  token_ptr = strchr(uart2_buffer_rx_temp, token);
  
  if(token_ptr == NULL)
  {
    return 0;
  }
  else
    length_to_write = (token_ptr - uart2_buffer_rx_temp + 1);

  // critical code: diabling intrrupts here keeps the access pointer values
  // proper
  if(UART2_INTERRUPT_RX)
    PIE3bits.RC2IE = 0;

  uart2_status.buffer_rx_full = 0;

  memcpy(data, uart2_buffer_rx_temp, length_to_write);
  uart2_buffer_rx_data_cnt -= length_to_write;

  if(uart2_buffer_rx_data_cnt == 0)
    uart2_status.buffer_rx_empty = 1;

  uart2_buffer_rx_rd_ptr += length_to_write;

  if(uart2_buffer_rx_rd_ptr >= UART2_BUFFER_SIZE_RX)
    uart2_buffer_rx_rd_ptr -= UART2_BUFFER_SIZE_RX;

  if(UART2_INTERRUPT_RX)
    PIE3bits.RC2IE = 1;

  return length_to_write;
}

//TODO: create uart1_buffer_read_filtered
//TODO: creare la variabile bookmark anche in uartx_buffer_read_filtered
/**************************************************
* Function name		: int uart2_buffer_read_multifiltered(char *data, char *token, int token_number)
*	return          : 0 = buffer empty
*                         # of bytes read = data read correctly
*	data            : unsigned char pointer where store the data from rx buffer
*       token           : array formed by end of frame character
*       token_number    : element's number (max 10)
* Created by		: Luca Lucci
* Date created		: 16/09/14
* Description		: This function search for tokens into rx buffer and
*                         store the frame into data.
* Notes			: In this function the interrupt will be disabled, so it
*                         can possible keeps the access pointer values proper.
*
*                         Warning! The null character can't be accepted due to
*                         strchr function that it will be parse it as end of
*                         string
**************************************************/
int uart2_buffer_read_multifiltered(char *data, char *token, char token_number)
{
  int length_to_write = 0;
  char uart2_buffer_rx_temp[UART2_BUFFER_SIZE_RX];
  char *token_ptr[10];
  char *token_winner = NULL;
  char *null_character = NULL;
  //int null_check_index = 0;
  int token_count = 0;
  static int bookmark = 0;

  if(uart2_status.buffer_rx_empty)
    return 0;

  if(token_number > 10)
    return -1;

  if((uart2_buffer_rx_rd_ptr + bookmark) >= UART2_BUFFER_SIZE_RX)
    bookmark = bookmark - UART2_BUFFER_SIZE_RX;

  if((uart2_buffer_rx_rd_ptr + bookmark) == uart2_buffer_rx_wr_ptr)
    return 0;
  
  // if it doesn't roll up then it copy message into temp buffer
  // else it copy the last part of the buffer and the first one until data
  // length
  if((uart2_buffer_rx_rd_ptr + bookmark) <= uart2_buffer_rx_wr_ptr)
  {
    length_to_write = uart2_buffer_rx_wr_ptr - (uart2_buffer_rx_rd_ptr  + bookmark);
    memcpy(uart2_buffer_rx_temp, &uart2_buffer_rx[uart2_buffer_rx_rd_ptr + bookmark], length_to_write);
  }
  else
  {
    length_to_write = (UART2_BUFFER_SIZE_RX - (uart2_buffer_rx_rd_ptr + bookmark));
    memcpy(uart2_buffer_rx_temp, &uart2_buffer_rx[uart2_buffer_rx_rd_ptr + bookmark], length_to_write);
    memcpy(&uart2_buffer_rx_temp[length_to_write], uart2_buffer_rx, uart2_buffer_rx_wr_ptr);

    length_to_write = length_to_write + uart2_buffer_rx_wr_ptr;
  }

  uart2_buffer_rx_temp[length_to_write] = '\0';

  // it checks for null character into the string
  // and replace it with 0x01 character.
  null_character = strchr(uart2_buffer_rx_temp, '\0');
  while(null_character != NULL)
  {
    if(null_character < &uart2_buffer_rx_temp[length_to_write])
      *null_character = 1;
    else
      break;

    null_character = strchr(uart2_buffer_rx_temp, '\0');
  }

  // it search for token
  for(token_count = 0; token_count < token_number; token_count++)
  {
    token_ptr[token_count] = strchr(uart2_buffer_rx_temp, token[token_count]);
    
    if(token_ptr[token_count] > token_winner)
      token_winner = token_ptr[token_count];
  }

  if(token_winner == NULL)
  {
    bookmark++;
    return 0;
  }

  bookmark = 0;

  // critical code: diabling intrrupts here keeps the access pointer values
  // proper
  if(UART2_INTERRUPT_RX)
    PIE3bits.RC2IE = 0;

  uart2_status.buffer_rx_full = 0;

  if(uart2_buffer_rx_rd_ptr < uart2_buffer_rx_wr_ptr)
  {
    length_to_write = uart2_buffer_rx_wr_ptr - uart2_buffer_rx_rd_ptr - (token_winner - uart2_buffer_rx_temp);
    memcpy(data, &uart2_buffer_rx[uart2_buffer_rx_rd_ptr], length_to_write);
  }
  else
  {
    length_to_write = (UART2_BUFFER_SIZE_RX - uart2_buffer_rx_rd_ptr);
    memcpy(data, &uart2_buffer_rx[uart2_buffer_rx_rd_ptr], length_to_write);
    memcpy(&data[length_to_write], uart2_buffer_rx, uart2_buffer_rx_wr_ptr - (token_winner - uart2_buffer_rx_temp));
    length_to_write = length_to_write + uart2_buffer_rx_wr_ptr - (token_winner - uart2_buffer_rx_temp);
  }

  data[length_to_write - 1] = 0;

  uart2_buffer_rx_data_cnt -= length_to_write;

  if(uart2_buffer_rx_data_cnt == 0)
    uart2_status.buffer_rx_empty = 1;

  uart2_buffer_rx_rd_ptr += length_to_write;

  if(uart2_buffer_rx_rd_ptr >= UART2_BUFFER_SIZE_RX)
    uart2_buffer_rx_rd_ptr -= UART2_BUFFER_SIZE_RX;

  if(UART2_INTERRUPT_RX)
    PIE3bits.RC2IE = 1;

  return length_to_write;
}

/**************************************************
* Function name		: unsigned char uart1_get_rx_data_size(void);
*	return			: number of byte to read through uart_buffer_rx_load
* Created by		: Luca Lucci
* Date created		: 19/10/12
* Description		: This function returns the number of bytes of data 
*					  available in receive buffer at the calling time. It helps
					  the user to read data from receive buffer at once.
* Notes				: -
**************************************************/
unsigned char uart1_get_rx_data_size(void)
{
  return uart1_buffer_rx_data_cnt;
}

/**************************************************
* Function name		: unsigned char uart2_get_rx_data_size(void);
*	return			: number of byte to read through uart_buffer_rx_load
* Created by		: Luca Lucci
* Date created		: 19/10/12
* Description		: This function returns the number of bytes of data 
*					  available in receive buffer at the calling time. It helps
*					  the user to read data from receive buffer at once.
* Notes				: -
**************************************************/
unsigned char uart2_get_rx_data_size(void)
{
  return uart2_buffer_rx_data_cnt;
}

/**************************************************
* Function name		: unsigned int uart1_buad_rate_set(unsigned long baud_rate, unsigned char freq_MHz)
*	return			: spbrg value
*	baud_rate		: desired baud_rate
*	freq_MHz		: working frequency that includes pll
* Created by		: Luca Lucci
* Date created		: 09/11/12
* Description		: This function calculate the spbrg value for uart to pass 
*					  to uart_open() function.
* Notes				: -
**************************************************/
unsigned int uart1_buad_rate_set(unsigned long baud_rate, long freq_Hz)
{
  unsigned char configuration_bit;	//make word as table 20-1

  configuration_bit = ((UART1_CONFIG & 0b00010000) >> 0x04) |
                      ((UART1_BAUD_CONFIG & 0b00001000) >> 0x02);

  switch(configuration_bit)
  {
    case 0:
      return ((freq_Hz / (baud_rate) / 64)) - 1;

    case 1:
    case 2:
      return ((freq_Hz / (baud_rate) / 16)) - 1;

    case 3:
      return ((freq_Hz / (baud_rate) / 4)) - 1;
	
    default:
      return 0xFF;	// return the minimum baud rate
  }
}

/**************************************************
* Function name		: unsigned int uart2_buad_rate_set(unsigned int baud_rate, unsigned char freq_MHz)
*	return			: spbrg value
*	baud_rate		: desired baud_rate
*	freq_MHz		: working frequency that includes pll
* Created by		: Luca Lucci
* Date created		: 09/11/12
* Description		: This function calculate the spbrg value for uart to pass 
*					  to uart_open() function.
* Notes				: -
**************************************************/
unsigned int uart2_buad_rate_set(unsigned long baud_rate, long freq_Hz)
{
  unsigned char configuration_bit;	//make word as table 20-1

  configuration_bit = ((UART2_CONFIG & 0b00010000) >> 0x04) |
                      ((UART2_BAUD_CONFIG & 0b00001000) >> 0x02);

  switch(configuration_bit)
  {
    case 0:
      return ((unsigned int)(freq_Hz / baud_rate) >> 6) - 1;

    case 1:
    case 2:
      return ((unsigned int)(freq_Hz / baud_rate) >> 4) - 1;

    case 3:
      return ((unsigned int)(freq_Hz / baud_rate) >> 2) - 1;
	
    default:
      return 0xFF;	// return the minimum baud rate
  }
}

/**************************************************
* Function name		: void uart1_isr(void)
*
* Created by		: Luca Lucci
* Date created		: 19/10/12
* Description		: This is the interrupt service routine which is called in
*			  the user application's ISR portion. This function
*			  actually sends the data from transmit buffer to USART and
*			  updates the data count and read pointer variables of 
*			  transmit buffer. For the receiver portion, it reads the data
*			  from USART and places the data into receive buffer (if no
*			  error accured) and updates data count and write pointer 
*			  variables of receive buffer. error flag is set if the 
*			  receive buffer is full  and it receives more data. If frame
*			  errors (FERR) occur it sets the error flag. If over flow
*			  error (OERR) accurs, it clears and sets the CREN bit, so
*			  that USART can receive further data.
* Notes				: -
**************************************************/
void uart1_isr(void)
{
  int interrupt_enable_rx = PIE3bits.RC2IE;
  int interrupt_enable_tx = PIE3bits.TX2IE;

  if(interrupt_enable_rx)
    PIE1bits.RC1IE = 0;

  if(interrupt_enable_tx)
    PIE1bits.TX1IE = 0;

  // For the transmitter

  if(UART1_INTERRUPT_TX)
  {
    if(PIR1bits.TX1IF && PIE1bits.TX1IE)
      uart1_buffer_send();
  }

  // For the receiver

  if (UART1_INTERRUPT_RX)
  {
    if(PIR1bits.RC1IF && PIE1bits.RC1IE)
      uart1_buffer_rx_load();
  }

  if(interrupt_enable_rx)
    PIE1bits.RC1IE = 1;

  if(interrupt_enable_tx)
    PIE1bits.TX1IE = 1;
}

/**************************************************
* Function name		: void uart2_isr(void)
*
* Created by		: Luca Lucci
* Date created		: 19/10/12
* Description		: This is the interrupt service routine which is called in
*			  the user application's ISR portion. This function 
*			  actually sends the data from transmit buffer to USART and
*			  updates the data count and read pointer variables of 
*			  transmit buffer. For the recive portion, it reads the data
*			  from USART and places the data into receive buffer (if no
*			  error accured) and updates data count and write pointer 
*			  variables of receive buffer. error flag is set if the 
*			  receive buffer is full  and it receives more data. If frame
*			  errors (FERR) occur it sets the error flag. If over flow
*			  error (OERR) accurs, it clears and sets the CREN bit, so
*			  that USART can receive further data.
* Notes				: -
**************************************************/
void uart2_isr(void)
{
  int interrupt_enable_rx = PIE3bits.RC2IE;
  int interrupt_enable_tx = PIE3bits.TX2IE;

  if(interrupt_enable_rx)
    PIE3bits.RC2IE = 0;

  if(interrupt_enable_tx)
    PIE3bits.TX2IE = 0;
  
  // For the transmitter

  if(UART2_INTERRUPT_TX)
  {
    if(PIR3bits.TX2IF && interrupt_enable_tx)
      uart2_buffer_send();
  }

  // For the receiver

  if(UART2_INTERRUPT_RX)
  {
    if(PIR3bits.RC2IF && interrupt_enable_rx)
      uart2_buffer_rx_load();
  }
  
  if(interrupt_enable_rx)
    PIE3bits.RC2IE = 1;

  if(interrupt_enable_tx)
    PIE3bits.TX2IE = 1;
}

/**************************************************
* Function name		: void uart1_tsr_poll(void)
*
* Created by		: Luca Lucci
* Date created		: 22/01/13
* Description		: This function update the rs485 tx control flag that control
*					  transmission
* Notes				: This should be called in the main loop
**************************************************/
void uart1_tsr_poll(void)
{
  if(TXSTA1bits.TRMT)
    uart1_rs485_tx_enable = 0;
}

/**************************************************
* Function name		: void uart2_tsr_poll(void)
*
* Created by		: Luca Lucci
* Date created		: 22/01/13
* Description		:  This function update the rs485 tx control flag that control
*			  transmission
* Notes			: This should be called in the main loop
***************************************************/
void uart2_tsr_poll(void)
{
  if(TXSTA2bits.TRMT)
  {
#ifdef uart2_rs485_tx_enable
    uart2_rs485_tx_enable = 0;
#endif
  }
}

/**************************************************
* Function name		: unsigned char uart1_error_handle()
* 	return			: 0 - no error occurred
*					  x - error code
* Created by		: Luca Lucci
* Date created		: 27/03/13
* Description		: This function check if there has been error in uart 
*					  communication and return error code
* Notes				: Only one error will be passed each time, so user must be
*					  poll this function until it returns 0. The if position
*					  decides the priority of the error message.
**************************************************/
unsigned char uart1_error_handle()
{
  if(uart1_status.buffer_rx_overflow)
  {
    uart1_status.buffer_rx_overflow = 0;
    return BUFFER_RX_OVERFLOW;
  }
  else if(uart1_status.buffer_rx_micro_overflow)
  {
    uart1_status.buffer_rx_micro_overflow = 0;
    return BUFFER_RX_MICRO_OVERFLOW;
  }
  else if(uart1_status.buffer_rx_error_frame)
  {
	uart1_status.buffer_rx_error_frame = 0;
    return FRAME_ERROR;
  }

  return 0;
}

/**************************************************
* Function name		: unsigned char uart1_error_translate(unsigned char *error_message, unsigned char code, unsigned char *length)
* 	return			: 0 - no error occurred
*					  1 - error message loaded
*	buffer			: pointer to a buffer where store the error message
*   code			: error code
*   length			: char where to store the length of the message 
* Created by		: Luca Lucci
* Date created		: 27/03/13
* Description		: This function load the error message into the buffer
*					  passed as param. Also return the length of the message 
*					  loaded.
* Notes				: The minimum buffer must be 30 byte
**************************************************/
unsigned char uart1_error_translate(unsigned char *error_message, unsigned char code, unsigned char *length)
{
  switch(code)
  {
    case BUFFER_RX_OVERFLOW:
      *length = sprintf(error_message, "Uart Overrun\r\n");
      break;

    case FRAME_ERROR:
      *length = sprintf(error_message, "Uart Frame error\r\n");
      break;

    default:
      return 0;
  }

  return 1;
}

/**************************************************
* Function name		: unsigned char uart2_error_handle()
* 	return			: 0 - no error occurred
*					  x - error code
* Created by		: Luca Lucci
* Date created		: 27/03/13
* Description		: This function check if there has been error in uart 
*					  communication and return error code.
* Notes				: Only one error will be passed each time, so user must be
*                                 poll this function until it returns 0. The if position
*                                 decides the priority of the error message.
**************************************************/
unsigned char uart2_error_handle()
{
  if(uart2_status.buffer_rx_overflow)
  {
    uart2_status.buffer_rx_overflow = 0;
    return BUFFER_RX_OVERFLOW;
  }
  else if(uart2_status.buffer_rx_micro_overflow)
  {
    uart2_status.buffer_rx_micro_overflow = 0;
    return BUFFER_RX_MICRO_OVERFLOW;
  }
  else if(uart2_status.buffer_rx_error_frame)
  {
    uart2_status.buffer_rx_error_frame = 0;
    return FRAME_ERROR;
  }

  return 0;
}

/**************************************************
* Function name		: unsigned char uart2_error_translate(unsigned char *error_message, unsigned char code, unsigned char *length)
* 	return			: 0 - no error occurred
*					  1 - error message loaded
*	buffer			: pointer to a buffer where store the error message
*   code			: error code
*   length			: char where to store the length of the message 
* Created by		: Luca Lucci
* Date created		: 27/03/13
* Description		: This function load the error message into the buffer
*					  passed as param. Also return the length of the message 
*					  loaded.
* Notes				: The minimum buffer must be 30 byte
**************************************************/
unsigned char uart2_error_translate(unsigned char *error_message, unsigned char code, unsigned char *length)
{
  switch(code)
  {
    case BUFFER_RX_OVERFLOW:
      *length = sprintf(error_message, "Uart Overrun\r\n");
      break;

    case FRAME_ERROR:
      *length = sprintf(error_message, "Uart Frame error\r\n");
      break;

    default:
      return 0;
  }

  return 1;
}