/***************************************************
* FileName:        io_cfg.h
* Dependencies:    See INCLUDES section below
* Processor:       PIC18
* Compiler:        XC18 1.21
* Company:         SpinItalia s.r.l.
* 
* All Rights Reserved.
*
*  The information contained herein is confidential 
* property of SpinItalia s.r.l. The user, copying, transfer or 
* disclosure of such information is prohibited except
* by express written agreement with SpinItalia s.r.l.
*
* First written on 08/07/2014 by Luca Lucci.
*
* Module Description:
* Here it's defined the hardware configuration
*
****************************************************/
/**
 * \file
 * Configurazione hardware del microcontrollore.
 */

#ifndef _IO_CFG_H
#define _IO_CFG_H

/** I N C L U D E S ********************************/

/** O S C I L L A T O R  ***************************/
#define FOSC_MHZ	16000000

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 16000000
#endif
 
/** T R I S         ********************************/
#define INPUT_PIN 1
#define OUTPUT_PIN 0

/** I 2 C           ********************************/
#define i2c_sck_tris TRISCbits.TRISC3
#define i2c_sda_tris TRISCbits.TRISC4

/** S P I           ********************************/
#define spi_cs_tris TRISDbits.TRISD3
#define spi_sck_tris TRISDbits.TRISD6
#define spi_sdo_tris TRISDbits.TRISD4
#define spi_sdi_tris TRISDbits.TRISD5

#define spi_cs LATDbits.LATD3

/** U S A R T       ********************************/
#define uart1_tx_tris TRISCbits.TRISC6
#define uart1_rx_tris TRISCbits.TRISC7
#define uart1_rs485_tx_enable_tris TRISDbits.TRISD7
#define uart1_rs485_tx_enable LATDbits.LATD7

#define uart2_tx_tris TRISGbits.TRISG1
#define uart2_rx_tris TRISGbits.TRISG2

/** ADC             ********************************/
#define ADC_RESOLUTION_BIT 12
#define ADC_REFERENCE 2.048
#define BATTERY_PART 2

#define battery_an_tris TRISAbits.TRISA0
#define battery_an_analog ANCON0bits.ANSEL0
#define battery_an      PORTAbits.AN0

#define temp_an_tris TRISAbits.TRISA1
#define temp_an_analog ANCON0bits.ANSEL1
#define temp_an      PORTAbits.AN1

#define ext1_an_tris TRISAbits.TRISA2
#define ext1_an_analog ANCON0bits.ANSEL2
#define ext1_an      PORTAbits.AN2

#define ext2_an_tris TRISFbits.TRISF7
#define ext2_an_analog ANCON0bits.ANSEL5
#define ext2_an      PORTFbits.AN5

#define ext_shut_an_tris TRISAbits.TRISA5
#define ext_shut_an_analog ANCON0bits.ANSEL4
#define ext_shut      PORTAbits.RA5

/** L E D           ********************************/
#define led_err_tris TRISEbits.TRISE4
#define led_err      LATEbits.LATE4
#define led_no_err_tris TRISEbits.TRISE5
#define led_no_err   LATEbits.LE5

/** B U T T O N     ********************************/

/** C O N T R O L    ********************************/
#define alim_3_3V_enable_tris TRISFbits.TRISF3
#define alim_3_3V_enable LATFbits.LF3

#define charger_ce_tris TRISGbits.TRISG3
#define charger_prg_tris TRISGbits.TRISG4
#define charger_prg LATGbits.LATG4

/** R N 1 3 1    ************************************/
#define rn131_rts_tris TRISBbits.TRISB2
#define rn131_rts PORTBbits.RB2
#define rn131_int_edge INTCON2bits.INTEDG2
#define rn131_int_enable INTCON3bits.INT2IE
#define rn131_int_flag INTCON3bits.INT2F

/** B A T T  A N A L O G  ***************************/
#define adc_batt_enable_tris TRISFbits.TRISF2
#define adc_batt_enable LATFbits.LATF2

#endif // _IO_CFG_H
