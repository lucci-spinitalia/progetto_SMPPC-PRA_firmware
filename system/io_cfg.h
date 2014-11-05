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

/** S P I           ********************************/
#define spi_cs1_tris TRISDbits.TRISD3
#define spi_cs2_tris TRISDbits.TRISD2
#define spi_sck_tris TRISDbits.TRISD6
#define spi_sdo_tris TRISDbits.TRISD4
#define spi_sdi_tris TRISDbits.TRISD5

#define spi_cs1 LATDbits.LATD3
#define spi_cs2 LATDbits.LATD2


/** U S A R T       ********************************/
#define uart1_tx_tris TRISCbits.TRISC6
#define uart1_rx_tris TRISCbits.TRISC7
#define uart1_rs485_tx_enable_tris TRISDbits.TRISD4
#define uart1_rs485_tx_enable LATDbits.LATD4

#define uart2_tx_tris TRISGbits.TRISG1
#define uart2_rx_tris TRISGbits.TRISG2
#define uart2_rs485_tx_enable_tris TRISDbits.TRISD5
#define uart2_rs485_tx_enable LATDbits.LATD5

/** ADC             ********************************/
#define ADC_RESOLUTION_BIT 10
#define ADC_REFERENCE 2.5
#define BATTERY_PART 1.680851
#define battery_ref_tris TRISAbits.RA3
#define battery_ref_digital ANCON0bits.PCFG3

#define battery_an_tris TRISAbits.TRISA0
#define battery_an_digital ANCON0bits.PCFG0
#define battery_an      PORTAbits.AN0

#define temp_an_tris TRISAbits.TRISA5
#define temp_an_digital ANCON0bits.PCFG4
#define temp_an      PORTAbits.AN4

/** L E D           ********************************/
#define led_err_tris TRISEbits.TRISE3
#define led_err      LATEbits.LATE3
#define led_no_err_tris TRISAbits.TRISA4
#define led_no_err   LATAbits.LA4

/** B U T T O N     ********************************/

/** C O N T R O L    ********************************/
#define alim_5V_enable_tris TRISFbits.TRISF5
#define alim_5V_enable LATFbits.LF5

#define alim_3_3V_enable_tris TRISFbits.TRISF7
#define alim_3_3V_enable LATFbits.LF7

#define backligth_enable_tris TRISGbits.RG0
#define backligth_enable LATGbits.LG0

#define contrast_cs_tris TRISDbits.RD7
#define contrast_cs LATDbits.LD7

#define contrast_ck_tris TRISDbits.RD0
#define contrast_ck LATDbits.LD0

#define contrast_ud_tris TRISDbits.RD1
#define contrast_ud LATDbits.LD1

#define MCP73811_ce_tris TRISGbits.RG3
#define MCP73811_prg_tris TRISGbits.RG4
#define MCP73811_prg LATGbits.LATG4

#define v_usb_tris TRISBbits.TRISB0
#define v_usb PORTBbits.RB0

/** R N 1 3 1    ************************************/
#define rn131_rts_tris TRISBbits.TRISB2
#define rn131_rts PORTBbits.RB2
#define rn131_int_edge INTCON2bits.INTEDG2
#define rn131_int_enable INTCON3bits.INT2IE
#define rn131_int_flag INTCON3bits.INT2F

/** U S B    ****************************************/
#define usb_d_min_tris TRISFbits.TRISF3
#define usb_d_plus_tris TRISFbits.TRISF4

/** B A T T  A N A L O G  ***************************/
#define adc_batt_tris TRISFbits.TRISF2
#define adc_batt LATFbits.LATF2

/**          ****************************************/
#define csn_tris TRISCbits.RC2
#define csn LATCbits.LATC2

#endif // _IO_CFG_H
