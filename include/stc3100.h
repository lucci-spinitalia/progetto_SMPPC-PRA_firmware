/* 
 * File:   stc3100.h
 * Author: Luca Lucci
 *
 * Created on 8 gennaio 2015, 16.59
 */

/**
* \file
* Header file del modulo STC3100.
*/
#ifndef STC3100_H
#define	STC3100_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#include "i2c.h"

#define STC3100_ADDR 0x70 /**< Indirizzo del battery gauge */
#define STC3100_MODE 0x00 /**< Registro MODE */
#define STC3100_CTRL 0x01 /**< Registro di controllo e di stato */
#define STC3100_CHARGE_LOW 0x02 /**< LSB per la carica della batteria */
#define STC3100_CHARGE_HIGH 0x03 /**< MSB per la carica della batteria */
#define STC3100_COUNTER_LOW 0x04 /**< LSB per il numero di conversioni */
#define STC3100_COUNTER_HIGH 0x05 /**< MSB per il numero di conversioni */
#define STC3100_CURRENT_LOW 0x06  /**< LSB per il valore di corrente */
#define STC3100_CURRENT_HIGH 0x07 /**< MSB per il valore di corrente */
#define STC3100_VOLTAGE_LOW 0x08 /**< LSB per il valore di tensione della batteria */
#define STC3100_VOLTAGE_HIGH 0x09 /**< MSB per il valore di tensione della batteria */
#define STC3100_TEMPERATURE_LOW 0x0A /**< LSB per il valore di temperatura */
#define STC3100_TEMPERATURE_HIGH 0x0B /**< MSB per il valore di temperatura */
#define STC3100_ID0 0x18 /**< Registro per l'ID */

/** P R O T O T Y P E S ****************************/
  void stc3100_get(unsigned char addr , unsigned char reg, unsigned char byte_to_read, unsigned char *stc3100_var);
  void stc3100_write(unsigned char addr , unsigned char reg, unsigned char value);

  unsigned char stc3100_id[8]; /**< contiene l'id del battery gauge */
  float stc3100_soc_mAh; /**< State of charge della batteria in mAh */
  //float stc3100_current_value_mA; /**< Corrente misurata dalla batteria */
  float stc3100_voltage_value_mV; /**< Tensione misurata dalla batteria */
  //float stc3100_temperature_value; /**< Temperatura misurata della batteria */

  union
  {
    int value[2];
    unsigned char bytes[4];

    struct
    {
      union
      {
        int integer;
        unsigned char bytes[2];
      } charge;

      union
      {
        int integer;
        unsigned char bytes[2];
      } counter;
    } reg;
  } stc3100_charge_state;

  union
  {
    int integer;
    unsigned char bytes[2];
  } stc3100_current;

  union
  {
    int integer;
    unsigned char bytes[2];
  } stc3100_voltage;

  union
  {
    int integer;
    unsigned char bytes[2];
  } stc3100_temperature;

  union
  {
    unsigned char bytes;

    struct
    {
      unsigned io0data :1;
      unsigned gg_rst :1;
      unsigned gg_eoc :1;
      unsigned gg_vtm_eoc :1;
      unsigned pordet :1;
      unsigned unused :3;
    } bits;
  } stc3100_ctrl;
#endif	/* STC3100_H */

