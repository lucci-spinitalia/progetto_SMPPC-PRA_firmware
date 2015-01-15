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

/** P R O T O T Y P E S ****************************/
  void stc3100_get(unsigned char addr , unsigned char reg, unsigned char byte_to_read, unsigned char *stc3100_var);
  void stc3100_write(unsigned char addr , unsigned char reg, unsigned char value);

  unsigned char stc3100_id[8]; /**< contiene l'id del battery gauge */
  float stc3100_soc_mAh; /**< State of charge della batteria in mAh */
  float stc3100_current_value_mA; /**< Corrente misurata dalla batteria */
  float stc3100_voltage_value_mV; /**< Tensione misurata dalla batteria */
  float stc3100_temperature_value; /**< Temperatura misurata della batteria */

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

#endif	/* STC3100_H */

