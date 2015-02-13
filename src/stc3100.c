/**
* \file
* Collezione di funzioni per la gestione del battery gauge STC3100.
*/

#include "../include/stc3100.h"
#include "../system/io_cfg.h"

/**
 * Scrive su un registro.
 *
 * @param addr indirizzo i2c del battery gauge
 * @param reg registro da scrivere
 * @param value valore da scrivere nel registro
 */
void stc3100_write(unsigned char addr , unsigned char reg, unsigned char value)
{
  unsigned char i2c_data = 0; /**< Valore letto dalla i2c */
  signed char i2c_status = 0; /**< Stato del bus i2c */

  do
  {
    IdleI2C();
    StartI2C();
    i2c_data = SSPBUF;

    // 7 bit d'Indirizzo del modulo più il bit di scrittura

    i2c_status = WriteI2C((addr << 1));

    if(i2c_status == -1)  // controllo che non ci sia stata collisione
    {
      i2c_data = SSPBUF;   // azzero il buffer
      SSPCON1bits.WCOL = 0;  // azzero il bit indicante la collisione
    }
    else if(i2c_status == -2)
    {
      StopI2C();
      continue;
    }
  } while(i2c_status != 0);

  // Registro da leggere
  do
  {
    i2c_status = WriteI2C(reg);

    if(i2c_status == -1)  // controllo che non ci sia stata collisione
    {
      i2c_data = SSPBUF;   // azzero il buffer
      SSPCON1bits.WCOL = 0;  // azzero il bit indicante la collisione
    }
    else if(i2c_status == -2)
    {
      StopI2C();
      continue;
    }
  } while(i2c_status != 0);

  // Valore da scrivere nel registro
  do
  {
    i2c_status = WriteI2C(value);

    if(i2c_status == -1)  // controllo che non ci sia stata collisione
    {
      i2c_data = SSPBUF;   // azzero il buffer
      SSPCON1bits.WCOL = 0;  // azzero il bit indicante la collisione
    }
    else if(i2c_status == -2)
    {
      StopI2C();
      continue;
    }
  } while(i2c_status != 0);

  NotAckI2C();
  StopI2C();

  while(SSPCON2bits.ACKEN != 0);
}

/**
 * Legge un valore da un registro.
 *
 * @param addr indirizzo i2c del battery gauge
 * @param reg registro da leggere
 * @param byte_to_read numero di byte consecutivi da leggere
 * @param stc3100_var dove memorizzare i dati letti
 *
 * @remark Accertarsi che la variabile dove memorizzare i dati sia abbastanza capiente.
 */
void stc3100_get(unsigned char addr , unsigned char reg, unsigned char byte_to_read, unsigned char *stc3100_var)
{
  unsigned char i2c_data = 0; /**< Valore letto dalla i2c */
  signed char i2c_status = 0; /**< Stato del bus i2c */

  do
  {
    IdleI2C();
    StartI2C();
    i2c_data = SSPBUF;

    // 7 bit d'Indirizzo del modulo più il bit di scrittura

    i2c_status = WriteI2C((addr << 1));

    if(i2c_status == -1)  // controllo che non ci sia stata collisione
    {
      i2c_data = SSPBUF;   // azzero il buffer
      SSPCON1bits.WCOL = 0;  // azzero il bit indicante la collisione
    }
    else if(i2c_status == -2)
    {
      StopI2C();
      continue;
    }
  } while(i2c_status != 0);

  // Registro da leggere
  do
  {
    i2c_status = WriteI2C(reg);

    if(i2c_status == -1)  // controllo che non ci sia stata collisione
    {
      i2c_data = SSPBUF;   // azzero il buffer
      SSPCON1bits.WCOL = 0;  // azzero il bit indicante la collisione
    }
    else if(i2c_status == -2)
    {
      StopI2C();
      continue;
    }
  } while(i2c_status != 0);

  // acknowladge
  RestartI2C();
  i2c_data = SSPBUF;

  // 7 bit d'Indirizzo del modulo più il bit di lettura
  do
  {
    i2c_status = WriteI2C((addr << 1) | 0x01);

    if(i2c_status == -1)  // controllo che non ci sia stata collisione
    {
      i2c_data = SSPBUF;   // azzero il buffer
      SSPCON1bits.WCOL = 0;  // azzero il bit indicante la collisione
    }
    else if(i2c_status == -2)
    {
      StopI2C();
      continue;
    }
  } while(i2c_status != 0);

  // acknowladge
  while(getsI2C(stc3100_var, byte_to_read));

  NotAckI2C();
  StopI2C();

  while(SSPCON2bits.ACKEN != 0);
}