/* 
 * File:   windbond.h
 * Author: Luca Lucci
 *
 * Created on 4 novembre 2014, 16.41
 */

/**
* \file
* Header file del modulo winbond.
*/

#ifndef WINDBOND_H
#define	WINDBOND_H

#ifdef	__cplusplus
extern "C" {
#endif

/** D E F I N E ************************************/
    
/**
 * Capienza della memoria eeprom.
 *
 * Rappresenta la capienza della memoria. Questo vuol dire che, se si dispone di una
 * memoria da 64 Mbit, il buffer degli indirizzi sarà di 23 bit ed andrà da 0 a
 * 0x7FFFFF).
 *
 */
#define EEPROM_SIZE 0x8000000UL

/** S H A R E D  V A R I A B L E S  ****************/
extern unsigned char eeprom_at_work; /**< indica quando si stanno compiendo delle operazioni sull'eeprom*/
extern unsigned long eeprom_ptr_wr;
extern unsigned long eeprom_ptr_send;
extern unsigned long eeprom_data_count;

/** S T R U C T S **********************************/

/**
 * Contiene tutte le informazioni riguardanti la eeprom presente sulla scheda
 */
struct eeprom_struct
{
  unsigned char device_id[2]; /**< manufacturer id e device id del modulo eeprom */

  union
  {
    unsigned char word;

    struct
    {
      unsigned busy:1; /**< Viene impostato quando è in esecuzione una scrittura od una formattazione */
      unsigned write_enable:1; /**< Viene impostato dopo aver eseguito l'istruzione Write Enable */
      unsigned block_protect_bits:3; /**< Indica una protezione alla scrittura */
      unsigned top_bottom_protect:1;
      unsigned sector_block_protect:1; /**< Indica se i bit di protezione si riferiscono ad un settore da 4KB (=1) oppure ad un blocco da 64K (=0) */
      unsigned status_register_protect_0:1; /**< Determina se la protezione è di tipo hardware o software */
    } bits;
  } status_register_1;

  union
  {
    unsigned char word;

    struct
    {
      unsigned status_register_protect_0:1; /**< Determina se la protezione è di tipo hardware o software */
      unsigned quad_enable:1; /**< Indica se è abilitata la modalità quad */
      unsigned riservato:6; /**< Indica una protezione alla scrittura */
    } bits;
  } status_register_2;
} winbond;

/** P R O T O T Y P E S ****************************/
int winbond_identification(unsigned char *device_id);
int winbond_status_register_read(unsigned char *status_register, unsigned char status_register_num);
int winbond_write_enable(void);
int winbond_try_write_enable(unsigned int num_of_try);
int winbond_write_disable(void);
int winbond_try_write_disable(unsigned int num_of_try);
int winbond_sector_erase(unsigned short long address);
int winbond_page_program(unsigned char *data, int data_bytes, unsigned short long address);
int winbond_data_read(unsigned char *data, unsigned short long data_bytes, unsigned short long address);
unsigned long winbond_empty_space(void);
int winbond_data_load(unsigned char *data, int data_bytes);
unsigned short long winbond_data_send(unsigned char *data, int max_data_bytes);
int winbond_powerdown(void);
int winbond_release_powerdown(void);


#ifdef	__cplusplus
}
#endif

#endif	/* WINDBOND_H */

