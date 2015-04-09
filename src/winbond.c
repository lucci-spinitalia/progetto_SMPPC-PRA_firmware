/**
* \file
* Collezione di funzioni per la gestione della eeprom winbond.
*/

#include "../include/winbond.h"
#include "../system/io_cfg.h"
#include "spi.h"

#include <timers.h>
#include <stdlib.h>
#include <string.h>

unsigned char flash_at_work = 0; /**< indica quando si stanno compiendo delle operazioni sull'eeprom*/
unsigned long flash_ptr_wr = 0;
unsigned long flash_ptr_send = 0;
unsigned long flash_data_count = 0; /**< indica il numero di byte caricati nella flash */

/**
 * Ottiene il manufacturer id ed il device id.
 *
 * Invia una richiesta "Read Manufacturer / Device ID" (codice 0x90) e legge
 * due byte dalla risposta della eeprom.
 *
 * @param device_id unsigned char dove memorizzare i due byte di id
 * @return 0: successo -1: fallimento
 * @remark Accertarsi che il parametro passato sia un array di almeno 2 byte
 */
int winbond_identification(unsigned char *device_id)
{
  // enable chip
  spi_cs = 0;

  // send "Read Manufacturer / Device ID" instruction (90h)
  if(WriteSPI2(0x90) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  // write address 0x000000
  if(WriteSPI2(0x00) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  if(WriteSPI2(0x00) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  if(WriteSPI2(0x00) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  getsSPI2(device_id, 2);

  // disable chip
  spi_cs = 1;

  return 0;
}

/**
 * Imposta la memoria in modalità power-down.
 *
 * L'ingresso in questa modalità porta ad un consumo di corrente pari a 1/10 di
 * quella di standby.
 *
 * @return 0: successo -1: fallimento
 * @remark Una volta entrati in questa modalità, l'unico comando accettato
 * è "Release from Power Down".
 */
int winbond_powerdown(void)
{
  // enable chip
  spi_cs = 0;

  // send "Power-down" instruction (B9h)
  if(WriteSPI2(0xb9) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  __delay_us(3);
  spi_cs = 1;

  return 0;
}

/**
 * Ripristina il dispositivo in modalità normale partendo dal powerdown.
 *
 * @return 0: successo -1: fallimento
 */
int winbond_release_powerdown(void)
{
  // enable chip
  spi_cs = 0;

  // send "Power-down" instruction (B9h)
  if(WriteSPI2(0xab) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  __delay_us(3);
  spi_cs = 1;

  return 0;
}

/**
 * Invia il messaggio "Write Enable" (codice 0x06) all'eeprom
 *
 * @return 0: successo;  -1: fallimento
 */
int winbond_write_enable(void)
{
  // enable chip
  spi_cs = 0;

  // send "Write Enable" instruction
  if(WriteSPI2(0x06) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  // disable chip
  spi_cs = 1;

  return 0;
}

/**
 * Prova ad abilitare la scrittura sulla eeprom per il numero di tentativi indicati.
 *
 * Invia messaggi di abilitazione di scrittura su eeprom per il numero di tentativi
 * indicati dal parametro passato.
 *
 * @param num_of_try numero di tentativi prima di generare un errore
 *
 * @return integer che indica se l'operazione è avvenuta con successo (0) oppure
 * non è stato possibile abilitare l'eeprom alla scrittura (-1).
 */
int winbond_try_write_enable(unsigned int num_of_try)
{
  int status_register_read_count;

  if(winbond.status_register_1.bits.write_enable == 1)
    return 0;

  // abilita la scrittura
  if(winbond_write_enable() < 0)
    return -1;

  // controllo che la eeprom sia entrata in modalità scrittura. Ha num_of_try tentativi
  for(status_register_read_count = 0; status_register_read_count < num_of_try; status_register_read_count++)
  {
    if(winbond_status_register_read(&winbond.status_register_1.word, 1) < 0)
      return -1;

    if(winbond.status_register_1.bits.write_enable == 1)
      break;
    else
    {
      // abilita la scrittura
      if(winbond_write_enable() < 0)
        return -1;
    }
  }

  if(winbond.status_register_1.bits.write_enable == 1)
    return 0;
  else
    return -1;
}

/**
 * Invia il messaggio "Write Disable" (codice 0x04) all'eeprom
 * @return  0: successo;  -1: fallimento
 */
int winbond_write_disable(void)
{
  // enable chip
  spi_cs = 0;

  // send "Write Disable" instruction
  if(WriteSPI2(0x04) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  // disable chip
  spi_cs = 1;

  return 0;
}

/**
 * Prova a disabilitare la scrittura sulla eeprom per il numero di tentativi indicati.
 *
 * Invia messaggi di disabilitazione alla scrittura su eeprom per il numero di tentativi
 * indicati dal parametro passato.
 *
 * @param num_of_try numero di tentativi prima di generare un errore
 *
 * @return integer che indica se l'operazione è avvenuta con successo (0) oppure
 * non è stato possibile abilitare l'eeprom alla scrittura (-1).
 */
int winbond_try_write_disable(unsigned int num_of_try)
{
  int status_register_read_count;

  if(winbond.status_register_1.bits.write_enable == 0)
    return 0;

  // abilita la scrittura
  if(winbond_write_disable() < 0)
    return -1;

  // controllo che la eeprom sia entrata in modalità scrittura. Ha num_of_try tentativi
  for(status_register_read_count = 0; status_register_read_count < num_of_try; status_register_read_count++)
  {
    if(winbond_status_register_read(&winbond.status_register_1.word, 1) < 0)
      return -1;

    if(winbond.status_register_1.bits.write_enable == 0)
      break;
    else
    {
      // abilita la scrittura
      if(winbond_write_disable() < 0)
        return -1;
    }
  }

  if(winbond.status_register_1.bits.write_enable == 0)
    return 0;
  else
    return -1;
}


/**
 * Cancella un settore dell'eeprom.
 *
 * Dopo aver abilitato la scrittura sull'eeprom, viene inviato un messaggio
 * "Sector Erase" all'indirizzo passato come parametro. Appena viene disabilitato
 * il chip select, la eeprom inizia a cancellare i dati; in questo caso si fa un
 * polling sul bit BUSY del registro di stato 1 intervallato del tempo tipico di
 * formattazione (30ms), in modo da capire quando il lavoro è stato concluso. Il
 * tempo massimo per cancellare un settore è di 400 ms, mentre la funzione attende
 * fino a 600 ms.
 *
 * @param address indirizzo del settore da cancellare
 * @return 0: successo -1: fallimento
 * @remark Il flag write_enable viene automaticamente azzerato quando l'operazione
 * di formattazione è conclusa
 */
int winbond_sector_erase(unsigned short long address)
{
  unsigned char address_byte;
  unsigned char try_count;

  if(winbond_try_write_enable(10) < 0)
    return -1;

  // enable chip
  spi_cs = 0;

  // send "Sector Erase" instruction
  if(WriteSPI2(0x20) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  // Il settore inizia sempre con gli ultimi 3 nibble a 0
  address = (unsigned short long)address & (unsigned short long)0xFFF000;
  address_byte = (unsigned char) (address >> 16);
  if(WriteSPI2(address_byte) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  address_byte = (unsigned char) (address >> 8);
  if(WriteSPI2(address_byte) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  address_byte = (unsigned char) address;
  if(WriteSPI2((unsigned char) address) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  // disable chip
  spi_cs = 1;

  // attendo che l'operazione di formattazione sia terminata facendo un polling
  // sul bit BUSY dello status_register 1. Da datasheet, il tempo tipico per cancellare
  // un settore è di 30 ms, mentre quello massimo è di 400 ms. Controllo il registro
  // ad intervalli di 30 ms per 20 volte
  for(try_count = 0; try_count < 20; try_count++)
  {
    winbond_status_register_read(&winbond.status_register_1.word, 1);

    if((winbond.status_register_1.bits.busy == 0) &&
       (winbond.status_register_1.bits.write_enable == 0))
      return 0;
    else
    {
      __delay_ms(10);
      __delay_ms(10);
      __delay_ms(10);
    }
  }

  return -1;
}

/**
 * Legge il registro di stato passato come parametro.
 *
 * Invia una richiesta di lettura per il registro di stato (codice 0x05 per il
 * registro 1 e codice 0x35 per il registro 2) e legge il byte restituito.
 *
 * @param status_register unsigned char dove memorizzare il registro di stato ottenuto
 * @param status_register_num Il registro di stato da leggere
 * @return 0: successo -1: fallimento
 * @remark Il numero del registro di stato deve essere 1 o 2. Valori differenti
 * genereranno un errore
 */
int winbond_status_register_read(unsigned char *status_register, unsigned char status_register_num)
{
  char instruction;

  // I registri sono solo 2
  if ((status_register_num != 1) && (status_register_num != 2))
    return -1;

  if(status_register_num == 1)
    instruction = 0x05;
  else
    instruction = 0x35;

  // enable chip
  spi_cs = 0;

  // send "Read Status Register" instruction
  if(WriteSPI2(instruction) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  getsSPI2(status_register, 1);

  // disable chip
  spi_cs = 1;

  return 0;
}

/**
 * Scrive una serie di byte sull'indirizzo passato.
 *
 * Dopo aver verificato che tutti i parametri passati siano validi ed aver abilitato
 * la scrittura, la funzione invia un comando di "Page Program".
 *
 * @param data array contenente i dati da inviare
 * @param data_bytes numero di byte da inviare
 * @param address indirizzo della locazione di memoria da scrivere
 * @return 0: successo -1: fallimento
 * @remark Si da per scontato che la locazione di memoria su cui si va a scrivere
 * sia stata precedentemente cancellata (i valori dei byte che contiene devono
 * essere tutti a 0xFF). Inoltre si deve sempre tenere a mente che, se i byte che
 * vengono scritti in modo sequenziale superano la dimensione della pagina, allora
 * i restanti verranno scritti all'inizio della stessa. Per evitare questo tipo
 * di errore, quando avviene un simile evento verrà generato un errore senza che
 * sia stato scritto nulla sulla memoria.
 */
int winbond_page_program(unsigned char *data, int data_bytes, unsigned short long address)
{
  unsigned char try_count;

  unsigned char address_sector;
  unsigned char address_byte;
  int data_bytes_count;

  // La dimensione di una pagina è di 256 byte
  if((data_bytes > 256) || (data_bytes < 1))
    return -1;

  // Se il numero di byte che devono essere scritti superano la dimensione della
  // pagina che rimane partendo da address, allora viene generato un errore
  address_sector = (unsigned char)address; //il byte meno significativo indica a che punto parto nella pagina

  if((address_sector + data_bytes) > 256)
    return -1;

  if(winbond_try_write_enable(10) < 0)
    return -1;

  // enable chip
  spi_cs = 0;

  // send "Page Program" instruction
  if(WriteSPI2(0x02) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  address_byte = (unsigned char) (address >> 16);
  if(WriteSPI2(address_byte) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  address_byte = (unsigned char) (address >> 8);
  if(WriteSPI2(address_byte) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  address_byte = (unsigned char) address;
  if(WriteSPI2((unsigned char) address) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  for(data_bytes_count = 0; data_bytes_count < data_bytes; data_bytes_count++)
  {
    if(WriteSPI2(data[data_bytes_count]) == -1)
    {
      spi_cs = 1;
      return -1;
    }
  }

  spi_cs = 1;

  // attendo che l'operazione di scrittura sia terminata facendo un polling
  // sul bit BUSY dello status_register 1. Da datasheet, il tempo tipico per scrivere
  // una pagina è di 0.7 ms, mentre quello massimo è di 30 ms. Controllo il registro
  // ad intervalli di 0.7 ms per 5 volte
  for(try_count = 0; try_count < 5; try_count++)
  {
    winbond_status_register_read(&winbond.status_register_1.word, 1);

    if((winbond.status_register_1.bits.busy == 0) &&
       (winbond.status_register_1.bits.write_enable == 0))
      return 0;
    else
      __delay_us(700);
  }

  return -1;
}


/**
 * Legge dalla eeprom un numero determinato di byte.
 *
 * Invia alla eeprom un messaggio "Read Data" (codice 0x03) con l'indirizzo da
 * cui iniziare a leggere, per poi memorizzare in data il numero di byte richiesti
 *
 * @param data buffer sul quale scrivere i dati letti
 * @param data_bytes numero di byte da leggere
 * @param address indirizzo di partenza dal quale leggere i dati
 * @return 0: successo -1: fallimento
 * @remark Da datasheet la frequenza massima per la lettura dei dati è di 33 MHz.
 * Assicurarsi di rispettare la specifica. Accertarsi anche che il buffer passato
 * sia abbastanza capiente da contenere il numero di byte richiesto.
 */
int winbond_data_read(unsigned char *data, unsigned short long data_bytes, unsigned short long address)
{
  unsigned char address_byte;
  uldiv_t block_to_read;
  int byte_count;

  // verifico che il chip non sia occupato
  winbond_status_register_read(&winbond.status_register_1.word, 1);

  if(winbond.status_register_1.bits.busy == 1)
    return -1;

  if(data_bytes <= 0)
    return 0;
  
  // enable chip
  spi_cs = 0;

  // send "Read Data" instruction
  if(WriteSPI2(0x03) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  address_byte = (unsigned char) (address >> 16);
  if(WriteSPI2(address_byte) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  address_byte = (unsigned char) (address >> 8);
  if(WriteSPI2(address_byte) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  address_byte = (unsigned char) address;
  if(WriteSPI2((unsigned char) address) == -1)
  {
    spi_cs = 1;
    return -1;
  }

  if(data_bytes > 255)
  {
    block_to_read = uldiv(data_bytes, 255);

    for(byte_count = 0; byte_count < block_to_read.quot; byte_count++)
      getsSPI2(&data[byte_count * 255], 255);

    getsSPI2(&data[block_to_read.quot * 255], block_to_read.rem);
  }
  else
    getsSPI2(data, data_bytes);

  // disable chip
  spi_cs = 1;

  return 0;
}

/**
 * Spazio ancora disponibile per scrivere sull'eeprom.
 *
 * Restituisce lo spazio disponibile, escludendo le locazioni che non sono state
 * ancora inviate.
 *
 * @return unsigned short long rappresentante lo spazio libero sull'eeprom
 */
unsigned long winbond_empty_space(void)
{
  if(flash_data_count < EEPROM_SIZE)
    return(EEPROM_SIZE - flash_data_count);
  else
    return 0;
}

/**
 * Scrive i dati sull'eeprom aggiornando il buffer circolare.
 *
 * Prima di eseguire la scrittura, vengono eseguite delle operazioni di controllo,
 * come il test sullo spazio disponibile, oppure se il settore che sta per essere
 * utilizzato è stato formattato. Inoltre questa funzione si cura di suddividere
 * in pagine il dato passato, facendo attenzione a non superare il limite di fine
 * pagina anche quando il puntatore circolare inizia a metà.
 *
 * @param data array di byte da inviare
 * @param data_bytes numero di byte da inviare
 * @return 0: successo -1: errore di collisione -2: eeprom piena -3: dato inconsistente
 * @remark La funzione si occupa di cancellare il settore su cui si va
 * a scrivere, a patto che il puntatore passi almeno una volta sulla prima locazione
 * dello stesso. Visto che i dati sono scritti a multipli di pagina, l'unica
 * condizione per cui la suddenta condizione non viene verificata è quando il
 * puntatore ::flash_ptr_wr viene inizializzato nel mezzo di un settore che
 * non è stato precedentemente formattato. Questo potrebbe portare ad un'errata
 * scrittura dei dati.
 * Il numero massimo di dati che possono essere scritti con una singola chiamata
 * della funzione sono pari alla dimensione dell'int, ovvero 2^16, in quanto questo
 * è il tipo del parametro d'ingresso che indica il numero di byte da scrivere.
 */
int winbond_data_load(unsigned char *data, int data_bytes)
{
  int process_return = 0;
  unsigned short long page_end_address;
  unsigned short long sector_address;
  unsigned short long data_count = 0; /**< numero di elementi che sono stati scritti tra quelli passati alla funzione */
  int data_to_write = 0;
  int data_to_check = 0; /**< numero di byte da verificare dopo la scrittura */
  int data_checked = 0;  /**< numero di byte verificati */
  int empty_space = 0;
  unsigned char data_check[64];

  // imposto il flag di operazione su memeoria che consente di bloccare tutte
  // le istruzioni di sleep
  flash_at_work = 1;

  // controllo che il numero di byte da scrivere sia compatibile con la memoria
  // che mi rimane a disposizione
  if(winbond_empty_space() < data_bytes)
  {
    flash_at_work = 0;
    return -2;
  }

  /*  Scrivo i dati una pagina alla volta.
   * Bisogna fare attenzione che:
   *   - il dato da scrivere non superi la dimensione della pagina, altrimenti
   *     verranno sovrascritti i dati all'inizio della stessa.
   *   - quando viene cambiata pagina non si cambi settore: in caso contrario
   *     bisogna prima formattarlo
   *   - non si sia superata l'ultima pagina: in questo caso bisogno iniziare
   *     dalla locazione 0x000000
  */

  while(data_count != data_bytes)
  {
    // verifico se sto iniziando un nuovo settore. In questo caso devo prima
    // formattarlo
    sector_address = flash_ptr_wr & 0xFFF000;
    if(flash_ptr_wr == sector_address)
    {
      process_return = winbond_sector_erase(sector_address);
      if(process_return < -1)
      {
        flash_at_work = 0;
        return -1;
      }
    }

    /* Il problema nella scrittura è che non è possibile passare
     * direttamente alla prossima pagina.
     * Per capire quanti byte devo scrivere su una pagina per evitare il rollup,
     * basta sottrarre l'indirizzo di fine pagina a quello corrente.
     */
    // Per capire quanto spazio mi rimane da scrivere su una determinata pagina
    page_end_address = (flash_ptr_wr & 0xFFFF00) | 0x0000FF;
    empty_space = page_end_address - flash_ptr_wr + 1;

    if((data_bytes - data_count) >= empty_space)
      data_to_write = empty_space;
    else
      data_to_write = data_bytes - data_count;
    
    // inizializzo la variabile data_to_check per non sforare l'array data_check
    if((data_to_write - data_checked) < 64)
      data_to_check = data_to_write;
    else
      data_to_check = 64;

    if(winbond_page_program(&data[data_count], data_to_write, flash_ptr_wr) > -1)
    {
      while(data_checked < data_to_write)
      {
        if(winbond_data_read(data_check, data_to_check, flash_ptr_wr) == 0)
        {
          data_checked += data_to_check;

          if(memcmp(&data[data_count], data_check, data_to_write) == 0)
          {
            // inizializzo la variabile data_to_check per non sforare l'array data_check
            if((data_to_write - data_checked) < 64)
              data_to_check = data_to_write;
            else
              data_to_check = 64;

            data_count += data_to_write;
            flash_ptr_wr += data_to_write;

            if(flash_ptr_wr == (unsigned short long)EEPROM_SIZE)
              flash_ptr_wr = 0;

            flash_data_count += data_to_write;
          }
          else
          {
            flash_at_work = 0;
            return -3;
          }
        }
      }
    }
    else
    {
      flash_at_work = 0;
      return -1;
    }
  }

  // Risprisino il flag di operazione su memoria
  flash_at_work = 0;

  return 0;
}

/**
 * Legge i dati dalla eeprom e li segna come inviati.
 *
 * @param data buffer dove memorizzare i dati
 * @param max_data_bytes la massima dimensione del buffer
 * @return numero di byte letti dalla eeprom
 */
long winbond_data_send(unsigned char *data, unsigned short long max_data_bytes)
{
  unsigned short long data_to_read;
  unsigned short long data_sent = 0;

  // imposto il flag di operazione su memeoria che consente di bloccare tutte
  // le istruzioni di sleep
  flash_at_work = 1;

  // Se non ci sono dati da inviare, esco
  if(flash_data_count == 0)
    return 0;

  // Confronto il numero di dati da inviare con quello massimo ammissibile.
  // Se è minore, allora leggo tutto quello che c'è, altrimenti ne leggo solo
  // una parte.
  if(max_data_bytes >= flash_data_count)
    data_to_read = flash_data_count;
  else
    data_to_read = max_data_bytes;

  // Nel caso in cui l'indirizzo finale risulti fuori dalla rosa degli indirizzi
  // ammissibili, è necessario tornare all'inizio del buffer
  if((flash_ptr_send + data_to_read) >= EEPROM_SIZE)
  {
    if(winbond_data_read(&data[data_sent], EEPROM_SIZE - flash_ptr_send, flash_ptr_send) > -1)
    {
      data_to_read -= (EEPROM_SIZE - flash_ptr_send);
      data_sent += (EEPROM_SIZE - flash_ptr_send);
    }
    else
    {
      flash_at_work = 0;
      return -1;
    }
  }

  if(winbond_data_read(&data[data_sent], data_to_read, flash_ptr_send) > -1)
    data_sent += data_to_read;
  else
  {
    flash_at_work = 0;
    return -1;
  }

  // Risprisino il flag di operazione su memoria
  flash_at_work = 0;

  return data_sent;
}

/**
 * Aggiorna i puntatori alla flash.
 *
 * Questa funzione deve essere richiamata dopo che il dato precedentemente
 * letto sia stato inviato correttamente.
 *
 * @param data_sent numero di byte con cui incrementare i puntatori.
 */
void update_circular_send_buffer(long data_sent)
{
  if(data_sent <= 0)
    return;
  
  if((flash_ptr_send + data_sent) >= EEPROM_SIZE)
    flash_ptr_send = data_sent - (EEPROM_SIZE - flash_ptr_send);
  else
    flash_ptr_send += data_sent;
  
  flash_data_count -= data_sent;
}