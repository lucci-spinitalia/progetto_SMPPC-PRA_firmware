/**
 * \file main.c
 * Implementa il ciclo principale.
 * 
 * \author Luca Lucci
 * \version 1.0
 * \date 04/08/2014
 */
//#define CONFIGURE_MODE /**< Abilita direttamente il webserver senza interferire con la seriale */

#define USE_OR_MASKS /**< Abilita l'opeartore or per concatenare i parametri del timer1*/

// Le seguenti definizioni contengono l'indirizzo di program memory per salvare
// i puntatori circolari in caso di mancanza di batteria.
#define EEPROM_COUNT_PTR 0x00
#define EEPROM_WRITE_PTR 0x04
#define EEPROM_SEND_PTR 0x08
#define ADC_TIMEOUT_PTR 0x0C
#define DATA_DELAY_PTR 0x10
#define BATT_WARN_PTR 0x14
#define BATT_MIN_PTR 0x18
#define SLEEP_PTR 0x1C
#define CATEGORY_PTR 0x20

#define TIMEOUT_UPDATE_RQST_MS 1000 /**< Periodo di invio delle richieste in centesimi di secondo*/

/**
 * Offset per il Timer1.
 *
 * Il Timer1 scandisce il periodo di aggiornamento delle variabili di sistema.
 * Incrementa con frequenza pari a quella d'istruzione (fosc/4) e passa per un
 * prescaler pari a 4. Nel caso in cui la frequenza di clock sia di 64 MHz
 *
 * \f[
 *   n = \frac {1 \times 10^{-3}} {t_{ck} \times 4 \times prescaler} = {10^{-3} \times 4 \times 10^{6}} = 4000
 * \f]
 *
 * Per avere un overflow ogni 1 ms si dovrà precaricare il registro di:
 *
 * \f[
 *   TIMER\_OFFSET = (2^{16} -1) - n = 61535
 * \f]
 */
#define TIMER_OFFSET 64535

#define END_OF_MESSAGE 0x2a /**< Char che indica la fine del messaggio */
//#define RESET_ON_UART_OVERFLOW /**< Resetta il micro sull'evento di overflow della seriale */
#define REQUEST_NUMBER_OF_TRY 10 /**< Numero di tentativi prima di rinunciare */

#define BATT_AN_CHANNEL ADC_CH0 /**< Porta analogica dove è collegata la batteria */
#define LVDT_AN_CHANNEL ADC_CH4 /**< Porta analogica dove è collegato il sensore LVDT */
#define TEMP_AN_CHANNEL ADC_CH1 /**< Porta analogica dove è collegato il sensore di temperatura */
#define BATT_ADC_TIMEOUT_S 30 /**< Tempo tra un'acquisizione e l'altra della batteria*/
#define LED_UPDATE_S 3600 /**< Ritardo nell'accensione del led di stato*/
#define CONNECTION_TIMEOUT_MS 20000 /**< Tempo limite oltre il quale si entra in modalità ap*/

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

#define RTCC_ERROR_S 0.038
#define STC3100_RESISTOR_MOHM 33.0

#include <stdio.h>
#include <stdlib.h>
#include <timers.h>
#include <adc.h>

#include "include/uart_interface.h"
#include "include/winbond.h"
#include "include/rn131.h"
#include "include/stc3100.h"
#include <xc.h>
#include "system/io_cfg.h"
#include "i2c.h"
#include "spi.h"
#include "EEP.h"
#include "rtcc.h"


/* C O N F I G ***************************************************************/
#pragma config RETEN = 0  /**< Vreg Sleep Enable bit (negate) */
#pragma config INTOSCSEL = 1  /**< LF-INTOSCSEL Low-power Enable bit (negate) */
#pragma config SOSCSEL = 2  /**< SOSC Power Selection and mode Configuration bits */
#pragma config XINST = 0  /**< Extended Instruction Set */
#pragma config FOSC = INTIO2  /**< Oscillator */
#pragma config PLLCFG = 0  /**< PLL x4 Enable bit */
#pragma config FCMEN = 0  /**< Fail-Safe Clock Monitor */
#pragma config IESO = 0  /**< Internal External Oscillator Switch Over Mode */
#pragma config PWRTEN = 1  /**< Power Up Timer (negate) */
#pragma config BOREN = 0  /**< Brown Out Reset */
#pragma config BORV = 3  /**< Brown Out Reset Voltage bits */
#pragma config BORPWR = 0  /**< BORMV Power Level */
#pragma config WDTEN = 0  /**< Watchdog Timer */
#pragma config WDTPS = 0  /**< Watchdog Postscaler */
#pragma config RTCOSC = 0  /**< RTCC Clock Select */
#pragma config CCP2MX = 0  /**< CCP2 Mux */
#pragma config MSSPMSK = 0  /**< MSSP address masking */
#pragma config MCLRE = 1  /**< Master Clear Enable */
#pragma config STVREN = 1  /**< Stack Overflow Reset */
#pragma config BBSIZ = 0  /**< Boot Block Size */


/* P R O T O T Y P E *********************************************************/
void initialize_system(void);
//void user_timer_ms(void);
void update_date(float error_sec);
void message_load_uart(const char *buffer);
int ap_mode(void);
int download_configuration_ftp(void);
int ping(void);
void circular_buffer_save(long start_address);
int BCDToDecimal(char bcdByte);
char DecimalToBCD(int decimalByte);
void adc_open(int an_channel);

/* G L O B A L   V A R I A B L E *********************************************/
time_t time_by_rn131 = 0; /**< Detiene i secondi trascorsi dall'epoc*/
struct tm *date_structure;  /**< Detiene l'ora ed il giorno in formato parametrico s*/

/** Indica quando è possibile terminare il programma.
 * 
 * Viene impostata quando si vuole terminare il programma dopo aver eseguito delle
 * operazioni propedeutiche, per esempio nel caso in cui viene meno l'alimentazione.
 */
volatile int done = 0;

/**
 * Indica quando può essere rinnovata una richiesta.
 *
 * Quando sono passati TIMEOUT_UPDATE_RQST_HS centesimi di secondo, questa
 * variabile viene impostata.
 *
 */
volatile int rqst_update_flag = 0;

/**
 * Indica quando si può leggere il valore dell'ultima acquisizione.
 *
 * Quando sono passati TIMEOUT_UPDATE_ADC_HS centesimi di secondo, questa
 * variabile viene impostata tramite l'interrupt del Timer1.
 *
 */
volatile int adc_update = 0;

volatile int config_error = 0;

/**
 * Canale adc che è stato campionato.
 *
 * Prima di avviare l'acquisizione, questo indice viene cambiato in accordo con
 * il canale da campionare. Una volta finito il campionamento, tramite questa
 * variabile è possibile associare il valore al sensore.
 */
volatile int an_channel = -1;

/**
 * Fa partire il timer prima dell'invio dei dati letti dalla eeprom.
 *
 * Con questo flag viene fatto partire il timer per creare un ritardo prima di 
 * leggere ed inviare i dati dalla eeprom. Infatti è necessario dare il tempo al
 * modulo wifly di comunicare eventuali errori di connessione.
 *
 */
volatile int tcp_start_timer_flag = -1;

unsigned char timer_config = 0x00; /**< Registro di configurazione 1 per il Timer*/
unsigned char timer_config1 = 0x00; /**< Registro di configurazione 2 per il Timer*/

/**
 * Contatore per la lettura/scrittura seriale sulla eeprom
 */
unsigned int eeprom_addr_count = 0;

union
{
  char bytes[38];
  
  struct
  {
    union
    {
      unsigned long value;
      unsigned char bytes[4];
    } eeprom_data_count; /**< quantità di dati presente nell'eeprom da inviare */

    union
    {
      unsigned long value;
      unsigned char bytes[4];
    } eeprom_ptr_wr; /**< locazione di memoria su cui si può scrivere */

    union
    {
      unsigned long value;
      unsigned char bytes[4];
    } eeprom_ptr_send; /**< prima locazione di memoria da inviare wifi */

    union
    {
      long value;
      unsigned char bytes[4];
    } timeout_update_adc_temp_ms; /**< Periodo di acquisizione dall'adc per il sensore di temperatura */

    union
    {
      long value;
      unsigned char bytes[4];
    } timeout_update_adc_ext1_ms; /**< Periodo di acquisizione dall'adc per il sensore LVDT */

    union
    {
      long value;
      unsigned char bytes[4];
    } timeout_data_eeprom_s; /**< Periodo di attesa prima di inviare i dati letti dalle eeprom */

    union
    {
      float value;
      unsigned char bytes[4];
   } battery_warn_value; /**< Valore di guardia per batteria scarica */

    union
    {
      float value;
      unsigned char bytes[4];
    } battery_min_value; /**< Valore minimo della batteria per cui può essere considerata scarica */

    union
    {
      long value;
      unsigned char bytes[4];
    } timeout_sleep_s; /**< Periodo di acquisizione dall'adc */

    union
    {
      unsigned int value;
      unsigned int bytes[2];
    } category;
  };
} configuration;

/**
 *  Tiene traccia dello stato interno.
 *
 */
union status_flags_struct
{
  unsigned char byte;
  
  struct
  {
    unsigned battery_warning :1;  /**< batteria quasi scarica */
    unsigned spi_bus_collision :1; /**< collisione sul bus spi */
    unsigned eeprom_full :1; /**< memoria eeprom piena */
    unsigned micro_reset_fault :1; /**< reset inaspettato */
    unsigned uart_overflow :1; /**< uart overflow */
    unsigned eeprom_fail :1; /**< eeprom fault */
    unsigned rttc_update_fail :1; /**< rtcc fault*/
  } bits;
} status_flags;

char buffer[UART2_BUFFER_SIZE_RX];/**< buffer globale d'appoggio sia in lettura che in scrittura*/

/**
 *  Indica quando il micro può inviare dati sulla seriale.
 *
 * Il microcontrollore può inviare i dati solo quando è certo che il dispositivo
 * rn131 sia sveglio ed associato ad un access point. Un'altra condizione necessaria
 * è che l'ora sia stata acquisita correttamente. Tutto questo viene riflesso sulla
 * variabile communication_ready.
 * 
 */
volatile char communication_ready = 0;


/**
 *  Contiene tutti i dati utili che devono essere memorizzati.
 */
union sample_union
{
  char sample_array[8]; /**< array che racchiude tutta la struttura */

  struct
  {
    long date; /**< data ed ora nel fomato unix */
    char category; /**< categoria di appartenenza del dato*/
    char state; /**< stato interno del microcontrollore */

    union
    {
      int adc_value_int; /**< valore dell'adc in formato integer*/

      struct
      {
        unsigned char adc_value[2];
      } adc_value_bytes; /**< valore dell'adc in formato array di byte */
    };
  } sample_struct; /**< valore campionato dall'adc*/
};
  

union sample_union sample;

/* I N T E R R U P T **********************************************************/

/**
 * Gestisce gli interrupt.
 *
 * Rappresenta la routine chiamata da un qualsiasi evento di interrupt.
 *
 */
void interrupt myIsr(void)
{
  static short long adc_timer_temp = 0; /**< tiene il tempo trascorso tra un'acquisizione e l'altra della temperatura. */
  static short long adc_timer_lvdt = 0; /**< tiene il tempo trascorso tra un'acquisizione e l'altra dell'LVDT */
  static short long adc_timer_batt = BATT_ADC_TIMEOUT_S; /**< tiene il tempo trascorso tra un'acquisizione e l'altra della batteria. */

  if(UART2_INTERRUPT_RX > 0)
    uart2_isr();

  if((rn131_int_flag == 1) && (rn131_int_enable == 1))
  {
    static int rn131_rts_pin = 0;
    
    rn131_int_flag = 0;

    if(rn131_rts_pin == 0)
    {
      rn131_rts_pin = 1;
      rn131_int_edge = 0;

      // Reset status flags
      rn131.connected = 0;
      rn131.tcp_open = 0;
      rn131.tcp_error = 0;
      rn131.cmd_mode_exit_rqst = 0;
      rn131.cmd_mode_reboot_rqst = 0;
      rn131.cmd_mode_sleep_rqst = 0;
      rn131.cmd_http_rqst = 0;
      rn131.cmd_mode_rqst = 0;
      rn131.time_set_rqst = 0;
      rn131.cmd_mode = 0;

      rn131.ready = 0;

      communication_ready = 0;

      rn131.wakeup = 1;

      OSCCONbits.SCS = 3;
      
      uart2_tx_tris = OUTPUT_PIN;
      uart2_rx_tris = INPUT_PIN;
      uart2_open(9600);

    }
    else
    {
      rn131_rts_pin = 0;
      rn131_int_edge = 1;
    }
  }

  /**
   * Studio consumo:
   *   - all off but 3.3V, stc3100 stand-by -> (PARZIALE 1)0.07mA  -> (MODULO) 0.07mA
   *   - adc idle -> 0  (PARZIALE 2)0.07 -> (MODULO) 0.07mA
   *   - uart idle -> (PARZIALE 3)0.32mA -> (MODULO)0.25mA
   *   - i2c idle -> (PARZIALE 3)0.32mA -> (MODULO)0.32mA
   *   - stc3100 start -> (PARZIALE 3)0.41mA -> (MODULO)0.09mA
   *
   *   - idlen = 0, 3_3VSW off, timer1 off, uart off, flash off;  -> (PARZIALE 3)1.23mA -> (MODULO)?mA
   *   - idlen = 0, 3_3VSW off, timer1 off, uart off, flash off, spi off;  -> (PARZIALE 3)0.8mA -> (MODULO SPI)0.43mA
   *   - idlen = 0, 3_3VSW on, timer1 off, uart off, flash off, spi off, i2c off;  -> (PARZIALE 3)0.60mA
   *   - idlen = 0, 3_3VSW off, timer1 off, uart off, flash off, spi off, i2c off;  -> (PARZIALE 4)0.14mA
   *
   *   - idlen = 1, 3_3VSW off, timer1 off, uart off, flash off, spi off;  -> (PARZIALE 3)2.74mA -> (MODULO)?mA
   *   - idlen = 1; alim_3_3V_enable = 0; -> 
   */
  if(PIR3bits.RTCCIF && PIE3bits.RTCCIE)
  {
    static short long led_timer = 0; /**< tiene il tempo per il led di stato */
    static int rqst_update_timer = 0;
    
    PIR3bits.RTCCIF = 0;
    //led_err = ~led_err;

    if(rn131.ap_mode == 0)
    {
      if((rn131.time_set == 1))
      {
        if(rn131.wakeup == 0)
        {
          if(TXSTA2bits.TXEN)
          {
            uart2_close();
            uart2_tx_tris = INPUT_PIN;
            uart2_rx_tris = INPUT_PIN;
          }
        }
        else
        {
          /**
           * Abilita nuova richiesta
           *
           * Se c'è stata una richiesta non soddisfatta dopo ::TIMEOUT_UPDATE_RQST_HS
           * allora si abilita il rinvio. In questo caso il firmware non deve andare
           * in sleep.
           *
           * \attention Ogni flag di richiesta che si aggiunge a ::rn131_struct deve
           * essere aggiunta anche nella condizione per incrementare il timer
           * ::rqst_update_timer
           */
          if(rn131.cmd_mode_exit_rqst || rn131.cmd_mode_reboot_rqst || rn131.cmd_mode_rqst ||
             rn131.time_set_rqst || rn131.cmd_mode_sleep_rqst || rn131.cmd_http_rqst)
            rqst_update_timer++;
          else
            rqst_update_timer = 0;

          if(rqst_update_timer * 1000 == TIMEOUT_UPDATE_RQST_MS)
          {
            rqst_update_timer = 0;
            rqst_update_flag = 1;
          }
        }
      }

      led_timer++;
      if(led_timer >= LED_UPDATE_S)
      {
        if(led_timer < (LED_UPDATE_S + 60))
        {
          if(status_flags.bits.micro_reset_fault || status_flags.bits.spi_bus_collision)
            led_err = 1;
          else if(status_flags.bits.battery_warning || status_flags.bits.eeprom_full ||
                status_flags.bits.uart_overflow)
          {
            led_err = 1;
            led_no_err = 1;
          }
          else
            led_no_err = 1;
        }
        else
        {
          led_err = 0;
          led_no_err = 0;
          led_timer = 0;

          circular_buffer_save(EEPROM_COUNT_PTR);
        }
      }

      if(tcp_start_timer_flag > 0)
      {
        tcp_start_timer_flag++;

        if(tcp_start_timer_flag == configuration.timeout_data_eeprom_s.value)
          tcp_start_timer_flag = 0;
      }

      if((rn131.time_set == 1) && (rn131.wakeup == 0))
      {
        adc_timer_batt++;
        adc_timer_temp += 1000;
        adc_timer_lvdt += 1000;

        if(adc_timer_batt >= BATT_ADC_TIMEOUT_S)
        {
          if((ADCON0bits.GO == 0) && (an_channel == -1))
          {
            adc_update = 1;
            adc_timer_batt = 0;
            an_channel = BATT_AN_CHANNEL;
          }
        }
        else if(adc_timer_temp >= configuration.timeout_update_adc_temp_ms.value)
        {
          // Avvia acquisizione
          if((ADCON0bits.GO == 0) && (an_channel == -1))
          {
            adc_timer_temp = 0;
            an_channel = TEMP_AN_CHANNEL;
            update_date(RTCC_ERROR_S);
            SelChanConvADC(an_channel);
          }
        }
        
        if(adc_timer_lvdt >= configuration.timeout_update_adc_ext1_ms.value)
        {
          if(!T1CONbits.TMR1ON)
          {
            OSCCONbits.IDLEN = 1;
            WriteTimer1(TIMER_OFFSET);
            OpenTimer1(timer_config, timer_config1);
          }
        }
        else 
        {
          if(T1CONbits.TMR1ON == 1)
            CloseTimer1();

          // Se ho finito anche di campionare, allora posso tornare in sleep mode
          if(an_channel == -1)
          {
            OSCCONbits.IDLEN = 0;
          }
        }
      }
      else
      {
        adc_timer_batt = 0;
        adc_timer_temp = 0;
        adc_timer_lvdt = 0;
      }
    }
  }
  
  if(PIR1bits.ADIF && PIE1bits.ADIE)
  {
    adc_update = 1;

    PIR1bits.ADIF = 0;
  }

  // this function must be the last because can send in sleep the micro
  if(PIR1bits.TMR1IF && PIE1bits.TMR1IE)
  {
    static int rqst_update_timer = 0;
    static short long connection_timer = 0; /**< tiene il tempo per lo scadere del tentativo di connession */

    WriteTimer1(TIMER_OFFSET);
    PIR1bits.TMR1IF = 0;

    if(rn131.ap_mode == 0)
    {
      if((rn131.wakeup == 1) && (rn131.ready == 1) && 
         (((rn131.time_set == 1) && (rn131.connected == 0)) || (rn131.time_set == 0))
          )
      {
        connection_timer++;

        if(connection_timer >= CONNECTION_TIMEOUT_MS)
        {
          connection_timer = 0;
          config_error = 1;
        }
      }
      else if(connection_timer != 0)
        connection_timer = 0;

      /**
       * Abilita nuova richiesta
       *
       * Se c'è stata una richiesta non soddisfatta dopo ::TIMEOUT_UPDATE_RQST_HS
       * allora si abilita il rinvio. In questo caso il firmware non deve andare
       * in sleep.
       *
       * \attention Ogni flag di richiesta che si aggiunge a ::rn131_struct deve
       * essere aggiunta anche nella condizione per incrementare il timer
       * ::rqst_update_timer
       */
      if((rn131.wakeup == 1) || (rn131.time_set == 0))
      {
        if(rn131.cmd_mode_exit_rqst || rn131.cmd_mode_reboot_rqst || rn131.cmd_mode_rqst ||
           rn131.time_set_rqst || rn131.cmd_mode_sleep_rqst || rn131.cmd_http_rqst)
          rqst_update_timer++;
        else
          rqst_update_timer = 0;

        if(rqst_update_timer == TIMEOUT_UPDATE_RQST_MS)
        {
          rqst_update_timer = 0;
          rqst_update_flag = 1;
        }
      }

      if((rn131.time_set == 1) && (rn131.wakeup == 0))
      {
        adc_timer_lvdt++;
      }
      else
      {
        adc_timer_lvdt = 0;
      }

      if(adc_timer_lvdt >= configuration.timeout_update_adc_ext1_ms.value)
      {
        // Avvia acquisizione
        if((ADCON0bits.GO == 0) && (an_channel == -1))
        {
          // Il dispositivo ha bisogno di almeno 3 ms per tirare fuori un segnale valido.
          // Inoltre anche lo step-up interno ha bisogno di un certo tempo di assestamento.
          // 100 ms di attesa sono sufficienti per ottenere un segnale valido.
          if(ext_shut == 0)
          {
            ext_shut = 1;
            adc_timer_lvdt = configuration.timeout_update_adc_ext1_ms.value - 100;
          }
          else
          {
            adc_timer_lvdt = 0;
            an_channel = LVDT_AN_CHANNEL;
            update_date(RTCC_ERROR_S);
            SelChanConvADC(an_channel);
          }
        }
      }
    }
  }

  /* Devo entrare in sleep se:
   * - ho inizializzato l'ora
   * - il modulo wifi non si è ancora svegliato
   * - non devo aggiornare il valore campionato dall'adc
   * - non devo inviare i dati tramite il modulo wifi. Per far questo però ho
   *   bisogno di attendere che il modulo sia connesso. Una volta che ho inviato
   *   tutti i dati imposto communication_ready a zero, così da ricadere nel
   *   primo caso
   * - la eeprom ha concluso le sue operazioni
   * -
  */
  if(((rn131.time_set == 1) && (communication_ready == 0)) && (adc_update == 0) && (eeprom_at_work == 0) &&
     (rn131.wakeup == 0) && (uart2_status.buffer_tx_empty == 1) && !BusyADC())
  {
    // cambio la sorgente del clock con quella secondaria
    // aspetto che il messaggio sia stato inviato sulla seriale fisica
    while(TXSTA2bits.TRMT == 0)
      NOP();

    SLEEP();
  }
}

/**
 * \brief Program's entry.
 *
 * @return Error code for exit
 *
 * @bug
 *   - prima di provare la presenza del webserver, sono costretto a fare un ping a google.it, altrimenti
 *     la connessione non va a buon fine "Connection FAILED".
 *   - se attivo l'interrupt sulla seriale, il micro si resetta
 *   - quando vado in sleep il tempo che trascorre tra un timeout e l'altro del timer 1 è più lungo.
 *     Questo è dovuto al ritardo introdotto dall'uscita dalla modalità sleep. Questo errore me lo porto dietro sia nel calcolo del tempo trascorso che nel numero di campioni
 *     presi dall'adc
 *
 *
 * @todo
 * - testare se il battery gauge misura bene
 * - calibrare il modulo stc3100
 * - portare il sistema a 31 kHz quando il modulo wifly è spento
 * - scrivere sulla eeprom solo dopo 32 byte, così da ridurre il consumo di corrente
 * - usare lvd feature per capire quando la batteria è scarica
 * - implementare codice errore FTP timeout=2
 * - scrivere i codici di stato interno
 * - verificare che vada in sleep passati 40 secondi, anche quando si trova in modalità web server
 * - caricare il file di configurazione spin all'avvio
 * - aggiungere comando "salva configurazione"
 * - aggiungere comando "reset parametri"
 * - provare che i valori siano effettivamente salvati nella eeprom interna
 * - memorizzare nell'eeprom la configurazione per il modulo wifly
 * - aggiungere l'acquisizione da un sensore esterno ed aggiornare la categoria nel messaggio
 * - capire se un modulo è stato configurato attraverso la versione del firmware
 * - reset del micro da remoto
 * - comando salva ed spegni
 * - rivedere l'acquisizione del RTC da remoto: è stata divisa per farla diventare
 *   più veloce, però ha bisogno dei messaggi che seguono per finire il ciclo di
 *   acquisizione
 * - gestire meglio l'interrupt del timer
 * - provare l'interrupt sul fronte del pin b3 quando viene tolta la batteria
 * - utilizzare il campo riservato per inoltrare gli errori(batteria quasi scarica,
 *   errore di lettura/scrittura eeprom, )
 * - prendere il carattere di fine messaggio direttamente dal modulo wifly,
 *   tra le informazioni del comando "get comm"
 * - invece di fare il polling sui messaggi inviati dal modulo, un altro modo per
 *   capire se il modulo è stato associato all'access point è tramite il comando
 *   "show net" oppure "show connection
 * - descrivere come è stato impostato il tempo di campionamento dell'adc
 * - utilizzare pin CTS ed RTS per svegliare il dispositivo ed inviare dati
 * - valutare se conviene impostare come clock il timer1 durante lo sleep.
 * - scrivere documentazione temporizzazione
 * - scrivere documentazione
 *
 */
int main(void)
{
  char ascii_buffer[256];
  char end_of_message[2] = {END_OF_MESSAGE, 0};
  int ascii_buffer_size = 0;
  long timeout_update_hs_temp = 0;

  char *cmd_http_start = rn131.cmd_http;
  char *cmd_http_mark = NULL;
  char cmd_http_parse[32];

  unsigned int uart_empty_space = 0;
  char uart_token[] = {'\n', '*'};
  unsigned short long eeprom_data_to_send; /**< numero di byte caricati dall'eeprom e pronti per essere inviati*/
  
  //unsigned int adc_result = 0; /**< valore del canale analogico*/
  float battery_value_temp = 0; /**< Valore di apporggio per impostare il livello della batteria da remoto*/

  int windbond_return_value;

  WDTCONbits.REGSLP = 1; // on-chip regulator enters low-power operation when device enters in Sleep mode
  OSCTUNEbits.INTSRC = 0;
  OSCCON2bits.MFIOSEL = 0;
  OSCCONbits.IRCF = 7;
  OSCTUNEbits.PLLEN = 0;
  OSCCONbits.IDLEN = 0;
  OSCCONbits.SCS = 3;

  initialize_system();

  // power on reset
  if(RCONbits.POR == 0)
  {
    RCONbits.POR = 1;
    RCONbits.BOR = 1;
  }

  // bor reset
  if(RCONbits.BOR == 0)
  {
    status_flags.bits.micro_reset_fault = 1;
    RCONbits.BOR = 1;
    RCONbits.RI = 1;
  }

  // reset instruction
  if(RCONbits.RI == 0)
  {
    status_flags.bits.micro_reset_fault = 1;
    RCONbits.RI = 1;
  }

  // configuration mismatch reset
  if(RCONbits.CM == 0)
  {
    status_flags.bits.micro_reset_fault = 1;
    RCONbits.CM = 1;
  }

  /************ INIT GLOBAL ***********/
  configuration.timeout_sleep_s.value = 120;
  configuration.timeout_update_adc_temp_ms.value = 30000;
  configuration.timeout_update_adc_ext1_ms.value = 30000;
  configuration.timeout_data_eeprom_s.value = 10;
  configuration.battery_warn_value.value = 3.2;
  configuration.battery_min_value.value = 3.0;
  configuration.category.value = 2;
  windbond_return_value = 0;

  // Disable all peripheral
  PMD0 = 0b11101000;
  PMD1 = 0b11111101;
  PMD2 = 0xff;
  PMD3 = 0xff;

  /************ INIT ADC ***********/
  // clear adc interrupt and turn off adc if in case was on previously
  CloseADC();
  OpenADC(ADC_FOSC_RC | ADC_RIGHT_JUST | ADC_2_TAD,
          ADC_CH0 | ADC_INT_ON,
          ADC_NEG_CH0 | ADC_REF_VDD_INT_VREF_2 | ADC_REF_VDD_VSS);
  
  /******* INIT SERIAL COMM ********/
  uart1_close();
  uart2_open(9600);

  /******* INIT I2C   ******************/
  CloseI2C1();
  OpenI2C1(MASTER, SLEW_OFF);

  // 400kHz Baud Clock
  SSPADD = FOSC_MHZ / (4 * 300000) - 1;

  // Read ID
  stc3100_get(STC3100_ADDR, STC3100_ID0, 8, stc3100_id);

  // Azzera l'accumulatore ed il contatore
  stc3100_write(STC3100_ADDR, STC3100_CTRL, 0x02);

  // Avvia il modulo per la calibrazione
  //stc3100_write(STC3100_ADDR, STC3100_MODE, 0x18);
  // Avvia il modulo con risoluzione a 14-bit
  stc3100_write(STC3100_ADDR, STC3100_MODE, 0x10);

  CloseI2C1();
  
  /******* INIT RTCC   ******************/
  rtccTimeDate rtcc_time_date;

  /******* INIT SPI   ******************/
  // init spi
  spi_sck_tris = OUTPUT_PIN;
  spi_sdo_tris = OUTPUT_PIN;
  spi_sdi_tris = INPUT_PIN;
  spi_cs_tris = OUTPUT_PIN;

  spi_cs = 1;

  CloseSPI2();
  OpenSPI2(SPI_FOSC_16, MODE_11, SMPEND);

  /************ INIT TIMER *********/
  timer_config = T1_16BIT_RW | T1_SOURCE_FOSC_4 | T1_PS_1_4 | T1_OSC1EN_OFF
          | T1_SYNC_EXT_OFF | TIMER_INT_ON;

  timer_config1 = TIMER_GATE_OFF;
  
  OpenTimer1(timer_config, timer_config1);
  WriteTimer1(TIMER_OFFSET);

  /************ INTERRUPT *********/
  
  /********* RTS INTERRUPT ********/
  rn131_int_edge = 1;
  rn131_int_flag = 0;
  rn131_int_enable = 1;

  /********* RTCC INTERRUPT ********/
  PIE3bits.RTCCIE = 1;

  /********* ADC INTERRUPT ********/
  ADC_INT_ENABLE();
  
  INTCONbits.PEIE = 1;
  ei(); // enable all interrupt
  
  /******* INIT DATA ********/
  rn131.wakeup = 0;
  rn131.ready = 0;
  rn131.connected = 0;
  rn131.tcp_open = 0;
  rn131.tcp_error = 0;
  rn131.cmd_mode = 0;
  rn131.ap_mode = 0;
  rn131.cmd_mode_rqst = 0;
  rn131.cmd_mode_exit_rqst = 0;
  rn131.cmd_mode_reboot_rqst = 0;
  rn131.cmd_mode_sleep_rqst = 0;
  rn131.cmd_http_rqst = 0;
  rn131.time_set = 0;
  rn131.time_set_rqst = 0;
  
  int rn131_message_length = 0;

  alim_3_3V_enable_tris = OUTPUT_PIN;
  alim_3_3V_enable =  1;
  
  memset(winbond.device_id, 0, sizeof(winbond.device_id));

  // leggo dalla EEPROM i valori dei puntatori circolari. Nel caso fosse
  // la prima esecuzione del programma dopo la programmazione, allora inizializzo
  // i valori a zero. Questa condizione è verificata quando tutti i valori risultano
  // uguali al valore limite
  //ReadFlash((UINT32)EEPROM_COUNT_PTR, (UINT16)sizeof(configuration.bytes), configuration.bytes);
  for(eeprom_addr_count = 0; eeprom_addr_count < sizeof(configuration.bytes); eeprom_addr_count++)
    configuration.bytes[eeprom_addr_count] = Read_b_eep(EEPROM_COUNT_PTR + eeprom_addr_count);

  if((configuration.eeprom_ptr_send.value == 0xFFFFFFFF) && (configuration.eeprom_ptr_wr.value == 0xFFFFFFFF) && (configuration.eeprom_data_count.value == 0xFFFFFFFF))
  {
    configuration.eeprom_ptr_send.value = 0;
    configuration.eeprom_ptr_wr.value = 0;
    configuration.eeprom_data_count.value = 0;
    configuration.timeout_update_adc_temp_ms.value = 30000;
    configuration.timeout_update_adc_ext1_ms.value = 30000;
    configuration.timeout_data_eeprom_s.value = 10;
    configuration.battery_warn_value.value = 3.2;
    configuration.battery_min_value.value = 3.0;
    configuration.category.value = 2;
    configuration.timeout_sleep_s.value = 120;
  }
  
  eeprom_at_work = 0;

  status_flags.byte = 0;
  
  __delay_ms(5);
  
  /* MANUFACTURER ID Winbond: 0xEF
   * DEVICE ID W25Q64BV:      0x16
   */
  if(winbond_identification(winbond.device_id) < 0)
    status_flags.bits.spi_bus_collision = 1;

  if(winbond.device_id[0] != 0xff)
  {
    if(winbond_status_register_read(&winbond.status_register_1.word, 1) < 0)
      status_flags.bits.spi_bus_collision = 1;

    if(winbond_status_register_read(&winbond.status_register_2.word, 2) < 0)
      status_flags.bits.spi_bus_collision = 1;
  }
  else
    status_flags.bits.eeprom_fail = 1;

  if(SSP2CON1bits.SSPEN == 1)
  {
    CloseSPI2();
    spi_sck_tris = INPUT_PIN;
    spi_sdo_tris = INPUT_PIN;
    spi_sdi_tris = INPUT_PIN;
    spi_cs_tris = INPUT_PIN;
  }
  
  while(!done)
  {
    /*************************************************************** Timers
     * I can't syncronize request with timer, so I discard the first occurrance.
     * In this way the max elapsed time is (TIMEOUT * 2)
     */
    if(rqst_update_flag == 1)
    {
      if(rn131.cmd_mode_rqst == 1)
        rn131.cmd_mode_rqst++;
      else if((rn131.cmd_mode_rqst > 1) && (rn131.cmd_mode == 0) && (uart2_status.buffer_tx_empty == 1))
      {
        message_load_uart("$$$");
        
        rn131.cmd_mode_rqst++;
        if(rn131.cmd_mode_rqst == REQUEST_NUMBER_OF_TRY)
          rn131.cmd_mode_rqst = 0;
      }

      if(rn131.time_set_rqst == 1)
        rn131.time_set_rqst++;
      else if((rn131.time_set_rqst > 1) && (uart2_status.buffer_tx_empty == 1))
      {
        message_load_uart("show t t\r");

        rn131.time_set_rqst++;
        if(rn131.time_set_rqst == REQUEST_NUMBER_OF_TRY)
          rn131.time_set_rqst = 0;
      }

      if(rn131.cmd_mode_exit_rqst == 1)
        rn131.cmd_mode_exit_rqst++;
      else if((rn131.cmd_mode_exit_rqst > 1) && (uart2_status.buffer_tx_empty == 1))
      {
        message_load_uart("exit\r");

        rn131.cmd_mode_exit_rqst++;
        if(rn131.cmd_mode_exit_rqst == REQUEST_NUMBER_OF_TRY)
          rn131.cmd_mode_exit_rqst = 0;
      }

      if(rn131.cmd_mode_reboot_rqst == 1)
        rn131.cmd_mode_reboot_rqst++;
      else if((rn131.cmd_mode_reboot_rqst > 1)&& (uart2_status.buffer_tx_empty == 1))
      {
        message_load_uart("reboot\r");

        rn131.cmd_mode_reboot_rqst++;
        if(rn131.cmd_mode_reboot_rqst == REQUEST_NUMBER_OF_TRY)
          rn131.cmd_mode_reboot_rqst = 0;
      }

      if(rn131.cmd_mode_sleep_rqst == 1)
        rn131.cmd_mode_sleep_rqst++;
      else if(rn131.cmd_mode_sleep_rqst > 1)
      {
        if(rn131.cmd_mode == 0)
        {
          if(rn131.cmd_mode_rqst == 0)
          {
            message_load_uart("$$$");
            rn131.cmd_mode_rqst = 1;
          }
        }
        else
        {
          // richiedo di entrare in modalità sleep. Non posso controllare che ci
          // entri veramente; se ci fosse stato un qualsiasi problema, si fa affidamento
          // al timer interno che lo porterà comunque in sleep
          message_load_uart("sleep\r");

          // devo essere certo di inviare tutta la stringa prima di entrare in sleep,
          // però non posso ignorare i messaggi che ricevo, altrimenti ottengon un
          // overflow sulla seriale.
          while(uart2_status.buffer_tx_empty == 0)
          {
            uart2_buffer_send();
            uart2_buffer_rx_load();
          }

          // aspetto che il messaggio sia stato inviato sulla seriale fisica
          while(TXSTA2bits.TRMT == 0)
            NOP();

          rn131_sleep();
          communication_ready = 0;
          tcp_start_timer_flag = -1;
        }
      }

      if(rn131.cmd_http_rqst == 1)
        rn131.cmd_http_rqst++;
      else if((rn131.cmd_http_rqst > 1)  && (uart2_status.buffer_tx_empty == 1))
      {
        buffer[0] = 0;
        
        // rn131.cmd_http può contenere più di un messaggio di configurazione,
        // ognuno delimitato da un \r        
        cmd_http_mark = strchr(cmd_http_start, '\r');

        if(cmd_http_mark == NULL)
          rn131.cmd_http_rqst = 0;
        
        while(cmd_http_mark != NULL)
        {
          memcpy(cmd_http_parse, cmd_http_start, cmd_http_mark - cmd_http_start);
          cmd_http_parse[cmd_http_mark - cmd_http_start] = 0;

          //il caso del web_server è particolare: devo attendere che si passi in
          //modalità comando senza aggiorare il puntatore alla richiesta, altrimenti
          //non riesco a ricordare cosa stavo facendo. Una volta che è stata avviato
          //il web server tutte le altre richieste devono essere annullate.
          if(strncmp(cmd_http_parse, "web_server", 10) == 0)
          {
            if(rn131.cmd_mode == 0)
            {
              if(rn131.cmd_mode_rqst == 0)
              {
                message_load_uart("$$$");

                rn131.cmd_http_rqst = 2;
                rn131.cmd_mode_rqst = 1;
              }

              // esco dal ciclo while
              cmd_http_mark = NULL;
            }
            else
            {
              message_load_uart("run web_app\r");

              rn131.cmd_http_rqst = 0;
              // annullo le altre richieste in sospeso
              *cmd_http_start = 0;
              cmd_http_mark = NULL;
            }
          }
          else if(strncmp(cmd_http_parse, "set_sleep=", 10) == 0)
          {
            if(rn131.cmd_mode == 0)
            {
              if(rn131.cmd_mode_rqst == 0)
              {
                message_load_uart("$$$");

                rn131.cmd_http_rqst = 2;
                rn131.cmd_mode_rqst = 1;
              }

              // esco dal ciclo while
              cmd_http_mark = NULL;
            }
            else
            {
              configuration.timeout_sleep_s.value = atol(cmd_http_parse + 10);

              sprintf(buffer, "set sys wake %ld\rsave\r", configuration.timeout_sleep_s.value);
              uart2_buffer_tx_seq_load(buffer, strlen(buffer));
              buffer[0] = 0;

              cmd_http_start = cmd_http_mark + 1;
              cmd_http_mark = strchr(cmd_http_start, '\r');

              if(cmd_http_mark == NULL)
                 rn131.cmd_http_rqst = 0;
              else
                 rn131.cmd_http_rqst = 2;

              // esco dal ciclo while
              cmd_http_mark = NULL;
              rn131.cmd_mode_exit_rqst = 1;
            }
          }
          else if((strncmp(cmd_http_parse, "get_config", 10) == 0) && (rn131.cmd_mode == 0))
          {
            ascii_buffer[0] = 0;
            
            if(buffer[0] == 0)
              sprintf(buffer, "%sx:", rn131.mac);

            sprintf(ascii_buffer, "sleep=%ld,adc=%ld,adc_temp=%ld,category=%d,data_delay=%ld,batt_warn=%1.2f,batt_min=%1.2f,",
                    configuration.timeout_sleep_s.value, configuration.timeout_update_adc_ext1_ms.value, configuration.timeout_update_adc_temp_ms.value, configuration.category.value, configuration.timeout_data_eeprom_s.value,
                    configuration.battery_warn_value.value, configuration.battery_min_value.value);

            strcat(buffer, ascii_buffer);

            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if((strncmp(cmd_http_parse, "set_config", 10) == 0) && (rn131.cmd_mode == 0))
          {
            circular_buffer_save(EEPROM_COUNT_PTR);

            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if((strncmp(cmd_http_parse, "get_batt", 8) == 0) && (rn131.cmd_mode == 0))
          {
            ascii_buffer[0] = 0;

            if(buffer[0] == 0)
              sprintf(buffer, "%sx:", rn131.mac);

            sprintf(ascii_buffer, "batt=%1.2f,", stc3100_voltage_value_mV);
            strcat(buffer, ascii_buffer);

            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if((strncmp(cmd_http_parse, "set_category=", 13) == 0) && (rn131.cmd_mode == 0))
          {
            configuration.category.value = atoi(cmd_http_parse + 13);
            
            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if((strncmp(cmd_http_parse, "set_adc=", 8) == 0) && (rn131.cmd_mode == 0))
          {
            timeout_update_hs_temp = atol(cmd_http_parse + 8);

            if(timeout_update_hs_temp > 0)
              configuration.timeout_update_adc_ext1_ms.value = timeout_update_hs_temp;

            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if((strncmp(cmd_http_parse, "set_adc_temp=", 13) == 0) && (rn131.cmd_mode == 0))
          {
            timeout_update_hs_temp = atol(cmd_http_parse + 8);

            if(timeout_update_hs_temp > 0)
              configuration.timeout_update_adc_temp_ms.value = timeout_update_hs_temp;

            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if((strncmp(cmd_http_parse, "set_data_delay=", 15) == 0) && (rn131.cmd_mode == 0))
          {
            timeout_update_hs_temp = atol(cmd_http_parse + 15);

            if(timeout_update_hs_temp > 0)
              configuration.timeout_data_eeprom_s.value = timeout_update_hs_temp;

            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if((strncmp(cmd_http_parse, "set_batt_warn=", 14) == 0) && (rn131.cmd_mode == 0))
          {
            battery_value_temp = atof(cmd_http_parse + 14);

            if(battery_value_temp > 2.7)
              configuration.battery_warn_value.value = battery_value_temp;

            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if((strncmp(cmd_http_parse, "set_batt_min=", 13) == 0) && (rn131.cmd_mode == 0))
          {
            battery_value_temp = atof(cmd_http_parse + 13);

            if(battery_value_temp >= 2.7)
              configuration.battery_min_value.value = battery_value_temp;

            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if(rn131.cmd_mode == 0)
          {
            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
        }

        if(buffer[0] != 0)
        {
          strcat(buffer, end_of_message);

          ascii_buffer_size = strlen(buffer);
          uart2_buffer_tx_seq_load(buffer, ascii_buffer_size);

          while(uart2_status.buffer_tx_empty == 0)
          {
            uart2_buffer_send();
            uart2_buffer_rx_load();
          }

          // aspetto che il messaggio sia stato inviato sulla seriale fisica
          while(TXSTA2bits.TRMT == 0)
            NOP();

          // resetto il buffer, così che i valori vengano appesi partendo dall'inizio
          ascii_buffer[0] = 0;
        }

        // se non ho dati dell'adc da inviare, allora posso pure portare il
        // modulo rn131 in sleep
        if((rn131.cmd_http_rqst == 0) && (eeprom_data_count == 0))
          rn131.cmd_mode_sleep_rqst = 1;

        if(rn131.cmd_mode == 1)
        {
          message_load_uart("exit\r");
          rn131.cmd_mode_exit_rqst = 1;
        }
      }

      rqst_update_flag = 0;
    }
    
    switch(uart2_error_handle())
    {
      case BUFFER_RX_OVERFLOW:
#ifdef RESET_ON_UART_OVERFLOW
        RESET();
#else
        status_flags.bits.uart_overflow = 1;
        break;

      case BUFFER_RX_MICRO_OVERFLOW:
#ifdef RESET_ON_UART_OVERFLOW
        RESET();
#else
        status_flags.bits.uart_overflow = 1;
#endif
        break;
        
      default:
        break;
    }
#endif
 /******************************************************************  Send ****/
    uart2_buffer_send();
    
/****************************************************************** Read     */     
    do
    {
      if(UART2_INTERRUPT_RX == 0)
        while(uart2_buffer_rx_load());
        
      if(uart2_status.buffer_rx_empty == 0)
      {
        rn131_message_length = uart2_buffer_read_multifiltered(buffer, uart_token, 2);

        if(rn131_message_length > 0)
        {
          // Check for a new connection
          if(rn131_connection_init(buffer, rn131_message_length) == 1)
          {
            memset(buffer, 0, rn131_message_length);
            
            continue;
          }

          // Controllo che riceva il messaggio CMD dopo una richiesta d'ingresso
          // in modalità command
          if(rn131.cmd_mode == 0)
          {
            if(rn131.cmd_mode_rqst > 0)
            {
              //if(rn131_command_enter() == 1)
              if(rn131_parse_message(buffer, "CMD") == 1)
              {
                rn131.cmd_mode = 1;
                rn131.cmd_mode_rqst = 0;
                continue;
              }
            }

            if(rn131.connected == 1)
            {
              if(rn131.time_set == 1)
              {
                if(rn131.tcp_open == 0)
                {
                  //if(rn131_tcp_connection())
                  if(rn131_parse_message(buffer, "OPEN"))
                  {
                    rn131.tcp_open = 1;
                    tcp_start_timer_flag = 1;

                    continue;
                  }

                  //if(rn131_warninng_connection_failed())
                  if(rn131_parse_message(buffer, "Connect FAILED") || rn131_parse_message(buffer, "HTTP/1.0 404 Not Found"))
                  {
                    rn131.tcp_error = 1;
                    rn131.cmd_mode_sleep_rqst = 1;

                    continue;
                  }
                }
                else
                {
                  if(rn131_http_response(buffer))
                  {
                    strcpy(rn131.cmd_http, buffer);
                    continue;
                  }

                  if(rn131_parse_message(buffer, "CLOS"))
                  {
                    rn131.tcp_open = 0;
                    
                    if(rn131.cmd_http[0] == 0)
                      tcp_start_timer_flag = configuration.timeout_data_eeprom_s.value - 1;
                      //tcp_start_timer_flag = 0;
                    else
                    {
                      rn131.cmd_http_rqst = 1;
                      cmd_http_start = rn131.cmd_http;
                    }
                    continue;
                  }
                }
              }
            }
          }
          else
          {
            if(rn131.cmd_mode_exit_rqst > 0)
            {
              //if(rn131_command_exit() == 1)
              if(rn131_parse_message(buffer, "EXIT"))
              {
                rn131.cmd_mode = 0;
                rn131.cmd_mode_exit_rqst = 0;

                continue;
              }
            }

            if(rn131.connected == 1)
            {
              if(rn131.time_set == 0)
              {
                if(rn131.time_set_rqst > 0)
                {
                  if(rn131_command_show_time(buffer, &time_by_rn131) == 1)
                  {
                    if(rn131.time_set == 1)
                    {
                      date_structure = gmtime(&time_by_rn131);

                      // date_structure indica l'anno partendo dal 1900, mentre rtcc_date lo indica partendo
                      // dal 2000
                      rtcc_time_date.f.year = DecimalToBCD(date_structure->tm_year - 100);
                      rtcc_time_date.f.mon = DecimalToBCD(date_structure->tm_mon + 1);
                      rtcc_time_date.f.mday = DecimalToBCD(date_structure->tm_mday);
                      rtcc_time_date.f.wday = DecimalToBCD(date_structure->tm_wday);
                      rtcc_time_date.f.hour = DecimalToBCD(date_structure->tm_hour);
                      rtcc_time_date.f.min = DecimalToBCD(date_structure->tm_min);
                      rtcc_time_date.f.sec = DecimalToBCD(date_structure->tm_sec);

                      RtccWrOn(); //write enable the rtcc registers

                      if(!RtccWriteTimeDate(&rtcc_time_date , 1))
                        status_flags.bits.rttc_update_fail = 1;

                      rtcc_time_date.f.year = DecimalToBCD(date_structure->tm_year - 100);
                      rtcc_time_date.f.mon = DecimalToBCD(date_structure->tm_mon + 1);
                      rtcc_time_date.f.mday = DecimalToBCD(date_structure->tm_mday);
                      rtcc_time_date.f.wday = DecimalToBCD(date_structure->tm_wday);
                      rtcc_time_date.f.hour = DecimalToBCD(date_structure->tm_hour);
                      rtcc_time_date.f.min = DecimalToBCD(date_structure->tm_min);
                      rtcc_time_date.f.sec = DecimalToBCD(date_structure->tm_sec);

                      ALRMCFGbits.CHIME = 1;
                      ALRMCFGbits.AMASK = 1;

                      if(!RtccWriteAlrmTimeDate(&rtcc_time_date))
                        status_flags.bits.rttc_update_fail = 1;
                      
                      if(status_flags.bits.rttc_update_fail != 1)
                      {
                        PADCFG1bits.RTSECSEL = 2;
                        // Il clock interno è di 31kHz, invece dei 32758 Hz attesi.
                        // Quindi bisogna compensare come descritto nell'equazione 18-1
                        // del datasheet. Però non è possibile compensare così tanto, in quanto
                        // il registro di autocompensazione è da soli 8 bit. Con il valore massimo
                        // arrivo ad una frequenza di 31500. L'errore commesso è di 0.038 secondi al secondo,
                        // quindi in un'ora commetterò un errore pari a 136.8 secondi all'ora.
                        //RTCCAL = 0x00;
                        mRtccOn();
                        mRtccClearAlrmPtr();
                        mRtccAlrmEnable();
                      }

                      mRtccWrOff();
                      
                      rn131.time_set_rqst = 0;

                      // It need to be rebooted otherwise the web server will kick off
                      // everyone trying to connect to it
                      if(rn131.cmd_mode_reboot_rqst == 0)
                      {
                        message_load_uart("reboot\r");

                        rn131_sleep();
                        communication_ready = 0;
                        tcp_start_timer_flag = -1;
                        
                        rn131.cmd_mode_reboot_rqst = 1;
                      }
                    }

                    continue;
                  }
                }
              }
            }
          }
        }
      }
    } while((rn131_message_length > 0) && (uart2_status.buffer_rx_empty == 0));
    
    /***********************************************************  Update  ***/
    if(rn131.connected == 1)
    {
      if(rn131.time_set == 0)
      {
        if(config_error == 1)
        {
          if(download_configuration_ftp())
            config_error = 0;
        }
        else if(rn131.cmd_mode == 0)
        {
          if(rn131.cmd_mode_rqst == 0)
          {
            message_load_uart("$$$");
            rn131.cmd_mode_rqst = 1;
          }
        }
        else
        {
          if(rn131.time_set_rqst == 0)
          {
            // Approfitto dell'inizializzazione dell'orologio per impostare anche il tempo di risveglio
            // del modulo, nel caso fosse nuovo.
            sprintf(buffer, "set sys wake %ld\rsave\r", configuration.timeout_sleep_s.value);
            message_load_uart(buffer);
            message_load_uart("time\r");

            rn131.time_set_rqst = 1;
          }
        }
      }
      else if((rn131.wakeup == 1) && (communication_ready == 0) && (rn131.cmd_mode_sleep_rqst == 0))
      {
        if(ping())
        {
          sprintf(buffer, "%s%s", rn131.mac, end_of_message);
          uart2_buffer_tx_seq_load(buffer, strlen(buffer));

          while(uart2_status.buffer_tx_empty == 0)
            uart2_buffer_send();

          // aspetto che il messaggio sia stato inviato sulla seriale fisica
          while(TXSTA2bits.TRMT == 0)
            NOP();
  
          communication_ready = 1;
        }
      }

      if((rn131.tcp_error == 0) && (rn131.cmd_mode_sleep_rqst == 0) && (tcp_start_timer_flag == 0))
      {
        // se mi trovo in modalità comandi, esco
        if(rn131.cmd_mode == 1)
        {
          if(rn131.cmd_mode_exit_rqst == 0)
            rn131.cmd_mode_exit_rqst = 1;
        }
        else
        {
          // il timer viene disabilitato ed i dati vengono inviati a blocchi della dimensione massima consentita.
          // il timer viene automaticamente abilitato alla ricezione di una stringa OPEN o CLOS da parte del
          // web server. In questo modo il modulo wifi ha il tempo di compiere tutte le operazioni di cui ha bisogno
          // senza perdere caratteri dalla seriale.
          tcp_start_timer_flag = -1;
          
          if(eeprom_data_count > 0)
          {
            if(SSP2CON1bits.SSPEN == 0)
            {
              spi_sck_tris = OUTPUT_PIN;
              spi_sdo_tris = OUTPUT_PIN;
              spi_sdi_tris = INPUT_PIN;
              spi_cs_tris = OUTPUT_PIN;

              OpenSPI2(SPI_FOSC_16, MODE_11, SMPEND);
            }

            // Voglio convertire i dati esadecimali in stringhe ascii: visto che ogni
            // byte è rappresentato da 2 caratteri, ho bisogno del doppio dello spazio
            // nel buffer di trasmissione. Inoltre voglio inserire il MAC address
            // del dispositivo rn131, che è di 18 caratteri
            uart_empty_space = (uart2_get_tx_buffer_empty_space() >> 1) - 18;
            udiv_t data_frame_number;
            data_frame_number = udiv(uart_empty_space - 1, sizeof(union sample_union));
        
            // leggo dall'eeprom e carico i dati sulla seriale. Devo riservarmi l'ultimo
            // carattere per inserire quello di fine messaggio. Inoltre devo inviare
            // dati non frammentati.
            eeprom_data_to_send = winbond_data_send(buffer, data_frame_number.quot * sizeof(union sample_union));
            configuration.eeprom_ptr_send.value = eeprom_ptr_send;
            configuration.eeprom_data_count.value = eeprom_data_count;

            if(eeprom_data_to_send > 0)
            {
              int i;
              char ascii_hex[3];

              strcat(ascii_buffer,  rn131.mac);
              for(i = 0; i < eeprom_data_to_send; i++)
              {
                /*ascii_hex[0] = buffer[i] >> 4;
                ascii_hex[1] = buffer[i] & 0x0f;
                ascii_hex[2] = '\0';*/
                sprintf(ascii_hex, "%.02x", buffer[i]);
                strcat(ascii_buffer,  ascii_hex);
              }

              strcat(ascii_buffer, end_of_message);

              ascii_buffer_size = strlen(ascii_buffer);
              uart2_buffer_tx_seq_load(ascii_buffer, ascii_buffer_size);

              while(uart2_status.buffer_tx_empty == 0)
              {
                uart2_buffer_send();
                uart2_buffer_rx_load();
              }

              // aspetto che il messaggio sia stato inviato sulla seriale fisica
              while(TXSTA2bits.TRMT == 0)
                NOP();

              // resetto il buffer, così che i valori vengano appesi partendo dall'inizio
              ascii_buffer[0] = '\0';
            }

            if(SSP2CON1bits.SSPEN == 1)
            {
              CloseSPI2();
              spi_sck_tris = INPUT_PIN;
              spi_sdo_tris = INPUT_PIN;
              spi_sdi_tris = INPUT_PIN;
              spi_cs_tris = INPUT_PIN;
            }
          }
          else
            rn131.cmd_mode_sleep_rqst = 1;
        }
      }
    }
    else
    {
      if(config_error == 1)
      {
        if(ap_mode())
          config_error = 0;
      }
    }

    if((rn131.time_set == 1) && (adc_update == 1) && (rn131.wakeup == 0))
    {
      if((an_channel > -1) && (SSP2CON1bits.SSPEN == 0))
      {
        spi_sck_tris = OUTPUT_PIN;
        spi_sdo_tris = OUTPUT_PIN;
        spi_sdi_tris = INPUT_PIN;
        spi_cs_tris = OUTPUT_PIN;

        OpenSPI2(SPI_FOSC_16, MODE_11, SMPEND);
      }

      switch(an_channel)
      {
        case BATT_AN_CHANNEL:
          OpenI2C1(MASTER, SLEW_OFF);

          // Leggo lo stato della batteria
          // Coulomb counter reading
          /*stc3100_get(STC3100_ADDR, STC3100_CHARGE_LOW, 4, stc3100_charge_state.bytes);
          stc3100_soc_mAh = 6.70 * (float)(stc3100_charge_state.reg.charge.integer / STC3100_RESISTOR_MOHM);*/

          // Battery current reading
          /*stc3100_get(STC3100_ADDR, STC3100_CURRENT_LOW, 2, stc3100_current.bytes);
          if((stc3100_current.integer & 0x2000) > 0)
            stc3100_current.integer |= 0xE000;

          stc3100_current_value_mA = 11.77 * (float)stc3100_current.integer / STC3100_RESISTOR_MOHM;*/

          // Battery voltage reading
          stc3100_get(STC3100_ADDR, STC3100_VOLTAGE_LOW, 2, stc3100_voltage.bytes);

          if((stc3100_voltage.integer & 0x0800) > 0)
            stc3100_voltage.integer |= 0xF000;

          stc3100_voltage_value_mV = 2.44 * stc3100_voltage.integer;

          if((stc3100_voltage_value_mV / 1000) <= configuration.battery_warn_value.value)
            status_flags.bits.battery_warning = 1;

          if((stc3100_voltage_value_mV / 1000) <= configuration.battery_min_value.value)
          {
            circular_buffer_save(EEPROM_COUNT_PTR);

            // disabilito tutte le periferiche
            di();
            CloseTimer1();
            CloseADC();
            ADC_INT_DISABLE();

            // Disabilito il modulo STC3100
            OpenI2C1(MASTER, SLEW_OFF);
            stc3100_write(STC3100_ADDR, STC3100_MODE, 0x00);
            CloseI2C1();

            CloseSPI2();
            spi_sck_tris = INPUT_PIN;
            spi_sdo_tris = INPUT_PIN;
            spi_sdi_tris = INPUT_PIN;
            spi_cs_tris = INPUT_PIN;

            // disalimento tutto
            alim_3_3V_enable = 0;
            adc_batt_enable = 1;

            // buona notte
            OSCCONbits.IDLEN = 0;
            SLEEP();
          }

          CloseI2C1();
          break;

        case TEMP_AN_CHANNEL:
          sample.sample_struct.adc_value_int = ReadADC();
          if(sample.sample_struct.adc_value_int != 0)
          {
            sample.sample_struct.category = 1;
            sample.sample_struct.date = time_by_rn131;
            sample.sample_struct.state = status_flags.byte;

            windbond_return_value = winbond_data_load(sample.sample_array, sizeof(sample));

            if(windbond_return_value == 0)
            {
              configuration.eeprom_ptr_wr.value = eeprom_ptr_wr;
              configuration.eeprom_data_count.value = eeprom_data_count;
            }
            else if(windbond_return_value == -1)
              status_flags.bits.spi_bus_collision = 1;
            else if(windbond_return_value == -2)
              status_flags.bits.eeprom_full = 1;
          }
          break;

        case LVDT_AN_CHANNEL:
          ext_shut = 0;
          sample.sample_struct.adc_value_int = ReadADC();
          if(sample.sample_struct.adc_value_int != 0)
          {
            sample.sample_struct.category = configuration.category.value;
            sample.sample_struct.date = time_by_rn131;
            sample.sample_struct.state = status_flags.byte;

            windbond_return_value = winbond_data_load(sample.sample_array, sizeof(sample));

            if(windbond_return_value == 0)
            {
              configuration.eeprom_ptr_wr.value = eeprom_ptr_wr;
              configuration.eeprom_data_count.value = eeprom_data_count;
            }
            else if(windbond_return_value == -1)
              status_flags.bits.spi_bus_collision = 1;
            else if(windbond_return_value == -2)
              status_flags.bits.eeprom_full = 1;
          }
          break;

        default:
          break;
      }

      adc_update = 0;
     
      if(an_channel > -1)
      {
        if(SSP2CON1bits.SSPEN == 1)
        {
          CloseSPI2();
          spi_sck_tris = INPUT_PIN;
          spi_sdo_tris = INPUT_PIN;
          spi_sdi_tris = INPUT_PIN;
          spi_cs_tris = INPUT_PIN;
        }
        
        an_channel = -1;

        SLEEP();
      }
    }
  }

  return(EXIT_SUCCESS);
}

void initialize_system(void)
{
  TRISA = OUTPUT_PIN;
  TRISB = OUTPUT_PIN;
  TRISC = OUTPUT_PIN;
  TRISD = OUTPUT_PIN;
  TRISE = OUTPUT_PIN;
  TRISF = OUTPUT_PIN;
  TRISG = OUTPUT_PIN;

  PORTA = 0;
  PORTB = 0;
  PORTC = 0;
  PORTD = 0;
  PORTE = 0;
  PORTF = 0;
  PORTG = 0;

  // init batt analog pin
  adc_batt_enable_tris = OUTPUT_PIN;
  adc_batt_enable = 1;
  
  // init adc
  // set all port to digital
  ANCON0 = 0xff;
  ANCON1 = 0b00001110;
  ANCON2 = 0;
  
  battery_an_tris = INPUT_PIN;
  battery_an_analog = 1;

  temp_an_tris = INPUT_PIN;
  temp_an_analog = 1;

  ext1_an_tris = INPUT_PIN;
  ext1_an_analog = 1;
  
  ext2_an_tris = INPUT_PIN;
  ext2_an_analog = 1;
  
  ext_shut_an_tris = OUTPUT_PIN;
  ext_shut_an_analog = 0;
  
  // init led
  led_err_tris = OUTPUT_PIN;
  led_err = 0;

  led_no_err_tris = OUTPUT_PIN;
  led_no_err = 0;

  // enable 3.3V
  alim_3_3V_enable_tris = INPUT_PIN;
  
  // init serial
#ifdef CONFIGURE_MODE
  uart2_tx_tris = INPUT_PIN;
  SLEEP();
#else
  uart2_tx_tris = OUTPUT_PIN;
#endif
  
  uart2_rx_tris = INPUT_PIN;

  // init rn131
  rn131_rts_tris = INPUT_PIN;

  // init MCP73811
  charger_ce_tris = INPUT_PIN;
  charger_prg_tris = OUTPUT_PIN;
  charger_prg = 0; // questo pin consuma molto perchè c'è un pull-down attaccato

  // init I2C
  i2c_sck_tris = INPUT_PIN;
  i2c_sda_tris = INPUT_PIN;

#ifndef CONFIGURE_MODE
  uart2_init(0);
#endif


}

/**
 * @todo aggiornare anche il giorno della settimana
 */
/*void user_timer_ms(void)
{
  static unsigned int msec = 0;
  msec++;

  if(msec >= 1000)
  {
    msec=0;

    time_by_rn131++;
  }
}*/

void update_date(float error_sec)
{
  rtccTimeDate rtcc_time_date;

  struct tm date_structure;
  static time_t time_by_rn131_start = 0;
  if(time_by_rn131_start == 0)
    time_by_rn131_start = time_by_rn131;
  
  RtccReadTimeDate(&rtcc_time_date);

  date_structure.tm_year = BCDToDecimal(rtcc_time_date.f.year) + 100;
  date_structure.tm_mon = BCDToDecimal(rtcc_time_date.f.mon) - 1;
  date_structure.tm_mday = BCDToDecimal(rtcc_time_date.f.mday);
  date_structure.tm_wday = BCDToDecimal(rtcc_time_date.f.wday);
  date_structure.tm_hour = BCDToDecimal(rtcc_time_date.f.hour);
  date_structure.tm_min = BCDToDecimal(rtcc_time_date.f.min);
  date_structure.tm_sec = BCDToDecimal(rtcc_time_date.f.sec);

  time_by_rn131 = mktime(&date_structure);

  // ho bisogno di compensare l'errore commesso dal rtc dovuto ad un clock diverso
  // da quello atteso. Siccome la frequenza è più piccola di quella prevista, un secondo
  // durerà di meno, commettendo un errore in eccesso sulla data. Per questo devo
  // sottrarre l'errore moltiplicato per i secondi passati
  time_by_rn131 -= (unsigned long)((time_by_rn131 - time_by_rn131_start) * error_sec);
}

int BCDToDecimal(char bcdByte)
{
  return (((bcdByte & 0xF0) >> 4) * 10) + (bcdByte & 0x0F);
}

char DecimalToBCD(int decimalByte)
{
  return (((decimalByte / 10) << 4) | (decimalByte % 10));
}

void message_load_uart(const char *message)
{
  char buffer[20];
  strcpy(buffer, message);

  uart2_buffer_tx_seq_load(buffer, strlen(buffer));
}

int ping(void)
{
  static int ping_status = 0;

  switch(ping_status)
  {
    case 0:
      if(rn131.cmd_mode == 0)
      {
        if(rn131.cmd_mode_rqst == 0)
        {
          message_load_uart("$$$");

          rn131.cmd_mode_rqst = 1;
        }
      }
      else
        ping_status = 1;
      break;

    case 1:
      message_load_uart("ping dgoogle.it\rexit\r");
      rn131.cmd_mode_exit_rqst = 1;
      ping_status = 2;
      break;

    case 2:
      if(rn131.cmd_mode == 0)
      {
        ping_status = 0;
        return 1;
      }
      break;
  }

  return 0;
}

int download_configuration_ftp(void)
{
  static int config_req_sent = 0;

  if(rn131.cmd_mode == 0)
  {
    if(rn131.cmd_mode_rqst == 0)
    {
      message_load_uart("$$$");

      rn131.cmd_mode_rqst = 1;
    }
  }
  else if(config_req_sent == 0)
  {
    message_load_uart("set dns backup babaracus.no-ip.org\r");
    message_load_uart("set ftp user pi\r");
    message_load_uart("set ftp pass raspberry\r");
    message_load_uart("set ftp remote 21\r");
    message_load_uart("set ftp dir /var/www\r");
    message_load_uart("ftp update wifly_config.cfg\r");

    config_req_sent = 1;
  }
  else
  {
    if(rn131_parse_message(buffer, "FTP OK"))
    {
      message_load_uart("load spin_c\r");
      message_load_uart("save\r");

      rn131.cmd_mode_reboot_rqst = 1;
      return 1;
    }
  }

  return 0;
}

int ap_mode(void)
{
  if(rn131.cmd_mode == 0)
  {
    if(rn131.cmd_mode_rqst == 0)
    {
      message_load_uart("$$$");

      rn131.cmd_mode_rqst = 1;
    }
  }
  else
  {
    message_load_uart("run web_app\r");

    // annullo ogni eventuale richiesta inoltrata
    rn131.cmd_http_rqst = 0;

    return 1;
  }

  return 0;
}

void circular_buffer_save(long start_address)
{
  for(eeprom_addr_count = 0; eeprom_addr_count < sizeof(configuration.bytes); eeprom_addr_count++)
    Write_b_eep(start_address + eeprom_addr_count, configuration.bytes[eeprom_addr_count]);
}