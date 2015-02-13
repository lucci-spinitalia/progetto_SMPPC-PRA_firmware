/**
 * \file main.c
 * Implementa il ciclo principale.
 * 
 * \author Luca Lucci
 * \version 1.0
 * \date 04/08/2014
 */
#define ERROR_MONITOR

#ifdef ERROR_MONITOR
#define TIMEOUT_ERROR_MONITOR_S 90 /**< Tempo prima di inviare il report di stallo */
#endif

#define USE_OR_MASKS /**< Abilita l'opeartore or per concatenare i parametri del timer1*/

// Le seguenti definizioni contengono l'indirizzo di program memory per salvare
// i puntatori circolari in caso di mancanza di batteria.
#define EEPROM_COUNT_PTR 0x00
#define EEPROM_WRITE_PTR 0x04
#define EEPROM_SEND_PTR 0x08
#define ADC_TIMEOUT_PTR 0x0C
#define EXT1_TIMEOUT_PTR 0x10
#define DATA_DELAY_PTR 0x14
#define BATT_WARN_PTR 0x18
#define BATT_MIN_PTR 0x1C
#define SLEEP_PTR 0x20
#define CATEGORY_PTR 0x24
#define TIME_PTR 0x26
#define BATTERY_PTR  0x2A
#define ERROR_PTR 0x40

#define TIMEOUT_UPDATE_RQST_MS 1000 /**< Periodo di invio delle richieste in centesimi di secondo*/
#define TIMEOUT_CALIB_MODE_S 300 /**< Scandenza della modalità calibrazione in secondi */
#define TIMEOUT_CALIB_SUCCESS_S 10 /**< Scandenza della modalità calibrazione in secondi */

#define EXT1_CALIB_RANGE_MIN 0x4B5  // 0.5 mm
#define EXT1_CALIB_RANGE_MAX 0x579  // -0.5 mm

#define spi_open() {spi_sck_tris = OUTPUT_PIN; spi_sdo_tris = OUTPUT_PIN; spi_sdi_tris = INPUT_PIN; spi_cs_tris = OUTPUT_PIN; OpenSPI2(SPI_FOSC_16, MODE_11, SMPEND);}
#define spi_close() {CloseSPI2(); spi_sck_tris = INPUT_PIN; spi_sdo_tris = INPUT_PIN; spi_sdi_tris = INPUT_PIN; spi_cs_tris = INPUT_PIN;}
/**
 * L'error monitor
 *
 * L'error monitor è uno strumento di debug che memorizza alcune variabili nella
 * eeprom interna allo scadere di un certo periodo TIMEOUT_ERROR_MONITOR_S in cui
 * il modulo rn131 è rimasto acceso.
 * Questo consente al manutentore di trovare in modo più agevole il motivo dello
 * stallo del dispositivo. L'error monitor può essere attivato decommentando
 * il define ERROR_MONITOR e ricompilando il codice.
 */

/**
 * L'avvio del modulo rn131.
 *
 * Per funzionare correttamente, il modulo wifly deve essere configurato in tutte
 * le sue parti, altrimenti non sarà possibile inizializzare l'ora di sistema e
 * far partire l'acquisizione.
 *
 * Esistono 3 diverse modalità all'accensione del modulo:
 * - modalità normale: in questo caso il modulo si è avviato in modalità client
 *     e viene ricevuta la stringa di inizializzazione wifly-GSX Ver: ". A questo
 *     punto il micro tenta di collegarsi alla rete preimpostata. Anche in questo
 *     caso esistono diversi casi:
 *        - associazione all'access point conclusa con successo, condizione verificata
 *          alla ricezione della stringa "Listen on ". Il micro comincerà allora
 *          a richiedere l'aggiornamento dell'ora di sistema. In caso di problemi
 *          viene scaricata la configurazione via ftp, nel caso in cui il problema
 *          sia dovuto alle impostazioni errate. Se anche questo tentativo fallische,
 *          allora si manda il modulo in sleep e si riproverà al prossimo risveglio
 *
 *        - associazione all'access point fallita, condizione risolta dopo che
 *          non è stata ancora ricevuta la stringa "Listen on " allo scadere di
 *          un periodo prestabilito. In questo caso il modulo viene riavviato
 *          in modalità AP per consentire all'utente di selezionare la giusta rete
 *
 * - modalità server, con AP abilitato e DHCP server. Questa condizione si verifica
 *     alla ricezione della stringa "wifly-GSX Ver: " seguita da "AP mode" durante
 *     l'inizializzazione. Da notare che viene comunque ricevuta la stringa
 *     "Listen on ". Quando viene rilevato il suddetto caso, il valore ap_mode
 *     nella struttura rn131 viene impostato: è possibile distinguere questa
 *     modalità da quella del softAP in quanto per il micro non è possibile
 *     avviare l'access point se prima non viene impostata l'ora.
 *
 * - modalità softAP: in questo caso al modulo è stato comandato di partire come
 *     access point con la piattaforma webserver attiva. Tale condizione è
 *     verificata quando, invece della solita stringa d'inizializzazione
 *     "wifly-GSX Ver: ", viene ricevuta "WiFly WebConfig". In questo caso il
 *     microcontrollore non interviene ed attende di ricevere l'altra stringa.
 */

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

/**
 * Offset per il Timer3.
 *
 * Il Timer3 è utilizzato per gestire le tempistiche della 1-wire. Da specifica,
 * per una trasmissione standard, deve avere un periodo di 1us, mentre per una
 * trasmissione overdrive di 0.25us.
 *
 * \f[
 *   n = \frac {1 \times 10^{-6}} {t_{ck} \times 4 \times prescaler} = {10^{-6} \times 4 \times 10^{6}} = 4
 * \f]
 *
 * Per avere un overflow ogni 1 us si dovrà precaricare il registro di:
 *
 * \f[
 *   TIMER\_OFFSET = (2^{16} -1) - n = 65531
 * \f]
 */
#define _1_WIRE_OFFSET 65531

#define END_OF_MESSAGE 0x2a /**< Char che indica la fine del messaggio */
//#define RESET_ON_UART_OVERFLOW /**< Resetta il micro sull'evento di overflow della seriale */
#define REQUEST_NUMBER_OF_TRY 10 /**< Numero di tentativi prima di rinunciare */

#define BATT_AN_CHANNEL ADC_CH31 /**< Porta analogica dove è collegata la batteria */
#define EXT1_AN_CHANNEL ADC_CH2 /**< Porta analogica dove è collegato il sensore EXT1 */
#define EXT2_AN_CHANNEL ADC_CH5 /**< Porta analogica dove è collegato il sensore EXT2 */
#define TEMP_AN_CHANNEL ADC_CH1 /**< Porta analogica dove è collegato il sensore di temperatura */
#define CALIBRATION_AN_CHANNEL ADC_CH0 /**< Riferimento interno a 1.024V usato per calibrare l'adc */
#define BATT_ADC_TIMEOUT_S 60 /**< Tempo tra un'acquisizione e l'altra della batteria*/
#define CIRCULAR_UPDATE_S 3600 /**< Intervallo per il salvataggio automatico dei valori associati al buffer circolare nella eeprom interna */
#define CONNECTION_TIMEOUT_MS 20000 /**< Tempo limite oltre il quale si entra in modalità ap*/
#define EXT1_DELAY_MS 100
#define EEPROM_DELAY_MS 10

#define ADC_FILTER_NUMBER 10 /**< Numero di acquisizioni su cui fare una media*/

#define BATT_CAPACITY 560.0 /**< Capacità della batteria in mAh */

/**
 * La frequenza reale dell'oscillatore interno si aggira circa sui 33.333kHz misurati.
 * Questo significa che il suddetto periodo, moltiplicato per il prescaler di 32768,
 * fornisce una base dei tempi per l'rtc pari a circa 0.983s. Questo significa che,
 * per compensare la maggiore velocità, è necessario togliere ad ogni secondo
 * (1-0.983)=0.01695s.
 */
#define RTCC_ERROR_S 0.01695
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
#pragma config RETEN = 1  /**< Vreg Sleep Enable bit (negate) */
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
int ping_send(void);
void circular_buffer_save(long start_address);
int BCDToDecimal(char bcdByte);
char DecimalToBCD(int decimalByte);
void adc_open(int an_channel);
void sleep_enter(void);

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
volatile char rqst_update_flag = 0;

/**
 * Indica quando si può leggere il valore dell'ultima acquisizione.
 */
volatile char adc_update = 0;

/**
 * Indica quando si possono aggiornare i valori del buffer circolare nella
 * eeprom interna.
 */
volatile char circular_buffer_update = 0;

#ifdef ERROR_MONITOR
/**
 * Indica quando si devono scrivere i valori di debug nella eeprom interna
 */
volatile char error_monitor_update = 0;
#endif

/**
 * Indica quando c'è stato un errore di connessione verso la rete internet.
 *
 * Può assumere tre valori:
 *   0 - nessun errore di connessione
 *   1 - primo tentativo di connessione al web server fallito. Si tenta di scaricare
 *       la configurazione tramite ftp, nel caso uno dei parametri fosse errato.
 *   2 - secondo tentativo fallito. Si azzera il flag e si spegne l'alimentazione
 *       secondaria.
 */
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

/**
 * Tiene conto delle volte che è stato acquisito un canale adc.
 *
 * Viene utilizzata per acquisire in modo sequenziale un canale adc per poi
 * fare la media dei valori.
 */
volatile char adc_acquisition_number = 0;

volatile char calib_mode = 0;

long rn131_timer = 0; /**< tempo trascorso prima di svegliare il modulo rn131 */
unsigned char rn131_start_flag = 0; /**< viene posto alto dal micro appena il modulo viene alimentato */


unsigned char timer_config = 0x00; /**< Registro di configurazione 1 per il Timer*/
unsigned char timer_config1 = 0x00; /**< Registro di configurazione 2 per il Timer*/
//unsigned char _1_wire_timer_config = 0x00; /**< Registro di configurazione 1 per il Timer*/
//unsigned char _1_wire_timer_config1 = 0x00; /**< Registro di configurazione 2 per il Timer*/

/**
 * Contatore per la lettura/scrittura seriale sulla eeprom
 */
unsigned int eeprom_addr_count = 0;

int eeprom_delay_flag = 0; /**< indica se è stato attivato il timer con il tempo necessario alla eeprom per accendersi */
int adc_offset = 0; /**< memorizza l'errore in offset dell'adc */

/* S  T R U C T *********************************************************/
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
    } timeout_update_adc_ext1_ms; /**< Periodo di acquisizione dall'adc per il sensore EXT1 */

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
    unsigned battery_undervoltage :1; /**< batteria scesa sotto 2V */
    unsigned spi_bus_collision :1; /**< collisione sul bus spi */
    unsigned eeprom_error :2; /**< 01b: memoria piena, 10b: errore di scrittura, 11b: eeprom assente */
    unsigned micro_reset_fault :1; /**< reset inaspettato */
    unsigned uart_overflow :1; /**< uart overflow */
    unsigned rttc_update_fail :1; /**< rtcc fault*/
  } bits;
} status_flags;

char buffer[UART2_BUFFER_SIZE_RX];/**< buffer globale d'appoggio sia in lettura che in scrittura*/

/**
 *  Indica quando il micro può inviare dati sulla seriale.
 *
 * Il microcontrollore può inviare i dati solo quando è certo che il dispositivo
 * rn131 sia sveglio, associato ad un access point ed il web server sia in ascolto.
 * Un'altra condizione necessaria è che l'ora sia stata acquisita correttamente.
 * Tutto questo viene riflesso sulla variabile communication_ready.
 * Quando viene impostato ad 1, significa che è stata mandata una richiesta GET
 * vuota per controllare che il webserver risponda.
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
  static short long adc_timer_ext1 = 0; /**< tiene il tempo trascorso tra un'acquisizione e l'altra dell'EXT1 */
  static short long adc_timer_batt = BATT_ADC_TIMEOUT_S; /**< tiene il tempo trascorso tra un'acquisizione e l'altra della batteria. */
  static char rn131_sleep_flag = 0;


  if(UART2_INTERRUPT_RX > 0)
  {
    if(uart2_isr())
      return;
  }

  if((rn131_int_flag == 1) && (rn131_int_enable == 1))
  {
    static int rn131_rts_pin = 0;
    
    rn131_int_flag = 0;

    if(rn131_rts_pin == 0)
    {
      rn131_rts_pin = 1;

      if(rn131_start_flag == 1)
      {
        rn131_int_edge = 0;
        
        // Reset status flags
        rn131_reset_flag();

        communication_ready = 0;
        
        rn131.wakeup = 1;
      }
      else
      {
        // non voglio il modulo acceso, quindi lo spengo subito
        rn131_sleep_flag = 1;
      }

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
    static short long circular_timer = 0; /**< tiene il tempo per il led di stato */
    static int rqst_update_timer = 0; /**< tempo trascorso prima di abilitare l'invio di una nuova richiesta */
    static short long connection_timer = 0; /**< tiene il tempo per lo scadere del tentativo di connession */
    static int calib_timer = 0; /**< tempo trascorso prima di tornare nella modalità normale */
    static int calib_success_start = 0; /**< detiene la prima occorrenza di calibrazione ottima senza interruzioni */
    static const int timeout_update_rqst_s = TIMEOUT_UPDATE_RQST_MS / 1000;
    static const int timeout_connection_s = CONNECTION_TIMEOUT_MS / 1000;

    PIR3bits.RTCCIF = 0;
    //led_err = ~led_err;
    
#ifdef ERROR_MONITOR
    static signed char error_monitor_timer = 0; /**< tempo trascorso prima di lanciare il report */
#endif
    
    if(rn131.ap_mode == 0)
    {
      if(rn131.time_set == 1)
      {
        if(rn131_start_flag == 0)
        {
          if(RCSTA2bits.SPEN == 1)
          {
            uart2_close();
            uart2_tx_tris = INPUT_PIN;
            uart2_rx_tris = INPUT_PIN;
          }

          if(calib_mode == 0)
          {
            rn131_timer++;
            if((rn131_timer >= configuration.timeout_sleep_s.value) && (an_channel == -1) && (alim_3_3V_enable == 0))
            {
              rn131_timer = 0;
              rn131_start_flag = 1;

              OSCCONbits.IDLEN = 0;

              uart2_tx_tris = OUTPUT_PIN;
              uart2_rx_tris = INPUT_PIN;
              uart2_open(9600);

              alim_3_3V_enable =  1;
            }
          }
        }
        else //if(rn131_start_flag == 0)
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
             rn131.time_set_rqst || rn131.cmd_http_rqst)
          {
            rqst_update_timer++;

            if(rqst_update_timer == timeout_update_rqst_s)
            {
              rqst_update_timer = 0;
              rqst_update_flag = 1;
            }
          }
          else
            rqst_update_timer = 0;

          if((rn131.ready == 1) && (rn131.connected == 0))
          {
            connection_timer++;

            if(connection_timer >= timeout_connection_s)
            {
              connection_timer = 0;
              config_error = 1;
            }
          }
          else if(connection_timer != 0)
            connection_timer = 0;
        }
      }

      if(calib_mode == 0)  // non voglio sovrapposizione di luci durante la modalità calibrazione
      {
        circular_timer++;
        if(circular_timer >= CIRCULAR_UPDATE_S)
        {
          circular_timer = 0;
          circular_buffer_update = 1;
        }

        if((rn131.time_set == 1) && (rn131.wakeup == 0) && (rn131_start_flag == 0))
        {
          adc_timer_batt++;
          adc_timer_temp += 1000;
          adc_timer_ext1 += 1000;

          if(adc_offset == 0)
          {
            if((ADCON0bits.GO == 0) && (an_channel == -1))
            {
              adc_update = 1;
              an_channel = CALIBRATION_AN_CHANNEL;
              OSCCONbits.IDLEN = 0;
              SelChanConvADC(an_channel);
            }
          }

          if(adc_timer_batt >= BATT_ADC_TIMEOUT_S)
          {
            if((ADCON0bits.GO == 0) && (an_channel == -1))
            {
              adc_update = 1;
              adc_timer_batt = 0;
              an_channel = BATT_AN_CHANNEL;
            }
          }
          else if((adc_timer_temp >= configuration.timeout_update_adc_temp_ms.value) ||
                  (adc_timer_ext1 >= configuration.timeout_update_adc_ext1_ms.value)
                 )
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
      }
      else //if(calib_mode == 0)
      {
        calib_timer++;

        if((led_err != 0) || (led_no_err != 1))
          calib_success_start = calib_timer;

        if((calib_timer >= TIMEOUT_CALIB_MODE_S) || ((calib_timer - calib_success_start) >= TIMEOUT_CALIB_SUCCESS_S))
        {
          led_err = 0;
          led_no_err = 0;
          calib_timer = 0;
          calib_success_start = 0;

          adc_timer_ext1 = 0;
          if(T1CONbits.TMR1ON == 1)
            CloseTimer1();
          an_channel = -1;
          ext_shut = 0;
          OSCCONbits.IDLEN = 0;
          
          calib_mode = 0;
        }
        else
        {
          // avvio l'acquisizione del canale
          if((ADCON0bits.GO == 0) && (an_channel == -1))
          {
            adc_timer_ext1 = configuration.timeout_update_adc_ext1_ms.value;

            if(!T1CONbits.TMR1ON)
            {
              OSCCONbits.IDLEN = 1;
              WriteTimer1(TIMER_OFFSET);
              OpenTimer1(timer_config, timer_config1);
            }
          }
        }
      }

      if(tcp_start_timer_flag > 0)
      {
        tcp_start_timer_flag++;

        if(tcp_start_timer_flag >= configuration.timeout_data_eeprom_s.value)
          tcp_start_timer_flag = 0;
      }
    }

#ifdef ERROR_MONITOR
    if((alim_3_3V_enable == 1) && (error_monitor_timer >= 0))
    {
      if(rn131.ap_mode == 0)
      {
        error_monitor_timer++;
      }
      else
        error_monitor_timer = 0;
    }
    else
      error_monitor_timer = 0;

    if(error_monitor_timer >= TIMEOUT_ERROR_MONITOR_S)
    {
      error_monitor_timer = -1;
      error_monitor_update = 1;
    }
#endif
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

    /**
     * \attention Il timer1 sostituisce l'rtc finchè questo non è stato inizializzato,
     * evento che avviene dopo aver acquisito l'ora.
     */
    if(rn131.ap_mode == 0)
    {
      // Questo if serve soltanto per inizializzare l'ora, quindi ho bisogno di
      // sapere quando avvisare dell'errore e di inoltrare le richieste in modo
      // regolare. Una volta acquisita l'ora, non c'è  più motivo di attivare il
      // timer1 mentre il modulo wifi è acceso.
      if((rn131.ready == 1) && (rn131.time_set == 0))
      {
        connection_timer++;

        if(connection_timer >= CONNECTION_TIMEOUT_MS)
        {
          connection_timer = 0;
          config_error = 1;
        }

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
           rn131.time_set_rqst || rn131.cmd_http_rqst)
        {
          rqst_update_timer++;

          if(rqst_update_timer == TIMEOUT_UPDATE_RQST_MS)
          {
            rqst_update_timer = 0;
            rqst_update_flag = 1;
          }
        }
        else
          rqst_update_timer = 0;
      }

      if(rn131_sleep_flag == 1)
      {
        adc_batt_enable_tris = OUTPUT_PIN;
        adc_batt_enable = 1;
        rn131_sleep_flag++;
      }
      else if(rn131_sleep_flag == 2)
      {
        adc_batt_enable = 0;
        adc_batt_enable_tris = INPUT_PIN;
        rn131_sleep_flag = 0;
      }
      
      if((rn131.time_set == 1) && (rn131.wakeup == 0) && (rn131_start_flag == 0))
      {
        adc_timer_ext1++;
        adc_timer_temp++;

        if(adc_timer_ext1 >= configuration.timeout_update_adc_ext1_ms.value)
        {
          // Avvia acquisizione
          if((ADCON0bits.GO == 0) && (an_channel == -1))
          {
            // Il dispositivo ha bisogno di almeno 3 ms per tirare fuori un segnale valido.
            // Inoltre anche lo step-up interno ha bisogno di un certo tempo di assestamento.
            // 100 ms di attesa sono sufficienti per ottenere un segnale valido.
#if(EXT1_DELAY_MS > EEPROM_DELAY_MS)
            if(eeprom_delay_flag < EXT1_DELAY_MS)
#else
            if(eeprom_delay_flag < EEPROM_DELAY_MS)
#endif
            {
              ext_shut = 1;

              eeprom_delay_flag++;
              
              alim_3_3V_enable =  1;
              adc_timer_ext1 = configuration.timeout_update_adc_ext1_ms.value;
            }
            else
            {
              adc_timer_ext1 = 0;
              an_channel = EXT1_AN_CHANNEL;
              update_date(RTCC_ERROR_S);
              OSCCONbits.IDLEN = 0;
              SelChanConvADC(an_channel);
            }
          }
        }
        else if(adc_timer_temp >= configuration.timeout_update_adc_temp_ms.value)
        {
          // Avvia acquisizione
          if((ADCON0bits.GO == 0) && (an_channel == -1))
          {
            // Il dispositivo ha bisogno di almeno 3 ms per tirare fuori un segnale valido.
            // Inoltre anche lo step-up interno ha bisogno di un certo tempo di assestamento.
            // 100 ms di attesa sono sufficienti per ottenere un segnale valido.
            if(eeprom_delay_flag < EEPROM_DELAY_MS)
            {
              eeprom_delay_flag++;
              alim_3_3V_enable =  1;
              adc_timer_temp = configuration.timeout_update_adc_temp_ms.value;
            }
            else
            {
              adc_timer_temp = 0;
              an_channel = TEMP_AN_CHANNEL;
              update_date(RTCC_ERROR_S);
              OSCCONbits.IDLEN = 0;
              SelChanConvADC(an_channel);
            }
          }
        }

        // Se per ogni acquisizione devo attendere un tempo maggiore di EEPROM_DELAY_MS allora
        // mi conviene spegnere l'alimentazione secondaria, altrimenti rimane accesa per non
        // dover attendere di nuovo il tempo di setup della flash.
        if((adc_timer_temp < (configuration.timeout_update_adc_temp_ms.value - EEPROM_DELAY_MS)) &&
#if(EXT1_DELAY_MS > EEPROM_DELAY_MS)
           (adc_timer_ext1 < (configuration.timeout_update_adc_ext1_ms.value - EXT1_DELAY_MS)) &&
#else
           (adc_timer_ext1 < (configuration.timeout_update_adc_ext1_ms.value - EEPROM_DELAY_MS)) &&
#endif
           (an_channel == -1) && (adc_acquisition_number == 0)
          )
        {

          // Se manca più di un secondo allo scadere dei timer, allora posso chiudere il timer
          // ed affidarmi all'rtc.
          if((adc_timer_temp < (configuration.timeout_update_adc_temp_ms.value - 1000)) &&
#if(EXT1_DELAY_MS > EEPROM_DELAY_MS)
             (adc_timer_ext1 < (configuration.timeout_update_adc_ext1_ms.value - 1000))
#else
             (adc_timer_ext1 < (configuration.timeout_update_adc_ext1_ms.value - 1000))
#endif
             )
          {
            CloseTimer1();
          }
          
          alim_3_3V_enable =  0;
          eeprom_delay_flag = 0;
        }
      }
      else
      {
        adc_timer_ext1 = 0;
        adc_timer_temp = 0;
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
   * - la seriale ha finito di inviare tutti i dati
   * - non devo aggiornare i dati di errore nella eeprom interna
  */
  if(((rn131.time_set == 1) && (communication_ready == 0)) && (adc_update == 0) && 
     (circular_buffer_update == 0) && (error_monitor_update != 1) && (eeprom_at_work == 0) &&
     (rn131.wakeup == 0) && (rn131_start_flag == 0) && (uart2_status.buffer_tx_empty == 1) &&
     !BusyADC() && (TXSTA2bits.TRMT == 1))
  {
    SLEEP();
  }
}

/*
 * Modifiche sulla scheda SMPPC-EL-001-01 (il simbolo * significa "necessaria")
 *   - dissaldare il transistor Q1 ed i componenti associati (R1, R2, R4, R5, R6, C2).
 *       Ora la temperatura viene letta dal modulo STC3100
 *   * - Cortocircuitare i pin 4 e 5 del connettore USB. Il pin di ground è il 5,
 *       ma nel circuito la massa è collegata al 4.
 *   - abilitare il regolatore interno del modulo rn131 lasciando le resistenze
 *     RP1 e RP2 e rimuovendo RP3. Nonostante sul datasheet si afferma che senza il
 *     regolatore la tensione di alimentazione può scendere fino a 2.5V, in realtà
 *     non si può andare sotto i 3.1V. Nel caso di regolatore spento, si è
 *     sperimentato che la tensione di alimentazione può arrivare fino a 2.5V.
 *   - Se si utilizzano le linee AN2 e AN6, rimuovere le resistenze R39 e R40.
 *   - Saldare una resistenza da 2k al posto di R6. In questo modo il firmware
 *       può effettuare la calibrazione andando a campionare il canale 0
 *   - Cortocircuitare il pin 25 di rn131 con il pad di R2 (che è stata precedentemente
 *     dissaldata), quello opposto pin adc_bat. Tramite questa modifica è possibile
 *     mandare il modulo in sleep senza dover inviare dati sulla seriale e quindi molto più
 *     velocemente. Montare al posto di R2 una resistenza da 10k per limitare la corrente
 *   - Dissaldare tutte le resistenze ed i condensatori dell'adc. Dovrà essere il circuito
 *     di condizionamento del sensore a dover rispettare la resistenza di carico e la tensione
 *     massima consentita. Collegare delle resistenza da 0 ohm per mantenere il collegamento.
 *
 *
 */
/**
 * \brief Program's entry.
 *
 * @return Error code for exit
 *
 * @bug
 *   - prima di provare la presenza del webserver, sono costretto a fare un ping a google.it, altrimenti
 *     la connessione non va a buon fine "Connection FAILED".
 *   - se attivo l'interrupt sulla seriale, il micro si resetta
 *   - il primo invio viene saltato
 *
 *
 * @todo
 * - aggiornare offset adc ad intervalli regolari
 * - speficare nella documentazione che ext2 viene acquisito subito dopo ext1 e viene inviato come variabile
 *   di supporto con il codice 0x03
 * - testare se il battery gauge misura bene
 * - scrivere sulla eeprom solo dopo 32 byte, così da ridurre il consumo di corrente
 * - usare lvd feature per capire quando la batteria è scarica
 * - aggiungere comando "reset parametri"
 * - capire se un modulo è stato configurato attraverso la versione del firmware
 * - reset del micro da remoto
 * - provare l'interrupt sul fronte del pin b3 quando viene tolta la batteria
 * - invece di fare il polling sui messaggi inviati dal modulo, un altro modo per
 *   capire se il modulo è stato associato all'access point è tramite il comando
 *   "show net" oppure "show connection
 * - descrivere come è stato impostato il tempo di campionamento dell'adc
 * - scrivere documentazione temporizzazione
 * - scrivere documentazione
 *
 */
int main(void)
{
  char ascii_buffer[256];
  char end_of_message[2] = {END_OF_MESSAGE, 0};
  int ascii_buffer_size = 0;
  long timeout_update_ms_temp = 0;

  char *cmd_http_start = rn131.cmd_http;
  char *cmd_http_mark = NULL;
  char cmd_http_parse[32];

  unsigned int uart_empty_space = 0;
  char uart_token[] = {'\n', '*'};
  unsigned short long eeprom_data_to_send = 0; /**< numero di byte caricati dall'eeprom e pronti per essere inviati*/
  udiv_t data_frame_number; /**< da qui prendo il quoziente per il calcolo del numero massimo di byte che posso leggere dalla eeprom */

  float battery_value_temp = 0; /**< Valore di apporggio per impostare il livello della batteria da remoto*/

  long adc_filtered = 0;
  
  int winbond_return_value;
  int ftp_update_return_value = 0;
  int ping_result = 0;

  float batt_soc = 0; /**< carica della batteria in percentuale */

  WDTCONbits.REGSLP = 1; // on-chip regulator enters low-power operation when device enters in Sleep mode
  OSCTUNEbits.INTSRC = 0;
  OSCCON2bits.MFIOSEL = 0;
  OSCCONbits.IRCF = 7;
  OSCTUNEbits.PLLEN = 0;
  OSCCONbits.IDLEN = 1;
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
    led_err = 1;
    led_no_err = 1;
  }

  // reset instruction
  if(RCONbits.RI == 0)
  {
    status_flags.bits.micro_reset_fault = 1;
    RCONbits.RI = 1;
    led_err = 1;
    led_no_err = 1;
  }

  // configuration mismatch reset
  if(RCONbits.CM == 0)
  {
    status_flags.bits.micro_reset_fault = 1;
    RCONbits.CM = 1;
    led_err = 1;
    led_no_err = 1;
  }

  /************ INIT GLOBAL ***********/
  configuration.timeout_sleep_s.value = 240;
  configuration.timeout_update_adc_temp_ms.value = 30000;
  configuration.timeout_update_adc_ext1_ms.value = 30000;
  configuration.timeout_data_eeprom_s.value = 10;
  configuration.battery_warn_value.value = 3.2;
  configuration.battery_min_value.value = 3.0;
  configuration.category.value = 2;
  winbond_return_value = 0;

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
  SSPADD = FOSC_HZ / (4 * 300000) - 1;

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
  spi_cs = 1;

  CloseSPI2();
  spi_open();

  /************ INIT TIMER *********/
  CloseTimer0();
  CloseTimer1();
  timer_config = T1_16BIT_RW | T1_SOURCE_FOSC_4 | T1_PS_1_4 | T1_OSC1EN_OFF
          | T1_SYNC_EXT_OFF | TIMER_INT_ON;

  timer_config1 = TIMER_GATE_OFF;
  
  OpenTimer1(timer_config, timer_config1);
  WriteTimer1(TIMER_OFFSET);

  /*_1_wire_timer_config = T3_16BIT_RW | T3_SOURCE_FOSC_4 | T3_PS_1_4 | T3_OSC1EN_OFF
          | T3_SYNC_EXT_OFF | TIMER_INT_OFF;

  _1_wire_timer_config1 = TIMER_GATE_OFF;
  WriteTimer3(_1_WIRE_OFFSET);*/
  
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
  rn131.web_server_mode = 0;
  rn131.cmd_mode_rqst = 0;
  rn131.cmd_mode_exit_rqst = 0;
  rn131.cmd_mode_reboot_rqst = 0;
  rn131.cmd_http_rqst = 0;
  rn131.time_set = 0;
  rn131.time_set_rqst = 0;
  
  int rn131_message_length = 0;

  rn131_start_flag = 1;
  alim_3_3V_enable =  1;
  
  memset(winbond.device_id, 0, sizeof(winbond.device_id));
  memset(ascii_buffer, 0, sizeof(ascii_buffer));

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
    configuration.timeout_sleep_s.value = 240;
  }
  
  eeprom_at_work = 0;

  status_flags.byte = 0;
  
  __delay_ms(10);
  
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
    status_flags.bits.eeprom_error = 3;

  if(SSP2CON1bits.SSPEN == 1)
  {
    spi_close();
  }
  
  while(!done)
  {
    if(UART2_INTERRUPT_RX == 0)
      uart2_buffer_rx_load(); //Lettura del buffer di ricezione per aumentarne la frequenza

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
          rn131.time_set_rqst = -1;
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
            if(ap_mode() == 0)
            {
              rn131.cmd_http_rqst = 2; // rientro al prossimo aggiornamento di richiesta
              break;
            }
            else
            {
               rn131.cmd_http_rqst = 2;
              *cmd_http_start = 0; //annullo tutte le altre richieste

              // da questo punto in poi il modulo deve entrare in access point
              // prima che la richiesta venga rinnovata.

              break;
            }
          }
          else if(strncmp(cmd_http_parse, "calib", 5) == 0)
          {
            // Se il sensore permette questa possibilità, entro nella modalità
            // calibrazione
            if(configuration.category.value == 0x02)
              calib_mode = 1;
          }
          else if(strncmp(cmd_http_parse, "set_sleep=", 10) == 0)
          {
            configuration.timeout_sleep_s.value = atol(cmd_http_parse + 10);
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
          }
          else if((strncmp(cmd_http_parse, "set_config", 10) == 0) && (rn131.cmd_mode == 0))
          {
            circular_buffer_save(EEPROM_COUNT_PTR);
          }
          else if((strncmp(cmd_http_parse, "get_batt", 8) == 0) && (rn131.cmd_mode == 0))
          {
            ascii_buffer[0] = 0;

            if(buffer[0] == 0)
              sprintf(buffer, "%sx:", rn131.mac);

            sprintf(ascii_buffer, "batt=%.0f,", batt_soc);
            strcat(buffer, ascii_buffer);
          }
          else if((strncmp(cmd_http_parse, "set_category=", 13) == 0) && (rn131.cmd_mode == 0))
          {
            configuration.category.value = atoi(cmd_http_parse + 13);
          }
          else if((strncmp(cmd_http_parse, "set_adc=", 8) == 0) && (rn131.cmd_mode == 0))
          {
            timeout_update_ms_temp = atol(cmd_http_parse + 8);

            if(timeout_update_ms_temp > 0)
              configuration.timeout_update_adc_ext1_ms.value = timeout_update_ms_temp;
          }
          else if((strncmp(cmd_http_parse, "set_adc_temp=", 13) == 0) && (rn131.cmd_mode == 0))
          {
            timeout_update_ms_temp = atol(cmd_http_parse + 8);

            if(timeout_update_ms_temp > 0)
              configuration.timeout_update_adc_temp_ms.value = timeout_update_ms_temp;
          }
          else if((strncmp(cmd_http_parse, "set_data_delay=", 15) == 0) && (rn131.cmd_mode == 0))
          {
            timeout_update_ms_temp = atol(cmd_http_parse + 15);

            if(timeout_update_ms_temp > 0)
              configuration.timeout_data_eeprom_s.value = timeout_update_ms_temp;
          }
          else if((strncmp(cmd_http_parse, "set_batt_warn=", 14) == 0) && (rn131.cmd_mode == 0))
          {
            battery_value_temp = atof(cmd_http_parse + 14);

            if(battery_value_temp > 2.7)
              configuration.battery_warn_value.value = battery_value_temp;
          }
          else if((strncmp(cmd_http_parse, "set_batt_min=", 13) == 0) && (rn131.cmd_mode == 0))
          {
            battery_value_temp = atof(cmd_http_parse + 13);

            if(battery_value_temp >= 2.7)
              configuration.battery_min_value.value = battery_value_temp;
          }

          cmd_http_start = cmd_http_mark + 1;
          cmd_http_mark = strchr(cmd_http_start, '\r');

          if(cmd_http_mark == NULL)
            rn131.cmd_http_rqst = 0;
        }

        if(buffer[0] != 0)
        {
          strcat(buffer, end_of_message);

          ascii_buffer_size = strlen(buffer);
          uart2_buffer_tx_seq_load(buffer, ascii_buffer_size);

          while(uart2_status.buffer_tx_empty == 0)
          {
            uart2_buffer_send();

            if(UART2_INTERRUPT_RX == 0)
              uart2_buffer_rx_load();
          }

          // resetto il buffer, così che i valori vengano appesi partendo dall'inizio
          ascii_buffer[0] = 0;
          tcp_start_timer_flag = 1;
        }
      }

      rqst_update_flag = 0;
    }

    // se non ho dati dell'adc da inviare, richieste da servire, dati da inviare
    // sulla uart ed ho già verificato la presenza del web server (per prendere
    // eventuali richieste), allora posso spegnere il dispositivo
    if(((rn131.wakeup == 1) || (rn131_start_flag == 1)) && (rn131.time_set == 1) && (rn131.ap_mode == 0) &&
       (rn131.cmd_http_rqst == 0) && (eeprom_data_count == 0) && (TXSTA2bits.TRMT == 1) &&
       (communication_ready == 1) && (tcp_start_timer_flag == -1) &&  (rn131.cmd_mode_reboot_rqst == 0))
      sleep_enter();

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

        if(UART2_INTERRUPT_RX == 0)
          uart2_buffer_rx_load(); //Lettura del buffer di ricezione per aumentarne la frequenza

        if(rn131_message_length > 0)
        {
          // Check for a new connection
          if(rn131_connection_init(buffer, rn131_message_length) == 1)
          {
            buffer[0] = '\0';
            if((rn131.web_server_mode == 1) && (rn131.ap_mode == 0)) // sono tornato dalla modalità web_server
            {
              rn131.web_server_mode = 0;
              
              if(rn131.time_set == 1)
                sleep_enter();

            }
            
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

            if((rn131.connected == 1) && (rn131.time_set == 1))
            {
              if(rn131.tcp_open == 0)
              {
                if(rn131_parse_message(buffer, "OPEN"))
                {
                  rn131.tcp_open = 1;
                  tcp_start_timer_flag = 1;

                  continue;
                }

                if(rn131_parse_message(buffer, "Connect FAILED") || rn131_parse_message(buffer, "HTTP/1.0 404 Not Found"))
                {
                  rn131.tcp_error = 1;
                  sleep_enter();

                  continue;
                }
              }
              else
              {
                if((rn131.cmd_http_rqst == 0) && (rn131_http_response(buffer)))
                {
                  strcpy(rn131.cmd_http, buffer);
                  continue;
                }

                if(rn131_parse_message(buffer, "CLOS"))
                {
                  rn131.tcp_open = 0;

                  tcp_start_timer_flag = configuration.timeout_data_eeprom_s.value - 1;  // diminuisco il tempo di attesa ad 1 secondo
                  if(rn131.cmd_http[0] != 0)
                  {
                    rn131.cmd_http_rqst = 1;
                    cmd_http_start = rn131.cmd_http;
                  }
                  continue;
                }
              }
            }
          }
          else //if(rn131.cmd_mode == 0)
          {
            if(rn131.cmd_mode_exit_rqst > 0)
            {
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
                      if(T1CONbits.TMR1ON == 1)
                        CloseTimer1();
                      
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

                      if(UART2_INTERRUPT_RX == 0)
                        uart2_buffer_rx_load(); //Lettura del buffer di ricezione per aumentarne la frequenza


                      // It need to be rebooted otherwise the web server will kick off
                      // everyone trying to connect to it
                      if(rn131.cmd_mode_reboot_rqst == 0)
                      {
                        message_load_uart("reboot\r");

                        rn131_reset_flag();
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


    if(UART2_INTERRUPT_RX == 0)
      uart2_buffer_rx_load(); //Lettura del buffer di ricezione per aumentarne la frequenza


    /***********************************************************  Update  ***/
    if(rn131.connected == 1) // implica anche che il modulo è acceso
    {
      if(rn131.time_set == 0)
      {
        if(config_error == 1)
        {
          rn131.time_set_rqst = 0;
          ftp_update_return_value = download_configuration_ftp();
          
          if(ftp_update_return_value == 1)
          {
            config_error = 0;
            rn131.time_set_rqst = 1;
          }
          else if(ftp_update_return_value == -1)
            config_error = 0;
        }
        else
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
            if(rn131.ap_mode == 0)
            {
              if(rn131.time_set_rqst == 0)
              {
                message_load_uart("time\r");

                rn131.time_set_rqst = 1;
              }
            }
            else
            {
              message_load_uart("set wlan join 1\rset ip dhcp 3\rsave\r");
              message_load_uart("reboot\r");

              rn131_reset_flag();
              communication_ready = 0;
              tcp_start_timer_flag = -1;
              rn131.cmd_mode_reboot_rqst = 1;
            }
          }
        }
      }
      else if(communication_ready == 0)  // verifico che il webserver sia presente
      {
        ping_result = ping();
        if(ping_result == 1)
        {
          sprintf(buffer, "%s%s", rn131.mac, end_of_message);
          uart2_buffer_tx_seq_load(buffer, strlen(buffer));

          while(uart2_status.buffer_tx_empty == 0)
          {
            uart2_buffer_send();

            if(UART2_INTERRUPT_RX == 0)
              uart2_buffer_rx_load();
          }
  
          communication_ready = 1;
          tcp_start_timer_flag = 1;
        }
        else if(ping_result == -1)
        {
          sleep_enter();
        }
      }
      else if((rn131.tcp_error == 0) && (tcp_start_timer_flag == 0))
      {
        // se mi trovo in modalità comandi, esco
        if(rn131.cmd_mode == 1)
        {
          if(rn131.cmd_mode_exit_rqst == 0)
          {
            message_load_uart("exit\r");
            rn131.cmd_mode_exit_rqst = 1;
          }
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
              spi_open();
            }

            // Voglio convertire i dati esadecimali in stringhe ascii: visto che ogni
            // byte è rappresentato da 2 caratteri, ho bisogno del doppio dello spazio
            // nel buffer di trasmissione. Inoltre voglio inserire il MAC address
            // del dispositivo rn131, che è di 18 caratteri
            uart_empty_space = (uart2_get_tx_buffer_empty_space() >> 1) - 18;
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
                sprintf(ascii_hex, "%.02x", buffer[i]);
                strcat(ascii_buffer,  ascii_hex);
              }

              strcat(ascii_buffer, end_of_message);

              ascii_buffer_size = strlen(ascii_buffer);
              uart2_buffer_tx_seq_load(ascii_buffer, ascii_buffer_size);

              while(uart2_status.buffer_tx_empty == 0)
              {
                uart2_buffer_send();

                if(UART2_INTERRUPT_RX == 0)
                  uart2_buffer_rx_load();
              }

              // resetto il buffer, così che i valori vengano appesi partendo dall'inizio
              ascii_buffer[0] = '\0';

              tcp_start_timer_flag = 1;
            }

            if(SSP2CON1bits.SSPEN == 1)
            {
              spi_close();
            }
          } //if(eeprom_data_count > 0)
        }
      }
    }
    else //if(rn131.connected == 1)
    {
      if(config_error == 1)
      {
        if(ap_mode())
          config_error = 0;
      }
    }

    if(error_monitor_update == 1)
    {
      int error_monitor_count = 0;
      error_monitor_update = 0;

      // Memorizzo lo stato in cui si è verificato lo stallo del dispositivo
      led_err = 1;
      circular_buffer_save(ERROR_PTR);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 2, T1CONbits.TMR1ON); // ptr = 0x66
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 3, rn131_start_flag);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 4, an_channel >> 8);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 5, an_channel & 0x00ff);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 6,  adc_update);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 7,  RCSTA2bits.SPEN);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 8,  SSP2CON1bits.SSPEN);

      for(error_monitor_count = 0; error_monitor_count < 30; error_monitor_count++)
        Write_b_eep(ERROR_PTR + BATTERY_PTR + 9 + error_monitor_count,  buffer[error_monitor_count]);

      Write_b_eep(ERROR_PTR + BATTERY_PTR + 39, rn131.wakeup);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 40, rn131.ap_mode);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 41, rn131.cmd_mode);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 42, rn131.cmd_http_rqst);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 43, rn131.cmd_mode_exit_rqst);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 44, rn131.cmd_mode_reboot_rqst);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 45, rn131.cmd_mode_rqst);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 46, rn131.connected);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 47, rn131.ready);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 48, rn131.tcp_error);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 49, rn131.tcp_open);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 50, rn131.time_set);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 51, rn131.time_set_rqst);

      Write_b_eep(ERROR_PTR + BATTERY_PTR + 52, uart2_status.buffer_rx_empty);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 53, uart2_status.buffer_rx_error_frame);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 54, uart2_status.buffer_rx_full);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 55, uart2_status.buffer_rx_micro_overflow);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 56, uart2_status.buffer_tx_empty);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 57, uart2_status.buffer_tx_full);

      Write_b_eep(ERROR_PTR + BATTERY_PTR + 58, communication_ready);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 59, tcp_start_timer_flag & 0x00ff);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 60, tcp_start_timer_flag >> 8);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 61, eeprom_data_to_send & 0x0000ff);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 62, (eeprom_data_to_send >> 8) & 0x0000ff);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 63, eeprom_data_to_send >> 16);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 64, data_frame_number.quot);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 65, rqst_update_flag);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 66, config_error);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 67, status_flags.bits.battery_undervoltage);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 68, status_flags.bits.battery_warning);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 69, status_flags.bits.eeprom_error);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 70, status_flags.bits.micro_reset_fault);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 71, status_flags.bits.rttc_update_fail);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 72, status_flags.bits.spi_bus_collision);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 73, status_flags.bits.uart_overflow);

      Write_b_eep(ERROR_PTR + BATTERY_PTR + 74, eeprom_data_count & 0x0000ff);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 75, (eeprom_data_count >> 8) & 0x0000ff);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 76, eeprom_data_count >> 16);
      Write_b_eep(ERROR_PTR + BATTERY_PTR + 77, eeprom_data_count >> 24);

      Write_b_eep(ERROR_PTR + CATEGORY_PTR + 78, time_by_rn131 & 0x0000ff);
      Write_b_eep(ERROR_PTR + CATEGORY_PTR + 79, (time_by_rn131 >> 8) & 0x0000ff);
      Write_b_eep(ERROR_PTR + CATEGORY_PTR + 80, time_by_rn131 >> 16);
      Write_b_eep(ERROR_PTR + CATEGORY_PTR + 81, time_by_rn131 >> 24);
    }

    if((rn131.time_set == 1) && (rn131.wakeup == 0) && (rn131_start_flag == 0))
    {
      if(circular_buffer_update == 1)
      {
        circular_buffer_save(EEPROM_COUNT_PTR);
        circular_buffer_update = 0;
      }

      if(adc_update == 1)
      {
        if((an_channel > -1) && (SSP2CON1bits.SSPEN == 0))
        {
          spi_open();
        }

        switch(an_channel)
        {
          case BATT_AN_CHANNEL:
            OpenI2C1(MASTER, SLEW_OFF);

            // Battery voltage reading
            stc3100_get(STC3100_ADDR, STC3100_CTRL, 1, &stc3100_ctrl.bytes);

            if(stc3100_ctrl.bits.pordet == 1)
            {
              status_flags.bits.battery_undervoltage = 1;

              // devo far ripartire il gauge
              stc3100_write(STC3100_ADDR, STC3100_MODE, 0x10);
            }
            else
            {
              if(stc3100_ctrl.bits.gg_eoc == 1)
              {
                // Leggo lo stato della batteria
                // Coulomb counter reading
                stc3100_get(STC3100_ADDR, STC3100_CHARGE_LOW, 4, stc3100_charge_state.bytes);
                stc3100_soc_mAh = 6.70 * (float)(stc3100_charge_state.reg.charge.integer / STC3100_RESISTOR_MOHM);

                if(stc3100_soc_mAh <= 0)
                  batt_soc = (BATT_CAPACITY + stc3100_soc_mAh) * 100 / BATT_CAPACITY;
                else
                  batt_soc = 100;
                
                // Battery current reading
                /*stc3100_get(STC3100_ADDR, STC3100_CURRENT_LOW, 2, stc3100_current.bytes);
                if((stc3100_current.integer & 0x2000) > 0)
                  stc3100_current.integer |= 0xE000;

                stc3100_current_value_mA = 11.77 * (float)stc3100_current.integer / STC3100_RESISTOR_MOHM;*/
              }

              if(stc3100_ctrl.bits.gg_vtm_eoc == 1)
              {
                stc3100_get(STC3100_ADDR, STC3100_VOLTAGE_LOW, 2, stc3100_voltage.bytes);

                if((stc3100_voltage.integer & 0x0800) > 0)
                  stc3100_voltage.integer |= 0xF000;

                stc3100_voltage_value_mV = 2.44 * stc3100_voltage.integer / 1000;

                if((stc3100_voltage_value_mV) <= configuration.battery_warn_value.value)
                  status_flags.bits.battery_warning = 1;

                if((stc3100_voltage_value_mV) <= configuration.battery_min_value.value)
                {
                  circular_buffer_save(EEPROM_COUNT_PTR);

                  // disabilito tutte le periferiche
                  di();
                  mRtccAlrmDisable();
                  mRtccOff();

                  CloseTimer1();
                  CloseADC();
                  ADC_INT_DISABLE();

                  // Disabilito il modulo STC3100
                  stc3100_write(STC3100_ADDR, STC3100_MODE, 0x00);
                  CloseI2C1();

                  spi_close();

                  // disalimento tutto
                  alim_3_3V_enable = 0;

                  // buona notte
                  OSCCONbits.IDLEN = 0;
                  SLEEP();
                }
              }
            }

            CloseI2C1();
            break;

          case TEMP_AN_CHANNEL:
            sample.sample_struct.adc_value_int = ReadADC();
            if(sample.sample_struct.adc_value_int != 0)
            {
              sample.sample_struct.adc_value_int -= adc_offset;
            
              sample.sample_struct.category = 1;
              sample.sample_struct.date = time_by_rn131;
              sample.sample_struct.state = status_flags.byte;

              winbond_return_value = winbond_data_load(sample.sample_array, sizeof(sample));

              if(winbond_return_value == 0)
              {
                configuration.eeprom_ptr_wr.value = eeprom_ptr_wr;
                configuration.eeprom_data_count.value = eeprom_data_count;
              }
              else if(winbond_return_value == -1)
                status_flags.bits.spi_bus_collision = 1;
              else if(winbond_return_value == -2)
                status_flags.bits.eeprom_error = 1;
              else if(winbond_return_value == -3)
                status_flags.bits.eeprom_error = 2;
            }
            break;

          case EXT1_AN_CHANNEL:
            if((configuration.category.value != 0x02) && 
               ((adc_acquisition_number + 1) >= ADC_FILTER_NUMBER) &&
               (calib_mode == 0))
              ext_shut = 0;
          
            sample.sample_struct.adc_value_int = ReadADC();
            if(sample.sample_struct.adc_value_int != 0)
            {
              sample.sample_struct.adc_value_int -= adc_offset;

              adc_acquisition_number++;
              if((adc_acquisition_number < ADC_FILTER_NUMBER))
              {
                adc_filtered += sample.sample_struct.adc_value_int;

                an_channel = EXT1_AN_CHANNEL;
                ConvertADC();
              }
              else
              {
                adc_acquisition_number = 0;
                adc_filtered += sample.sample_struct.adc_value_int;
                sample.sample_struct.adc_value_int = adc_filtered / ADC_FILTER_NUMBER;

                adc_filtered = 0;

                if(calib_mode == 0)
                {
                  // Se si tratta del sensore lvdt, allora il risultato lo devo pesare
                  // con la tensione di alimentazione che acquisisco con ext2. Se
                  // si tratta di un'altro sensore generico, allora salvo il dato
                  sample.sample_struct.category = configuration.category.value;
                  sample.sample_struct.date = time_by_rn131;
                  sample.sample_struct.state = status_flags.byte;
              
                  winbond_return_value = winbond_data_load(sample.sample_array, sizeof(sample));

                  if(winbond_return_value == 0)
                  {
                    configuration.eeprom_ptr_wr.value = eeprom_ptr_wr;
                    configuration.eeprom_data_count.value = eeprom_data_count;
                  }
                  else if(winbond_return_value == -1)
                    status_flags.bits.spi_bus_collision = 1;
                  else if(winbond_return_value == -2)
                    status_flags.bits.eeprom_error = 1;
                  else if(winbond_return_value == -3)
                    status_flags.bits.eeprom_error = 2;
                }
                else
                {
                  if(sample.sample_struct.adc_value_int < EXT1_CALIB_RANGE_MIN)
                  {
                    led_err = 1;
                    led_no_err = 1;
                  }
                  else if(sample.sample_struct.adc_value_int > EXT1_CALIB_RANGE_MAX)
                  {
                    led_err = 1;
                    led_no_err = 0;
                  }
                  else
                  {
                    led_err = 0;
                    led_no_err = 1;
                  }
                }
              }
            }
            break;

          case EXT2_AN_CHANNEL:
            if((adc_acquisition_number + 1) >= ADC_FILTER_NUMBER)
              ext_shut = 0;
          
            sample.sample_struct.adc_value_int = ReadADC();

            if(sample.sample_struct.adc_value_int != 0)
            {
              // compenso l'offset
              sample.sample_struct.adc_value_int -= adc_offset;

              adc_acquisition_number++;
              if((adc_acquisition_number < ADC_FILTER_NUMBER))
              {
                adc_filtered += sample.sample_struct.adc_value_int;

                an_channel = EXT2_AN_CHANNEL;
                ConvertADC();
              }
              else
              {
                adc_acquisition_number = 0;
                adc_filtered += sample.sample_struct.adc_value_int;
                sample.sample_struct.adc_value_int = adc_filtered / ADC_FILTER_NUMBER;
                adc_filtered = 0;

                if(calib_mode == 0)
                {
                  sample.sample_struct.category = configuration.category.value + 1;
                  sample.sample_struct.date = time_by_rn131;
                  sample.sample_struct.state = status_flags.byte;

                  winbond_return_value = winbond_data_load(sample.sample_array, sizeof(sample));

                  if(winbond_return_value == 0)
                  {
                    configuration.eeprom_ptr_wr.value = eeprom_ptr_wr;
                    configuration.eeprom_data_count.value = eeprom_data_count;
                  }
                  else if(winbond_return_value == -1)
                    status_flags.bits.spi_bus_collision = 1;
                  else if(winbond_return_value == -2)
                    status_flags.bits.eeprom_error = 1;
                  else if(winbond_return_value == -3)
                    status_flags.bits.eeprom_error = 2;
                }
              }
            }
            break;

          case CALIBRATION_AN_CHANNEL:
            adc_offset = ReadADC();
            break;
          
          default:
            break;
        }

        adc_update = 0;
     
        // Se ho finito con le acquisizioni ripetute
        if(adc_acquisition_number == 0)
        {
          if((configuration.category.value == 0x02) && (an_channel == EXT1_AN_CHANNEL) && (calib_mode == 0))
          {
            an_channel = EXT2_AN_CHANNEL;
            SelChanConvADC(an_channel);
          }
          else
          {
            if(SSP2CON1bits.SSPEN == 1)
            {
              spi_close();
            }
        
            an_channel = -1;
            if(!T1CONbits.TMR1ON)
              OSCCONbits.IDLEN = 0;
            else
              OSCCONbits.IDLEN = 1;
          }
        }
      }
      
      SLEEP();
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
  adc_batt_enable_tris = INPUT_PIN;
  
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

  _1_wire_eeprom_an_tris = INPUT_PIN;
  _1_wire_eeprom_an_analog = 0;
  _1_wire_eeprom_in_tris = INPUT_PIN;
  _1_wire_eeprom_out_tris = OUTPUT_PIN;
  _1_wire_eeprom_out = 0;
  
  // init led
  led_err_tris = OUTPUT_PIN;
  led_err = 0;

  led_no_err_tris = OUTPUT_PIN;
  led_no_err = 0;

  // enable 3.3V
  alim_3_3V_enable_tris = OUTPUT_PIN;
  alim_3_3V_enable =  0;
  
  // init serial
  uart2_tx_tris = OUTPUT_PIN;
  uart2_tx = 1;
  
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

  uart2_init(0);
}

/**
 * Ottiene il giorno e l'ora correnti ed applica un fattore di correzione per
 * allineare la differenza tra la frequenza reale del clock e quella dell'orologio.
 *
 * @param error_sec  errore commesso ogni secondo in secondi.
 *
 * @remark La prima volta che si richiama questa funzione, viene salvato il valore
 * del timer in formato unix, il quale sarà utilizzato per applicare la correzione
 * dell'ora.
 */
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
  // da quello atteso. Siccome la frequenza tipica dell'oscillatore interno è più
  // elevata dei 32768 Hz previsti, un secondo verrà conteggiato in anticipo
  // commettendo un errore in eccesso sulla data. Per questo devo
  // sottrarre l'errore moltiplicandolo per i secondi passati
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
  char buffer[UART2_BUFFER_SIZE_TX];
  strcpy(buffer, message);

  uart2_buffer_tx_seq_load(buffer, strlen(buffer));
}

int ping(void)
{
  static int ping_status = 0;
  int ping_result = 0;

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
        break;
      }
      else
        ping_status = 1;

    case 1:
      ping_result = ping_send();
      if(ping_result == 1)
      {
        message_load_uart("exit\r");
        rn131.cmd_mode_exit_rqst = 1;
        ping_status = 2;
      }
      else if(ping_result == -1)
      {
        ping_status = 0;
        return -1;
      }
      break;

    case 2:
      if(rn131.cmd_mode == 0)
      {
        ping_status = 0;
        return 1;
      }
      else if(rn131.cmd_mode_exit_rqst == 0)
      {
        message_load_uart("exit\r");
        rn131.cmd_mode_exit_rqst = 1;
      }
      break;
  }

  return 0;
}

int ping_send(void)
{
  static int ping_status = 0;

  switch(ping_status)
  {
    case 0:
      message_load_uart("ping dgoogle.it\r");
      
      ping_status = 1;
      break;

    case 1:
      if(rn131_parse_message(buffer, "Ping try"))
      {
        ping_status = 0;
        return 1;
      }
      else if(rn131_parse_message(buffer, "ERR: "))
      {
        ping_status = 0;
        return -1;
      }
      break;
  }

  return 0;
}

int download_configuration_ftp(void)
{
  static int config_req_sent = 0;
  int ping_result = 0;

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
    ping_result = ping_send();
    
    if(ping_result == 1)
    {
      message_load_uart("set dns backup www.spinitalia.com\r");
      message_load_uart("set ftp user Spinitalia_ftp\r");
      message_load_uart("set ftp pass aqquario1234\r");
      message_load_uart("set ftp remote 21\r");
      message_load_uart("set ftp dir ./\r");
      message_load_uart("set ftp addr 0\r");
      message_load_uart("ftp update wifly_config.cfg\r");

      /*message_load_uart("set dns backup babaracus.no-ip.org\r");
      message_load_uart("set ftp user pi\r");
      message_load_uart("set ftp pass raspberry\r");
      message_load_uart("set ftp remote 21\r");
      message_load_uart("set ftp dir /var/www\r");
      message_load_uart("ftp update wifly_config.cfg\r");*/

      config_req_sent = 1;
    }
    else if(ping_result == 0)
      return 0;
    else
      return -1;
  }
  else
  {
    if(rn131_parse_message(buffer, "FTP OK"))
    {
      message_load_uart("load spin_c\r");
      message_load_uart("save\r");
      message_load_uart("reboot\r");

      rn131_reset_flag();
      communication_ready = 0;
      tcp_start_timer_flag = -1;

      rn131.cmd_mode_reboot_rqst = 1;
      return 1;
    }
    else if(rn131_parse_message(buffer, "FTP timeout"))
    {
      config_req_sent = 0;
      return -1;
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

  Write_b_eep(start_address + eeprom_addr_count, time_by_rn131 & 0x0000ff);
  Write_b_eep(start_address + eeprom_addr_count + 1, (time_by_rn131 >> 8) & 0x0000ff);
  Write_b_eep(start_address + eeprom_addr_count + 2, time_by_rn131 >> 16);
  Write_b_eep(start_address + eeprom_addr_count + 3, time_by_rn131 >> 24);

  Write_b_eep(start_address + eeprom_addr_count + 4, stc3100_voltage.integer & 0x00ff);
  Write_b_eep(start_address + eeprom_addr_count + 5, (stc3100_voltage.integer >> 8) & 0x00ff);
}

/**
 * Resetta le variabili riguardanti il modulo rn131, toglie l'alimentazione
 * secondaria e manda il modulo in sleep.
 */
void sleep_enter(void)
{
  // Finisco di acquisire i dati dalla seriale, nel caso ce ne fossero
  if(UART2_INTERRUPT_RX == 0)
    while(uart2_buffer_rx_load());

  // scarico i buffer circolari
  uart2_flush_buffers();

  rn131_start_flag = 0;

  adc_batt_enable_tris = OUTPUT_PIN;
  adc_batt_enable = 1;
  __delay_us(10);
  adc_batt_enable = 0;
  adc_batt_enable_tris = INPUT_PIN;
  
  alim_3_3V_enable =  0;
  rn131_reset_flag();
  communication_ready = 0;
  tcp_start_timer_flag = -1;

  SLEEP();
}