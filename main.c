/**
 * \file main.c
 * Implementa il ciclo principale.
 * 
 * \author Luca Lucci
 * \version 1.0
 * \date 04/08/2014
 */

#define USE_OR_MASKS /**< Abilita l'opeartore or per concatenare i parametri del timer1*/

// Le seguenti definizioni contengono l'indirizzo di program memory per salvare
// i puntatori circolari in caso di mancanza di batteria.
#define EEPROM_COUNT_PTR 0x7000
#define EEPROM_WRITE_PTR 0x7004
#define EEPROM_SEND_PTR 0x7008
#define ADC_TIMEOUT_PTR 0x700C
#define DATA_DELAY_PTR 0x7010
#define BATT_WARN_PTR 0x7014
#define BATT_MIN_PTR 0x7018
#define SLEEP_PTR 0x701C
#define CATEGORY_PTR 0x7020

//#define LCD_WINSTAR /**< Abilita il display Winstar*/
#define TIMEOUT_UPDATE_RQST_HS 50 /**< Periodo di invio delle richieste in centesimi di secondo*/

#ifdef LCD_WINSTAR
#define TIMEOUT_UPDATE_LCD_HS 100 /**< Periodo di aggiornamento del display lcd in centesimi di secondo*/
#endif
/**
 * Offset per il Timer1.
 *
 * Il Timer1 è l'orologio di sistema ed è collegato ad un oscillatore esterno
 * con un quarzo da 32768 Hz. Utilizzando il registro a 16 bit ed il prescaler
 * 1:1, un periodo di 10ms equivale a:
 *
 * \f[
 *   n = \frac {10 \times 10^{-3}} {t_{ck}} = {10^{-2} \times 32768} = 327,68
 * \f]
 *
 * Per avere un overflow ogni 10ms si dovrà precaricare il registro di:
 *
 * \f[
 *   TIMER\_OFFSET = (2^{16} -1) - n = 6507,32
 * \f]
 */
#define TIMER_OFFSET 65208

#define END_OF_MESSAGE 0x2a /**< Char che indica la fine del messaggio */
//#define RESET_ON_UART_OVERFLOW /**< Resetta il micro sull'evento di overflow della seriale */
#define REQUEST_NUMBER_OF_TRY 10 /**< Numero di tentativi prima di rinunciare */

#define BATT_AN_CHANNEL 0 /**< Porta analogica dove è collegata la batteria */
#define LVDT_AN_CHANNEL 1 /**< Porta analogica dove è collegato il sensore LVDT */
#define TEMP_AN_CHANNEL 4 /**< Porta analogica dove è collegato il sensore di temperatura */
#define BATT_ADC_TIMEOUT_HS 1000 /**< Tempo tra un'acquisizione e l'altra della batteria*/
#define LED_UPDATE_HS 30000 /**< Ritardo nell'accensione del led di stato*/
#define CONNECTION_TIMEOUT_HS 2000 /**< Tempo limite oltre il quale si entra in modalità ap*/

#include <stdio.h>
#include <stdlib.h>
#include <timers.h>
#include <adc.h>

#ifdef LCD_WINSTAR
#include "include/lcd.h"
#endif

#include "include/uart_interface.h"
#include "include/winbond.h"
#include "include/rn131.h"
#include <xc.h>
#include "system/io_cfg.h"
#include "spi.h"
#include "flash.h"

/* C O N F I G ***************************************************************/
#pragma config PLLDIV = 4  /**< Divide by 4 (16MHz oscillator input) */
#pragma config DEBUG = OFF /**< Background debugger disabled; RB6 and RB7 configured
                            as general purpose I/O pins*/
#pragma config WDTEN = OFF /**< WDT disabled (control is placed on SWDTEN bit)*/
#pragma config STVREN = OFF /**< Reset on stack overflow/underflow disabled */
#pragma config XINST = OFF /**< Instruction se extension and Indexed Addressing
                            mode disabled*/

#pragma config CPUDIV = OSC1 /**< No CPU system clock divide */
#pragma config CP0 = OFF /**< Program memory is not code-protected */

#pragma config IESO = OFF /**< Two-Speed start-up disabled */
#pragma config FOSC = HSPLL /**< HS oscillator, PLL enabled, HSPLL used by USB */
#pragma config FCMEN = OFF /**< Fail-safe clock monitor disabled */

#pragma config WDTPS = 1 /**< Watchdog timer postscaler */

#pragma config MSSPMSK = 1 /**< 7-bit address mode enable RE6 and RE5 */
#pragma config CCP2MX = 1 /**< ECCP2/P2A is multiplexed with RC1 */

/* P R O T O T Y P E *********************************************************/
void initialize_system(void);
void user_timer_hs(void);
void message_load_uart(const char *buffer);
int ap_mode(void);
int ping(void);
void circular_buffer_save(void);

/* G L O B A L   V A R I A B L E *********************************************/
time_t time_by_rn131 = 0; /**< Detiene i secondi trascorsi dall'epoc*/

#ifdef LCD_WINSTAR
/** Indica quando l'LCD può essere aggiornato.
 *
 * Update_lcd è impostato tramite l'interrupt generato dal timer1, dopo che sono
 * trascorsi TIMEOUT_UPDATE_LCD_HS centesimi di secondo. Viene azzerato dal main
 * dopo l'avvenuta scrittura sul display.
 */
volatile int lcd_update = 0;
#endif

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

volatile int ap_mode_enter = 0;

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
    } timeout_update_adc_temp_hs; /**< Periodo di acquisizione dall'adc per il sensore di temperatura */

    union
    {
      long value;
      unsigned char bytes[4];
    } timeout_update_adc_lvdt_hs; /**< Periodo di acquisizione dall'adc per il sensore LVDT */

    union
    {
      long value;
      unsigned char bytes[4];
    } timeout_data_eeprom_hs; /**< Periodo di attesa prima di inviare i dati letti dalle eeprom */

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

#ifdef LCD_WINSTAR
char lcdlines[]=
{ LCD_HOME_TO_1ST_LINE,
  LCD_HOME_TO_2ND_LINE,
  LCD_HOME_TO_3RD_LINE,
  LCD_HOME_TO_4TH_LINE
};/*!< Se usiamo l'LCD WINSTAR questo ha 4 linee da 16 caratteri per cui
     rispetto all'LCD COG devo aggiungere un indirizzo di inizio linea in piu'*/
#endif


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
    }
    else
    {
      rn131_rts_pin = 0;
      rn131_int_edge = 1;
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
#ifdef LCD_WINSTAR
    static int lcd_timer = 0;
#endif
    static int rqst_update_timer = 0;
    static short long adc_timer_batt = 0; /**< tiene il tempo trascorso tra un'acquisizione e l'altra della batteria. */
    static short long adc_timer_temp = 0; /**< tiene il tempo trascorso tra un'acquisizione e l'altra della temperatura. */
    static short long adc_timer_lvdt = 0; /**< tiene il tempo trascorso tra un'acquisizione e l'altra dell'LVDT */
    static short long led_timer = 0; /**< tiene il tempo per il led di stato */
    static int connection_timer = 0; /**< tiene il tempo per lo scadere del tentativo di connession */

    /*Quando rientro dallo sleep devo aspettare che il processore sia correttamente
     agganciato al clock. Per esserne sicuro testo il bit meno significativo di
     TMR1L: quando leggo il nuovo stato posso dire che il processore è pronto*/
    asm("BTFSC TMR1L,0");
    asm("BRA $-2");
    asm("BTFSS TMR1L,0");
    asm("BRA $-2");

    WriteTimer1(TIMER_OFFSET);
    PIR1bits.TMR1IF = 0;
    
    user_timer_hs();

    if(rn131.ap_mode == 0)
    {
      if((rn131.wakeup == 1) && (rn131.ready == 1) && (rn131.connected == 0) && (rn131.ap_mode == 0))
      {
        connection_timer++;
        if(connection_timer >= CONNECTION_TIMEOUT_HS)
        {
          connection_timer = 0;
          ap_mode_enter = 1;
        }
      }
      else if(connection_timer != 0)
        connection_timer = 0;

      led_timer++;
      if(led_timer >= LED_UPDATE_HS)
      {
        if(led_timer < (LED_UPDATE_HS + 256))
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
        }
      }
#ifdef LCD_WINSTAR
      lcd_timer++;

      if(lcd_timer == TIMEOUT_UPDATE_LCD_HS)
      {
        lcd_update = 1;
        lcd_timer = 0;
      }
#endif

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

      if(rqst_update_timer == TIMEOUT_UPDATE_RQST_HS)
      {
        rqst_update_timer = 0;
        rqst_update_flag = 1;
      }

      if((rn131.time_set == 1) && (rn131.wakeup == 0))
      {
        adc_timer_batt++;
        adc_timer_temp++;
        adc_timer_lvdt++;
      }
      else
      {
        adc_timer_batt = 0;
        adc_timer_temp = 0;
        adc_timer_lvdt = 0;
      }

      if(adc_timer_temp >= configuration.timeout_update_adc_temp_hs.value)
      {
        // Avvia acquisizione
        if((ADCON0bits.GO == 0) && (an_channel == -1))
        {
          adc_timer_temp = 0;
          an_channel = 4;
          SelChanConvADC(an_channel << 3);
        }
      }
      else if(adc_timer_lvdt >= configuration.timeout_update_adc_lvdt_hs.value)
      {
        // Avvia acquisizione
        if((ADCON0bits.GO == 0) && (an_channel == -1))
        {
          // Il dispositivo ha bisogno di almeno 3 ms per tirare fuori un segnale valido.
          // Inoltre anche lo step-up interno ha bisogno di un certo tempo di assestamento.
          // 100 ms di attesa sono sufficienti per ottenere un segnale valido.
          if(backligth_enable == 0)
          {
            backligth_enable = 1;
            adc_timer_lvdt = configuration.timeout_update_adc_lvdt_hs.value - 10;
          }
          else
          {
            adc_timer_lvdt = 0;
            an_channel = 1;
            SelChanConvADC(an_channel << 3);
          }
        }
      }
      else if(adc_timer_batt >= BATT_ADC_TIMEOUT_HS)
      {
        // Avvia acquisizione
        if((ADCON0bits.GO == 0) && (an_channel == -1))
        {
          if(adc_batt == 1)
          {
            adc_batt = 0;
            adc_timer_batt = BATT_ADC_TIMEOUT_HS - 10;
          }
          else
          {
            adc_timer_batt = 0;
            an_channel = 0;
            SelChanConvADC(an_channel << 3);
          }
        }
      }

      if(tcp_start_timer_flag > 0)
      {
        tcp_start_timer_flag++;

        if(tcp_start_timer_flag == configuration.timeout_data_eeprom_hs.value)
          tcp_start_timer_flag = 0;
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
  unsigned char timer_config = 0x00;
  char uart_token[] = {'\n', '*'};
  unsigned short long eeprom_data_to_send; /**< numero di byte caricati dall'eeprom e pronti per essere inviati*/
  
  //unsigned int adc_result = 0; /**< valore del canale analogico*/
  float adc_V = 0; /**< valore del canale analogico convertito in V*/
  float battery_value_temp = 0; /**< Valore di apporggio per impostare il livello della batteria da remoto*/

  int windbond_return_value;

  WDTCONbits.REGSLP = 1; // on-chip regulator enters low-power operation when device enters in Sleep mode
  OSCTUNEbits.PLLEN = 1;
  OSCCONbits.IDLEN = 0;
  
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
  configuration.timeout_sleep_s.value = 60;
  configuration.timeout_update_adc_temp_hs.value = 1000;
  configuration.timeout_update_adc_lvdt_hs.value = 1000;
  configuration.timeout_data_eeprom_hs.value = 1000;
  configuration.battery_warn_value.value = 3.2;
  configuration.battery_min_value.value = 3.0;
  configuration.category.value = 2;
  windbond_return_value = 0;

  /************ INIT ADC ***********/
  // clear adc interrupt and turn off adc if in case was on previously
  CloseADC();

  /** refer to pag. 306 */
  OpenADC(ADC_FOSC_RC | ADC_RIGHT_JUST | ADC_2_TAD,
          ADC_CH0 | ADC_INT_ON | ADC_REF_VREFPLUS_VSS,
          ADC_0ANA);

  // calibratura ADC
  ADCON1bits.ADCAL = 1;
  ConvertADC();
  while(BusyADC());
  ADCON1bits.ADCAL = 0;

  ADC_INT_ENABLE();

  /************ INIT TIMER *********/
  timer_config = T1_16BIT_RW | T1_SOURCE_EXT | T1_PS_1_1 | T1_OSC1EN_ON
          | T1_SYNC_EXT_OFF | TIMER_INT_ON;

  OpenTimer1(timer_config);
  WriteTimer1(TIMER_OFFSET);
  /******* INIT LCD ********/
#ifdef LCD_WINSTAR
  Lcd_Init();
#endif

  /******* INIT SERIAL COMM ********/
  uart2_open(9600);

  /******* INIT SPI   ******************/
  CloseSPI2();
  OpenSPI2(SPI_FOSC_4, MODE_11, SMPEND);

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

  memset(winbond.device_id, 0, sizeof(winbond.device_id));

  // leggo dalla program memory i valori dei puntatori circolari. Nel caso fosse
  // la prima esecuzione del programma dopo la programmazione, allora inizializzo
  // i valori a zero. Questa condizione è verificata quando tutti i valori risultano
  // uguali al valore limite
  ReadFlash((UINT32)EEPROM_COUNT_PTR, (UINT16)sizeof(configuration.bytes), configuration.bytes);

  if((configuration.eeprom_ptr_send.value == 0xFFFFFFFF) && (configuration.eeprom_ptr_wr.value == 0xFFFFFFFF) && (configuration.eeprom_data_count.value == 0xFFFFFFFF))
  {
    configuration.eeprom_ptr_send.value = 0;
    configuration.eeprom_ptr_wr.value = 0;
    configuration.eeprom_data_count.value = 0;
    configuration.timeout_update_adc_temp_hs.value = 1000;
    configuration.timeout_update_adc_lvdt_hs.value = 1000;
    configuration.timeout_data_eeprom_hs.value = 1000;
    configuration.battery_warn_value.value = 3.2;
    configuration.battery_min_value.value = 3.0;
    configuration.category.value = 2;
    configuration.timeout_sleep_s.value = 60;
  }
  
  eeprom_at_work = 0;

  status_flags.byte = 0;
  
  /* MANUFACTURER ID Winbond: 0xEF
   * DEVICE ID W25Q64BV:      0x16
   */
  if(winbond_identification(winbond.device_id) < 0)
    status_flags.bits.spi_bus_collision = 1;

  if(winbond.device_id[0] > 0)
  {
    if(winbond_status_register_read(&winbond.status_register_1.word, 1) < 0)
      status_flags.bits.spi_bus_collision = 1;

    if(winbond_status_register_read(&winbond.status_register_2.word, 2) < 0)
      status_flags.bits.spi_bus_collision = 1;
  }
  else
    status_flags.bits.eeprom_fail = 1;
  
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

        if(rn131.ap_mode)
          led_no_err = 1;
        
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
                    configuration.timeout_sleep_s.value, configuration.timeout_update_adc_lvdt_hs.value, configuration.timeout_update_adc_temp_hs.value, configuration.category.value, configuration.timeout_data_eeprom_hs.value,
                    configuration.battery_warn_value.value, configuration.battery_min_value.value);

            strcat(buffer, ascii_buffer);

            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if((strncmp(cmd_http_parse, "set_config", 10) == 0) && (rn131.cmd_mode == 0))
          {
            circular_buffer_save();

            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if((strncmp(cmd_http_parse, "get_batt", 8) == 0) && (rn131.cmd_mode == 0))
          {
            ascii_buffer[0] = 0;

            if(buffer[0] == 0)
              sprintf(buffer, "%sx:", rn131.mac);

            sprintf(ascii_buffer, "batt=%1.2f,", adc_V);
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
              configuration.timeout_update_adc_lvdt_hs.value = timeout_update_hs_temp;

            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if((strncmp(cmd_http_parse, "set_adc_temp=", 13) == 0) && (rn131.cmd_mode == 0))
          {
            timeout_update_hs_temp = atol(cmd_http_parse + 8);

            if(timeout_update_hs_temp > 0)
              configuration.timeout_update_adc_temp_hs.value = timeout_update_hs_temp;

            cmd_http_start = cmd_http_mark + 1;
            cmd_http_mark = strchr(cmd_http_start, '\r');

            rn131.cmd_http_rqst = 0;
          }
          else if((strncmp(cmd_http_parse, "set_data_delay=", 15) == 0) && (rn131.cmd_mode == 0))
          {
            timeout_update_hs_temp = atol(cmd_http_parse + 15);

            if(timeout_update_hs_temp > 0)
              configuration.timeout_data_eeprom_hs.value = timeout_update_hs_temp;

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
    
#ifdef LCD_WINSTAR 
    if(lcd_update == 1)
    {
      Lcd_CLS();

      switch(uart2_error_handle())
      {
        case BUFFER_RX_OVERFLOW:
#ifdef RESET_ON_UART_OVERFLOW
          RESET();
#else
         sprintf(buffer, "!Overflow!!\n");
         Lcd_Printf(buffer);
#endif
          break;

        case FRAME_ERROR:
          sprintf(buffer, "Frame Error!\n");
          Lcd_Printf(buffer);
          break;

        default:
          if(rn131.ready == 0)
          {
            sprintf(buffer, "Searching. . .\n");
            Lcd_Printf(buffer);
          }
          else if(rn131.connected == 0)
          {
            sprintf(buffer, "Connecting. . .\n");
            Lcd_Printf(buffer);
          }
          else if(rn131.time_set == 0)
          {
            //time_struct = localtime(&time_by_rn131);
            sprintf(buffer, "Getting time. . .\n", asctime(localtime(&time_by_rn131)));
            Lcd_Printf(buffer);
          }
          else
          {
            //time_struct = localtime(&time_by_rn131);
            sprintf(buffer, "%s\n", asctime(localtime(&time_by_rn131)));
            Lcd_Printf(buffer);
          }
          break;
      }

      lcd_update = 0;
    }
#else
    switch(uart2_error_handle())
    {
      case BUFFER_RX_OVERFLOW:
#ifdef RESET_ON_UART_OVERFLOW
        RESET();
#else
        status_flags.bits.uart_overflow = 1;
#endif
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
            /*if(rn131.ap_mode == 1)
            {
              rn131_sleep();
              communication_ready = 0;
              tcp_start_timer_flag = -1;
              
              SLEEP();
            }*/
            
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
                      tcp_start_timer_flag = configuration.timeout_data_eeprom_hs.value - 2;
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
          if(rn131.time_set_rqst == 0)
          {
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
                sprintf(ascii_hex, "%02x", buffer[i]);
                strcat(ascii_buffer,  &ascii_hex);
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
          }
          else
            rn131.cmd_mode_sleep_rqst = 1;
        }
      }
    }
    else
    {
      if(ap_mode_enter == 1)
      {
        if(ap_mode())
          ap_mode_enter = 0;
      }
    }

    if((rn131.time_set == 1) && (adc_update == 1))
    {
      sample.sample_struct.adc_value_int = ReadADC();

      switch(an_channel)
      {
        case BATT_AN_CHANNEL:
          adc_batt = 1;
          
          adc_V = (sample.sample_struct.adc_value_int * ADC_REFERENCE) * BATTERY_PART / (2 << (ADC_RESOLUTION_BIT - 1));

          if(adc_V <= configuration.battery_warn_value.value)
            status_flags.bits.battery_warning = 1;

          if(adc_V <= configuration.battery_min_value.value)
          {
            circular_buffer_save();

            // disabilito tutte le periferiche
            di();
            CloseTimer1();
            CloseADC();
            ADC_INT_DISABLE();
            CloseSPI2();

            // disalimento tutto
            alim_3_3V_enable = 0;
            adc_batt = 1;

            #if (defined(LCD_WINSTAR) || defined(LCD))
              backligth_enable = 0;

              contrast_cs_tris = INPUT_PIN;
              contrast_ck_tris = INPUT_PIN;
              contrast_ud_tris = INPUT_PIN;
            #endif

            // buona notte
            OSCCONbits.IDLEN = 0;
            SLEEP();
          }
          break;

        case TEMP_AN_CHANNEL:
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
          backligth_enable = 0;
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
      
      an_channel = -1;
      adc_update = 0;
      SLEEP();
    }
  }
  
  CloseTimer1();

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

  csn_tris = OUTPUT_PIN;
  csn = 0;

  // init batt analog pin
  adc_batt_tris = OUTPUT_PIN;
  adc_batt = 1;
  
  // init usb
  UCONbits.USBEN = 0;
  usb_d_min_tris = INPUT_PIN;
  usb_d_min_tris = INPUT_PIN;
  
  // init adc
  battery_ref_tris = INPUT_PIN;
  battery_ref_digital = 0;
  
  battery_an_tris = INPUT_PIN;
  battery_an_digital = 0;

  temp_an_tris = INPUT_PIN;
  temp_an_digital = 0;

  lvdt_an_tris = INPUT_PIN;
  lvdt_an_digital = 0;
  
  // init led
  led_err_tris = OUTPUT_PIN;
  led_err = 0;

  led_no_err_tris = OUTPUT_PIN;
  led_no_err = 0;
  
  /********* INIT RTS INTERRUPT ********/
  rn131_int_edge = 1;
  rn131_int_flag = 0;
  rn131_int_enable = 1;

  // enable 5V
  alim_5V_enable_tris = OUTPUT_PIN;
  alim_5V_enable = 0;

  // enable 3.3V
  alim_3_3V_enable_tris = OUTPUT_PIN;
  alim_3_3V_enable = 1;
  
  // init lcd
  backligth_enable_tris = OUTPUT_PIN;
#if (defined(LCD_WINSTAR) || defined(LCD))
  backligth_enable = 1;

  contrast_cs_tris = OUTPUT_PIN;
  contrast_ck_tris = OUTPUT_PIN;
  contrast_ud_tris = OUTPUT_PIN;

  contrast_cs = 1;
  contrast_ck = 1;
  contrast_ud = 1;

  NOP();
  contrast_ud = 0;
  contrast_cs = 0;

  for(i = 0; i < 128; i++)
  {
    contrast_ck = 0;
    NOP();
    NOP();
    contrast_ck = 1;
  }

  contrast_cs = 1;
  contrast_ud = 1;
#else
  backligth_enable = 0;

  contrast_cs_tris = INPUT_PIN;
  contrast_ck_tris = INPUT_PIN;
  contrast_ud_tris = INPUT_PIN;
#endif

  // init serial
  uart2_tx_tris = OUTPUT_PIN;
  uart2_rx_tris = INPUT_PIN;

  // init rn131
  rn131_rts_tris = INPUT_PIN;

  // init MCP73811
  MCP73811_ce_tris = INPUT_PIN;
  MCP73811_prg_tris = OUTPUT_PIN;
  MCP73811_prg = 1;

  // init v_usb
  v_usb_tris = INPUT_PIN;
  
  // init spi
  spi_sck_tris = OUTPUT_PIN;
  spi_sdo_tris = OUTPUT_PIN;
  spi_sdi_tris = INPUT_PIN;
  spi_cs1_tris = OUTPUT_PIN;
  spi_cs2_tris = OUTPUT_PIN;

  spi_cs1 = 1;
  spi_cs2 = 1;

  uart2_init(0);
}

/**
 * @todo aggiornare anche il giorno della settimana
 */
void user_timer_hs(void)
{
  static unsigned int hsec = 0;
  hsec++;

  if(hsec >= 100)
  {
    hsec=0;

    time_by_rn131++;
  }
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
    //message_load_uart("factory RESET\r");
    //message_load_uart("save\r");
    message_load_uart("run web_app\r");

    // annullo ogni eventuale richiesta inoltrata
    rn131.cmd_http_rqst = 0;

    return 1;
  }

  return 0;
}

void circular_buffer_save(void)
{
  EraseFlash((UINT32)0x7000,(UINT32)0x73FF);
  WriteBytesFlash((UINT32)EEPROM_COUNT_PTR, (UINT16)sizeof(configuration.bytes), configuration.bytes);
}