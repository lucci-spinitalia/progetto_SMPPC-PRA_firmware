/* 
 * File:   rn131.h
 * Author: Luca Lucci
 *
 * Created on 4 novembre 2014, 17.29
 *
 */

/**
* \file
* Header file del modulo rn131.
*/

#ifndef RN131_H
#define	RN131_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <time.h>

 /** S T R U C T S **********************************/
    
 /**
 * Contiene tutti i flag inerenti al funzionamento del modulo rn131.
 *
 * Sono presenti due tipi di flag:
 *   - di stato
 *   - di richiesta
 * dove gli ultimi sono indicati con il suffisso rqst.
 *
 * I flag di stato sono inizializzati, oltre all'avvio del programma, ogni volta
 * che viene ricevuta la stringa di inizializzazione (vedi \ref principle_of_function_sec
 *  ).
 * I flag di richiesta sono necessari per poter ripetere le richieste al modulo
 * rn131 fino a quando non viene ricevuto il messaggio di richiesta ricevuta.
 */
struct rn131_struct
{
  char version[5]; /**< versione del firmware presente sul modulo rn131*/
  char mac[18]; /**< mac address del modulo rn131*/
  char cmd_mode_rqst; /**< flag di richiesta abilitazione modalità comandi*/
  char cmd_mode_exit_rqst; /**< richiesta di uscita dalla modalità comandi*/
  char cmd_mode_reboot_rqst; /**< richieta riavvio modulo*/
  char cmd_mode_ping_rqst; /**< richiesta ping */
  char cmd_http_rqst; /**< richiesta di esecuzione comando dal server http */
  char cmd_http[256]; /**< comando dal server http */
  char time_set_rqst; /**< richiesta di sincronizzazione del tempo con il server ntp*/
  unsigned wakeup :1; /**< indica quando il modulo si è svegliato dallo sleep */
  unsigned ready :1; /**< indica che il modulo è pronto per ricevere dati*/
  unsigned connected :1; /**< indica che è stato assegnato un indirizzo ip al modulo ed ha accesso alla rete*/
  unsigned tcp_open :1; /**< indica se è stata aperta una connessione tcp */
  unsigned network_error :1; /**< indica se c'è stato un errore nel connettersi all'host predefinito */
  unsigned cmd_mode :1; /**< indica se ci si trova in modalità comandi*/
  unsigned time_set :1; /**< tempo sincronizzato correttamente con il server ntp*/
  unsigned ap_mode :1; /**< indica se si trova in modalità access point */
  unsigned web_server_mode :1; /**< indica se si trova in modalità web server. Deve essere resettato dall'utente */
} rn131; /**< Rappresenta lo stato e le richieste per il modulo rn131*/

/** P R O T O T Y P E S ****************************/
char rn131_connection_init(char *buffer, int rn131_message_length);
char rn131_http_response(char *message);
char rn131_command_show_time(char *message, time_t *time_by_rn131);
void rn131_reset_flag(void);
char rn131_parse_message(char *buffer, const char *message);


#ifdef	__cplusplus
}
#endif

#endif	/* RN131_H */

