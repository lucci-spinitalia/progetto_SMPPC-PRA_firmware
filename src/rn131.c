/**
* \file
* Collezione di funzioni per la gestione del modulo rn131 ed il parsing dei messaggi ricevuti.
*/

#include <stdlib.h>
#include <string.h>

#include "../include/rn131.h"
/**
 * Traduce il messaggio in buffer ed aggiorna lo stato del modulo rn131.
 *
 * Controlla se il messaggio contenuto nel buffer globale corrisponda ad uno
 * di quelli di inizializzazione. In caso affermativo, aggiorna le variabili
 * di stato contenute nella struttura rn131.
 * @param rn131_message_length lunghezza del messaggio ricevuto
 * @return Un char che indica se è stato trovato o meno un messaggio conosciuto
 *   - 0: il buffer non contiene messaggi conosciuti
 *   - 1: messaggio trovato
 * @remarks Se viene trovata la stringa di inizializzazione del modulo, tutti i
 * parametri della struttura rn131 vengono azzerati, tranne il time_set. Se
 * invece viene catturato il messaggio dell'indirizzo del gateway, significa che
 * il modulo è in rete e viene aggiornata la variabile connected della struttura
 * rn131_struct.
 */
char rn131_connection_init(char *buffer, int rn131_message_length)
{
  char *rn131_parse_ptr = NULL;

  /* MESSAGE 1 */
  /* This is the first message sent after boot */
  if(rn131_parse_message(buffer, "WiFly WebConfig") == 1)
  {
    rn131.ap_mode = 1;
    return 1;
  }

  if(rn131_parse_message(buffer, "wifly-GSX Ver: ") == 1)
  {
    rn131_parse_ptr = buffer;
    rn131_parse_ptr += 15;
    strncpy(rn131.version, rn131_parse_ptr, strchr(rn131_parse_ptr, ' ') - rn131_parse_ptr);

    // Reset status flags
    rn131.connected = 0;
    rn131.tcp_open = 0;
    rn131.tcp_error = 0;
    rn131.cmd_mode_exit_rqst = 0;
    rn131.cmd_mode_reboot_rqst = 0;
    rn131.cmd_mode_sleep_rqst = 0;
    rn131.cmd_http_rqst = 0;
    rn131.cmd_mode_rqst = 0;
    rn131.cmd_mode = 0;
    rn131.ready = 0;

    if(rn131.ap_mode)
    {
      rn131.ap_mode = 0;
      rn131.cmd_mode_sleep_rqst = 1;
    }
    
    return 1;
  }

  /* MESSAGE 2 */
  if(rn131_parse_message(buffer, "MAC Addr=") == 1)
  {
    rn131_parse_ptr = buffer;
    rn131_parse_ptr += 9;

    strncpy(rn131.mac, rn131_parse_ptr, rn131_message_length - 9 - 2);
    return 1;
  }

  if(rn131_parse_message(buffer, "READY") == 1)
  {
    rn131.ready = 1;
    return 1;
  }

  if(rn131.ready == 1)
  {
    if(rn131_parse_message(buffer, "Listen on ") == 1)
    {
      rn131.connected = 1;

      return 1;
    }
  }

  return 0;
}

char rn131_http_response(char *message)
{
  static int response_status = 0;

 /* - *OPEN*HTTP/1.1 200 OK
 * Date: Tue, 30 Sep 2014 15:54:04 GMT
 * Server: Apache/2.2.22 (Debian)
 * X-Powered-By: PHP/5.4.4-14+deb7u14
 * Vary: Accept-Encoding
 * Content-Length: 16
 * Connection: close
 * Content-Type: text/html
 * comando1\rcomando2\r**CLOS*/

  switch(response_status)
  {
    case 0:
      if(rn131_parse_message(message, "Content-Type:") == 0)
        break;

    case 1: //leggo un \r\n aggiuntivo
      response_status++;
      break;

    case 2: //leggo i comandi
      response_status = 0;
      return 1;

    default:
      response_status = 0;
      break;
  }

  return 0;
}

/**
 * Traduce il messaggio in buffer
 *
 * @return Un char che indica se è stato trovato o meno un messaggio conosciuto
 *   - 0: il buffer non contiene messaggi conosciuti
 *   - 1: messaggio trovato
 */
char rn131_parse_message(char *buffer, const char *message)
{
  char rn131_parse = 0;

  rn131_parse = strncmp(buffer, message, strlen(message));

  if(rn131_parse == 0)
    return 1;

  return 0;
}

/**
 * Aggancia il messaggio orario
 *
 * Verifica se è presente un messaggio orario aggiornato dal server ntp ed
 * aggiorna di conseguenza la varibile globale time_by_rn131. Per ottenere l'ora
 * giusta è necessario prima ottere un messaggio "Time=" e poi estrapolare dal
 * messaggio RTC i secondi dall'epoc.
 *
 * @return Un char che indica se è stato riconosciuto uno dei messaggi associati
 * all'ora
 *
 * @remarks Una volta che viene acquisita l'ora esatta, viene aggiornata la
 * variabile globale time_by_rn131.
 */
char rn131_command_show_time(char *message, time_t *time_by_rn131)
{
  char *rn131_parse_ptr = NULL;
  static char time_set = 0;
  static char time[12];

  /**************************************************************** Time */
  /* I will find "Time=" only when time has been retreived else I will
   * read "Time NOT SET". I have to use strncmp function to don't confusing
   * it with "UpTime" parameter
  */

  // Sono constretto a dividere la conversione del tempo da ascii a long perchè
  // altrimenti mi porterebbe all'overflow della seriale
  switch(time_set)
  {
    case 0:
      if(rn131_parse_message(message, "Time=") == 1)
      {
        time_set = 1;
        return 1;
      }
      break;

    case 1:
      if(rn131_parse_message(message, "RTC=") == 1)
      {
        time_set++;

        rn131_parse_ptr = message;
        rn131_parse_ptr += 4;
        strcpy(time, rn131_parse_ptr);

        return 1;
      }
      break;

    case 2:
      time_set++;

      *time_by_rn131 = atol(time + 5);
      return 1;

    case 3:
      time_set = 0;

      time[5] = 0;
      *time_by_rn131 += atol(time) * 100000;
      rn131.time_set = 1;
      return 1;

    default:
      time_set = 0;
      break;
  }

  return 0;
}

void rn131_sleep(void)
{
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
  rn131.wakeup = 0;
}
