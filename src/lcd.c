/**
* \file
* Modulo per la gestione del display.
*/

#include "../include/lcd.h"
#include <timers.h>
#include <stdio.h>
#include "../system/io_cfg.h"
#include "../include/lcd.h"

#define LCD_CONF_MAX_Y_LINES 0x03

//extern char lcdlines[];
char lcdlines[]=
{ LCD_HOME_TO_1ST_LINE,
  LCD_HOME_TO_2ND_LINE,
  LCD_HOME_TO_3RD_LINE,
  LCD_HOME_TO_4TH_LINE
};

static char lcdline=0;
static char curx=0;


// ************************************************************************** //
// ************************************************************************** //
// *****************                                         **************** //
// *****************                                         **************** //
// *****************       DA QUI IN POI NON TOCCARE         **************** //
// *****************                                         **************** //
// *****************                                         **************** //
// ************************************************************************** //
// ************************************************************************** //




// __________________________________________________________________ [Lcd_Init]
/*!
\brief Init dell'LCD.
L'LCD lo usiamo nella modalità nibble. Questo modello avrebbe anche 
l'SPI ma non lo usiamo... magari nel DACS2.
*/
void Lcd_Init(void)
{
  // tutti i pin dell'LCD sono outputs e all'inizio sono low.
  //LcdPort    = 0;
  PORTE = PORTE & 0x8;
  TRISE = 0;
    //tr_LcdPort = 0;
  
  // dal momento in cui tutti i pin di controllo sono a 0
  // devono passare 100ms.
  //delayT0(T0_CONST_0100_MS);
  __delay_ms(10);
  __delay_ms(10);
  __delay_ms(10);
  __delay_ms(10);
  __delay_ms(10);
  
  Lcd_SendNibble(LCD_CONF_8BIT_IF);
  //delayT0(T0_CONST_0002_MS);
  //__delay_ms(5);
  __delay_us(50);

  Lcd_SendNibble(LCD_CONF_8BIT_IF);
  //delayT0(T0_CONST_0030_US);
  //__delay_us(100);
  __delay_us(50);

  Lcd_SendNibble(LCD_CONF_8BIT_IF);
  //delayT0(T0_CONST_0030_US);
  //__delay_ms(4);
  __delay_us(50);

  Lcd_SendNibble(LCD_CONF_4BIT_IF);
  //delayT0(T0_CONST_0030_US);
  __delay_us(30);
  // da qui in poi sto in 4bit mode

  Lcd_SendByte(LCD_FLAG_INSTRUCTION,LCD_CONF_LINE_CURSOR);
  //delayT0(T0_CONST_0030_US);
  __delay_us(30);

  Lcd_SendByte(LCD_FLAG_INSTRUCTION,LCD_CONF_BIAS_LINES);
  //delayT0(T0_CONST_0030_US);
  __delay_us(30);

  Lcd_SendByte(LCD_FLAG_INSTRUCTION,LCD_CONF_CONTRAST);
  //delayT0(T0_CONST_0005_MS);
  __delay_ms(5);

  Lcd_SendByte(LCD_FLAG_INSTRUCTION,LCD_CONF_POWER_CTL);
  //delayT0(T0_CONST_0030_US);
  __delay_us(30);

  Lcd_SendByte(LCD_FLAG_INSTRUCTION,LCD_CONF_FOLLOWER);
  //delayT0(T0_CONST_0030_US);
  __delay_us(30);

  Lcd_SendByte(LCD_FLAG_INSTRUCTION,LCD_CONF_DISPLAY_IO);
  //delayT0(T0_CONST_0030_US);
  __delay_us(30);

  Lcd_SendByte(LCD_FLAG_INSTRUCTION,LCD_CONF_CLEAR_DISPL);
  //delayT0(T0_CONST_0002_MS);
  __delay_ms(2);

  Lcd_SendByte(LCD_FLAG_INSTRUCTION,LCD_CONF_ENTRY_MODE);
  //delayT0(T0_CONST_0030_US);
  __delay_us(30);

}//end: Lcd_Init _______________________________________________________________


// ____________________________________________________________ [Lcd_SendNibble]
/*!
\brief Manda un nibble all'LCD.
Manda un nubble all'LCD. Il nibble deve stare engli LSb del byte 
passato come parametro. 
Chiama: delayT0
\param nibble: è il byte che contiene nei suoi LSb il nibble da
spedire. 
*/
void Lcd_SendNibble(char nibble)
{
 
  // i dati vengono caricati sul falling edge della linea enable
  // quindi la alzo
  PORTEbits.RE0 = 1;

  // attendo 4 Cicli
  _delay(4);

  // preparo il nibble
  PORTE &= 0x0f;
  PORTE |= ((nibble & 0x0f)<<4 );
  
  // attendo 4 Cicli il dato stabile
  _delay(4);

  // asserisco la linea enable per caricare il nibble
  PORTEbits.RE0 = 0;

  // attendo prima di consentire altre operazioni
  __delay_us(2);

}// end: ________________________________________________________ Lcd_SendNibble


// ______________________________________________________________ [Lcd_SendByte]
/*!
\brief Manda un Byte all'LCD
Questa funzione manda un byte all'LCD, chiama la funzione 
Lcd_SendNibble due volte.
La linea RS indica la scrittura nella RAM dell'LCD di un 
instruction solo quando questa è a 1.
*/
void Lcd_SendByte(char addr, char byte)
{
  PORTEbits.RE2 = 0;
  //delayT0(T0_CONST_0010_US);
  __delay_us(10);

  PORTEbits.RE2 = addr;

  Nop();
  PORTEbits.RE1 = 0;
  Nop();
  PORTEbits.RE0 = 0;

  Lcd_SendNibble(byte >> 4);

  _delay(4);
  _delay(4);

  Lcd_SendNibble(byte & 0x0f);

  __delay_us(50);

}// end: __________________________________________________________ Lcd_SendByte


// ____________________________________________________________________ [Lcd_XY]
void Lcd_XY( char x, char y)
{

  // l'ascissa non puo' essere maggiore di LCD_CONF_MAX_X_CHARS
  x &= LCD_CONF_MAX_X_CHARS;

  // l'ordinata non puo' mai essere maggiore del num di linee
  if(y>LCD_CONF_MAX_Y_LINES)
    y=LCD_CONF_MAX_Y_LINES;

  curx=x;
  lcdline=y;
  
  Lcd_SendByte(LCD_FLAG_INSTRUCTION,lcdlines[y]|(x));
  _delay(4);
}//end: _________________________________________________________________ Lcd_XY





// __________________________________________________________________ [Lcd_Putc]
/*!
\brief Manda un carattere all'lcd.
CHIEDERE DELUCIDAZIONI SUI "\f" E "\b". COSI' MESSO lcdline NON SERVE A
NIENTE
*/
void Lcd_Putc(char c) 
{
  switch (c)
  {
    case '\n':
      curx=0;
      lcdline++;
      if(lcdline>LCD_CONF_MAX_Y_LINES)
        lcdline=0;     
      Lcd_XY(curx,lcdline);

      break;

    case 4: break;

    default: 
      //questo è il problema della stampa che salta di una riga
      //sull'LCD winstar: se x scavalla devo impostare lcdlines
      //altrimenti salta di una riga
      if((c < 32)||(c > 128))
        return;
      
      if(curx > LCD_CONF_MAX_X_CHARS)
      {
        curx=0;
        
        lcdline++;
        if(lcdline>LCD_CONF_MAX_Y_LINES)
          lcdline=0;
        Lcd_XY(curx,lcdline);  /* */
      }
      
      Lcd_SendByte(1,c);     
      curx++;
      break;
  }
}//end: _______________________________________________________________ Lcd_Putc




// ________________________________________________________________ [Lcd_Printf]
/*!
\brief Implementa la scrittura di stringhe costanti predefinite.
\param stringa è un puntatore ad array di caratteri che devono essere
definiti in fase di compilazione.
*/
void Lcd_Printf(const unsigned char * stringa)
{
  while(*stringa)
  {
    Lcd_Putc(*stringa);
    stringa++;
  }
}// end: ____________________________________________________________ Lcd_Printf



// _______________________________________________________________ [Lcd_nPrintf]
/*!
\brief Implementa la scrittura di stringhe da array di chars
\param stringa e' un puntatore ad array di caratteri
\param len e' la lunghezza della stringa
*/
void Lcd_nPrintf(unsigned char *stringa, unsigned char len)
{
  unsigned char k;
  for(k=0;k<len;k++)
    Lcd_Putc(stringa[k]);
}// end: ___________________________________________________________ Lcd_nPrintf




void Lcd_CLS(void)
{
  Lcd_SendByte(LCD_FLAG_INSTRUCTION,LCD_CLEAR_DISPLAY);
  curx=0;
  lcdline=0;
  __delay_ms(2);
}

#ifdef TEST_LCD

// _________________________________________________________ [Lcd_Test]
/*!
\brief Test dell'LCD
Parte dal primo carattere stampabile e finche' puo scrive a video
caratteri
*/
void Lcd_Test(void)
{
  char i,k,j;
  j=32;
  for(k=0;k<(LCD_CONF_MAX_Y_LINES+1);k++){
    for(i=0;i<(LCD_CONF_MAX_X_CHARS+1);i++){
      Lcd_XY(i,k);
      Lcd_Putc(j++);
      delayT0(T0_CONST_0010_MS);
    }
  }

}//end: ______________________________________________________ Lcd_Test
#endif


