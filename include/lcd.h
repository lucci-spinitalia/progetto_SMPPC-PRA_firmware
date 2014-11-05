/*!
\file
Header file del modulo LCD. Il modulo LCD non ha interrupts.


*/

//#define TEST_LCD

#define LCD_CONF_8BIT_IF           3
#define LCD_CONF_4BIT_IF           2

#define LCD_CONF_LINE_CURSOR      41
#define LCD_CONF_4BIT_LINE_CURSOR  9


#define LCD_CONF_BIAS_LINES       21
#define LCD_CONF_CONTRAST        123
#define LCD_CONF_POWER_CTL        94
#define LCD_CONF_FOLLOWER        108  
#define LCD_CONF_DISPLAY_IO       12
#define LCD_CONF_DISPLAY_OFF       8
#define LCD_CONF_CLEAR_DISPL       1
#define LCD_CONF_ENTRY_MODE        6
#define LCD_CONF_RETURN_HOME       2


#define LCD_CONF_MAX_X_CHARS    0x0F

#ifdef LCD_COG
  #define LCD_CONF_MAX_Y_LINES    0x02
#elif defined(LCD_WINSTAR)
  #define LCD_CONF_MAX_Y_LINES    0x03
#endif

#define LCD_FLAG_INSTRUCTION       0
#define LCD_FLAG_DATA              1

#ifdef LCD_COG
  #define LCD_HOME_TO_1ST_LINE    0x80
  #define LCD_HOME_TO_2ND_LINE    0x90
  #define LCD_HOME_TO_3RD_LINE    0xA0
#else
  #define LCD_HOME_TO_1ST_LINE    0x80
  #define LCD_HOME_TO_2ND_LINE    0xC0
  #define LCD_HOME_TO_3RD_LINE    0x90
  #define LCD_HOME_TO_4TH_LINE    0xD0
#endif

#define LCD_CLEAR_DISPLAY          1



#define Lcd_ON();   Lcd_SendByte(LCD_FLAG_INSTRUCTION,LCD_CONF_DISPLAY_IO);
#define Lcd_OFF();  Lcd_SendByte(LCD_FLAG_INSTRUCTION,LCD_CONF_DISPLAY_OFF);




#ifdef FW_DBG
void LcdDBG(void);
#endif
void Lcd_CLS(void);
void Lcd_Init(void);
void Lcd_SendNibble(char nibble);
void Lcd_SendByte(char addr, char byte);
void Lcd_XY( char x, char y);
void Lcd_Putc(char c);
void Lcd_Printf(const unsigned char* stringa);
void Lcd_nPrintf(unsigned char *stringa, unsigned char len);

// _____________________________________________ Application Specific Prototypes
void LcdIO(void);


