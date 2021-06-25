//-----------------------------------------------------------------------------
//                       Nixie Pin Mapping Arduino NANO
//
//                                ----ICSP----
// Dig.01 (TX)          (TXD) PD1 [01]    [26] VIN
// Dig.00 (RX)          (RXD) PD0 [02]    [27] GND
//                    (RESET) PC6 [03]    [28] PC6 (RESET)
//                            GND [04]    [27] VCC
// Dig.02 - - -        (INT0) PD2 [05]    [26] PC2 (ADC7)     - - - analog 7
// Dig.03 - - -        (INT1) PD3 [06]    [25] PC1 (ADC6)     - - - analog 6
// Dig.04 - - -      (XCK/TO) PD4 [07]    [24] PC5 (ADC5/SCL) SCL   analog 5
// Dig.05 SDIN           (T1) PD5 [08]    [23] PC4 (ADC4/SDA) SDA   analog 4
// Dig.06 STCP         (AIN0) PD6 [09]    [22] PC3 (ADC3)     - - - IN-19A RGB   Dig.17
// Dig.07 SHCP         (AIN1) PD7 [10]    [21] PC2 (ADC2)     - - - IN-19A Deg C Dig.16
// Dig.08 RGB_R        (ICP1) PB0 [11]    [20] PC1 (ADC1)     - - - IN-19A P     Dig.15
// Dig.09 RGB_G        (OC1A) PB1 [12]    [19] PC0 (ADC0)     - - - IN-19A %     Dig.14
// Dig.10 RGB_B        (OC1B) PB2 [13]    [18] AREF
// Dig.11 IR_RCV   (MOSI/OC2) PB3 [14]    [17] 3V3
// Dig.12 DHT22        (MISO) PB4 [15]    [16] PB5 (SCK)      TIME_MEAS Dig.13
//                               -----USB----
//                               Arduino NANO
//-----------------------------------------------------------------------------
#ifndef _Nixie_h
#define _Nixie_h

#include "i2c.h"

//------------------------
// PORTB Defines
//------------------------
#define TIME_MEAS (0x20)
#define DHT22     (0x10)
#define IR_RCV    (0x08)
#define RGB_B     (0x04)
#define RGB_G     (0x02)
#define RGB_R     (0x01)

//------------------------
// PORTC Defines
//------------------------
#define HUMIDITYSYMBOL	(0x01)
#define PRESSURESYMBOL  (0x02)
#define DEGREESYMBOL	(0x04)
#define LED_IN19A		(0x08)

//------------------------
// PORTD Defines
//------------------------
#define SHCP (0x80)
#define STCP (0x40)
#define SDIN (0x20)

//------------------------
// RGB LED Defines
//------------------------
#define RED         (0x01)
#define GREEN       (0x02)
#define BLUE        (0x04)
#define YELLOW      (RED |GREEN)
#define CYAN        (BLUE|GREEN)
#define MAGENTA     (RED | BLUE)
#define WHITE       (RED |GREEN|BLUE)
#define BLACK       (0x00)

#define NIXIE_CLEAR (10UL)
#define NIXIE_CLEAR_ALL ((NIXIE_CLEAR<<20)|(NIXIE_CLEAR<<16)|(NIXIE_CLEAR<<12)|(NIXIE_CLEAR<<8)|(NIXIE_CLEAR<<4)|NIXIE_CLEAR)
#define WH_MAX          (10)

// RGB-pattern mode
#define OFF     (0) /* RGB leds are off */
#define RANDOM  (1) /* Color is selected random */
#define DYNAMIC (2) /* Color is selected by seconds */
#define FIXED   (3) /* Color is selected by fixed_rgb_colour */
#define SET_WIT (4) /* acknowledgement for e0 response from esp8266 */

//------------------------
// Nixie Decimal Points
//------------------------
#define LEFT_DP5  (0x80000000)
#define LEFT_DP3  (0x40000000)
#define RIGHT_DP6 (0x20000000)
#define RIGHT_DP5 (0x10000000)
#define RIGHT_DP4 (0x08000000)
#define RIGHT_DP3 (0x04000000)
#define RIGHT_DP2 (0x02000000)
#define RIGHT_DP1 (0x01000000)

//------------------------
// BMP180 STD Defines
//------------------------
#define S180_START_T (0)
#define S180_GET_T   (1)
#define S180_START_P (2)
#define S180_GET_P   (3)

//-----------------------------------------------------------------------
// States for esp8266_std in update_esp8266_time()
//-----------------------------------------------------------------------
#define ESP8266_INIT    (0)  /* Default state */
#define ESP8266_UPDATE  (1)  /* Update time from ESP8266 NTP Server */
#define ESP8266_RETRY   (2)  /* Retry getting a response from the ESP8266 */

#define ESP8266_HOURS   (12) /* Time in hours between updates from ESP8266 */
#define ESP8266_MINUTES (ESP8266_HOURS * 60)
#define ESP8266_SECONDS ((uint16_t)ESP8266_HOURS * 3600)

//-----------------------------------------------------------------------
// States for ir_cmd_std in handle_ir_command()
//-----------------------------------------------------------------------
#define IR_CMD_IDLE      (0)
#define IR_CMD_0         (1)
#define IR_CMD_1         (2) /* Show version number for 5 seconds */
#define IR_CMD_2         (3) /* Show last response from ESP8266 */
#define IR_CMD_3         (4) /* Get Date & Time from ESP8266 NTP Server */
#define IR_CMD_4         (5) /* Show DS3231 Temperature for 5 seconds */
#define IR_CMD_5         (6) /* Set intensity of colors */
#define IR_CMD_6         (7) /* Invert Blanking-Active signal */
#define IR_CMD_7         (8) /* Enable Testpattern */
#define IR_CMD_8         (9) /* Set blanking-begin time */
#define IR_CMD_9        (10) /* Set blanking-end time */
#define IR_CMD_HASH     (11) /* Show date & year for 10 seconds */
#define IR_CMD_CURSOR   (12)
#define IR_CMD_COL_CURSOR (13)

//-----------------------------------------------------------------------
// Definitions for numbering of Nixie Tubes
//-----------------------------------------------------------------------
#define POS0    (0) /* Left-most Nixie, typically displays MSB hours */
#define POS1    (1) /* Typically displays LSB hours */
#define POS2    (2) /* Typically displays MSB minutes */
#define POS3    (3) /* Typically displays LSB minutes */
#define POS4    (4) /* Typically displays MSB seconds */
#define POS5    (5) /* Right-most Nixie, typically displays LSB seconds */

//-----------------------------------------------------------------
// Constants for ssd[] array with characters for 7-segment display
//-----------------------------------------------------------------
#define DIG_0      (0)
#define DIG_1      (1)
#define DIG_2      (2)
#define DIG_3      (3)
#define DIG_4      (4)
#define DIG_5      (5)
#define DIG_6      (6)
#define DIG_7      (7)
#define DIG_8      (8)
#define DIG_9      (9)
#define DIG_SPACE (10) /* decodes to Nixie off */

uint32_t millis(void);
void     delay_msec(uint16_t ms);
void     set_rgb_colour(uint8_t color);
void     check_and_set_summertime(Time p);
void     dht22_task(void);
void     bmp180_task(void);
void     update_nixies(void);
uint8_t  encode_to_bcd(uint8_t x);
void     clear_nixie(uint8_t nr);
void     ftest_nixies(void);
void     fixed_random_rgb_colour(uint8_t s, bool rndm);
bool     blanking_active(Time p);
void	 dec_point_set(uint8_t dp);
void	 dec_point_clr(uint8_t dp);
void     display_task(void);
void     set_nixie_timedate(uint8_t x, uint8_t y, char z);
void     init_timer1(void);
void     init_timer2(void);
uint16_t tmr1_val(void);
void     init_ports(void);

#endif