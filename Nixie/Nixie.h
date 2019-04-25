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
// Dig.08 WS2812_DI    (ICP1) PB0 [11]    [20] PC1 (ADC1)     - - - IN-19A P     Dig.15
// Dig.09 HV_ON        (OC1A) PB1 [12]    [19] PC0 (ADC0)     - - - IN-19A %     Dig.14
// Dig.10 - - -        (OC1B) PB2 [13]    [18] AREF
// Dig.11 IR_RCV   (MOSI/OC2) PB3 [14]    [17] 3V3
// Dig.12 - - -        (MISO) PB4 [15]    [16] PB5 (SCK)      TIME_MEAS Dig.13
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
#define HV_ON     (0x02)
#define WS2812_DI (0x01)

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

//-----------------------------------
// RGB LED Defines
// 32-bits long: 0x00 red green blue
//-----------------------------------
#define RGB_BLACK       (0x00000000)
#define RGB_RED         (0x00400000)
#define RGB_GREEN       (0x00004000)
#define RGB_BLUE        (0x00000040)
#define RGB_YELLOW      (RGB_RED |RGB_GREEN)
#define RGB_CYAN        (RGB_BLUE|RGB_GREEN)
#define RGB_MAGENTA     (RGB_RED | RGB_BLUE)
#define RGB_WHITE       (RGB_RED |RGB_GREEN|RGB_BLUE)

#define BLACK   (0)
#define RED     (1)
#define GREEN   (2)
#define BLUE    (3)
#define YELLOW  (4)
#define MAGENTA (5)
#define CYAN    (6)
#define WHITE   (7)

#define NIXIE_CLEAR (10UL)
#define NIXIE_CLEAR_ALL ((NIXIE_CLEAR<<20)|(NIXIE_CLEAR<<16)|(NIXIE_CLEAR<<12)|(NIXIE_CLEAR<<8)|(NIXIE_CLEAR<<4)|NIXIE_CLEAR)
#define WH_MAX          (10)

// RGB-pattern mode
#define OFF     (0) /* WS2812 leds are off */
#define RANDOM  (1) /* Color is selected random */
#define DYNAMIC (2) /* Color is selected by seconds */
#define FIXED   (3) /* Color is selected by fixed_rgb_colour */

//-----------------------------------
// Nixie Decimal Point bit-values:
// 1:HH, 2:HL, 3:MH, 4:ML, 5:SH, 6:SL
//-----------------------------------
#define LEFT_DP1  (0x08) /* for nixie_bits8 */
#define LEFT_DP2  (0x04) /* for nixie_bits8 */
#define LEFT_DP3  (0x02) /* for nixie_bits8 */
#define LEFT_DP4  (0x01) /* for nixie_bits8 */

#define LEFT_DP5  (0x80000000) /* for nixie_bits */
#define LEFT_DP6  (0x40000000) /* for nixie_bits */
#define RIGHT_DP1 (0x20000000) /* for nixie_bits */
#define RIGHT_DP2 (0x10000000) /* for nixie_bits */
#define RIGHT_DP3 (0x08000000) /* for nixie_bits */
#define RIGHT_DP4 (0x04000000) /* for nixie_bits */
#define RIGHT_DP5 (0x02000000) /* for nixie_bits */
#define RIGHT_DP6 (0x01000000) /* for nixie_bits */

//-----------------------------------
// Nixie Decimal Point constant:
// 1:HH, 2:HL, 3:MH, 4:ML, 5:SH, 6:SL
//-----------------------------------
#define DP_HH_LEFT  (0)
#define DP_HH_RIGHT (1)
#define DP_HL_LEFT  (2)
#define DP_HL_RIGHT (3)
#define DP_MH_LEFT  (4) /* minutes indicator */
#define DP_MH_RIGHT (5)
#define DP_ML_LEFT  (6)
#define DP_ML_RIGHT (7)  /* second blinking indicator, inverted */
#define DP_SH_LEFT  (8)  /* second blinking indicator */
#define DP_SH_RIGHT (9)  
#define DP_SL_LEFT  (10) /* dec. point for pressure and temperature */
#define DP_SL_RIGHT (11) /* 1 = DST-active */

//------------------------
// BMP180 STD Defines
//------------------------
#define S180_START_T (0)
#define S180_GET_T   (1)
#define S180_START_P (2)
#define S180_GET_P   (3)

//-----------------------------------------------------------------------------------------------
// https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
//
// At 16 MHz, 1 NOP is approx. 62.5 nsec.
//
// Symbol Parameter	                Min	Typical	Max	Units Measured
// T0H	  0 code ,high voltage time	200	350	500	ns    360..400
// T1H	  1 code ,high voltage time	550	700	5.500	ns    760
// TLD	  data, low voltage time	450	600	5.000	ns    1120
// TLL	  latch, low voltage time	6.000			ns    3120 (max)
//
//-----------------------------------------------------------------------------------------------
#define NR_LEDS   (6)
#define wait_T0H  asm("nop\n nop\n nop\n nop\n nop")
#define wait_T1H  wait_T0H; wait_T0H; asm("nop")

#define ws2812b_send_1   PORTB |=  WS2812_DI; /* WS2812_DI == 1 */  \
						 wait_T1H;                                  \
						 PORTB &= ~WS2812_DI; /* WS2812_DI == 0 */
#define ws2812b_send_0   PORTB |=  WS2812_DI; /* WS2812_DI == 1 */  \
						 wait_T0H;                                  \
						 PORTB &= ~WS2812_DI; /* WS2812_DI == 0 */

//------------------------
// Temp / Hum / mBar Calibration
//------------------------
#define Cal_Hum (100)

uint32_t millis(void);
void     delay_msec(uint16_t ms);
void     ws2812b_fill_rgb(uint32_t color);
void     ws2812b_send_byte(uint8_t bt);
void     ws2812_send_all(void);
void     set_rgb_colour(uint8_t color);
void     check_and_set_summertime(Time p);
void     bme280_task(void);
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
void     init_timer2(void);
void     init_ports(void);

#endif