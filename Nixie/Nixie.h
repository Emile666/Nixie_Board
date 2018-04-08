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
// Dig.06 STCP         (AIN0) PD6 [09]    [22] PC3 (ADC3)     - - - analog 3
// Dig.07 SHCP         (AIN1) PD7 [10]    [21] PC2 (ADC2)     - - - analog 2
// Dig.08 WS2812_DI    (ICP1) PB0 [11]    [20] PC1 (ADC1)     - - - analog 1
// Dig.09 HV_ON        (OC1A) PB1 [12]    [19] PC0 (ADC0)     - - - analog 0
// Dig.10 - - -        (OC1B) PB2 [13]    [18] AREF
// Dig.11 IR_RCV   (MOSI/OC2) PB3 [14]    [17] 3V3
// Dig.12 - - -        (MISO) PB4 [15]    [16] PB5 (SCK)      TIME_MEAS Dig.13
//                               -----USB----
//                               Arduino NANO
// $Id: Nixie.h,v 1.1 2016/05/07 09:37:27 Emile Exp $
// $Log: Nixie.h,v $
// Revision 1.1  2016/05/07 09:37:27  Emile
// - Project restructure
//
// Revision 1.1.1.1  2016/05/07 08:50:25  Emile
// - Initial version for 1st time check-in to CVS.
//
//-----------------------------------------------------------------------------
#ifndef _Nixie_h
#define _Nixie_h

// Set to 1 to print sensor outputs
//#define DEBUG_SENSORS (1)

//------------------------
// PORTB Defines
//------------------------
#define TIME_MEAS (0x20)
#define DHT22     (0x10)
#define IR_RCV    (0x08)
#define HV_ON     (0x02)
#define WS2812_DI (0x01)

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

#define NIXIE_CLEAR     (10UL)
#define NIXIE_CLEAR_ALL ((NIXIE_CLEAR<<20)|(NIXIE_CLEAR<<16)|(NIXIE_CLEAR<<12)|(NIXIE_CLEAR<<8)|(NIXIE_CLEAR<<4)|NIXIE_CLEAR)
#define WH_MAX          (10)

//-----------------------------------
// Nixie Decimal Points:
// 1:HH, 2:HL, 3:MH, 4:ML, 5:SH, 6:SL
//-----------------------------------
#define LEFT_DP1  (0x08)
#define LEFT_DP2  (0x04)
#define LEFT_DP3  (0x02)
#define LEFT_DP4  (0x01)

#define LEFT_DP5  (0x80000000)
#define LEFT_DP6  (0x40000000)
#define RIGHT_DP1 (0x20000000)
#define RIGHT_DP2 (0x10000000)
#define RIGHT_DP3 (0x08000000)
#define RIGHT_DP4 (0x04000000)
#define RIGHT_DP5 (0x02000000)
#define RIGHT_DP6 (0x01000000)

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

#endif