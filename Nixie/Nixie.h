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
// $Id: Nixie.h,v 1.1 2016/05/07 09:37:27 Emile Exp $
// $Log: Nixie.h,v $
// Revision 1.1  2016/05/07 09:37:27  Emile
// - Project restructure
//
// Revision 1.1.1.1  2016/05/07 08:50:25  Emile
// - Initial version for 1st time check-in to CVS.
//
// Revision 1.2.0.0  2016/07/09 21:39:30  Ronald.
// - Added PORTC Defines  
//
//-----------------------------------------------------------------------------
#ifndef _Nixie_h
#define _Nixie_h

// Set to 1 to print sensor outputs
#define DEBUG_SENSORS (1)

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

//------------------------
// Temp / Hum / mBar Calibration
//------------------------
#define Cal_Hum (100)

#endif