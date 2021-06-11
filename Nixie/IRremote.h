//******************************************************************************
// $Id: IRremote.h,v 1.1 2016/05/07 09:37:27 Emile Exp $
// Version 2.0.1 June, 2015
// Copyright 2009 Ken Shirriff
// For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
// Edited by Mitra to add new controller SANYO
//
// Interrupt code based on NECIRrcv by Joe Knapp
// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
// Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
//
// JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
// LG added by Darryl Smith (based on the JVC protocol)
// Whynter A/C ARC-110WD added by Francesco Meschia
//
// E vd Logt: converted into C-library with only NEC protocol
// $Log: IRremote.h,v $
// Revision 1.1  2016/05/07 09:37:27  Emile
// - Project restructure
//
// Revision 1.1.1.1  2016/05/07 08:50:25  Emile
// - Initial version for 1st time check-in to CVS.
//
//******************************************************************************
#ifndef IRremote_h
#define IRremote_h

// VS1838B IR infrared remote
//-------------------------------------------------
// see: https://www.sbprojects.net/knowledge/ir/nec.php

#define CLK_TICKS         (16) // 1 clock-tick = 16 usec: timer1 @ 62.5 kHz
// LSB first, 1 start bit + 16 bit address + 8 bit command + 8 bit inverted command + 1 stop bit.
#define NEC_ADDRESS_BITS  (16) // 16 bit address or 8 bit address and 8 bit inverted address
#define NEC_COMMAND_BITS  (16) // Command and inverted command
#define NEC_BITS          (NEC_ADDRESS_BITS + NEC_COMMAND_BITS)
#define NEC_UNIT          (560)

#define NEC_HDR_MARK      (16 * NEC_UNIT) /* mark (0) time for header */
#define NEC_HDR_SPACE     (8 * NEC_UNIT)  /* space (1) time for header */
#define NEC_BIT_MARK      (NEC_UNIT)
#define NEC_ONE_SPACE     (3 * NEC_UNIT)  /* space length of a one */
#define NEC_ZERO_SPACE    (NEC_UNIT)      /* space length of a zero */
#define NEC_RPT_SPACE     (4 * NEC_UNIT)  /* repeat space length */

#define HDR_MARK_LTICKS   ((NEC_HDR_MARK/CLK_TICKS)   - 40) /* approx.  7 % less */
#define HDR_MARK_HTICKS   ((NEC_HDR_MARK/CLK_TICKS)   + 40) /* approx.  7 % more */
#define HDR_SPACE_LTICKS  ((NEC_HDR_SPACE/CLK_TICKS)  - 20) /* approx.  7 % less */
#define HDR_SPACE_HTICKS  ((NEC_HDR_SPACE/CLK_TICKS)  + 20) /* approx.  7 % more */
#define ONE_SPACE_LTICKS  ((NEC_ONE_SPACE/CLK_TICKS)  - 10) /* approx. 10 % less */
#define ONE_SPACE_HTICKS  ((NEC_ONE_SPACE/CLK_TICKS)  + 10) /* approx. 10 % more */
#define BIT_MARK_LTICKS   ((NEC_BIT_MARK/CLK_TICKS)   -  6) /* approx. 17 % less */
#define BIT_MARK_HTICKS   ((NEC_BIT_MARK/CLK_TICKS)   +  6) /* approx. 17 % more */
#define ZERO_SPACE_LTICKS ((NEC_ZERO_SPACE/CLK_TICKS) -  6) /* approx. 17 % less */
#define ZERO_SPACE_HTICKS ((NEC_ZERO_SPACE/CLK_TICKS) +  6) /* approx. 17 % more */
#define RPT_SPACE_LTICKS  ((NEC_RPT_SPACE/CLK_TICKS)  - 20) /* approx. 14 % less */
#define RPT_SPACE_HTICKS  ((NEC_RPT_SPACE/CLK_TICKS)  + 20) /* approx. 14 % more */

//------------------------------------------------------------------------------
// ISR State-Machine : Receiver States
//------------------------------------------------------------------------------
#define STATE_IDLE      2
#define STATE_MARK      3
#define STATE_SPACE     4
#define STATE_STOP      5
#define STATE_OVERFLOW  6

//-----------------------------------------------------------------------
// KEY values for remote control, used in ir_key() and handle_ir_command()
//-----------------------------------------------------------------------
#define IR_CHARS    "0123456789ULRDOAHX?"
#define IR_0        (0x00)
#define IR_1		(0x01)
#define IR_2		(0x02)
#define IR_3		(0x03)
#define IR_4		(0x04)
#define IR_5		(0x05)
#define IR_6		(0x06)
#define IR_7		(0x07)
#define IR_8		(0x08)
#define IR_9		(0x09)
#define IR_UP       (0x0A)
#define IR_LEFT     (0x0B)
#define IR_RIGHT    (0x0C)
#define IR_DOWN     (0x0D)
#define IR_OK       (0x0E)
#define IR_ASTERISK (0x0F)
#define IR_HASH     (0x10)
#define IR_REPEAT   (0x11)
#define IR_NONE     (0x12)

//-----------------------------------------------------------------------
// Raw 32 bits codes from IR receiver, stored in ir_result
//-----------------------------------------------------------------------
#define IR_CODE_0        (0x00FF4AB5)
#define IR_CODE_1		 (0x00FF6897)
#define IR_CODE_2		 (0x00FF9867)
#define IR_CODE_3		 (0x00FFB04F)
#define IR_CODE_4		 (0x00FF30CF)
#define IR_CODE_5		 (0x00FF18E7)
#define IR_CODE_6		 (0x00FF7A85)
#define IR_CODE_7		 (0x00FF10EF)
#define IR_CODE_8		 (0x00FF38C7)
#define IR_CODE_9		 (0x00FF5AA5)
#define IR_CODE_UP       (0x00FF629D)
#define IR_CODE_LEFT     (0x00FF22DD)
#define IR_CODE_RIGHT    (0x00FFC23D)
#define IR_CODE_DOWN     (0x00FFA857)
#define IR_CODE_OK       (0x00FF02FD)
#define IR_CODE_ASTERISK (0x00FF42BD)
#define IR_CODE_HASH     (0x00FF52AD)
#define IR_CODE_REPEAT   (0xFFFFFFFF)

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
// Defines for show_date_IR variable
//-----------------------------------------------------------------------
#define IR_SHOW_TIME     (0) /* Default, show normal time */
#define IR_SHOW_DATE     (1) /* Show day and month */
#define IR_SHOW_YEAR     (2) /* Show year */
#define IR_SHOW_TEMP     (3) /* Show DS3231 temperature */
#define IR_SHOW_VER      (4) /* Show version number */
#define IR_SHOW_ESP_STAT (5) /* Show last response from ESP8266: 1 = ok */

//-----------------------------------------------------------------------
// Defines for set_time_IR variable
//-----------------------------------------------------------------------
#define IR_NO_TIME      (0) /* Default option */
#define IR_BB_TIME      (1) /* Show Blanking begin-time */
#define IR_BE_TIME      (2) /* Show Blanking end-time */

//------------------------------------------------------------------------------
// Main routines for receiving IR
//------------------------------------------------------------------------------
bool    check_ticks(uint16_t val, uint16_t low, uint16_t high);
bool    ir_decode_nec(void);
uint8_t ir_key(void);
void    handle_ir_command(uint8_t key);
void    ir_task(void);

#endif
