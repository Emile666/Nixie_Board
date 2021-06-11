//******************************************************************************
// $Id: IRremote.c,v 1.1 2016/05/07 09:37:27 Emile Exp $
// Version 2.0.1 June, 2015
// Copyright 2009 Ken Shirriff
// For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
//
// Modified by Paul Stoffregen <paul@pjrc.com> to support other boards and timers
// Modified  by Mitra Ardron <mitra@mitra.biz>
// Added Sanyo and Mitsubishi controllers
// Modified Sony to spot the repeat codes that some Sony's send
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
// $Log: IRremote.c,v $
// Revision 1.1  2016/05/07 09:37:27  Emile
// - Project restructure
//
// Revision 1.1.1.1  2016/05/07 08:50:25  Emile
// - Initial version for 1st time check-in to CVS.
//
// Revision 1.2.0.0. 2016/07/09 21:39:30  Ronald
// - Added and modified several functions
//
//******************************************************************************
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>
#include <util/atomic.h>
#include "IRremote.h"
#include "usart.h"
#include "Nixie.h"
#include "eep.h"
#include "scheduler.h"

uint8_t  ir_cmd_std = IR_CMD_IDLE;      // FSM state in handle_ir_command()
uint8_t  ir_cmd_tmr = 0;                // No-action timer for handle_ir_command()
uint32_t ir_result  = 0;                // 32 bit raw bit-code from IR is stored here
bool     ir_rdy     = false;            // flag for ir_task() that new IR code is received
uint8_t  time_arr[6];                   // Array for changing time or intensity with IR
uint8_t  time_arr_idx;                  // Index into time_arr[]

extern uint8_t  tmr1_std;               // FSM for reading IR codes
extern uint8_t  rgb_pattern;            // RGB color mode: [RANDOM, DYNAMIC, FIXED, OFF]
extern uint8_t  fixed_rgb_colour;       // Color when rgb_pattern is FIXED

extern uint8_t  test_nixies;		    // Sets Nixie clock in test mode
extern uint8_t  wheel_effect;			// Wheel-effect on every second and minute change

extern uint8_t  blank_begin_h;
extern uint8_t  blank_begin_m;
extern uint8_t  blank_end_h;
extern uint8_t  blank_end_m;
extern bool     blanking_invert;         // Invert blanking-active IR-command
extern char     nixie_ver[];
extern uint16_t esp8266_tmr;             // timer for updating ESP8266
extern bool     last_esp8266;            // true = last esp8266 command was successful
extern uint16_t rawbuf[];                // buffer with clock-ticks from IR-codes
extern uint8_t  rawlen;                  // number of bits read from IR
extern uint8_t  show_date_IR; // What to display on the 7-segment displays
extern uint8_t  set_time_IR;   // Show normal time or blanking begin/end time
extern bool     set_color_IR;     // true = set color intensity via IR

/*------------------------------------------------------------------
  Purpose  : This function checks if the IR bit timing is between a
             lower and a upper limit. All values are in clock-ticks,
             a clock-tick (timer 3) is 32 usec (f = 31.25 kHz)
  Variables: -
  Returns  : true = success ; false = error
  ------------------------------------------------------------------*/
bool check_ticks(uint16_t val, uint16_t low, uint16_t high)
{
    return ((val >= low) && (val <= high));
} // check_ticks()

/*------------------------------------------------------------------
  Purpose  : This function decodes the bits from rawbuf and stores
             the result in ir_result. It is called from ir_task().
  Variables: -
  Returns  : true = success ; false = error
  ------------------------------------------------------------------*/
bool ir_decode_nec(void)
{
    int16_t  offset = 1;  // Index in to results; Skip first entry!?
    //char     s[20];
    
    ir_result = 0; // We decode in to here; Start with nothing
	//sprintf(s,"rawlen=%d ",rawlen);
	//xputs(s);
	//for (uint8_t i = 0; i < rawlen; i++)
	//{
		//sprintf(s,"%u,",rawbuf[i]);
		//xputs(s);
	//} // for i		
    //// Check we have the right amount of data (68). 
    // The +4 is for initial gap, start bit mark and space + stop bit mark.
    if ((rawlen < 68) && (rawlen != 4)) 
    {
        //sprintf(s,"len (%d) err\n",rawlen);
        //xputs(s);
        return false;
    } // if
    
    if (!check_ticks(rawbuf[offset], HDR_MARK_LTICKS, HDR_MARK_HTICKS))  
    {   // Check header "mark" this must be done for repeat and data
        //xputs("hdr mark err\n");
        return false;
    } // if
    offset++;
    
    if (rawlen == 4)
    {   // Check for repeat - this has a different header space length
        //uart_printf("rpt hdr ");
        if (check_ticks(rawbuf[offset  ], RPT_SPACE_LTICKS, RPT_SPACE_HTICKS) &&
            check_ticks(rawbuf[offset+1], BIT_MARK_LTICKS , BIT_MARK_HTICKS))
        {
            //xputs("ok\n");
            return true;
        } // if
        //xputs("err\n");
        return false; // wrong repeat header
    } // if 
    
    if (!check_ticks(rawbuf[offset], HDR_SPACE_LTICKS, HDR_SPACE_HTICKS)) 
    {   // Check command header space
        //xputs("hdr space err\n");
        return false; // Header space length is wrong
    } // if
    offset++;
    
    // Build the data
    while (offset < rawlen-1)
    {   // Check data "mark"
        if (!check_ticks(rawbuf[offset], BIT_MARK_LTICKS, BIT_MARK_HTICKS)) 
        {
            //sprintf(s,"mark %d err\n",offset);
            //xputs(s);
            return false;
        } // if
        offset++;
        // Suppend this bit
        if      (check_ticks(rawbuf[offset], ONE_SPACE_LTICKS , ONE_SPACE_HTICKS ))  ir_result = (ir_result << 1) | 1 ;
        else if (check_ticks(rawbuf[offset], ZERO_SPACE_LTICKS, ZERO_SPACE_HTICKS))  ir_result = (ir_result << 1) | 0 ;
        else 
        {
            //sprintf(s,"space %d err\n",offset);
            //xputs(s);
            return false;
        } // else
        offset++;
    } // while
    return true; // success
} // ir_decode_nec()

/*------------------------------------------------------------------
  Purpose  : This function retrieves the proper key from the IR code
  Variables: global variable ir_result is used. It is called from ir_task().
  Returns  : code for key found
  ------------------------------------------------------------------*/
uint8_t ir_key(void)
{
    char    s[10];
    uint8_t key;
    
    switch (ir_result)
    {
        case IR_CODE_UP      : key = IR_UP      ; break;
        case IR_CODE_DOWN    : key = IR_DOWN    ; break;
        case IR_CODE_LEFT    : key = IR_LEFT    ; break;
        case IR_CODE_RIGHT   : key = IR_RIGHT   ; break;
        case IR_CODE_OK	     : key = IR_OK      ; break;
        case IR_CODE_ASTERISK: key = IR_ASTERISK; break;
        case IR_CODE_HASH    : key = IR_HASH    ; break;
        case IR_CODE_0       : key = IR_0       ; break;
        case IR_CODE_1       : key = IR_1       ; break;
        case IR_CODE_2       : key = IR_2       ; break;
        case IR_CODE_3       : key = IR_3       ; break;
        case IR_CODE_4       : key = IR_4       ; break;
        case IR_CODE_5       : key = IR_5       ; break;
        case IR_CODE_6       : key = IR_6       ; break;
        case IR_CODE_7       : key = IR_7       ; break;
        case IR_CODE_8       : key = IR_8       ; break;
        case IR_CODE_9       : key = IR_9       ; break;
        case IR_CODE_REPEAT  : key = IR_REPEAT  ; break;
        default              : key = IR_NONE    ; break;
    } // switch
    sprintf(s,"IR[%c]\n",IR_CHARS[key]);
    xputs(s); // output to terminal screen
    return key;
} // ir_key()

/*-----------------------------------------------------------------------------
  Purpose  : This function checks if a BCD number is allowed for a time.
  Variables: digit: [0..9], the number proposed for a certain digit.
  Globals  : time_arr_idx: index into timer_arr[], indicates which digit is about
                           to change, POS0 = left-most, POS5 = right-most digit
             time_arr[]  : array to store the proposed digits in.
  Returns  : -
  ---------------------------------------------------------------------------*/
void check_possible_digit(uint8_t digit)
{
    if (digit > DIG_9) return; // only valid digits allowed
    
    switch (time_arr_idx)
    {
        case POS0: // MSB of hours for blanking begin/end
            if (digit < DIG_3) 
            {   // only 0, 1 or 2 allowed for MSB hours
                time_arr[POS0] = digit;
            } // else
            break;
        case POS1: // LSB of hours for blanking begin/end
            if ((time_arr[POS0] == DIG_2) && (digit < DIG_4))
            {   // only 20, 21, 22 and 23 allowed
                time_arr[POS1]  = digit;
            } // if
            else if (time_arr[POS0] < DIG_2)
            {   // time_arr[POS0] == 0 || time_arr[POS0] == 1
                time_arr[POS1]  = digit;
            } // else    
            break;
        case POS2: // MSB of minutes for blanking begin/end
            if (digit < DIG_6) 
            {   // only 0..5 allowed
                time_arr[POS2]  = digit;
            } // else
            break;
        default: // time_arr_idx == POS3, LSB of minutes for blanking begin/end
            time_arr[POS3]  = digit; // any digit between 0..9 allowed
            break;
    } // switch
} // check_possible_digit()

/*------------------------------------------------------------------------
  Purpose  : Encode a byte into 2 BCD numbers.
  Variables: x: the byte to encode
  Returns  : the two encoded BCD numbers
  ------------------------------------------------------------------------*/
uint8_t encode_to_bcd2(uint8_t x)
{
    uint8_t temp;
    uint8_t retv = 0;
    
    temp   = x / 10;
    retv  |= (temp & 0x0F);
    retv <<= 4; // SHL 4
    temp   = x - temp * 10;
    retv  |= (temp & 0x0F);
    return retv;
} // encode_to_bcd2()

/*------------------------------------------------------------------------
  Purpose  : Encode a 16-bit integer into 4 BCD numbers.
  Variables: x: the integer to encode
  Returns  : the four encoded BCD numbers
  ------------------------------------------------------------------------*/
uint16_t encode_to_bcd4(uint16_t x)
{
    uint16_t temp, rest = x;
    uint16_t retv = 0;
    
    temp   = rest / 1000;
    retv  |= (temp & 0x0F);
    retv <<= 4; // SHL 4
    rest  -= temp * 1000;

    temp   = rest / 100;
    retv  |= (temp & 0x0F);
    retv <<= 4; // SHL 4
    rest  -= temp * 100;
    
    temp   = rest / 10;
    retv  |= (temp & 0x0F);
    retv <<= 4; // SHL 4
    rest  -= temp * 10;
    retv  |= (rest & 0x0F);
    return retv;
} // encode_to_bcd4()

/*-----------------------------------------------------------------------------
  Purpose  : This function is called every 100 msec. and initiates all actions
             derived from IR remote keys
  Variables: key: the key pressed on the IR-remote
  Returns  : -
  ---------------------------------------------------------------------------*/
void handle_ir_command(uint8_t key)
{
    static uint16_t tmr_xsec; // seconds timer
    uint8_t  x;
    uint16_t temp,t2;
    
    if (key == IR_NONE)
    {   // increment no-action timer
        if (!blanking_invert && !test_nixies && (++ir_cmd_tmr > 200))
        {   // back to idle after 20 seconds
            set_time_IR  = IR_NO_TIME;  /* No blanking begin/end display */
            set_color_IR = false;       /* No color intensity display */
            ir_cmd_std   = IR_CMD_IDLE; /* default state */
            return; // exit
        } // if
    } // if
    else ir_cmd_tmr = 0; // reset timer if IR key is pressed
    
    switch (ir_cmd_std)
    {
        case IR_CMD_IDLE:
            if (key == IR_0)      ir_cmd_std = IR_CMD_0;
            else if (key == IR_1) 
            {
                ir_cmd_std = IR_CMD_1; // show version number for 5 seconds
                tmr_xsec   = 0;
            } // else if
            else if (key == IR_2) 
            {
                ir_cmd_std = IR_CMD_2; // show last response status from ESP8266
                tmr_xsec = 0;
            } // else if
            else if (key == IR_3) 
            {
                xputs("e0\n");   // Get Date & Time from ESP8266 NTP server
                tmr_xsec = 0;
            } // else if
            else if (key == IR_6) 
            {
                ir_cmd_std = IR_CMD_6; // Blanking invert
                tmr_xsec = 0;          // reset timer
            } // else if
            else if (key == IR_7) 
            {
                ir_cmd_std = IR_CMD_7; // Test mode
                tmr_xsec = 0;          // reset timer
            } // else if
            else if (key == IR_8) 
            {
                ir_cmd_std = IR_CMD_8; // Set Blanking-Begin time
                tmr_xsec = 0;          // reset timer
            } // else if
            else if (key == IR_9) 
            {
                ir_cmd_std = IR_CMD_9; // Set Blanking-End time
                tmr_xsec = 0;          // reset timer
            } // else if
            break;
            
        case IR_CMD_0:
            break;
            
        case IR_CMD_1: // show version number for 5 seconds
            if (++tmr_xsec >= 50)
            {
                show_date_IR = IR_SHOW_TIME;
                ir_cmd_std   = IR_CMD_IDLE;
            } // if
            else
            {   // no time-out yet
                x = strlen(nixie_ver);
                time_arr[POS0]  = (uint8_t)(nixie_ver[x-5] - '0');
                time_arr[POS1]  = (uint8_t)(nixie_ver[x-3] - '0');
                time_arr[POS2]  = (uint8_t)(nixie_ver[x-2] - '0');
                show_date_IR    = IR_SHOW_VER;
            } // else
            break;
            
        case IR_CMD_2: // show last response status from ESP8266
            if (++tmr_xsec >= 30)
            {
                show_date_IR = IR_SHOW_TIME;
                ir_cmd_std   = IR_CMD_IDLE;
            } // if
            else
            {   // no time-out yet
             temp = ESP8266_MINUTES - (esp8266_tmr / 60); // minutes left until next update
             t2   = encode_to_bcd4(temp);
             //time_arr[POS0]  = DIG_t; // t
             time_arr[POS1]  = (uint8_t)((t2 >> 8) & 0x0F); // MSB of minutes left
             time_arr[POS2]  = (uint8_t)((t2 >> 4) & 0x0F); // middle byte of minutes left
             time_arr[POS3]  = (uint8_t)(t2 & 0x0F);        // LSB of minutes left 
             time_arr[POS4]  = DIG_SPACE; // leave empty
             time_arr[POS5]  = (last_esp8266) ? DIG_1 : DIG_0;
             show_date_IR = IR_SHOW_ESP_STAT;
            } // else
            break;
            
        case IR_CMD_3:            
            break;
            
        case IR_CMD_4: // Show temperature for 5 seconds
            break;
            
        case IR_CMD_5: // Set intensity of colors
            break;
            
        case IR_CMD_6: // Invert Blanking Active for 60 seconds
            if (++tmr_xsec >= 600)
            {
                blanking_invert = false;
                ir_cmd_std      = IR_CMD_IDLE;
            } // if
            else blanking_invert = true;
            break;
            
        case IR_CMD_7: // Set test mode for 60 seconds
            if (++tmr_xsec >= 600)
            {
                test_nixies = false;
                ir_cmd_std     = IR_CMD_IDLE;
            } // if
            else test_nixies = true;
            break;
            
        case IR_CMD_8: // Set Blanking Begin
            x = encode_to_bcd2(blank_begin_h);
            time_arr[POS0] = (x >> 4) & 0x0F; // MSB of hours blanking-begin
            time_arr[POS1] = x & 0x0F;        // LSB of hours blanking-begin
            x = encode_to_bcd2(blank_begin_m);
            time_arr[POS2] = (x >> 4) & 0x0F; // MSB of minutes blanking-begin
            time_arr[POS3] = x & 0x0F;        // LSB of minutes blanking-begin 
            time_arr_idx   = POS0;            // start at MSB of blanking-begin hours
            set_time_IR    = IR_BB_TIME;      // indicate change blanking-begin time
            ir_cmd_std     = IR_CMD_CURSOR;   // use cursor keys to change time
            break;
            
        case IR_CMD_9: // Set Blanking End
            x = encode_to_bcd2(blank_end_h);
            time_arr[POS0] = (x >> 4) & 0x0F; // MSB of hours blanking-end
            time_arr[POS1] = x & 0x0F;        // LSB of hours blanking-end
            x = encode_to_bcd2(blank_end_m);
            time_arr[POS2] = (x >> 4) & 0x0F; // MSB of minutes blanking-end
            time_arr[POS3] = x & 0x0F;        // LSB of minutes blanking-end
            time_arr_idx   = POS0;            // start at MSB of blanking-end hours
            set_time_IR    = IR_BE_TIME;      // indicate change blanking-end time
            ir_cmd_std     = IR_CMD_CURSOR;   // use cursor keys to change time
            break;

        case IR_CMD_HASH: // Show date & year for 8 seconds
            break;
            
        case IR_CMD_CURSOR:  // use cursor keys to change time for blanking-begin & -end
            x = time_arr[time_arr_idx]; // get current digit
            switch (key)
            {
                case IR_0: case IR_1: case IR_2: case IR_3: case IR_4:
                case IR_5: case IR_6: case IR_7: case IR_8: case IR_9:
                    check_possible_digit(key);
                    break;
                case IR_UP: 
                    check_possible_digit(++x); 
                    break;
                case IR_DOWN: 
                    check_possible_digit(--x); 
                    break;
                case IR_LEFT: 
                    if (time_arr_idx == POS0) 
                         time_arr_idx = POS3;
                    else time_arr_idx--;
                    break;
                case IR_RIGHT: 
                    if (time_arr_idx == POS3) 
                         time_arr_idx = POS0;
                    else time_arr_idx++;
                     break;
                case IR_OK: 
                    if (set_time_IR == IR_BB_TIME)
                    {  // Blanking-time Begin
                       blank_begin_h = 10 * time_arr[POS0] + time_arr[POS1];
                       blank_begin_m = 10 * time_arr[POS2] + time_arr[POS3];
                       eeprom_write_byte(EEPARB_HR1  , blank_begin_h); // Start hour of blanking Nixies
                       eeprom_write_byte(EEPARB_MIN1 , blank_begin_m); // Start minute of blanking Nixies
                       set_time_IR = IR_NO_TIME;
                       ir_cmd_std  = IR_CMD_IDLE;
                    } // if
                    else if (set_time_IR == IR_BE_TIME)
                    {  // Blanking-time End
                       blank_end_h = 10 * time_arr[POS0] + time_arr[POS1];
                       blank_end_m = 10 * time_arr[POS2] + time_arr[POS3];
                       eeprom_write_byte(EEPARB_HR2  , blank_end_h); // End hour of blanking Nixies
                       eeprom_write_byte(EEPARB_MIN2 , blank_end_m); // End minute of blanking Nixies
                       set_time_IR = IR_NO_TIME;
                       ir_cmd_std  = IR_CMD_IDLE;
                    } // if
                    break;
                default: break; // ignore all other keys
            } // switch
            break;
            
        default:
            ir_cmd_std = IR_CMD_IDLE;
            break;
    } // switch   
} // handle_ir_command()

/*-----------------------------------------------------------------------------
  Purpose  : This is the 100 msec. task from the task-scheduler and it controls
             the IR remote controller. If the PORTC IRQ handler signals a new
             IR signal, it sets ir_rdy high. This function then decodes the
             IR signal into a key pressed and resets the PORTC IRQ handler for
             reception of a new IR signal.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void ir_task(void)
{
    uint8_t i, key = IR_NONE;
    
    if (ir_rdy)
    {
       if (ir_decode_nec()) 
       {
           key = ir_key(); // find the key pressed
       } // if
       for (i = 0; i < 99; i++) rawbuf[i] = 0; // clear buffer
       tmr1_std = STATE_IDLE; // reset state for next IR code
       ir_rdy   = false;      // done here
    } // if
    handle_ir_command(key);   // run this every 100 msec.
} // ir_task()

