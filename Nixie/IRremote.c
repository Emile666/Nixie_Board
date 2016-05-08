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
//******************************************************************************
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>
#include <util/atomic.h>
#include "IRremote.h"
#include "usart.h"
#include "Nixie.h"

// Allow all parts of the code access to the ISR data
// NB. The data can be changed by the ISR at any time, even mid-function
// Therefore we declare it as "volatile" to stop the compiler/CPU caching it
volatile irparams_t  irparams;
decode_results results; // create struct for IR-library
uint8_t        ir_remote_key = IR_NONE; // Result from ir_receive()

//+=============================================================================
// The match functions were (apparently) originally MACROs to improve code speed
//   (although this would have bloated the code) hence the names being CAPS
// A later release implemented debug output and so they needed to be converted
//   to functions.
// I tried to implement a dual-compile mode (DEBUG/non-DEBUG) but for some
//   reason, no matter what I did I could not get them to function as macros again.
// I have found a *lot* of bugs in the Arduino compiler over the last few weeks,
//   and I am currently assuming that one of these bugs is my problem.
// I may revisit this code at a later date and look at the assembler produced
//   in a hope of finding out what is going on, but for now they will remain as
//   functions even in non-DEBUG mode
//
int  MATCH (int measured,  int desired)
{
  bool passed = ((measured >= TICKS_LOW(desired)) && (measured <= TICKS_HIGH(desired)));
  return passed;
} // MATCH()

//---------------------------------------------------------------------
// Due to sensor lag, when received, Marks tend to be 100us too short
//---------------------------------------------------------------------
int  MATCH_MARK (int measured_ticks,  int desired_us)
{
  bool passed = ((measured_ticks >= TICKS_LOW (desired_us - MARK_EXCESS))
                && (measured_ticks <= TICKS_HIGH(desired_us - MARK_EXCESS)));
  return passed;
} // MATCH_MARK()

//---------------------------------------------------------------------
// Due to sensor lag, when received, Spaces tend to be 100us too long
//---------------------------------------------------------------------
int  MATCH_SPACE (int measured_ticks,  int desired_us)
{
  bool passed = ((measured_ticks >= TICKS_LOW (desired_us + MARK_EXCESS))
                && (measured_ticks <= TICKS_HIGH(desired_us + MARK_EXCESS)));
  return passed;
} // MATCH_SPACE()

/*------------------------------------------------------------------
  Purpose  : This function initializes the IR-library.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void ir_init(void)
{
	// Initialize state machine variables
	irparams.rcvstate = STATE_IDLE;
	irparams.rawlen   = 0;
} // ir_init()

//+=============================================================================
// Interrupt Service Routine - Fires every 50uS
// TIMER2 interrupt code to collect raw data.
// Widths of alternating SPACE, MARK are recorded in rawbuf.
// Recorded in ticks of 50uS [microseconds, 0.000050 seconds]
// 'rawlen' counts the number of entries recorded so far.
// First entry is the SPACE between transmissions.
// As soon as a the first [SPACE] entry gets long:
//   Ready is set; State switches to IDLE; Timing of SPACE continues.
// As soon as first MARK arrives:
//   Gap width is recorded; Ready is cleared; New logging starts
//
void ir_isr(void)
{
	// Read if IR Receiver -> SPACE [xmt LED off] or a MARK [xmt LED on]
	uint8_t  irdata = 0;
	if (PINB & IR_RCV) // PB3 (D11). NOTE: PINB, not PORTB for reading!
	     irdata = 1;
	else irdata = 0;
	irparams.timer++;  // One more 50uS tick
	if (irparams.rawlen >= RAWBUF)  irparams.rcvstate = STATE_OVERFLOW ;  // Buffer overflow

	switch(irparams.rcvstate) {
		//......................................................................
		case STATE_IDLE: // In the middle of a gap
			if (irdata == MARK) {
				if (irparams.timer < GAP_TICKS)  {  // Not big enough to be a gap.
					irparams.timer = 0;
				} else {
					// Gap just ended; Record duration; Start recording transmission
					irparams.overflow                  = false;
					irparams.rawlen                    = 0;
					irparams.rawbuf[irparams.rawlen++] = irparams.timer;
					irparams.timer                     = 0;
					irparams.rcvstate                  = STATE_MARK;
				} // else
			} // if
			break;
		//......................................................................
		case STATE_MARK:  // Timing Mark
			if (irdata == SPACE) {   // Mark ended; Record time
				irparams.rawbuf[irparams.rawlen++] = irparams.timer;
				irparams.timer                     = 0;
				irparams.rcvstate                  = STATE_SPACE;
			} // if
			break;
		//......................................................................
		case STATE_SPACE:  // Timing Space
			if (irdata == MARK) {  // Space just ended; Record time
				irparams.rawbuf[irparams.rawlen++] = irparams.timer;
				irparams.timer                     = 0;
				irparams.rcvstate                  = STATE_MARK;
			} else if (irparams.timer > GAP_TICKS) {  // Space
					// A long Space, indicates gap between codes
					// Flag the current code as ready for processing
					// Switch to STOP
					// Don't reset timer; keep counting Space width
					irparams.rcvstate = STATE_STOP;
			} // else if
			break;
		//......................................................................
		case STATE_STOP:  // Waiting; Measuring Gap
		 	if (irdata == MARK)  irparams.timer = 0 ;  // Reset gap timer
		 	break;
		//......................................................................
		case STATE_OVERFLOW:  // Flag up a read overflow; Stop the State Machine
			irparams.overflow = true;
			irparams.rcvstate = STATE_STOP;
		 	break;
	} // switch
} // ir_isr()

/*------------------------------------------------------------------
  Purpose  : This function decodes the bits from rawbuf and stores
             the result in results->value.
  Variables: results: pointer to decode_results struct
  Returns  : true = success ; false = error
  ------------------------------------------------------------------*/
bool  ir_decode_nec(decode_results *results)
{
	long  data   = 0;  // We decode in to here; Start with nothing
	int   offset = 1;  // Index in to results; Skip first entry!?

	if (results->rcvstate != STATE_STOP)  return false ;
	// Check header "mark"
	if (!MATCH_MARK(results->rawbuf[offset], NEC_HDR_MARK))  return false;
	offset++;
	// Check for repeat
	if ((results->rawlen == 4) &&
	MATCH_SPACE(results->rawbuf[offset  ], NEC_RPT_SPACE) &&
	MATCH_MARK (results->rawbuf[offset+1], NEC_BIT_MARK))
	{
		results->bits  = 0;
		results->value = REPEAT;
		return true;
	} // if
	// Check we have enough data
	if (results->rawlen < (2 * NEC_BITS) + 4) return false;
	// Check header "space"
	if (!MATCH_SPACE(results->rawbuf[offset], NEC_HDR_SPACE)) return false;
	offset++;
	// Build the data
	for (int i = 0;  i < NEC_BITS;  i++) {
		// Check data "mark"
		if (!MATCH_MARK(results->rawbuf[offset], NEC_BIT_MARK)) return false;
		offset++;
		// Suppend this bit
		if      (MATCH_SPACE(results->rawbuf[offset], NEC_ONE_SPACE ))  data = (data << 1) | 1 ;
		else if (MATCH_SPACE(results->rawbuf[offset], NEC_ZERO_SPACE))  data = (data << 1) | 0 ;
		else return false;
		offset++;
	} // for
	// Success
	results->bits  = NEC_BITS;
	results->value = data;
	return true;
} // ir_decode_nec()

/*------------------------------------------------------------------
  Purpose  : This function decodes the bits from rawbuf and stores
             the result in results->value
  Variables: results: pointer to decode_results struct
  Returns  : true = success ; false = error
  ------------------------------------------------------------------*/
bool  ir_decode(decode_results *results)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		results->rawbuf   = irparams.rawbuf;
		results->rawlen   = irparams.rawlen;
		results->overflow = irparams.overflow;
		results->rcvstate = irparams.rcvstate;
	}	
	if (ir_decode_nec(results)) return true;
	ir_resume(); // Throw away and start over
	return false;
} // ir_decode()

/*------------------------------------------------------------------
  Purpose  : This function checks if the state-machine for receiving
             IR bits has stopped.
  Variables: -
  Returns  : true = stopped ; false = still running
  ------------------------------------------------------------------*/
bool ir_is_idle(void)
{
	uint8_t rcvstate;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		rcvstate = irparams.rcvstate;
	}	
	return ((rcvstate == STATE_IDLE) || (rcvstate == STATE_STOP)) ? true : false;
} // ir_is_idle()

/*------------------------------------------------------------------
  Purpose  : This function restarts the state-machine.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void ir_resume(void)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		irparams.rcvstate = STATE_IDLE;
		irparams.rawlen   = 0;
	}	
} // ir_resume()

/*------------------------------------------------------------------
  Purpose  : This function checks is a command is entered via the
             IR sensor. If so, it executes it.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void std_cmd(void)
{
	static uint8_t cmd_state = NO_CMD;
	static uint8_t idx = 0;
	static uint8_t inp_arr[6];
	char           s[20];
	
	switch (cmd_state)
	{
		case NO_CMD:   if (ir_remote_key == IR_ASTERISK) 
					      cmd_state = CMD_MODE;
					   break;
		case CMD_MODE: if (ir_remote_key == IR_1)
					   {
						   xputs("Enter Time (hhmmss):");
						   idx  = 0;
						   cmd_state = CMD_TIME;
					   }			
					   else if (ir_remote_key == IR_2)
					   {
						   xputs("Enter Date (ddmmyy:");
						   idx  = 0;
						   cmd_state = CMD_DATE;
					   }		   
					   else if (ir_remote_key == IR_HASH)
					      cmd_state = NO_CMD;
					   break;
		case CMD_TIME: if (ir_remote_key <= IR_9)
					   {
						   inp_arr[idx] = ir_remote_key;
						   sprintf(s,"%d",ir_remote_key);
						   xputs(s);
						   if (++idx > 5) cmd_state = TIME_EXEC;
					   } // if
					   break;
		case TIME_EXEC: if (ir_remote_key == IR_HASH)
						{
							xputs("Set-Time EXEC:");
							sprintf(s,"%d%d:%d%d:%d%d\n",inp_arr[0],inp_arr[1],inp_arr[2],
							                             inp_arr[3],inp_arr[4],inp_arr[5]);
							xputs(s);
							cmd_state = NO_CMD;
						} // if
						break;
		case CMD_DATE:  if (ir_remote_key <= IR_9)
						{
							inp_arr[idx] = ir_remote_key;
							sprintf(s,"%d",ir_remote_key);
							xputs(s);
							if (++idx > 5) cmd_state = DATE_EXEC;
						} // if
						break;
		case DATE_EXEC: if (ir_remote_key == IR_HASH)
						{
							xputs("Set-Date EXEC:");
							sprintf(s,"%d%d-%d%d-%d\n",inp_arr[0],inp_arr[1],inp_arr[2],
														 inp_arr[3],2000+10*inp_arr[4]+inp_arr[5]);
							xputs(s);
							cmd_state = NO_CMD;
						} // if
						break;
	} // switch
} // std_cmd()

/*------------------------------------------------------------------
  Purpose  : This function is the entry-point for the IR-receive
             library. It can be called every 500 msec.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void ir_receive(void)
{
	char    s[20];
	static  uint8_t cmd_rcv = false;
	
	if (ir_decode(&results)) // have we received an IR signal?
	{
	  switch (results.value)
	  {
		case IR_CODE_UP      : ir_remote_key = IR_UP      ; break;
		case IR_CODE_DOWN    : ir_remote_key = IR_DOWN    ; break;
		case IR_CODE_LEFT    : ir_remote_key = IR_LEFT    ; break;
		case IR_CODE_RIGHT   : ir_remote_key = IR_RIGHT   ; break;
		case IR_CODE_OK		 : ir_remote_key = IR_OK      ; break;
		case IR_CODE_ASTERISK: ir_remote_key = IR_ASTERISK; break;
		case IR_CODE_HASH    : ir_remote_key = IR_HASH    ; break;
		case IR_CODE_0       : ir_remote_key = IR_0       ; break;
		case IR_CODE_1       : ir_remote_key = IR_1       ; break;
		case IR_CODE_2       : ir_remote_key = IR_2       ; break;
		case IR_CODE_3       : ir_remote_key = IR_3       ; break;
		case IR_CODE_4       : ir_remote_key = IR_4       ; break;
		case IR_CODE_5       : ir_remote_key = IR_5       ; break;
		case IR_CODE_6       : ir_remote_key = IR_6       ; break;
		case IR_CODE_7       : ir_remote_key = IR_7       ; break;
		case IR_CODE_8       : ir_remote_key = IR_8       ; break;
		case IR_CODE_9       : ir_remote_key = IR_9       ; break;
		case IR_CODE_REPEAT  : ir_remote_key = IR_REPEAT  ; break;
		default              : ir_remote_key = IR_NONE    ; break;
	  } // switch
	  sprintf(s,"ir[%c]\n",IR_CHARS[ir_remote_key]);
	  xputs(s);    // output to terminal screen
	  std_cmd();   // execute STD for commands
	  ir_resume(); // receive the next remote key
	} // if
} // ir_receive()
