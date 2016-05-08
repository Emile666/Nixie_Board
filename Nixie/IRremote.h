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

//==============================================================================
//                           N   N  EEEEE   CCCC
//                           NN  N  E      C
//                           N N N  EEE    C
//                           N  NN  E      C
//                           N   N  EEEEE   CCCC
//==============================================================================
#define NEC_BITS          32
#define NEC_HDR_MARK    9000
#define NEC_HDR_SPACE   4500
#define NEC_BIT_MARK     560
#define NEC_ONE_SPACE   1690
#define NEC_ZERO_SPACE   560
#define NEC_RPT_SPACE   2250

//------------------------------------------------------------------------------
// Mark & Space matching functions
//
int  MATCH       (int measured, int desired) ;
int  MATCH_MARK  (int measured_ticks, int desired_us) ;
int  MATCH_SPACE (int measured_ticks, int desired_us) ;

//------------------------------------------------------------------------------
// Information for the Interrupt Service Routine
//
#define RAWBUF  101  // Maximum length of raw duration buffer

typedef struct {
	// The fields are ordered to reduce memory over caused by struct-padding
	uint8_t       rcvstate;        // State Machine state
	uint8_t       rawlen;          // counter of entries in rawbuf
	unsigned int  timer;           // State timer, counts 50uS ticks.
	unsigned int  rawbuf[RAWBUF];  // raw data
	uint8_t       overflow;        // Raw buffer overflow occurred
} irparams_t;

// ISR State-Machine : Receiver States
#define STATE_IDLE      2
#define STATE_MARK      3
#define STATE_SPACE     4
#define STATE_STOP      5
#define STATE_OVERFLOW  6

//------------------------------------------------------------------------------
// Pulse parms are ((X*50)-100) for the Mark and ((X*50)+100) for the Space.
// First MARK is the one after the long gap
// Pulse parameters in uSec
//
// Due to sensor lag, when received, Marks  tend to be 100us too long and
//                                   Spaces tend to be 100us too short
#define MARK_EXCESS    100

// microseconds per clock interrupt tick
#define USECPERTICK    50

// Upper and Lower percentage tolerances in measurements
#define TOLERANCE       50
#define LTOL            (1.0 - (TOLERANCE/100.))
#define UTOL            (1.0 + (TOLERANCE/100.))

// Minimum gap between IR transmissions
#define _GAP            20000
#define GAP_TICKS       (_GAP/USECPERTICK)

#define TICKS_LOW(us)   ((int)(((us)*LTOL/USECPERTICK)))
#define TICKS_HIGH(us)  ((int)(((us)*UTOL/USECPERTICK + 1)))

//------------------------------------------------------------------------------
// IR detector output is active low
//
#define MARK   0
#define SPACE  1

//------------------------------------------------------------------------------
// Results returned from the decoder
//
typedef struct _decode_results
{
		unsigned long          value;        // Decoded value [max 32-bits]
		int                    bits;         // Number of bits in decoded value
		volatile unsigned int  *rawbuf;      // Raw intervals in 50uS ticks
		int                    rawlen;       // Number of records in rawbuf
		int                    overflow;     // true if IR raw code too long
		uint8_t                rcvstate;     // State Machine state
} decode_results;

// Decoded value for NEC when a repeat code is received
#define REPEAT 0xFFFFFFFF

// KEY values for the remote control
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

#define IR_CHARS         "0123456789ULRDOAHX?"

#define NO_CMD    (0)
#define CMD_MODE  (1)
#define CMD_TIME  (2)
#define TIME_EXEC (3)
#define CMD_DATE  (4)
#define DATE_EXEC (5)

//------------------------------------------------------------------------------
// Main routines for receiving IR
//------------------------------------------------------------------------------
void ir_init(void);
void ir_isr(void);
bool ir_decode(decode_results *results);
bool ir_is_idle(void);
void ir_resume(void);
void ir_receive(void);

#endif
