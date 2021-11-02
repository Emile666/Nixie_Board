//-----------------------------------------------------------------------------
// Created: 07/12/2011 15:17:35
// Author : Emile
// File   : Nixie.c
//-----------------------------------------------------------------------------
#include <avr/io.h>
#include <util/atomic.h>
#include "scheduler.h"
#include "usart.h"
#include "IRremote.h"
#include "Nixie.h"
#include "command_interpreter.h"
#include "bme280.h"
#include "eep.h"

char          nixie_ver[] = "Nixie New HW v0.33\n";
bool		  test_nixies = false;	    // S3 command / IR #9 command
bool          relay_status;             // Relay status, on (1) or off (0)
uint8_t       cnt_50usec  = 0;			// 50 usec. counter
unsigned long t2_millis   = 0UL;		// msec. counter
uint32_t      bme280_press = 0.0;		// Pressure E-1 mbar
int16_t       bme280_temp  = 0;	  		// Temperature in E-2 Celsius
uint32_t	  bme280_hum   = 0;			// Humidity in Q22.10 format		

bool          dst_active      = false; // true = Daylight Saving Time active
uint8_t       blank_begin_h   = 0;
uint8_t       blank_begin_m   = 0;
uint8_t       blank_end_h     = 0;
uint8_t       blank_end_m     = 0;

uint8_t rgb_pattern = FIXED;		  // RGB color mode: [OFF, RANDOM, DYNAMIC, FIXED]
uint8_t fixed_rgb_colour = CYAN;	  // RGB colour variable used in Nixie.c

uint8_t       wheel_effect    = 0;    // 0: none, 1: from 59->00, 2: every second/minute
bool          display_time    = true; // true = hh:mm:ss is on display 
uint8_t		  col_time;               // Colour for Time display
uint8_t		  col_date;               // Colour for Date & Year display
uint8_t       col_temp;               // Colour for Temperature display
uint8_t       col_humi;               // Colour for Humidity display
uint8_t       col_dewp;               // Colour for Dew-point display
uint8_t       col_pres;               // Colour for Pressure display
uint8_t       col_roll;               // Colour for second roll-over
uint8_t       led_r[NR_LEDS];         // Array with 8-bit red colour for all WS2812
uint8_t       led_g[NR_LEDS];         // Array with 8-bit green colour for all WS2812
uint8_t       led_b[NR_LEDS];         // Array with 8-bit blue colour for all WS2812
bool          hv_relay_sw;            // switch for hv_relay
bool          hv_relay_fx;            // fix for hv_relay

bool     set_col_white   = false; // true = esp8266 time update was successful
bool     blanking_invert = false; // Invert blanking-active IR-command
bool     enable_test_IR  = false; // Enable Test-pattern IR-command
bool     last_esp8266    = false; // true = last esp8266 command was successful
uint8_t  esp8266_std    = ESP8266_INIT;         // update time from ESP8266 every 18 hours
uint16_t esp8266_tmr    = ESP8266_SECONDS - 30; // ESP8266 timer, update 30 sec. after power-up
uint8_t  tmr1_std = STATE_IDLE;  // FSM for reading IR codes
uint16_t rawbuf[100];            // buffer with clock-ticks from IR-codes
uint8_t  rawlen     = 0;         // number of bits read from IR
uint16_t prev_ticks = 0;         // previous value of ticks, used for bit-length calc.
uint8_t  show_date_IR = IR_SHOW_TIME; // What to display on the 7-segment displays
uint8_t  set_time_IR  = IR_NO_TIME;   // Show normal time or blanking begin/end time
bool     set_color_IR = false;        // true = set color intensity via IR

extern char     rs232_inbuf[];      // RS232 input buffer
extern bool     ir_rdy;             // flag for ir_task() that new IR code is received
extern uint8_t  time_arr[];         // Array for changing time or intensity with IR
extern uint8_t  time_arr_idx;       // Index into time_arr[]

//---------------------------------------------------------------------------
// Copy to hardware PCB v0.21: MSB (bit 39) first --> LSB (bit 00)
//
// Bits 39..32: Decimal points:  -    -    -    -   LDP1 LDP2 LDP3 LDP4
// Bits 31..24: Decimal points: LDP5 LDP6 RDP1 RDP2 RDP3 RDP4 RDP5 RDP6
// Bits 23..16: Hours         : SLD  SLC  SLB  SLA  SHD  SHC  SHB  SHA 
// Bits 15..08: Minutes       : MLD  MLC  MLB  MLA  MHD  MHC  MHB  MHA
// Bits 07..00: Seconds       : HLD  HLC  HLB  HLA  HHD  HHC  HHB  HHA
//---------------------------------------------------------------------------
uint32_t nixie_bits  = 0UL;
uint8_t  nixie_bits8 = 0UL;
uint8_t  rgb_colour  = BLACK;

/*------------------------------------------------------------------
  Purpose  : This function returns the number of milliseconds since
			 power-up. It is defined in delay.h
  Variables: -
  Returns  : The number of milliseconds since power-up
  ------------------------------------------------------------------*/
unsigned long millis(void)
{
	unsigned long m;
	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		m = t2_millis;
	}
	return m;
} // millis()

/*------------------------------------------------------------------
  Purpose  : This function waits a number of milliseconds. It is 
             defined in delay.h. Do NOT use this in an interrupt.
  Variables: 
         ms: The number of milliseconds to wait.
  Returns  : -
  ------------------------------------------------------------------*/
void delay_msec(uint16_t ms)
{
	unsigned long start = millis();

	while ((millis() - start) < ms) ;
} // delay_msec()

/*------------------------------------------------------------------------
  Purpose  : This is the Timer-Interrupt routine which runs every msec. 
             (f=1 kHz). It is used by the task-scheduler.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------------*/
ISR(TIMER2_COMPA_vect)
{
	PORTB |= TIME_MEAS;  // Time Measurement
	scheduler_isr();     // call the ISR routine for the task-scheduler
	t2_millis++;         // update millisecond counter
	PORTB &= ~TIME_MEAS; // Time Measurement
} // ISR()

/*------------------------------------------------------------------
  Purpose  : This is the State-change interrupt routine for PORTB. 
             It interrupts whenever a change on IR_RCV is detected.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
ISR (PCINT0_vect)
{
    uint16_t diff_ticks;
    uint16_t ticks = tmr1_val(); // counts at f = 62.5 kHz, T = 16 usec.
    uint8_t  ir_rcvb = PINB & IR_RCV;
	
    if (ticks < prev_ticks)
         diff_ticks = ~prev_ticks + ticks;
    else diff_ticks = ticks  - prev_ticks;
    if (ir_rcvb) // copy IR-signal to debug output
         PORTB |=  IR_LED;
    else PORTB &= ~IR_LED;
    switch (tmr1_std)
    {
	    case STATE_IDLE:
	    if (!ir_rcvb)
	    {   // falling edge
		    rawbuf[0] = ticks;
		    rawlen    = 1;
		    tmr1_std  = STATE_MARK;
	    } // if
	    break;
	    case STATE_MARK: // A mark is a 0 for the VS1838B
	    if (ir_rcvb)
	    {   // rising edge, end of mark
		    if (rawlen < 99)
		    {
			    rawbuf[rawlen++] = diff_ticks;
			    tmr1_std         = STATE_SPACE;
		    } // if
		    else
		    {   // overflow
			    ir_rdy   = true;
			    tmr1_std = STATE_STOP;
		    } // else
	    } // if
	    break;
	    case STATE_SPACE:
	    if (!ir_rcvb)
	    {   // falling edge, end of space
		    if (diff_ticks > 1250) // 1250 = 20 msec. min. gap between transmissions
		    {   // long space received, ready to process everything
			    ir_rdy    = true;
			    tmr1_std  = STATE_STOP;
		    } // if
		    else if (rawlen < 99)
		    {
			    rawbuf[rawlen++] = diff_ticks;
			    tmr1_std         = STATE_MARK;
		    } // if
		    else
		    {   // overflow
			    ir_rdy   = true;
			    tmr1_std = STATE_STOP;
		    } // else
	    } // if
	    break;
	    case STATE_STOP:
	    // remain in this state unless ir_task() resets this
	    break;
    } // switch
    prev_ticks = ticks; // save ticks value	
} // ISR(PCINT0_vect)

/*-----------------------------------------------------------------------------
  Purpose  : This routine sends one byte to the WS2812B LED-string.
  Variables: bt: the byte to send
  Returns  : -
  ---------------------------------------------------------------------------*/
void ws2812b_send_byte(uint8_t bt)
{
    uint8_t i,x = 0x80; // Start with MSB first
    
    for (i = 0; i < 8; i++)
    {
        if (bt & x)
        {    // Send a 1   
             ws2812b_send_1;
        } // if
        else 
        {   // Send a 0
            ws2812b_send_0;
        } // else
        x >>= 1; // Next bit
    } // for i
} // ws2812b_send_byte()

/*-----------------------------------------------------------------------------
  Purpose  : This routine sends the RGB-bytes for every LED to the WS2812B
             LED string.
  Variables: 
      led_g: the green byte array
      led_r: the red byte array
      led_b: the blue byte array
  Returns  : -
  ---------------------------------------------------------------------------*/
void ws2812_send_all(void)
{
    uint8_t i;
	
    cli(); // disable IRQ for time-sensitive LED-timing
    for (i = 0; i < NR_LEDS; i++)
    {
        ws2812b_send_byte(led_g[i]); // Send one byte of Green
        ws2812b_send_byte(led_r[i]); // Send one byte of Red
        ws2812b_send_byte(led_b[i]); // Send one byte of Blue
    } // for i
    sei(); // enable IRQ again
} // ws2812_send_all()

/*------------------------------------------------------------------------
  Purpose  : This task is called every time the time is displayed on the
             Nixies. It checks for a change from summer- to wintertime and
			 vice-versa.
			 To start DST: Find the last Sunday in March: @2 AM advance clock to 3 AM.
			 To stop DST : Find the last Sunday in October: @3 AM set clock back to 2 AM (only once!).
  Variables: p: pointer to time-struct
  Returns  : -
  ------------------------------------------------------------------------*/
void check_and_set_summertime(Time p)
{
	uint8_t        hr,day,lsun03,lsun10,dst_eep;
	uint16_t       min;
    static uint8_t advance_time = 0;
	static uint8_t revert_time  = 0;
#ifdef DEBUG_SENSORS
	char           s[20];
#endif
	
	if (p.mon == 3)
	{
		day    = ds3231_calc_dow(31,3,p.year); // Find day-of-week for March 31th
		lsun03 = 31 - (day % 7);               // Find last Sunday in March
#ifdef DEBUG_SENSORS
		sprintf(s,"lsun03=%d\n",lsun03); xputs(s);
#endif
		switch (advance_time)
		{
			case 0: if (p.date == lsun03)
					{	// last Sunday in March
						min = (uint16_t)p.hour * 60 + p.min; // convert to minutes
						if (min < 120) dst_active = false;   // summertime not yet active
						else if (min == 120)
						{   // At 2:00 AM advance time to 3 AM, check for one minute
							advance_time = 1; // goto next state
						} // if
						else dst_active = true; // min > 120
					} // if
					else if (p.date < lsun03) dst_active = false;
					else if (p.date > lsun03) dst_active = true;
					break;
			case 1: // Now advance time, do this only once
					ds3231_settime(3,0,p.sec); // Set time to 3:00, leave secs the same
					eeprom_write_byte(EEPARB_DST,0x01); // set DST in eeprom
					advance_time = 2;                   // goto next state
					dst_active   = true;
					break;
			case 2: if (p.min > 0) advance_time = 0; // At 3:01:00 back to normal
					dst_active = true;
					break;
		} // switch
	} // if
	else if (p.mon == 10)
	{
		day    = ds3231_calc_dow(31,10,p.year); // Find day-of-week for October 31th
		lsun10 = 31 - (day % 7);                // Find last Sunday in October
#ifdef DEBUG_SENSORS
		sprintf(s,"lsun10=%d\n",lsun10); xputs(s);
#endif
		switch (revert_time)
		{
			case 0: if (p.date == lsun10)
					{	// last Sunday in October
						min = (uint16_t)p.hour * 60 + p.min; // convert to minutes
						if (min < 180) dst_active = true;    // summertime still active
						else if (dst_active && (min == 180)) // only when dst is active
						{   // At 3:00 AM revert time back to 2 AM, check for one minute
							revert_time = 1; // goto next state
						} // if
						else dst_active = false; // min > 180
					} // if
					else if (p.date < lsun10) dst_active = true;
					else if (p.date > lsun10) dst_active = false;
					break;
			case 1: // Now revert time, do this only once
					ds3231_settime(2,0,p.sec); // Set time back to 2:00, leave secs the same
					eeprom_write_byte(EEPARB_DST,0x00); // reset DST in eeprom
					revert_time = 2;
					dst_active  = false;
					break;
			case 2: // make sure that we passed 3 AM in order to prevent multiple reverts
					if (p.hour > 3) revert_time = 0; // at 4:00:00 back to normal
					dst_active = false;
					break;
		} // switch
	} // else if
	else if ((p.mon < 3) || (p.mon > 10)) dst_active = false; // January, February, November, December
	else                                  dst_active = true;  // April until September

    //------------------------------------------------------------------------
    // If, for some reason, the clock was powered-off during the change to
    // summer- or winter-time, the eeprom value differs from the actual
    // dst_active value. If so, set the actual sommer- and winter-time.
    //------------------------------------------------------------------------
    dst_eep = (uint8_t)eeprom_read_byte(EEPARB_DST);
    if (dst_active && !dst_eep)
    {   // It is summer-time, but clock has not been advanced yet
	    hr = (p.hour >= 23) ? 0 : p.hour + 1;
	    ds3231_settime(hr,p.min,p.sec); // Set summer-time to 1 hour later
	    eeprom_write_byte(EEPARB_DST,0x01); // set DST in eeprom
    } // if
    else if (!dst_active && dst_eep)
    {   // It is winter-time, but clock has not been moved back yet
	    hr = (p.hour > 0) ? p.hour - 1 : 23;
	    ds3231_settime(hr,p.min,p.sec); // Set summer-time to 1 hour earlier
	    eeprom_write_byte(EEPARB_DST,0x00); // reset DST in eeprom
    } // if
} // check_and_set_summertime()

/*------------------------------------------------------------------------
  Purpose  : This task is called by the Task-Scheduler every 5 seconds.
             It reads the BME280 pressure and temperature.
  Variables: bme280_pressure, bme280_temp
  Returns  : -
  ------------------------------------------------------------------------*/
void bme280_task(void)
{
	bme280_temp  = bme280_temperature();
	bme280_press = bme280_pressure();
	bme280_hum   = bme280_humidity();
} // bme280_task()

/*------------------------------------------------------------------------
  Purpose  : This task is called by the Task-Scheduler every 50 msec. 
             It updates the Nixie tubes with the new bits.
  Variables: nixie_bits (32 bits)
  Returns  : -
  ------------------------------------------------------------------------*/
void update_nixies(void)
{
	uint8_t         i, h, m, s;
	uint32_t        mask;        // start with MSB
	uint8_t         mask8;
	uint32_t        bitstream;   // copy of nixie_bits
	uint8_t         bitstream8;  // copy of nixie_bits8
	static uint8_t  wheel_cntr = 0;
	static uint8_t  bits_min_old, bits_sec_old, bits_hrs_old;
	uint8_t         wheel[WH_MAX] = {11,22,33,44,55,66,77,88,99,0};
			
	//bitstream = nixie_bits; // copy original bitstream for PCB < v0.21
	// Needed for PCB hardware v0.21 and higher
	// nixie_bits: LDP RDP HH HL MH ML SH SL
	// bitstream : LDP RDP SL SH ML MH HL HH
	bitstream  = (nixie_bits & 0xFF000000); // copy LDP and RDP bits
	bitstream |= (nixie_bits & 0x0000000F) << 20; // SL
	bitstream |= (nixie_bits & 0x000000F0) << 12; // SH
	bitstream |= (nixie_bits & 0x00000F00) <<  4; // ML
	bitstream |= (nixie_bits & 0x0000F000) >>  4; // MH
	bitstream |= (nixie_bits & 0x000F0000) >> 12; // HL
	bitstream |= (nixie_bits & 0x00F00000) >> 20; // HH
	
	bitstream8 = nixie_bits8; // copy original bitstream
	if (display_time)
	{
		//--------------------------------------------------------
		// WHEEL-EFFECT: Rotate digits + ANTI-POISONING Function
		//--------------------------------------------------------
		h = (uint8_t)((nixie_bits & 0x00FF0000) >> 16); // isolate minutes digits
		m = (uint8_t)((nixie_bits & 0x0000FF00) >>  8); // isolate minutes digits
		s = (uint8_t)(nixie_bits  & 0x000000FF);        // isolate seconds digits
		switch (wheel_effect)
		{
			case 0: // no wheel-effect
					if ((s == 0x00) && ((m == 0x00) || (m == 0x30)))
					{	// Anti-poisoning effect at every half- and full-hour
						if (wheel_cntr < WH_MAX-1)
						{
							bitstream |= (((uint32_t)wheel[wheel_cntr]) << 16); // seconds
							bitstream &= 0xFFFF0000; // clear minutes and hours bits
							bitstream |= (((uint32_t)wheel[wheel_cntr]) << 8);  // minutes
							bitstream |= ((uint32_t)wheel[wheel_cntr]);         // hours
						} // if
						if (++wheel_cntr > WH_MAX-1) wheel_cntr = WH_MAX-1;
					} // if
					else wheel_cntr = 0; // reset for next second
					break;
			case 1: // wheel-effect only from 59 -> 0 (minutes & seconds) + anti-poisoning effect
					if (s == 0x00)
					{	// seconds == 0, wheel-effect
						bitstream |= (((uint32_t)wheel[wheel_cntr]) << 16);
						if ((m == 0x00) || (m == 0x30))
						{	// // Anti-poisoning effect at every half- and full-hour
							if (wheel_cntr < WH_MAX-1)
							{
								bitstream &= 0xFFFF0000; // clear minutes and hours bits
								bitstream |= (((uint32_t)wheel[wheel_cntr]) << 8); // minutes
								bitstream |= ((uint32_t)wheel[wheel_cntr]);        // hours
							} // if							
						} // if
						if (++wheel_cntr > WH_MAX-1) wheel_cntr = WH_MAX-1;
					} // if
					else wheel_cntr = 0; // reset for next second
					break;
			case 2: // wheel-effect on every change in hours / minutes / seconds
					if (s != bits_sec_old)
					{	// change in seconds
						bitstream &= 0xFF00FFFF; // clear seconds bits
						bitstream |= (((uint32_t)wheel[wheel_cntr]) << 16);
						if (m != bits_min_old)
						{	// change in minutes
							bitstream   &= 0xFFFF00FF; // clear minutes bits
							bitstream   |= (((uint32_t)wheel[wheel_cntr]) << 8); // minutes
							if ((h != bits_hrs_old) || (m == 0x30))
							{	// change in hours or anti-poisoning effect
								bitstream   &= 0xFFFFFF00; // clear hours bits
								bitstream   |= ((uint32_t)wheel[wheel_cntr]); // hours
								if (wheel_cntr == WH_MAX-1)
								{
									bits_hrs_old = h;
								} // if
							} // if
							if (wheel_cntr == WH_MAX-1)
							{
								bits_min_old = m;
							} // if
						} // if
						if (++wheel_cntr > WH_MAX-1)
						{
							wheel_cntr   = WH_MAX-1;
							bits_sec_old = s;
						} // if
					} // if
					else wheel_cntr = 0; // reset for next second
					break;
		} // switch
	} // if	
	
	//--------------------------------------------------
	// Now send the first 8 nixie-bits to hardware
	// Bit-order: X X X X LDP1 LDP2 LDP3 LDP4
	//--------------------------------------------------
	mask8 = 0x80;
	for (i = 0; i < 8; i++)
	{
		PORTD &= ~(SHCP|STCP); // set clocks to 0
		if ((bitstream8 & mask8) == mask8)
		PORTD |=  SDIN; // set SDIN to 1
		else PORTD &= ~SDIN; // set SDIN to 0
		mask8 >>= 1;     // shift right 1
		PORTD |=  SHCP; // set clock to 1
		PORTD &= ~SHCP; // set clock to 0 again
	}
	//--------------------------------------------------
	// Now send the remaining 32 nixie-bits to hardware
	//--------------------------------------------------
	mask = 0x80000000;
	for (i = 0; i < 32; i++)
	{
		PORTD &= ~(SHCP|STCP); // set clocks to 0
		if ((bitstream & mask) == mask)
			 PORTD |=  SDIN; // set SDIN to 1
		else PORTD &= ~SDIN; // set SDIN to 0
		mask >>= 1;     // shift right 1
		PORTD |=  SHCP; // set clock to 1
		PORTD &= ~SHCP; // set clock to 0 again
	} // for i
	// Now clock bits from shift-registers to output-registers
	PORTD |=  STCP; // set clock to 1
	PORTD &= ~STCP; // set clock to 0 again
} // update_nixies()

/*------------------------------------------------------------------------
  Purpose  : Encode a byte into 2 BCD numbers.
  Variables: x: the byte to encode
  Returns  : the two encoded BCD numbers
  ------------------------------------------------------------------------*/
uint8_t encode_to_bcd(uint8_t x)
{
	uint8_t temp;
	uint8_t retv = 0;
		
	temp   = x / 10;
	retv  |= (temp & 0x0F);
	retv <<= 4; // SHL 4
	temp   = x - temp * 10;
	retv  |= (temp & 0x0F);
	return retv;
} // encode_to_bcd()

/*------------------------------------------------------------------------
  Purpose  : clears one Nixie tube. The 74141/K155ID1 ICs blank for
             codes > 9. This is used here.
  Variables: nr: [1..6], the Nixie tube to clear. 1=most left, 6= most right.
  Returns  : -
  ------------------------------------------------------------------------*/
void clear_nixie(uint8_t nr)
{
	uint8_t shift;
		
	shift = 24 - 4 * nr;
	nixie_bits |= (NIXIE_CLEAR << shift);
} // clear_nixie()

/*------------------------------------------------------------------------
  Purpose  : To test the Nixie tubes, showing 0 to 9, %, C, P, and RGB Leds 
  Variables: -
  Returns  : -
  ------------------------------------------------------------------------*/
void ftest_nixies(void)
{
	static uint8_t std_test = 0;
	
	PORTC &= ~(HUMIDITYSYMBOL | PRESSURESYMBOL | DEGREESYMBOL | LED_IN19A);
	nixie_bits8 = 0x00;
	
	switch (std_test)
	{
		case 0: nixie_bits = 0x00000000; 
		        dec_point_set(DP_HH_LEFT);
				dec_point_set(DP_HH_RIGHT);
				std_test = 1; 
				break;
		case 1: nixie_bits = 0x00111111; 
		        dec_point_set(DP_HL_LEFT);
		        dec_point_set(DP_HL_RIGHT);
				std_test = 2; 
				break;
		case 2: nixie_bits = 0x00222222; 
		        dec_point_set(DP_MH_LEFT);
		        dec_point_set(DP_MH_RIGHT);
		        PORTC   |= (DEGREESYMBOL | LED_IN19A); 
				std_test = 3; 
				break;
		case 3: nixie_bits = 0x00333333; 
		        dec_point_set(DP_ML_LEFT);
		        dec_point_set(DP_ML_RIGHT);
				std_test = 4; 
				break;
		case 4: nixie_bits = 0x00444444; 
				dec_point_set(DP_SH_LEFT);
				dec_point_set(DP_SH_RIGHT);
				std_test = 5; 
				break;
		case 5: nixie_bits = 0x00555555; 
		        PORTC   |= (HUMIDITYSYMBOL | LED_IN19A); 
		        dec_point_set(DP_SL_LEFT);
		        dec_point_set(DP_SL_RIGHT);
				std_test = 6; 
				break;
		case 6: nixie_bits = 0x00666666; std_test = 7; break;
		case 7: nixie_bits = 0x00777777; std_test = 8; break;
		case 8: nixie_bits = 0x00888888; 
		        PORTC   |= (PRESSURESYMBOL | LED_IN19A); 
				std_test = 9; 
				break;
		case 9: nixie_bits  = 0xFF999999; // LEFT DP5..DP6 + RIGHT DP1..DP6 on
				nixie_bits8 = 0x0F;       // LEFT DP1..DP4 on
				std_test = 0; 
				break;
	} // switch
	set_rgb_colour(std_test);
	ws2812_send_all(); // Send color-bits to WS2812 leds
} // ftest_nixies()

/*-----------------------------------------------------------------------------
  Purpose  : This routine fills the led_<x>[NR_LEDS] array. It uses the global
             variable nixie_bits to enable/disable a specific led, so make sure
			 this is set prior to calling this routine.
  Variables: color: the 24-bit color info
  Returns  : -
  ---------------------------------------------------------------------------*/
void ws2812b_fill_rgb(uint32_t color)
{
	uint8_t i,r,g,b;
	
	r = (uint8_t)((color & RGB_RED)   >> 16);
	g = (uint8_t)((color & RGB_GREEN) >>  8);
	b = (uint8_t)( color & RGB_BLUE);
	for (i = 0; i < NR_LEDS; i++)
	{
		if ((nixie_bits >> (i<<2) & 0x0000000F) == NIXIE_CLEAR)
		{
			led_g[i] = led_r[i] = led_b[i] = 0x00; // disable led
		}
		else
		{
			led_g[i] = g;
			led_r[i] = r;
			led_b[i] = b;
		} // else
	} // for i
} // ws2812b_fill_rgb()

/*------------------------------------------------------------------------
  Purpose  : This functions is called when random_rgb == true.
			 Called every second.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------------*/
void set_rgb_colour(uint8_t color)
{
	switch (color)
	{
		case WHITE:   ws2812b_fill_rgb(RGB_WHITE);   break;
		case RED:     ws2812b_fill_rgb(RGB_RED);     break;
		case GREEN:   ws2812b_fill_rgb(RGB_GREEN);   break;
		case BLUE:    ws2812b_fill_rgb(RGB_BLUE);    break;
		case YELLOW:  ws2812b_fill_rgb(RGB_YELLOW);  break;
		case MAGENTA: ws2812b_fill_rgb(RGB_MAGENTA); break;
		case CYAN:    ws2812b_fill_rgb(RGB_CYAN);    break;
		default:      ws2812b_fill_rgb(RGB_BLACK);   break;	
	} // switch
} // set_rgb_colour

/*------------------------------------------------------------------------
  Purpose  : This function converts hours and minutes to minutes.
             blanking time for the Nixies.
  Variables: p: Time struct containing actual time
  Returns  : true: blanking is true, false: no blanking
  ------------------------------------------------------------------------*/
uint16_t cmin(uint8_t h, uint8_t m)
{
	uint16_t x1 = (uint16_t)h << 2; // 4*h
	uint16_t x2;
	
	x2  = x1 << 1; // x2 = 8*h
	x1 += x2;      // x1 = 12*h
	x2 <<= 1;      // x2 = 16*h
	x1 += x2;      // x1 = 28*h
	x2 <<= 1;      // x2 = 32*h
	x1 += x2;      // x1 = 60*h
	return x1 + m; // 60*h + m
} // cmin()

/*------------------------------------------------------------------------
  Purpose  : This function decides if the current time falls between the
             blanking time for the Nixies.
  Variables: p: Time struct containing actual time
  Returns  : true: blanking is true, false: no blanking
  ------------------------------------------------------------------------*/
bool blanking_active(Time p)
{
	uint16_t x = cmin(p.hour       , p.min);
	uint16_t b = cmin(blank_begin_h, blank_begin_m);
	uint16_t e = cmin(blank_end_h  , blank_end_m);
	
	// (b>=e): Example: 23:30 and 05:30, active if x>=b OR  x<=e
	// (b< e): Example: 02:30 and 05:30, active if x>=b AND x<=e
	bool blanking = ((b >= e) && ((x >= b) || (x <= e))) || ((x >= b) && (x < e));
	if (blanking_invert) blanking = !blanking;
	return blanking;
} // blanking_active()

/*------------------------------------------------------------------------
  Purpose  : This function sets one decimal point of the six Nixies. Every
             Nixie can have a left and a right decimal-point. This function
			 uses the global variables nixie_bits (32 bit) and nixie_bits8 (8 bit).
  Variables: dp: a decimal-point constant indicating which one to set.
  Returns  : -
  ------------------------------------------------------------------------*/
void dec_point_set(uint8_t dp)
{
	switch (dp)
	{
		case DP_HH_LEFT:  // Nixie 1, most-left one
			nixie_bits8 |= LEFT_DP1;
			break;
		case DP_HH_RIGHT: // Nixie 1, most-left one
			nixie_bits  |= RIGHT_DP1;
			break;
		case DP_HL_LEFT:  // Nixie 2
			nixie_bits8 |= LEFT_DP2;
			break;
		case DP_HL_RIGHT: // Nixie 2
			nixie_bits  |= RIGHT_DP2;
			break;
		case DP_MH_LEFT:  // Nixie 3
			nixie_bits8 |= LEFT_DP3;
			break;
		case DP_MH_RIGHT: // Nixie 3
			nixie_bits  |= RIGHT_DP3;
			break;
		case DP_ML_LEFT:  // Nixie 4
			nixie_bits8 |= LEFT_DP4;
			break;
		case DP_ML_RIGHT: // Nixie 4
			nixie_bits  |= RIGHT_DP4;
			break;
		case DP_SH_LEFT:  // Nixie 5
			nixie_bits  |= LEFT_DP5;
			break;
		case DP_SH_RIGHT: // Nixie 5
			nixie_bits  |= RIGHT_DP5;
			break;
		case DP_SL_LEFT:  // Nixie 6, most-right one
			nixie_bits  |= LEFT_DP6;
			break;
		case DP_SL_RIGHT: // Nixie 6, most-right one
			nixie_bits  |= RIGHT_DP6;
			break;
	} // switch
} // dec_point_set()

/*------------------------------------------------------------------------
  Purpose  : This function sets one decimal point of the six Nixies. Every
             Nixie can have a left and a right decimal-point. This function
			 uses the global variables nixie_bits (32 bit) and nixie_bits8 (8 bit).
  Variables: dp: a decimal-point constant indicating which one to set.
  Returns  : -
  ------------------------------------------------------------------------*/
void dec_point_clr(uint8_t dp)
{
	switch (dp)
	{
		case DP_HH_LEFT:  // Nixie 1, most-left one
			nixie_bits8 &= ~LEFT_DP1;
			break;
		case DP_HH_RIGHT: // Nixie 1, most-left one
			nixie_bits  &= ~RIGHT_DP1;
			break;
		case DP_HL_LEFT:  // Nixie 2
			nixie_bits8 &= ~LEFT_DP2;
			break;
		case DP_HL_RIGHT: // Nixie 2
			nixie_bits  &= ~RIGHT_DP2;
			break;
		case DP_MH_LEFT:  // Nixie 3
			nixie_bits8 &= ~LEFT_DP3;
			break;
		case DP_MH_RIGHT: // Nixie 3
			nixie_bits  &= ~RIGHT_DP3;
			break;
		case DP_ML_LEFT:  // Nixie 4
			nixie_bits8 &= ~LEFT_DP4;
			break;
		case DP_ML_RIGHT: // Nixie 4
			nixie_bits  &= ~RIGHT_DP4;
			break;
		case DP_SH_LEFT:  // Nixie 5
			nixie_bits  &= ~LEFT_DP5;
			break;
		case DP_SH_RIGHT: // Nixie 5
			nixie_bits  &= ~RIGHT_DP5;
			break;
		case DP_SL_LEFT:  // Nixie 6, most-right one
			nixie_bits  &= ~LEFT_DP6;
			break;
		case DP_SL_RIGHT: // Nixie 6, most-right one
			nixie_bits  &= ~RIGHT_DP6;
			break;
	} // switch
} // dec_point_clr()

/*------------------------------------------------------------------------
  Purpose  : This function updates the DS3231 with the time from the
             ESP8266 NTP Server. It does this every 12 hours.
			 This function is called every second from display_task().
  Variables: -
  Returns  : -
  ------------------------------------------------------------------------*/
void update_esp8266_time(void)
{
    static uint8_t retry_tmr;
    static uint8_t retries = 0;
    
    switch (esp8266_std)
    {
	    case ESP8266_INIT:
			if (++esp8266_tmr >= ESP8266_SECONDS) // 12 hours * 60 min. * 60 sec.
			{
				last_esp8266 = false; // reset status
				retries      = 0;     // init. number of retries
				esp8266_std  = ESP8266_UPDATE;
			} // if
	    break;
		
	    case ESP8266_UPDATE:
			retry_tmr    = 0; // init. retry timer
			xputs("e0\n");    // update time from ESP8266
			esp8266_std  = ESP8266_RETRY;
	    break;
		
	    case ESP8266_RETRY:
			if (last_esp8266 == true)
			{   // valid e0 response back, set by command_interpreter()
				esp8266_std  = ESP8266_INIT;
			} // if
			else if (++retries > 5)
			{   // No response from esp8266 after 5 retries, stop trying
				esp8266_std = ESP8266_INIT;
				esp8266_tmr = 0; // reset timer here and try again in 12 hours
			} // else if
			else if (++retry_tmr >= 60)
			{   // retry 1 minute later
				retries++;
				esp8266_std = ESP8266_UPDATE; // retry once more
			} // else if
	    break;
		
	    default:
			esp8266_tmr = 0;
			esp8266_std = ESP8266_INIT;
	    break;
    } // switch
} // update_esp8266_time()

/*------------------------------------------------------------------------
  Purpose  : This task decides what to display on the Nixie Tubes.
			 Called once every second.
  Variables: nixie_bits (32 bits)
  Returns  : -
  ------------------------------------------------------------------------*/
void display_task(void)
{
	Time     p; // Time struct
	uint8_t  c,x,y;
	uint16_t r;
	static   uint8_t col_white_tmr = 0;
    static   bool    blink     = false;
		
	nixie_bits  = 0x00000000; // clear all bits
	nixie_bits8 = 0x00;       // clear upper 8 bits
	ds3231_gettime(&p);       // get time from DS32231 RTC 
	update_esp8266_time();    // try to get a time from ESP8266 every 12 hours
	display_time = false;     // start with no-time on display
	if (set_col_white)
	{   // show white color for 2 seconds
		rgb_pattern   = SET_WIT;
		if (++col_white_tmr > 2)
		{
			col_white_tmr = 0;
			set_col_white = false;
		} // if
	} // else
	else rgb_pattern = FIXED;

	switch (rgb_pattern)
	{
		case RANDOM:  c = rand() % 8;       break;
		case DYNAMIC: c = p.sec % 8;        break;
		case FIXED:   c = fixed_rgb_colour; break;
		case SET_WIT: c = WHITE;            break;
		case OFF:
		default:      c = BLACK;            break;   
	} // switch
	set_rgb_colour(c);
	
	if (test_nixies) 
	{   // S3 command or IR 7 command (60 sec.)
		ftest_nixies(); 
		return;
	} // if	
	else if (hv_relay_sw)
	{   // V0 or V1 command
		if (hv_relay_fx)
		     PORTB |=  HV_ON; // relay on
		else PORTB &= ~HV_ON; // relay off
	} // else if
	else if (blanking_active(p))
	{
		PORTB &= ~HV_ON; // relay off
	} // else if
	else 
	{	// blanking is not active
		PORTB |=  HV_ON; // relay on
	} // else	
	
	if (show_date_IR == IR_SHOW_VER)
	{	// Show version info
		nixie_bits = time_arr[POS0];
		nixie_bits <<= 4;
		nixie_bits |= time_arr[POS1];
		nixie_bits <<= 4;	
		nixie_bits |= time_arr[POS2];
		nixie_bits <<= 4;
		clear_nixie(1);
		clear_nixie(2);
		clear_nixie(6);
		dec_point_set(DP_SH_LEFT); // set decimal point
	} // if
	else if (show_date_IR == IR_SHOW_ESP_STAT)
	{	// Show response from ESP8266 NTP Server
		nixie_bits = time_arr[POS0];
		nixie_bits <<= 4;
		for (uint8_t i = POS1; i <= POS5; i++)
		{
			nixie_bits |= time_arr[i];
			nixie_bits <<= 4;
		} // for i	
		if (time_arr[POS5] == 1)
		     set_rgb_colour(GREEN); // last response ok
		else set_rgb_colour(RED);   // last response not ok
		clear_nixie(6);
	} // else if
	else if (set_time_IR != IR_NO_TIME)
	{	// Set blanking begin/end time
		nixie_bits = time_arr[POS0];
		nixie_bits <<= 4;
		for (uint8_t i = POS1; i <= POS3; i++)
		{
			nixie_bits |= time_arr[i];
			nixie_bits <<= 4;
		} // for i
		if (blink)
		{   // blinking color
			switch (time_arr_idx)
			{
				case POS0: nixie_bits |= 0x000F0000; break;
				case POS1: nixie_bits |= 0x0000F000; break;
				case POS2: nixie_bits |= 0x00000F00; break;
				default  : nixie_bits |= 0x000000F0; break;
			} // switch
		} // if
		dec_point_set(DP_ML_LEFT); // set decimal point
		clear_nixie(1);
		clear_nixie(6);
        blink = !blink;   // toggle blinking
	} // else if
	else switch (p.sec)
	{ 
		case 15: // display date & month
			nixie_bits = encode_to_bcd(p.date); // pos. 1 and 2
			nixie_bits <<= 12;
			nixie_bits |= encode_to_bcd(p.mon); // pos. 4 and 5
			nixie_bits <<= 4;
			clear_nixie(3);
			clear_nixie(6);
			
			if (rgb_pattern == FIXED)
			{
				set_rgb_colour(GREEN);
			} // if			
			break;
		
		case 16: // display year
			PORTC &= ~(HUMIDITYSYMBOL | PRESSURESYMBOL | DEGREESYMBOL | LED_IN19A);
			nixie_bits = encode_to_bcd(p.year / 100);
			nixie_bits <<= 8;
			nixie_bits |= encode_to_bcd(p.year % 100);
			nixie_bits <<= 4;
			clear_nixie(1);
			clear_nixie(6);
			
			if (rgb_pattern == FIXED)
			{
				set_rgb_colour(GREEN);
			} // if
			break;
		
		case 30: // display humidity
		case 31:
			r = bme280_hum + 50; // 50 is for round-off to next decimal
			x = (uint8_t)(r / 1000); // 46333: 46333 + 50 E-3 % = 46.383 %
			nixie_bits = encode_to_bcd(x);
			nixie_bits <<= 4;
			r = (uint16_t)(r - 1000 * x); // 383
			y = (uint8_t)(r / 100);       // 3
			nixie_bits |= y;
			dec_point_set(DP_SL_LEFT); // set decimal point
			
			PORTC &= ~(DEGREESYMBOL | PRESSURESYMBOL);
			PORTC |= HUMIDITYSYMBOL;
			PORTC |= LED_IN19A;
			
			clear_nixie(1);
			clear_nixie(2);
			clear_nixie(3);
			if (bme280_hum < 10000)	// Blank when hum < 10.000 %
			{
				clear_nixie(4);
			} // if

			if (rgb_pattern == FIXED)
			{
				set_rgb_colour(BLUE);
			} // if
			break;
		
		case 40: // display temperature
		case 41:
			r = bme280_temp + 5;    // 5 is for round-off to next decimal
			x = (uint8_t)(r / 100); // 3296 = 32.96 °C, display 33.0 
			nixie_bits   = encode_to_bcd(x);
			nixie_bits <<= 4;
			r = (uint16_t)(r - 100 * x); // 01
			y = (uint8_t)(r / 10);       // 0
			nixie_bits |= y;
			dec_point_set(DP_SL_LEFT); // set decimal point
			
			PORTC &= ~(HUMIDITYSYMBOL | PRESSURESYMBOL);
			PORTC |= DEGREESYMBOL;
			PORTC |= LED_IN19A;
			
			clear_nixie(1);
			clear_nixie(2);
			clear_nixie(3);
			if (bme280_temp < 1000)	// Blank when temp < 10.00 degrees
			{
				clear_nixie(4);
			} // if
			
			if (rgb_pattern == FIXED)
			{
				set_rgb_colour(RED);
			} // if
			break;

		case 50: // display Pressure in mbar
		case 51:
			x = (uint8_t)(bme280_press / 1000); // 10197 = 1019.7 mbar
			nixie_bits = encode_to_bcd(x);
			nixie_bits <<= 8;
			r = (uint16_t)(bme280_press - 1000 * x); // 197
			y = (uint8_t)(r / 10);                   // 19
			nixie_bits |= encode_to_bcd(y);
			nixie_bits <<= 4;
			r -= 10 * y;                             // 7
			nixie_bits |= (uint8_t)r;
			dec_point_set(DP_SL_LEFT); // set decimal point

			PORTC &= ~(HUMIDITYSYMBOL | DEGREESYMBOL);
			PORTC |= PRESSURESYMBOL;
			PORTC |= LED_IN19A;
			
			clear_nixie(1);
			if (bme280_press < 10000)	// Blank zero when pressure < 1000,0 mBar
			{
				clear_nixie(2);
			} // if
				
			if (rgb_pattern == FIXED)
			{
				set_rgb_colour(YELLOW);
			} // if			
			break;
	
		default: // display normal time
			display_time = true;
			check_and_set_summertime(p); // check for Summer/Wintertime change
			nixie_bits = encode_to_bcd(p.hour);
			nixie_bits <<= 8; // SHL 8
			nixie_bits |= encode_to_bcd(p.min);
			nixie_bits <<= 8; // SHL 8
			nixie_bits |= encode_to_bcd(p.sec);
			PORTC &= ~(HUMIDITYSYMBOL | PRESSURESYMBOL | DEGREESYMBOL | LED_IN19A);
			if (p.sec & 0x01) dec_point_set(DP_SH_LEFT);
			else              dec_point_set(DP_ML_RIGHT);
			dec_point_set(DP_MH_LEFT); // set hrs-min. separator
			if (dst_active)   dec_point_set(DP_SL_RIGHT);
			else              dec_point_clr(DP_SL_RIGHT);
			break;
	} // switch
	relay_status = ((PINB & HV_ON) == HV_ON); // On or Off
	if (!relay_status)
	{	// Is HV-relay off?
		set_rgb_colour(BLACK); // disable LEDs if so
	} // if	
	ws2812_send_all(); // Send color-bits to WS2812 leds
} // display_task()

/*------------------------------------------------------------------------
Purpose  : This function initializes Timer 1 for a 62.5 kHz (16 usec.)
           timer, which is needed for time-measurement of the IR library.
Variables: -
Returns  : -
------------------------------------------------------------------------*/
void init_timer1(void)
{
	// Set Timer 1 to 62.5 kHz interrupt frequency
	TCCR1A = 0x00;  // normal mode
	TCCR1B = 0x04;  // set pre-scaler to 256 (fclk = 62.5 kHz)
	TCNT1  =  0;    // start counting at 0
} // init_timer1()

/*------------------------------------------------------------------
  Purpose  : This function reads the value of TMR1 which runs at 62.5 kHz
  Variables: -
  Returns  : the value from TMR1
  ------------------------------------------------------------------*/
uint16_t tmr1_val(void)
{
	return TCNT1;
} // tmr1_val()

/*------------------------------------------------------------------------
Purpose  : This function initializes Timer 2 for 1 kHz (1 msec.)
           signal, needed for the scheduler interrupt.
Variables: -
Returns  : -
------------------------------------------------------------------------*/
void init_timer2(void)
{
	// Set Timer 2 to 1 kHz interrupt frequency: ISR(TIMER2_COMPA_vect)
	TCCR2A |= (0x01 << WGM21); // CTC mode, clear counter on TCNT2 == OCR2A
	TCCR2B =  (0x01 << CS22) | (0x01 << CS20); // set pre-scaler to 128
	OCR2A  =  124;   // this should set interrupt frequency to 1000 Hz
	TCNT2  =  0;     // start counting at 0
	TIMSK2 |= (0x01 << OCIE2A);   // Set interrupt on Compare Match
} // init_timer2()

/*------------------------------------------------------------------------
Purpose  : This function initializes PORTB, PORTC and PORTD pins.
           See Nixie.h header for a detailed overview of all port-pins.
Variables: -
Returns  : -
------------------------------------------------------------------------*/
void init_ports(void)
{
	DDRB  &= ~(IR_RCV); // clear bits = input
	PORTB |=  (IR_RCV); // enable pull-up resistor
	DDRB  |=  (TIME_MEAS | WS2812_DI | HV_ON | IR_LED); // set bits = output
	PORTB &= ~(TIME_MEAS | WS2812_DI | HV_ON | IR_LED); // init. outputs to 0

	PCICR |= (1<<PCIE0);   // Enable pin change interrupt 0
	PCMSK0 |= (1<<PCINT3); // Enable PCINT3 (PB3) for IR_RCV

	DDRC  |=  (0x0F);   // set bits C3 t/m C0 as output
	PORTC &= ~(HUMIDITYSYMBOL | PRESSURESYMBOL | DEGREESYMBOL | LED_IN19A);

	DDRD  |=  (SDIN | SHCP | STCP); // set bits = output
	PORTD &= ~(SDIN | SHCP | STCP); // set outputs to 0
} // init_ports()

int freeRam (void)
{
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
} // freeRam()

/*------------------------------------------------------------------------
Purpose  : main() function: Program entry-point, contains all init. functions 
		   and calls the task-scheduler in a loop.
Variables: -
Returns  : -
------------------------------------------------------------------------*/
int main(void)
{
	char s[20];
	
	init_timer1(); // init. timer for IR-measurements 
	init_timer2(); // init. timer for scheduler
	i2c_init(SCL_CLK_400KHZ); // Init. I2C HW with 400 kHz clock
	init_ports();  // init. PORTB, PORTC and PORTD port-pins 	
	srand(59);	   // Initialize random generator from 0 - 59
	
	// Initialize Serial Communication, See usart.h for BAUD
	// F_CPU should be a Project Define (-DF_CPU=16000000UL)
	usart_init(MYUBRR); // Initializes the serial communication
	bme280_init();      // Init. the BME280 sensor
	
	// Add tasks for task-scheduler here
	add_task(display_task ,"Display",  0, 1000); // What to display on the Nixies.
	add_task(update_nixies,"Update" ,100,   50); // Run Nixie Update every  50 msec.
	add_task(ir_task      ,"IR_recv",150,  100); // Run IR-process   every 100 msec.
	add_task(bme280_task  ,"BME280" ,250, 5000); // Run BMP180 sensor process every 5 seconds
	
	sei(); // set global interrupt enable, start task-scheduler
	check_and_init_eeprom();  // Init. EEPROM
	read_eeprom_parameters();
	dst_active = eeprom_read_byte(EEPARB_DST); // read from EEPROM
	xputs(nixie_ver);
	xputs("Blanking from ");
	sprintf(s,"%02d:%02d to %02d:%02d\n",blank_begin_h,blank_begin_m,blank_end_h,blank_end_m);
	xputs(s);
	sprintf(s,"Free RAM:%d bytes\n",freeRam());
	xputs(s);                    // print amount of free RAM

	// Main routine 
	while(1)
	{   // Run all scheduled tasks here
		dispatch_tasks(); // Run Task-Scheduler
		switch (rs232_command_handler()) // run command handler continuously
		{
			case ERR_CMD: 
			     xputs("Command Error\n");
				 break;
			case ERR_NUM: 
			     sprintf(s,"Number Error (%s)\n",rs232_inbuf);
				 xputs(s);
				 break;
			default: 
			     break;
		} // switch
	} // while
} // main()