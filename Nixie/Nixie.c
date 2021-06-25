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
#include "dht22.h"
#include "bmp180.h"
#include "eep.h"

char          nixie_ver[] = "Nixie Old HW v0.30\n";
bool		  test_nixies = false;	    // S3 command / IR #9 command

uint8_t       cnt_50usec  = 0;       // 50 usec. counter
unsigned long t2_millis   = 0UL;     // msec. counter
uint16_t      dht22_hum   = 0;       // Humidity E-1 %
int16_t       dht22_temp  = 0;       // Temperature E-1 Celsius
int16_t       dht22_dewp  = 0;       // Dewpoint E-1 Celsius
double        bmp180_pressure = 0.0; // Pressure E-1 mbar
double        bmp180_temp     = 0.0; // Temperature E-1 Celsius

bool          dst_active      = false;		// true = Daylight Saving Time active
uint8_t       blank_begin_h   = 0;
uint8_t       blank_begin_m   = 0;
uint8_t       blank_end_h     = 0;
uint8_t       blank_end_m     = 0;

uint8_t rgb_pattern = FIXED;		  // RGB color mode: [OFF, RANDOM, DYNAMIC, FIXED]
uint8_t fixed_rgb_colour = CYAN;	  // RGB colour variable used in Nixie.c

uint8_t       wheel_effect    = 1;    // 0: none, 1: from 59->00, 2: every second/minute
bool          display_time    = true; // true = hh:mm:ss is on display 
uint8_t		  col_time;				  // Colour for Time display
uint8_t		  col_date;				  // Colour for Date & Year display
uint8_t       col_temp; 			  // Colour for Temperature display
uint8_t       col_humi; 			  // Colour for Humidity display
uint8_t       col_dewp; 			  // Colour for Dew-point display
uint8_t       col_pres; 			  // Colour for Pressure display
uint8_t       col_roll; 			  // Colour for second roll-over

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
// Bits 31..24: Decimal points: LDP5 LDP3 RDP6 RDP5 RDP4 RDP3 RDP2 RDP1
// Bits 23..16: Hours         : HHD  HHC  HHB  HHA  HLD  HLC  HLB  HLA 
// Bits 15..08: Minutes       : MHD  MHC  MHB  MHA  MLD  MLC  MLB  MLA
// Bits 07..00: Seconds       : SHD  SHC  SHB  SHA  SLD  SLC  SLB  SLA
//---------------------------------------------------------------------------
uint32_t nixie_bits = 0UL;
uint8_t  rgb_colour = BLACK;

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
	PORTB |= TIME_MEAS; // Time Measurement
	scheduler_isr(); // call the ISR routine for the task-scheduler
	t2_millis++;     // update millisecond counter
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
    //if (ir_rcvb) // copy IR-signal to debug output
    //     PORTB |=  IR_LED;
    //else PORTB &= ~IR_LED;
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
			case 0: if ((p.date == lsun03) && (p.hour == 2) && (p.min == 0))
					{   // At 2:00 AM advance time to 3 AM, check for one minute
						advance_time = 1;
					} // if
					else if (p.date < lsun03) dst_active = false;
					else if (p.date > lsun03) dst_active = true;
					else if (p.hour < 2)      dst_active = false;
					break;
			case 1: // Now advance time, do this only once
					ds3231_settime(3,0,p.sec); // Set time to 3:00, leave secs the same
					advance_time = 2;
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
			case 0: if ((p.date == lsun10) && (p.hour == 3) && (p.min == 0))
					{   // At 3:00 AM revert time back to 2 AM, check for one minute
						revert_time = 1;
					} // if
					else if (p.date > lsun10) dst_active = false;
					else if (p.date < lsun10) dst_active = true;
					else if (p.hour < 3)      dst_active = true;
					break;
			case 1: // Now revert time, do this only once
					ds3231_settime(2,0,p.sec); // Set time back to 2:00, leave secs the same
					revert_time = 2;
					dst_active  = false;
					break;
			case 2: // make sure that we passed 3 AM in order to prevent multiple reverts
					if (p.hour > 3) revert_time = 0; // at 4:00:00 back to normal
					dst_active = false;
					break;
		} // switch
	} // else if
	else if ((p.mon < 3) || (p.mon > 10)) dst_active = false;
	else                                  dst_active = true; 

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
             It reads the DHT22 sensor Humidity and Temperature
  Variables: dht22_humidity, dht22_temperature
  Returns  : -
  ------------------------------------------------------------------------*/
void dht22_task(void)
{
	int8_t x;
	// read DHT22 sensor
	x = dht22_read(&dht22_hum,&dht22_temp);
	if (x == DHTLIB_OK) 
	{
		dht22_dewp = dht22_dewpoint(dht22_hum,dht22_temp);
#ifdef DEBUG_SENSORS
		char s[30];
		sprintf(s,"dht22: RH=%d E-1 %%, ",dht22_hum); 
		xputs(s);
		sprintf(s," T=%d E-1 °C,",dht22_temp);
		xputs(s);
		sprintf(s," dewpoint=%d E-1 °C\n",dht22_dewp);
		xputs(s);
#endif
	}
	else xputs("dht22 checksum error\n");	
} // dht22_task()

/*------------------------------------------------------------------------
  Purpose  : This task is called by the Task-Scheduler every second.
             It reads the BMP180 pressure and temperature.
  Variables: bmp180_pressure, bmp180_temperature
  Returns  : -
  ------------------------------------------------------------------------*/
void bmp180_task(void)
{
	static uint8_t std180 = S180_START_T;
	//char   s[30];
	bool   err;
	
	switch (std180)
	{
		case S180_START_T:
			err = bmp180_start_temperature();
			if (err) xputs("bmp180: start_T err\n");
			else std180 = S180_GET_T;
			break;

		case S180_GET_T:
			err = bmp180_get_temperature(&bmp180_temp);
			if (err) std180 = S180_START_T;
			else     std180 = S180_START_P;
#ifdef DEBUG_SENSORS
			xputs("bmp180: ");
			if (err) xputs("Terr\n");
			else
			{
				sprintf(s,"%4.1f °C\n",bmp180_temp);
				xputs(s);
			} // else
#endif
			break;
			
		case S180_START_P:
			err = bmp180_start_pressure(BMP180_COMMAND_PRESSURE3);
			if (err) 
			{
				 xputs("start_P err\n");
				 std180 = S180_START_T;
			} // if
			else std180 = S180_GET_P;
			break;

		case S180_GET_P:
			err = bmp180_get_pressure(&bmp180_pressure,bmp180_temp);
			std180 = S180_START_T;
#ifdef DEBUG_SENSORS
			xputs("bmp180: ");
			if (err) xputs("Perr\n");
			else
			{
				sprintf(s,"%4.1f mbar\n",bmp180_pressure);
				xputs(s);
			} // else
#endif
			break;
	} // switch
} // bmp180_task()

/*------------------------------------------------------------------------
  Purpose  : This task is called by the Task-Scheduler every 50 msec. 
             It updates the Nixie tubes with the new bits.
  Variables: nixie_bits (32 bits)
  Returns  : -
  ------------------------------------------------------------------------*/
void update_nixies(void)
{
	uint8_t         i, h, m, s;
	uint32_t        mask = 0x80000000; // start with MSB
	uint32_t        bitstream;         // copy of nixie_bits
	static uint8_t  wheel_cntr = 0;
	static uint8_t  bits_min_old, bits_sec_old, bits_hrs_old;
	uint8_t         wheel[WH_MAX] = {11,22,33,44,55,66,77,88,99,0};
			
	PORTB &= ~(RGB_R | RGB_G | RGB_B);               // clear  LED colours
	PORTB |= (rgb_colour & (RGB_R | RGB_G | RGB_B)); // update LED colours
	
	bitstream = nixie_bits; // copy original bitstream
	if (display_time)
	{
		//--------------------------------------------------------
		// WHEEL-EFFECT: Rotate digits + ANTI-POISONING Function
		//--------------------------------------------------------
		h = (uint8_t)((nixie_bits & 0x00FF0000) >> 16); // isolate hours   digits
		m = (uint8_t)((nixie_bits & 0x0000FF00) >>  8); // isolate minutes digits
		s = (uint8_t)(nixie_bits  & 0x000000FF);        // isolate seconds digits
		switch (wheel_effect)
		{
			case 0: // no wheel-effect
					if ((s == 0x00) && ((m == 0x00) || (m == 0x30)))
					{	// Anti-poisoning effect at every half- and full-hour
						if (wheel_cntr < WH_MAX-1)
						{
							bitstream |= ((uint32_t)wheel[wheel_cntr]);         // seconds
							bitstream &= 0xFF0000FF; // clear minutes and hours bits
							bitstream |= (((uint32_t)wheel[wheel_cntr]) << 8);  // minutes
							bitstream |= (((uint32_t)wheel[wheel_cntr]) << 16); // hours
						} // if
						if (++wheel_cntr > WH_MAX-1) wheel_cntr = WH_MAX-1;
					} // if
					else wheel_cntr = 0; // reset for next second
					break;
			case 1: // wheel-effect only from 59 -> 0 (minutes & seconds) + anti-poisoning effect
					if (s == 0x00)
					{	// seconds == 0, wheel-effect
						bitstream |= ((uint32_t)wheel[wheel_cntr]); // seconds
						if ((m == 0x00) || (m == 0x30))
						{	// // Anti-poisoning effect at every half- and full-hour
							if (wheel_cntr < WH_MAX-1)
							{
								bitstream &= 0xFF0000FF; // clear minutes and hours bits
								bitstream |= (((uint32_t)wheel[wheel_cntr]) <<  8); // minutes
								bitstream |= (((uint32_t)wheel[wheel_cntr]) << 16); // hours
							} // if							
						} // if
						if (++wheel_cntr > WH_MAX-1) wheel_cntr = WH_MAX-1;
					} // if
					else wheel_cntr = 0; // reset for next second
					break;
			case 2: // wheel-effect on every change in hours / minutes / seconds
					if (s != bits_sec_old)
					{	// change in seconds
						bitstream &= 0xFFFFFF00; // clear seconds bits
						bitstream |= ((uint32_t)wheel[wheel_cntr]); // seconds
						if (m != bits_min_old)
						{	// change in minutes
							bitstream   &= 0xFFFF00FF; // clear minutes bits
							bitstream   |= (((uint32_t)wheel[wheel_cntr]) << 8); // minutes
							if ((h != bits_hrs_old) || (m == 0x30))
							{	// change in hours or anti-poisoning effect
								bitstream   &= 0xFF00FFFF; // clear hours bits
								bitstream   |= (((uint32_t)wheel[wheel_cntr]) << 16); // hours
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
	//------------------------------------------
	// Now send the nixie bitstream to hardware
	//------------------------------------------
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
	if ((nr >= 1) && (nr <= 6))
	{
		shift = 24 - 4 * nr;
		nixie_bits |= (NIXIE_CLEAR << shift);
	} // if
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
	switch (std_test)
	{
		case 0: nixie_bits = 0x01000000; std_test = 1; break;
		case 1: nixie_bits = 0x02111111; std_test = 2; break;
		case 2: nixie_bits = 0x40222222; PORTC |= DEGREESYMBOL | LED_IN19A; std_test = 3; break;
		case 3: nixie_bits = 0x04333333; std_test = 4; break;
		case 4: nixie_bits = 0x08444444; std_test = 5; break;
		case 5: nixie_bits = 0x80555555; PORTC |= (HUMIDITYSYMBOL | LED_IN19A); std_test = 6; break;
		case 6: nixie_bits = 0x10666666; std_test = 7; break;
		case 7: nixie_bits = 0x20777777; std_test = 8; break;
		case 8: nixie_bits = 0xFF888888; PORTC |= (PRESSURESYMBOL | LED_IN19A); std_test = 9; break;
		case 9: nixie_bits = 0xFF999999; std_test = 0; test_nixies = false; break;
	} // switch
	rgb_colour = std_test & 0x07;
} // ftest_nixies()

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
	uint8_t  c,x;
	static uint8_t col_white_tmr = 0;
	static bool blink = false;
	
	nixie_bits = 0x00000000; // clear all bits
	ds3231_gettime(&p);      // get time from DS3231 RTC
	update_esp8266_time();   // try to get a time from ESP8266 every 12 hours
    display_time = false;    // start with no-time on display
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
	rgb_colour = c;
	
	if (test_nixies) 
	{   // S3 command or IR 7 command (60 sec.)
		ftest_nixies(); 
		return;
	} // if	
	else if (blanking_active(p))
	{
	     nixie_bits = NIXIE_CLEAR_ALL;
		 PORTC &= ~(HUMIDITYSYMBOL | PRESSURESYMBOL | DEGREESYMBOL | LED_IN19A);
		 rgb_colour = BLACK;
	} // else if
	
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
		nixie_bits |= RIGHT_DP3;   // TODO
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
		     rgb_colour = GREEN; // last response ok
		else rgb_colour = RED;   // last response not ok
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
		nixie_bits |= RIGHT_DP3;   // TODO
		clear_nixie(1);
		clear_nixie(6);
        blink = !blink;   // toggle blinking
	} // else if
	else switch (p.sec)
	{
		case 15: // display date & month
			PORTC &= ~(HUMIDITYSYMBOL | PRESSURESYMBOL | DEGREESYMBOL | LED_IN19A);
			nixie_bits = encode_to_bcd(p.date);
			nixie_bits <<= 12;
			nixie_bits |= encode_to_bcd(p.mon);
			nixie_bits <<= 4;
			clear_nixie(3);
			clear_nixie(6);
			if (rgb_pattern == FIXED)
			{
				rgb_colour = GREEN;
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
				rgb_colour = GREEN;
			} // if
			break;

		case 40: // display temperature
			x = (uint8_t)bmp180_temp;
			nixie_bits = encode_to_bcd(x);
			nixie_bits <<= 4;
			nixie_bits |= (uint8_t)(10.0 * (bmp180_temp - x));
			nixie_bits |= RIGHT_DP5;
			clear_nixie(1);
			clear_nixie(2);
			clear_nixie(3);
			PORTC &= ~(HUMIDITYSYMBOL | PRESSURESYMBOL);
			PORTC |=  DEGREESYMBOL | LED_IN19A;
			if (rgb_pattern == FIXED)
			{
				rgb_colour = RED;
			} // if
			break;

		case 50: // display Pressure in mbar
			x = (uint8_t)(bmp180_pressure / 100.0);
			nixie_bits = encode_to_bcd(x);
			nixie_bits <<= 8;
			x = (uint8_t)(bmp180_pressure - (int16_t)x * 100);
			nixie_bits |= encode_to_bcd(x);
			nixie_bits <<= 4;
			x = (uint8_t)(10 * (bmp180_pressure - (int16_t)bmp180_pressure));
			nixie_bits |= x;
			nixie_bits |= RIGHT_DP5;
			clear_nixie(1);
			PORTC &= ~(HUMIDITYSYMBOL | DEGREESYMBOL);
			PORTC |=  PRESSURESYMBOL | LED_IN19A;
			if (rgb_pattern == FIXED)
			{
				rgb_colour = YELLOW;
			} // if
			break;

		case 0: // set colour during roll-over
			if ((wheel_effect > 0) && (rgb_pattern == FIXED))
			{
				rgb_colour = col_roll;
			}
			// FALL-THROUGH, no break here

		default: // display normal time
		    display_time = true;
			check_and_set_summertime(p); // check for Summer/Wintertime change
			nixie_bits = encode_to_bcd(p.hour);
			nixie_bits <<= 8; // SHL 8
			nixie_bits |= encode_to_bcd(p.min);
			nixie_bits <<= 8; // SHL 8
			nixie_bits |= encode_to_bcd(p.sec);
			PORTC &= ~(HUMIDITYSYMBOL | PRESSURESYMBOL | DEGREESYMBOL | LED_IN19A);
			if (p.sec & 0x01) nixie_bits |=  RIGHT_DP4;
			else              nixie_bits |=  LEFT_DP5;
			if (p.min & 0x01) nixie_bits |=  RIGHT_DP2;
			else              nixie_bits |=  LEFT_DP3;
			if (dst_active)   nixie_bits |=  RIGHT_DP6;
			else              nixie_bits &= ~RIGHT_DP6;
			break;
	} // else switch
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
Purpose  : This function initializes Timer 2 for a 1 kHz (1 msec.)
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
	DDRB  &= ~(DHT22 | IR_RCV); // clear bits = input
	PORTB &= ~(DHT22 | IR_RCV); // disable pull-up resistors
	DDRB  |=  (TIME_MEAS | RGB_R | RGB_G | RGB_B); // set bits = output
	PORTB &= ~(TIME_MEAS | RGB_R | RGB_G | RGB_B); // set outputs to 0
	
	PCICR |= (1<<PCIE0);   // Enable pin change interrupt 0
	PCMSK0 |= (1<<PCINT3); // Enable PCINT3 (PB3) for IR_RCV

	DDRC  |=  (HUMIDITYSYMBOL | PRESSURESYMBOL | DEGREESYMBOL | LED_IN19A); // set bits C3..C0 as outputs
	PORTC &= ~(HUMIDITYSYMBOL | PRESSURESYMBOL | DEGREESYMBOL | LED_IN19A); // set outputs to 0
	
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
	srand(59);     // Initialize random generator from 0 - 59
	
	// Initialize Serial Communication, See usart.h for BAUD
	// F_CPU should be a Project Define (-DF_CPU=16000000UL)
	usart_init(MYUBRR); // Initializes the serial communication
	bmp180_init();      // Init. the BMP180 sensor
	
	// Add tasks for task-scheduler here
	add_task(display_task ,"Display",  0, 1000); // What to display on the Nixies.
	add_task(update_nixies,"Update" ,100,   50); // Run Nixie Update every  50 msec.
	add_task(ir_task      ,"IR_recv",150,  100); // Run IR-process   every 100 msec.
  //add_task(dht22_task   ,"DHT22"  ,250, 5000); // Run DHT22 sensor process every 5 sec.
	add_task(bmp180_task  ,"BMP180" ,350, 5000); // Run BMP180 sensor process every second.

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