//-----------------------------------------------------------------------------
// Created: 07/12/2011 15:17:35
// Author : Emile
// File   : $Id: Nixie.c,v 1.1 2016/05/07 09:37:27 Emile Exp $
//-----------------------------------------------------------------------------
// $Log: Nixie.c,v $
// Revision 1.1  2016/05/07 09:37:27  Emile
// - Project restructure
//
// Revision 1.1.1.1  2016/05/07 08:50:25  Emile
// - Initial version for 1st time check-in to CVS.
//
//-----------------------------------------------------------------------------
#include <avr/io.h>
#include <util/atomic.h>
#include "scheduler.h"
#include "usart.h"
#include "IRremote.h"
#include "Nixie.h"
#include "i2c.h"
#include "command_interpreter.h"
#include "dht22.h"

uint8_t       test_nixies = false; // S3 command
uint8_t       cnt_50usec  = 0;     // 50 usec. counter
unsigned long t2_millis   = 0UL;   // msec. counter
uint16_t      dht22_hum   = 0;     // Humidity E-1 %
int16_t       dht22_temp  = 0;     // Temperature E-1 Celsius
int16_t       dht22_dewp  = 0;     // Dewpoint E-1 Celsius

extern char   rs232_inbuf[];       // RS232 input buffer

//---------------------------------------------------------------------------
// Bits 31..24: Decimal points: LDP5 LDP3 RDP6 RDP5 RDP4 RDP3 RDP2 RDP1
// Bits 23..16: Hours         : HHD  HHC  HHB  HHA  HLD  HLC  HLB  HLA 
// Bits 15..08: Minutes       : MHD  MHC  MHB  MHA  MLD  MLC  MLB  MLA
// Bits 07..00: Seconds       : SHD  SHC  SHB  SHA  SLD  SLC  SLB  SLA
//---------------------------------------------------------------------------
unsigned long int nixie_bits = 0UL;
uint8_t           rgb_colour = BLACK;

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
  Purpose  : This is the Timer-Interrupt routine which runs every 50 usec. 
             (f=20 kHz). It is used by the task-scheduler and the IR-receiver.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------------*/
ISR(TIMER2_COMPA_vect)
{
	PORTB |= TIME_MEAS; // Time Measurement
	if (++cnt_50usec > 19) 
	{
		scheduler_isr(); // call the ISR routine for the task-scheduler
		t2_millis++;     // update millisecond counter
		cnt_50usec = 0;
	} // if	
	ir_isr();            // call the ISR routine for the IR-receiver
	PORTB &= ~TIME_MEAS; // Time Measurement
} // ISR()

/*------------------------------------------------------------------------
  Purpose  : This task is called by the Task-Scheduler every 5 seconds.
             It reads the DHT22 sensor Humidity and Temperature
  Variables: dht22_humidity, dht22_temperature
  Returns  : -
  ------------------------------------------------------------------------*/
void dht22_task(void)
{
	//char s[20];
	int8_t x;
	
	dht22_read(&dht22_hum,&dht22_temp); // read DHT22 sensor
	//x = dht22_hum / 10;
	//sprintf(s," hum=%d.%d,",x,dht22_hum-10*x); 
	//xputs(s);
	//x = dht22_temp / 10;
	//sprintf(s," temp=%d.%d,",x,dht22_temp-10*x);
	//xputs(s);
	dht22_dewp = dht22_dewpoint(dht22_hum,dht22_temp);
	//x = dht22_dewp / 10;
	//sprintf(s," dewpoint=%d.%d\n",x,dht22_dewp-10*x);
	//xputs(s);
} // dht22_task()

/*------------------------------------------------------------------------
  Purpose  : This task is called by the Task-Scheduler every 50 msec. 
             It updates the Nixie tubes with the new bits.
  Variables: nixie_bits (32 bits)
  Returns  : -
  ------------------------------------------------------------------------*/
void update_nixies(void)
{
	uint8_t i;
	uint32_t mask = 0x80000000; // start with MSB
	
	PORTB &= ~(RGB_R | RGB_G | RGB_B);               // clear  LED colours
	PORTB |= (rgb_colour & (RGB_R | RGB_G | RGB_B)); // update LED colours
	
	for (i = 0; i < 32; i++)
	{
		PORTD &= ~(SHCP|STCP); // set clocks to 0
		if ((nixie_bits & mask) == mask)
			 PORTD |=  SDIN; // set SDIN to 1
		else PORTD &= ~SDIN; // set SDIN to 0
		mask >>= 1;     // shift right 1
		PORTD |=  SHCP; // set clock to 1
		PORTD &= ~SHCP; // set clock to 0 again
	} // for x
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

void ftest_nixies(void)
{
	static uint8_t std_test = 0;
	
	switch (std_test)
	{
		case 0: nixie_bits = 0x01000000; std_test = 1; break;
		case 1: nixie_bits = 0x02111111; std_test = 2; break;
		case 2: nixie_bits = 0x40222222; std_test = 3; break;
		case 3: nixie_bits = 0x04333333; std_test = 4; break;
		case 4: nixie_bits = 0x08444444; std_test = 5; break;
		case 5: nixie_bits = 0x80555555; std_test = 6; break;
		case 6: nixie_bits = 0x10666666; std_test = 7; break;
		case 7: nixie_bits = 0x20777777; std_test = 8; break;
		case 8: nixie_bits = 0xFF888888; std_test = 9; break;
		case 9: nixie_bits = 0xFF999999; std_test = 0; test_nixies = false; break;
	} // switch
	rgb_colour = std_test & 0x07;
} // ftest_nixies()

/*------------------------------------------------------------------------
  Purpose  : This task decide what to display on the Nixie Tubes.
			 Called once every second.
  Variables: nixie_bits (32 bits)
  Returns  : -
  ------------------------------------------------------------------------*/
void display_task(void)
{
	Time    p; // Time struct
	uint8_t x;
	
	nixie_bits = 0x00000000; // clear all bits
	ds3231_gettime(&p);
	if (test_nixies) ftest_nixies(); // S3 command
	else switch(p.sec)
	{
		case 25: // display date & month
			nixie_bits = encode_to_bcd(p.date);
			nixie_bits <<= 12;
			nixie_bits |= encode_to_bcd(p.mon);
			nixie_bits <<= 4;
			clear_nixie(3);
			clear_nixie(6);
			rgb_colour = GREEN;
			break;
		case 26: // display year
			nixie_bits = encode_to_bcd(p.year / 100);
			nixie_bits <<= 8;
			nixie_bits |= encode_to_bcd(p.year % 100);
			nixie_bits <<= 4;
			clear_nixie(1);
			clear_nixie(6);
			rgb_colour = GREEN;
			break;
		case 35: // display humidity
			x = dht22_hum / 10;
			nixie_bits = encode_to_bcd(x);
			nixie_bits <<= 4;
			nixie_bits |= (dht22_hum - 10 * x);
			nixie_bits <<= 12;
			nixie_bits |= RIGHT_DP2;
			clear_nixie(4);
			clear_nixie(5);
			clear_nixie(6);
			rgb_colour = BLUE;
			break;
		case 36: // display temperature
			x = dht22_temp / 10;
			nixie_bits = encode_to_bcd(x);
			nixie_bits <<= 4;
			nixie_bits |= (dht22_temp - 10 * x);
			nixie_bits |= RIGHT_DP5;
			clear_nixie(1);
			clear_nixie(2);
			clear_nixie(3);
			rgb_colour = BLUE;
			break;
		case 37: // display dewpoint
			x = dht22_dewp / 10;
			nixie_bits = encode_to_bcd(x);
			nixie_bits <<= 4;
			nixie_bits |= (dht22_dewp - 10 * x);
			nixie_bits <<= 4;
			nixie_bits |= RIGHT_DP4;
			clear_nixie(1);
			clear_nixie(2);
			clear_nixie(6);
			rgb_colour = CYAN;
			break;
		default: // display normal time
			nixie_bits = encode_to_bcd(p.hour);
			nixie_bits <<= 8; // SHL 8
			nixie_bits |= encode_to_bcd(p.min);
			nixie_bits <<= 8; // SHL 8
			nixie_bits |= encode_to_bcd(p.sec);
			rgb_colour = BLACK;
			if (p.sec & 0x01)
				 nixie_bits |= RIGHT_DP4;
			else nixie_bits |= LEFT_DP5;
			if (p.min & 0x01)
				 nixie_bits |= RIGHT_DP2;
			else nixie_bits |= LEFT_DP3;
			break;
	} // else switch
} // display_task()

/*------------------------------------------------------------------------
Purpose  : This function initialises Timer 2 for a 20 kHz (50 usec.)
           signal, because the IR library needs a 50 usec interrupt.
Variables: -
Returns  : -
------------------------------------------------------------------------*/
void init_timer2(void)
{
	// Set Timer 2 to 20 kHz interrupt frequency: ISR(TIMER2_COMPA_vect)
	TCCR2A |= (0x01 << WGM21); // CTC mode, clear counter on TCNT2 == OCR2A
	TCCR2B =  (0x01 << CS21);  // set pre-scaler to 8 (fclk = 2 MHz)
	OCR2A  =  99;    // this should set interrupt frequency to 20 kHz
	TCNT2  =  0;     // start counting at 0
	TIMSK2 |= (0x01 << OCIE2A);   // Set interrupt on Compare Match
} // init_timer2()

/*------------------------------------------------------------------------
Purpose  : This function initialises PORTB and PORTD pins.
           See Nixie.h header for a detailed overview of all port-pins.
Variables: -
Returns  : -
------------------------------------------------------------------------*/
void init_ports(void)
{
	DDRB  &= ~(DHT22 | IR_RCV); // clear bits = input
	PORTB &= ~(DHT22 | IR_RCV); // disable pull-up resistors
	DDRB  |=  (TIME_MEAS | RGB_R | RGB_G | RGB_B); // set bits = output
	PORTB &= ~(TIME_MEAS | RGB_R | RGB_G | RGB_B); // init. outputs to 0
	DDRD  |=  (SDIN | SHCP | STCP); // set bits = output
	PORTD &= ~(SDIN | SHCP | STCP); // init. outputs to 0
} // init_ports()

/*------------------------------------------------------------------------
Purpose  : main() function: Program entry-point, contains all init. functions 
		   and calls the task-scheduler in a loop.
Variables: -
Returns  : -
------------------------------------------------------------------------*/
int main(void)
{
	char s[20];
	
	init_timer2(); // init. timer for scheduler and IR-receiver
	ir_init();     // init. IR-library
	i2c_init(SCL_CLK_400KHZ); // Init. I2C HW with 400 kHz clock
	init_ports();  // init. PORTB and PORTD port-pins 	
	
	// Initialize Serial Communication, See usart.h for BAUD
	// F_CPU should be a Project Define (-DF_CPU=16000000UL)
	usart_init(MYUBRR); // Initializes the serial communication
	
	// Add tasks for task-scheduler here
	add_task(display_task ,"Display",  0, 1000); // What to display on the Nixies.
	add_task(update_nixies,"Update" ,100,   50); // Run Nixie Update every  50 msec.
	add_task(ir_receive   ,"IR_recv",150,  500); // Run IR-process   every 500 msec.
	add_task(dht22_task   ,"DHT22"  ,250, 5000); // Run DHT22 sensor process every 5 sec.
	
	sei(); // set global interrupt enable, start task-scheduler
	xputs("Nixie board v0.1, Emile, Martijn, Ronald\n");
    while(1)
    {   // Run all scheduled tasks here
		dispatch_tasks(); // Run Task-Scheduler
		switch (rs232_command_handler()) // run command handler continuously
		{
			case ERR_CMD: xputs("Command Error\n"); 
						  break;
			case ERR_NUM: sprintf(s,"Number Error (%s)\n",rs232_inbuf);
						  xputs(s);
						  break;
			default     : break;
		} // switch
    } // while
} // main()