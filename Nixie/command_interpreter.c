//-----------------------------------------------------------------------------
// Created: 22-4-2013 07:26:24
// Author : Emile
// File   : $Id: command_interpreter.c,v 1.1 2016/05/07 09:37:27 Emile Exp $
//-----------------------------------------------------------------------------
// $Log: command_interpreter.c,v $
// Revision 1.1  2016/05/07 09:37:27  Emile
// - Project restructure
//
// Revision 1.1.1.1  2016/05/07 08:50:25  Emile
// - Initial version for 1st time check-in to CVS.
//
//-----------------------------------------------------------------------------
#include <string.h>
#include <ctype.h>
#include <util/atomic.h>
#include <stdio.h>
#include "i2c.h"
#include "command_interpreter.h"
#include "eep.h"
#include "Nixie.h"

char    rs232_inbuf[USART_BUFLEN];     // buffer for RS232 commands
uint8_t rs232_ptr = 0;                 // index in RS232 buffer
extern  uint8_t test_nixies;
extern  uint8_t blank_begin_h;
extern  uint8_t blank_begin_m;
extern  uint8_t blank_end_h;
extern  uint8_t blank_end_m;
extern  uint8_t wheel_effect;

extern  uint8_t col_time;
extern  uint8_t col_date;
extern  uint8_t col_temp;
extern  uint8_t col_humi;
extern  uint8_t col_dewp;
extern  uint8_t col_pres;
extern  uint8_t col_roll;
extern  uint8_t rgb_pattern;           // RGB color mode: [RANDOM, DYNAMIC, FIXED, OFF]

/*-----------------------------------------------------------------------------
  Purpose  : Non-blocking RS232 command-handler via the USB port
  Variables: -
  Returns  : [NO_ERR, ERR_CMD, ERR_NUM, ERR_I2C]
  ---------------------------------------------------------------------------*/
uint8_t rs232_command_handler(void)
{
  char    ch;
  static uint8_t cmd_rcvd = 0;
  
  if (!cmd_rcvd && usart_kbhit())
  { // A new character has been received
    ch = tolower(usart_getc()); // get character as lowercase
	switch (ch)
	{
		case '\r': break;
		case '\n': cmd_rcvd  = 1;
		           rs232_inbuf[rs232_ptr] = '\0';
		           rs232_ptr = 0;
				   break;
		default  : rs232_inbuf[rs232_ptr++] = ch;
				   break;
	} // switch
  } // if
  if (cmd_rcvd)
  {
	  cmd_rcvd = 0;
	  return execute_single_command(rs232_inbuf);
  } // if
  else return NO_ERR;
} // rs232_command_handler()

void print_dow(uint8_t dow)
{
	switch (dow)
	{
		case 1: xputs("Mon"); break;
		case 2: xputs("Tue"); break;
		case 3: xputs("Wed"); break;
		case 4: xputs("Thu"); break;
		case 5: xputs("Fri"); break;
		case 6: xputs("Sat"); break;
		case 7: xputs("Sun"); break;
	}
} // print_dow()

/*-----------------------------------------------------------------------------
  Purpose: interpret commands which are received via the USB serial terminal:
  Variables: 
          s: the string that contains the command from RS232 serial port 0
  Returns  : [NO_ERR, ERR_CMD, ERR_NUM, ERR_I2C] or ack. value for command
  ---------------------------------------------------------------------------*/
uint8_t execute_single_command(char *s)
{
   uint8_t  num  = atoi(&s[1]); // convert number in command (until space is found)
   uint8_t  val;
   uint8_t  rval = NO_ERR;
   char     s2[40]; // Used for printing to RS232 port
   char     *s1;
   uint8_t  d,m,h,sec;
   uint16_t y;
   Time     p;
   int16_t  temp;
   
   switch (s[0])
   {
				 
	   case 'c': // Set default colours of RGB LED [0..6]
				 val = atoi(&s[3]);
				 if ((num > 6) || (val > 7))
					rval = ERR_NUM;
				 else
				 {
					eeprom_write_byte(EEPARB_COL_TIME + (num << 1),val);
					switch (num)
					{
						case 0: xputs("col_time=");
								col_time = val;
								break;
						case 1: xputs("col_date=");
								col_date = val;
								break;
						case 2: xputs("col_temp=");
								col_temp = val;
								break;
						case 3: xputs("col_humi=");
								col_humi = val;
								break;
						case 4: xputs("col_dewp=");
								col_dewp = val;
								break;
						case 5: xputs("col_pres=");
								col_pres = val;
								break;
						case 6: xputs("col_roll=");
								col_roll = val;
								break;
					} // switch
				    switch (val)
				    {
					    case 0: xputs("black\n");   break;
					    case 1: xputs("red\n")  ;   break;
					    case 2: xputs("green\n");   break;
					    case 3: xputs("yellow\n");  break;
					    case 4: xputs("blue\n");    break;
					    case 5: xputs("magenta\n"); break;
					    case 6: xputs("cyan\n");    break;
					    case 7: xputs("white\n");   break;
				    } // switch
			    } // else
			    break;
	   
	   case 'd': // Set Date and Time
				 switch (num)
				 {
					 case 0: // Set Date
							s1 = strtok(&s[3],":-");
							d  = atoi(s1);
							s1 = strtok(NULL ,":-");
							m  = atoi(s1);
							s1 = strtok(NULL ,":-");
							y  = atoi(s1);
							xputs("Date: ");
							print_dow(ds3231_calc_dow(d,m,y));
							sprintf(s2," %02d-%02d-%d\n",d,m,y);
							xputs(s2);
							ds3231_setdate(d,m,y); // write to DS3231 IC
							break;
					 case 1: // Set Time
							s1 = strtok(&s[3],":-.");
							h  = atoi(s1);
							s1 = strtok(NULL ,":-.");
							m  = atoi(s1);
							s1 = strtok(NULL ,":-.");
							sec= atoi(s1);
							sprintf(s2,"Time: %02d:%02d:%02d\n",h,m,sec);
							xputs(s2);
							ds3231_settime(h,m,sec); // write to DS3231 IC
							break;
					 case 2: // Get Date & Time
							 ds3231_gettime(&p);
							 xputs("DS3231: ");
							 print_dow(p.dow);
							 sprintf(s2," %02d-%02d-%d, %02d:%02d.%02d\n",p.date,p.mon,p.year,p.hour,p.min,p.sec);
							 xputs(s2);
							 break;
					 case 3: // Get Temperature
							 temp = ds3231_gettemp();
							 sprintf(s2,"DS3231: %d.",temp>>2);
							 xputs(s2);
							 switch (temp & 0x03)
							 {
								 case 0: xputs("00 °C\n"); break;
								 case 1: xputs("25 °C\n"); break;
								 case 2: xputs("50 °C\n"); break;
								 case 3: xputs("75 °C\n"); break;
							 } // switch
							 break;
					 case 4: // Set Start-Time for blanking Nixies
							 s1 = strtok(&s[3],":-.");
							 h  = atoi(s1);
							 s1 = strtok(NULL ,":-.");
							 m  = atoi(s1);
							 if ((h < 24) && (m < 60))
							 {
								blank_begin_h = h;
								blank_begin_m = m;
								write_eeprom_parameters(); 
								sprintf(s2,"Start-Time for blanking Nixies: %02d:%02d\n",h,m);
								xputs(s2);
							 } // if
							 break;
					 case 5: // Set End-Time for blanking Nixies
							 s1 = strtok(&s[3],":-.");
							 h  = atoi(s1);
							 s1 = strtok(NULL ,":-.");
							 m  = atoi(s1);
							 if ((h < 24) && (m < 60))
							 {
								 blank_end_h = h;
								 blank_end_m = m;
								 write_eeprom_parameters();
								 sprintf(s2,"End-Time for blanking Nixies: %02d:%02d\n",h,m);
								 xputs(s2);
							 } // if
							 break;
					 default: rval = ERR_NUM;
							  break;
				 } // switch
				 break;
				 
	   case 'e': // E0: reset EEPROM
				 if (num > 0)
				 rval       = ERR_NUM;
				 else 
				 {
					 eeprom_write_byte(EEPARB_INIT, NO_INIT); // Eeprom init. flag;
					 xputs("EEPROM reset\n");
				 } // else					
				 break;

	   case 'r': // Set rgb_pattern
				 if (num > FIXED)
					rval = ERR_NUM;
				 else
				 {
					rgb_pattern = num;
					eeprom_write_byte(EEPARB_RGB_PATT,rgb_pattern);
					xputs("rgb_pattern = ");
					switch (rgb_pattern)
					{
						case OFF    : xputs("OFF\n");     break;
						case RANDOM : xputs("RANDOM\n");  break;
						case DYNAMIC: xputs("DYNAMIC\n"); break;
						case FIXED  : xputs("FIXED\n");   break;
					} // switch
				 } // else
				 break;

	   case 's': // System commands
				 switch (num)
				 {
					 case 0: // revision
							 xputs("Nixie board v0.2\n");
							 break;
					 case 1: // List all I2C devices
					         i2c_scan();
							 break;
					 case 2: // List all tasks
							 list_all_tasks(); 
							 break;
					 case 3: // test nixies
					         test_nixies = true;
							 break;	
					 default: rval = ERR_NUM;
							  break;
				 } // switch
				 break;

	   case 'w': // Set wheel-effect
				if (num > 2)
				rval = ERR_NUM;
				else 
				{
					wheel_effect = num;
					eeprom_write_byte(EEPARB_WHEEL,wheel_effect);
				} // else				
				break;
	   
	   default: rval = ERR_CMD;
				sprintf(s2,"ERR.CMD[%s]\n",s);
				xputs(s2);
	            break;
   } // switch
   return rval;	
} // execute_single_command()