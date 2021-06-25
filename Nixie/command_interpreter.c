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
extern  uint8_t rgb_colour;
extern  bool test_nixies;
extern  char nixie_ver[];

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
extern  bool    hv_relay_sw;  // switch for hv_relay
extern  bool    hv_relay_fx;  // fix for hv_relay
extern  uint8_t rgb_pattern;  // RGB color mode: [RANDOM, DYNAMIC, FIXED, OFF]
extern  bool    dst_active;   // true = Daylight Saving Time active
extern  bool    set_col_white;   // true = esp8266 time update was successful
extern  bool    blanking_invert; // Invert blanking-active IR-command
extern  bool    enable_test_IR;  // Enable Test-pattern IR-command
extern  bool    last_esp8266;    // true = last esp8266 command was successful
extern  uint16_t esp8266_tmr;    // timer for updating ESP8266

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
   uint8_t  d,mo,h,mi,sec;
   uint16_t y;
   Time     p;
   int16_t  temp;
   const char sep[] = ":-.";
   
   switch (s[0])
   {
	   case 'c': // Set all Colours
				 val = atoi(&s[3]); // convert number until EOL
				 switch (num)
				 {
					 case 0: // Colour for Time display
					 col_time = val;
					 eeprom_write_byte(EEPARB_COL_TIME,val);
					 break;
					 case 1: // Colour for Date & Year display
					 col_date = val;
					 eeprom_write_byte(EEPARB_COL_DATE,val);
					 break;
					 case 2: // Colour for Temperature display
					 col_temp = val;
					 eeprom_write_byte(EEPARB_COL_TEMP,val);
					 break;
					 case 3: // Colour for Humidity display
					 col_humi = val;
					 eeprom_write_byte(EEPARB_COL_HUMI,val);
					 break;
					 case 4: // Colour for Dew-point display
					 col_dewp = val;
					 eeprom_write_byte(EEPARB_COL_DEWP,val);
					 break;
					 case 5: // Colour for Pressure display
					 col_pres = val;
					 eeprom_write_byte(EEPARB_COL_PRES,val);
					 break;
					 case 6: // Colour for seconds rollover display
					 col_roll = val;
					 eeprom_write_byte(EEPARB_COL_ROLL,val);
					 break;
					 default: rval = ERR_NUM;
					 break;
				 } // switch
				 sprintf(s,"col[%d]=%d\n",num,val); xputs(s);
				 break;

	   case 'd': // Set Date and Time
				 switch (num)
				 {
					case 0: // Set Date
							s1 = strtok(&s[3],sep);
							d  = atoi(s1);
							s1 = strtok(NULL ,sep);
							mo = atoi(s1);
							s1 = strtok(NULL ,sep);
							y  = atoi(s1);
							xputs("Date: ");
							print_dow(ds3231_calc_dow(d,mo,y));
							sprintf(s2," %02d-%02d-%d\n",d,mo,y);
							xputs(s2);
							ds3231_setdate(d,mo,y); // write to DS3231 IC
							break;
							
					case 1: // Set Time
							s1 = strtok(&s[3],sep);
							h  = atoi(s1);
							s1 = strtok(NULL ,sep);
							mi = atoi(s1);
							s1 = strtok(NULL ,sep);
							sec= atoi(s1);
							sprintf(s2,"Time: %02d:%02d:%02d\n",h,mi,sec);
							xputs(s2);
							ds3231_settime(h,mi,sec); // write to DS3231 IC
							break;
							
					case 2: // Get Date & Time
							 ds3231_gettime(&p);
							 xputs("DS3231: ");
							 print_dow(p.dow);
							 sprintf(s2," %02d-%02d-%d, %02d:%02d:%02d",p.date,p.mon,p.year,p.hour,p.min,p.sec);
							 xputs(s2);
							 sprintf(s2," dst:%d, blanking:%d\n", dst_active, blanking_active(p));
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
							s1 = strtok(&s[3],sep);
							h  = atoi(s1);
							s1 = strtok(NULL ,sep);
							mi = atoi(s1);
							if ((h < 24) && (mi < 60))
							{
								blank_begin_h = h;
								blank_begin_m = mi;
								eeprom_write_byte(EEPARB_HR1 ,blank_begin_h);
								eeprom_write_byte(EEPARB_MIN1,blank_begin_m);
								sprintf(s2,"Start-Time for blanking Nixies: %02d:%02d\n",h,mi);
								xputs(s2);
							} // if
							break;
							
					case 5: // Set End-Time for blanking Nixies
							s1 = strtok(&s[3],sep);
							h  = atoi(s1);
							s1 = strtok(NULL ,sep);
							mi = atoi(s1);
							if ((h < 24) && (mi < 60))
							{
								blank_end_h = h;
								blank_end_m = mi;
								eeprom_write_byte(EEPARB_HR2 ,blank_end_h);
								eeprom_write_byte(EEPARB_MIN2,blank_end_m);
								sprintf(s2,"End-Time for blanking Nixies: %02d:%02d\n",h,mi);
								xputs(s2);
							} // if
							break;		 
							 
					 default: rval = ERR_NUM;
							  break;
				 } // switch
				 break;
				 
        case 'e': // The e commands are responses back from the ESP8266 NTP Server
        // Possible response: "e0 26-05-2021.15:55:23"
        switch (num)
        {
	        case 0: // E0 = Get Date & Time from the ESP8266
	        s1 = strtok(&s[3],sep);
	        d  = atoi(s1);
	        s1 = strtok(NULL ,sep);
	        mo = atoi(s1);
	        s1 = strtok(NULL ,sep);
	        y  = atoi(s1);
	        // Second part is the time from the ESP8266
	        s1 = strtok(NULL,sep);
	        h  = atoi(s1);
	        if (dst_active)
	        {
		        if (h == 23)
		        h = 0;
		        else h++;
	        } // if
	        s1  = strtok(NULL ,sep);
	        mi  = atoi(s1);
	        s1  = strtok(NULL ,sep);
	        sec = atoi(s1);
	        if (sec == 59) // add 1 second for the transmit delay
	        sec = 0;
	        else sec++;
	        if (y > 2020)
	        {   // Valid Date & Time received
		        ds3231_setdate(d,mo,y);   // write to DS3231 IC
		        ds3231_settime(h,mi,sec); // write to DS3231 IC
		        last_esp8266 = true;      // response was successful
		        set_col_white = true;     // show briefly on display
		        esp8266_tmr = 0;          // Reset update timer
	        } // if
	        else last_esp8266 = false;   // response not successful
	        xputs("Date: ");
	        print_dow(ds3231_calc_dow(d,mo,y));
	        sprintf(s2," %d-%d-%d ",d,mo,y);
	        xputs(s2);
	        sprintf(s2,"Time: %d:%d:%d\n",h,mi,sec);
	        xputs(s2);
	        break;
        } // switch
        break;

	   case 'l': // Set RGB LED [0..7]
				 if (num > 7) 
				      rval       = ERR_NUM;
				 else rgb_colour = num;
				 break;

	   case 'm': // Set RGB mode command [0=OFF, 1=RANDOM, 2=DYNAMIC, 3=FIXED]
				 if (num > 4)
				      rval = ERR_NUM;
				 else if (num == 4)
				 {
					 sprintf(s2,"rgb_pattern=%d\n",rgb_pattern);
					 xputs(s2);
				 } // else if
				 else rgb_pattern = num;
				 break;
				 
	   case 's': // System commands
				 switch (num)
				 {
					 case 0: // revision
							 xputs(nixie_ver);
							 break;
					 case 1: // List all I2C devices
					         i2c_scan();
							 break;
					 case 2: // List all tasks
							 list_all_tasks(); 
							 break;
					 case 3: // test nixies
					         test_nixies = !test_nixies;
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