/*==================================================================
  File Name    : $Id: $
  Function name: -
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : EEPROM routines
  ------------------------------------------------------------------
  $Log:$
  ================================================================== */ 
#include "eep.h"
#include "Nixie.h"

extern uint8_t blank_begin_h;
extern uint8_t blank_begin_m;
extern uint8_t blank_end_h;
extern uint8_t blank_end_m;
extern uint8_t wheel_effect;
extern uint8_t col_time;
extern uint8_t col_date;
extern uint8_t col_temp;
extern uint8_t col_humi;
extern uint8_t col_dewp;
extern uint8_t col_pres;
extern uint8_t col_roll;
extern bool    dst_active; // true = Daylight Saving Time active

void check_and_init_eeprom(void)
{
	uint8_t x;
	
	x = eeprom_read_byte(EEPARB_INIT);
	if (x == NO_INIT)
	{
		eeprom_write_byte(EEPARB_INIT , 0x01);     // Eeprom init. flag
		eeprom_write_byte(EEPARB_HR1  , 23);       // Start hour of blanking Nixies
		eeprom_write_byte(EEPARB_MIN1 , 30);       // Start minute of blanking Nixies
		eeprom_write_byte(EEPARB_HR2  ,  7);       // End hour of blanking Nixies
		eeprom_write_byte(EEPARB_MIN2 , 30);       // End minute of blanking Nixies
		eeprom_write_byte(EEPARB_WHEEL,  2);       // Wheel-effect for digits
		eeprom_write_byte(EEPARB_COL_TIME,BLACK);  // Colour for normal time-display
		eeprom_write_byte(EEPARB_COL_DATE,GREEN);  // Colour for Date & Year display
		eeprom_write_byte(EEPARB_COL_TEMP,YELLOW); // Colour for Temperature display
		eeprom_write_byte(EEPARB_COL_HUMI,BLUE);   // Colour for Humidity display
		eeprom_write_byte(EEPARB_COL_DEWP,CYAN);   // Colour for Dewpoint display
		eeprom_write_byte(EEPARB_COL_PRES,RED);    // Colour for Pressure display
		eeprom_write_byte(EEPARB_COL_ROLL,WHITE);  // Colour for Seconds rollover display
		eeprom_write_byte(EEPARB_DST,0);           // Daylight-savings time
	} // if
} // check_and_init_eeprom()

void read_eeprom_parameters(void)
{
	blank_begin_h = eeprom_read_byte(EEPARB_HR1);	
	blank_begin_m = eeprom_read_byte(EEPARB_MIN1);
	blank_end_h   = eeprom_read_byte(EEPARB_HR2);
	blank_end_m   = eeprom_read_byte(EEPARB_MIN2);
	wheel_effect  = eeprom_read_byte(EEPARB_WHEEL);
	col_time      = eeprom_read_byte(EEPARB_COL_TIME);
	col_date      = eeprom_read_byte(EEPARB_COL_DATE);
	col_temp      = eeprom_read_byte(EEPARB_COL_TEMP);
	col_humi      = eeprom_read_byte(EEPARB_COL_HUMI);
	col_dewp      = eeprom_read_byte(EEPARB_COL_DEWP);
	col_pres      = eeprom_read_byte(EEPARB_COL_PRES);
	col_roll      = eeprom_read_byte(EEPARB_COL_ROLL);
	dst_active    = eeprom_read_byte(EEPARB_DST);
} // read_eeprom_parameters()

void write_eeprom_parameters(void)
{
	eeprom_write_byte(EEPARB_HR1     ,blank_begin_h);
	eeprom_write_byte(EEPARB_MIN1    ,blank_begin_m);
	eeprom_write_byte(EEPARB_HR2     ,blank_end_h);
	eeprom_write_byte(EEPARB_MIN2    ,blank_end_m);
	eeprom_write_byte(EEPARB_WHEEL   ,wheel_effect);
	eeprom_write_byte(EEPARB_COL_TIME,col_time);
	eeprom_write_byte(EEPARB_COL_DATE,col_date);
	eeprom_write_byte(EEPARB_COL_TEMP,col_temp);
	eeprom_write_byte(EEPARB_COL_HUMI,col_humi);
	eeprom_write_byte(EEPARB_COL_DEWP,col_dewp);
	eeprom_write_byte(EEPARB_COL_PRES,col_pres);
	eeprom_write_byte(EEPARB_DST     ,dst_active);
} // write_eeprom_parameters()
