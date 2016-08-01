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

extern uint8_t blank_begin_h;
extern uint8_t blank_begin_m;
extern uint8_t blank_end_h;
extern uint8_t blank_end_m;

void check_and_init_eeprom(void)
{
	uint8_t x;
	
	x = eeprom_read_byte(EEPARB_INIT);
	if (x == NO_INIT)
	{
		eeprom_write_byte(EEPARB_INIT, 0x01); // Eeprom init. flag
		eeprom_write_byte(EEPARB_HR1 , 23);   // Start hour of blanking Nixies
		eeprom_write_byte(EEPARB_MIN1, 30);   // Start minute of blanking Nixies
		eeprom_write_byte(EEPARB_HR2 ,  7);   // End hour of blanking Nixies
		eeprom_write_byte(EEPARB_MIN2,  0);   // End minute of blanking Nixies
	} // if
} // check_and_init_eeprom()

void read_eeprom_parameters(void)
{
	blank_begin_h = eeprom_read_byte(EEPARB_HR1);	
	blank_begin_m = eeprom_read_byte(EEPARB_MIN1);
	blank_end_h   = eeprom_read_byte(EEPARB_HR2);
	blank_end_m   = eeprom_read_byte(EEPARB_MIN2);
} // read_eeprom_parameters()

void write_eeprom_parameters(void)
{
	eeprom_write_byte(EEPARB_HR1 ,blank_begin_h);
	eeprom_write_byte(EEPARB_MIN1,blank_begin_m);
	eeprom_write_byte(EEPARB_HR2 ,blank_end_h);
	eeprom_write_byte(EEPARB_MIN2,blank_end_m);
} // write_eeprom_parameters()
