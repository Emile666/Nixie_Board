/*==================================================================
  File Name    : $Id: $
  Function name: -
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : EEPROM routines
  ------------------------------------------------------------------
  $Log:$
  ================================================================== */ 
#ifndef EEP_H_
#define EEP_H_
#include <avr\eeprom.h>
#include <stdbool.h>

// EEprom Parameters
#define EEPTR        (void *)
#define EEPTR1       (uint8_t *)  
#define EEPTR2       (uint16_t *)
#define EEPTRF       (float *)

#define EEPARB_INIT   EEPTR1(0x08)
#define EEPARB_HR1    EEPTR1(0x0A)
#define EEPARB_MIN1   EEPTR1(0x0C)
#define EEPARB_HR2    EEPTR1(0x0E)
#define EEPARB_MIN2   EEPTR1(0x10)

#define NO_INIT      (0xff)

void check_and_init_eeprom(void);
void read_eeprom_parameters(void);
void write_eeprom_parameters(void);

#endif /* EEP_H_ */