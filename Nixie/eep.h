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

#define EEPARB_INIT     EEPTR1(0x08)
#define EEPARB_HR1      EEPTR1(0x0A)
#define EEPARB_MIN1     EEPTR1(0x0C)
#define EEPARB_HR2      EEPTR1(0x0E)
#define EEPARB_MIN2     EEPTR1(0x10)
#define EEPARB_WHEEL    EEPTR1(0x12)
#define EEPARB_COL_TIME EEPTR1(0x14)
#define EEPARB_COL_DATE EEPTR1(0x16)
#define EEPARB_COL_TEMP EEPTR1(0x18)
#define EEPARB_COL_HUMI EEPTR1(0x1A)
#define EEPARB_COL_DEWP EEPTR1(0x1C)
#define EEPARB_COL_PRES EEPTR1(0x1E)
#define EEPARB_COL_ROLL EEPTR1(0x20)
#define EEPARB_DST      EEPTR1(0x22)

#define NO_INIT      (0xff)

void check_and_init_eeprom(void);
void read_eeprom_parameters(void);
void write_eeprom_parameters(void);

#endif /* EEP_H_ */