/*==================================================================
  File name    : $Id: scheduler.h,v 1.1 2016/05/07 09:37:27 Emile Exp $
  Author       : E. van de Logt
  ------------------------------------------------------------------
  Purpose : This is the header-file for scheduler.c
  ------------------------------------------------------------------
  $Log: scheduler.h,v $
  Revision 1.1  2016/05/07 09:37:27  Emile
  - Project restructure

  Revision 1.1.1.1  2016/05/07 08:50:25  Emile
  - Initial version for 1st time check-in to CVS.

  ==================================================================
*/ 
#ifndef _SCHEDULER_H
#define _SCHEDULER_H

#include <string.h>
#include <avr/io.h>
#include <stdbool.h>
#include "usart.h"

#define MAX_TASKS	  (9)
#define MAX_MSEC      (60000)
#define TICKS_PER_SEC (1000L)
#define NAME_LEN      (12) 

#define TASK_READY    (0x01)
#define TASK_ENABLED  (0x02)

#define NO_ERR        (0x00)
#define ERR_CMD		  (0x01)
#define ERR_NUM		  (0x02)
#define ERR_I2C		  (0x03)
#define ERR_NAME      (0x04)
#define ERR_EMPTY     (0x05)
#define ERR_MAX_TASKS (0x06)

typedef struct _task_struct
{
	void     (* pFunction)(); // Function pointer
	char     Name[NAME_LEN];  // Task name
	uint16_t Period;          // Period between 2 calls in msec.
	uint16_t Delay;           // Initial delay before Counter starts in msec.
	uint16_t Counter;         // Running counter, is init. from Period
	uint8_t	 Status;          // bit 1: 1=enabled ; bit 0: 1=ready to run
	uint16_t Duration;        // Measured task-duration in clock-ticks
	uint16_t Duration_Max;    // Max. measured task-duration
} task_struct;

void    scheduler_isr(void);  // run-time function for scheduler
void    dispatch_tasks(void); // run all tasks that are ready
uint8_t add_task(void (*task_ptr), char *Name, uint16_t delay, uint16_t period);
uint8_t set_task_time_period(uint16_t Period, char *Name);
uint8_t enable_task(char *Name);
uint8_t disable_task(char *Name);
void    list_all_tasks(void);

#endif
