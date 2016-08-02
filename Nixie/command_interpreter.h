/*==================================================================
  File Name    : $Id: command_interpreter.h,v 1.1 2016/05/07 09:37:27 Emile Exp $
  Function name: -
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : Header file for command_interpreter.c
  ------------------------------------------------------------------
  $Log: command_interpreter.h,v $
  Revision 1.1  2016/05/07 09:37:27  Emile
  - Project restructure

  Revision 1.1.1.1  2016/05/07 08:50:25  Emile
  - Initial version for 1st time check-in to CVS.

  ================================================================== */ 
#ifndef _CI_H
#define _CI_H   1
#include <avr/io.h>

#include <stdlib.h>
#include "scheduler.h"
#include "eep.h"

uint8_t rs232_command_handler(void);
uint8_t execute_single_command(char *s);

#endif
