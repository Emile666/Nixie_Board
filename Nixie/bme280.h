/*==================================================================
  File Name    : bme280.h
  Author       : Ronald / Emile
  ------------------------------------------------------------------
  Purpose : This is the header-file for bme280.c
  ================================================================== */ 
#ifndef _BME280_H
#define _BME280_H   1
#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include "i2c.h"
#include "usart.h"

// BME280 defines
#define BME280_ADR				(0xEC)	/* I2C base address with SDO to GND   */
#define BME280_REG_ID			(0xD0)
#define BME280_ID_Value			(0x60)
#define BME280_REG_RESET		(0xE0)
#define BME280_RESET			(0xB6)
#define BME280_REG_CONFIG		(0xF5)
#define BME280_REG_CTRL_MEAS	(0xF4)
#define BME280_REG_CTRL_HUM		(0xF2)	/* Has to be written before CTRL_MEAS */ 
#define BME280_OSRS_H			(0x01)	/* Humidity oversampling x1           */		
#define BME280_OSRS_T			(0x40)	/* Temperature oversampling x2        */
#define BME280_OSRS_P			(0x14)	/* Pressure oversampling x16          */
#define BME280_MODE				(0x03)	/* BME280 set to normal mode          */
#define BME280_T_SB				(0x00)	/* In Normal mode T_standby = 0.5mS   */
#define BME280_FILTER			(0x10)	/* IIR filter set to x16              */


// Function prototypes
uint8_t bme280_read_reg(uint8_t reg);
void    bme280_init(void);
int32_t bme280_temperature(void);

#endif
