/*
	SFE_BMP180.h
	Bosch BMP180 pressure sensor library for the Arduino microcontroller
	Mike Grusin, SparkFun Electronics

	Uses floating-point equations from the Weather Station Data Logger project
	http://wmrx00.sourceforge.net/
	http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf

	Forked from BMP085 library by M.Grusin

	version 1.0 2013/09/20 initial version
	Verison 1.1.2 - Updated for Arduino 1.6.4 5/2015
	
	Our example code uses the "beerware" license. You can do anything
	you like with this code. No really, anything. If you find it useful,
	buy me a (root) beer someday.
*/
#ifndef _BMP180_h
#define _BMP180_h
#include <stdbool.h>

#define BMP180_ADR         (0xEE)
#define	BMP180_REG_CONTROL (0xF4)
#define	BMP180_REG_RESULT  (0xF6)

#define	BMP180_COMMAND_TEMPERATURE (0x2E)
#define	BMP180_COMMAND_PRESSURE0   (0x34)
#define	BMP180_COMMAND_PRESSURE1   (0x74)
#define	BMP180_COMMAND_PRESSURE2   (0xB4)
#define	BMP180_COMMAND_PRESSURE3   (0xF4)

bool bmp180_init(void);
bool bmp180_read_int(uint8_t address, int16_t *value);
bool bmp180_read_uint(uint8_t address, uint16_t *value);
bool bmp180_start_temperature(void);
bool bmp180_get_temperature(double *temperature);
bool bmp180_start_pressure(uint8_t oversampling);
bool bmp180_get_pressure(double *P, double temp);

#endif
