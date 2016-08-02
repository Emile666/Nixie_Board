/*
	SFE_BMP180.cpp
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
#include <stdio.h>
#include <math.h>
#include "bmp180.h"
#include "i2c.h"
#include "usart.h"

int16_t  AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
uint16_t AC4,AC5,AC6; 
double   c5,c6,mc,md,x0,x1,x2,yy0,yy1,yy2,p0,p1,p2;
double   c3,c4,b1;
char     _error;

// Initialize library for subsequent pressure measurements
bool bmp180_init(void)
{
	//i2c_init(SCL_CLK_400KHZ); // Start up the I2C library (if not already done)

	// The BMP180 includes factory calibration data stored on the device.
	// Each device has different numbers, these must be retrieved and
	// used in the calculations when taking pressure measurements.
	// Retrieve calibration data from device:
	if (!bmp180_read_int(0xAA,&AC1)  && !bmp180_read_int(0xAC,&AC2)  &&
		!bmp180_read_int(0xAE,&AC3)  && !bmp180_read_uint(0xB0,&AC4) &&
		!bmp180_read_uint(0xB2,&AC5) && !bmp180_read_uint(0xB4,&AC6) &&
		!bmp180_read_int(0xB6,&VB1)  && !bmp180_read_int(0xB8,&VB2)  &&
		!bmp180_read_int(0xBA,&MB)   && !bmp180_read_int(0xBC,&MC)   &&
		!bmp180_read_int(0xBE,&MD))
	{
		// All reads completed successfully!

		// If you need to check your math using known numbers,
		// you can uncomment one of these examples.
		// (The correct results are commented in the below functions.)

		// Example from Bosch datasheet
		// AC1 = 408; AC2 = -72; AC3 = -14383; AC4 = 32741; AC5 = 32757; AC6 = 23153;
		// B1 = 6190; B2 = 4; MB = -32768; MC = -8711; MD = 2868;

		// Example from http://wmrx00.sourceforge.net/Arduino/BMP180-Calcs.pdf
		// AC1 = 7911; AC2 = -934; AC3 = -14306; AC4 = 31567; AC5 = 25671; AC6 = 18974;
		// VB1 = 5498; VB2 = 46; MB = -32768; MC = -11075; MD = 2432;

/*
		
		char s[20];
		sprintf(s,"AC1:%d\n",AC1); xputs(s);
		sprintf(s,"AC2:%d\n",AC2); xputs(s);
		sprintf(s,"AC3:%d\n",AC3); xputs(s);
		sprintf(s,"AC4:%d\n",AC4); xputs(s);
		sprintf(s,"AC5:%d\n",AC5); xputs(s);
		sprintf(s,"AC6:%d\n",AC6); xputs(s);
		sprintf(s,"VB1:%d\n",VB1); xputs(s);
		sprintf(s,"VB2:%d\n",VB2); xputs(s);
		sprintf(s,"MB:%d\n",MB)  ; xputs(s);
		sprintf(s,"MC:%d\n",MC)  ; xputs(s);
		sprintf(s,"MD:%d\n",MD)  ; xputs(s);
*/		
		
		// Compute floating-point polynominals:

		c3 = 160.0 * pow(2,-15) * AC3;
		c4 = pow(10,-3) * pow(2,-15) * AC4;
		b1 = pow(160,2) * pow(2,-30) * VB1;
		c5 = (pow(2,-15) / 160) * AC5;
		c6 = AC6;
		mc = (pow(2,11) / pow(160,2)) * MC;
		md = MD / 160.0;
		x0 = AC1;
		x1 = 160.0 * pow(2,-13) * AC2;
		x2 = pow(160,2) * pow(2,-25) * VB2;
		yy0 = c4 * pow(2,15);
		yy1 = c4 * c3;
		yy2 = c4 * b1;
		p0 = (3791.0 - 8.0) / 1600.0;
		p1 = 1.0 - 7357.0 * pow(2,-20);
		p2 = 3038.0 * 100.0 * pow(2,-36);
		return(false); // No error
	} // if
	else
	{
		// Error reading calibration data; bad component or connection?
		return(true);
	} // else
} // bmp180_init()

// Read a signed integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
bool bmp180_read_int(uint8_t address, int16_t *value)
{
	bool err;

	err = (i2c_start(BMP180_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(address) == I2C_NACK); // address of register to read
	if (!err) err = (i2c_rep_start(BMP180_ADR | I2C_READ) == I2C_NACK);
	if (!err)
	{
		*value  = (int16_t)i2c_readAck() << 8; // Read MSB register
		*value |= (uint8_t)i2c_readNak();      // Read LSB register, generate I2C stop condition
	}
	else *value = 0;
	i2c_stop(); // close I2C bus
	return err;
} // bmp_180_read_int()

bool bmp180_read_uint(uint8_t address, uint16_t *value)
// Read an unsigned integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	bool err;

	err = (i2c_start(BMP180_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(address) == I2C_NACK); // address of register to read
	if (!err) err = (i2c_rep_start(BMP180_ADR | I2C_READ) == I2C_NACK);
	if (!err)
	{
		*value  = (uint16_t)i2c_readAck() << 8; // Read MSB register
		*value |= (uint8_t)i2c_readNak();       // Read LSB register, generate I2C stop condition
	}
	else *value = 0;
	i2c_stop(); // close I2C bus
	return err;
} // bmp_180_read_uint()

bool bmp180_read_bytes(uint8_t address, uint8_t *data, uint8_t nr)
// Reads nr bytes from bmp180 sensor at specified address.
// address: register to start reading (plus subsequent register)
// data   : external variable to store data (function modifies value)
// nr     : number of bytes to read   
{
	bool err;
	uint8_t i;

	err = (i2c_start(BMP180_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(address) == I2C_NACK); // address of register to read
	if (!err) err = (i2c_rep_start(BMP180_ADR | I2C_READ) == I2C_NACK);
	i = 0;
	while (!err && (i < nr-1))
	{
		data[i++] = i2c_readAck(); // Read register
	} // while
	if (!err) data[i] = i2c_readNak(); // Read register, generate I2C stop condition
	i2c_stop(); // close I2C bus
	return err;
} // bmp_180_read_bytes()

bool bmp180_start_temperature(void)
// Begin a temperature reading.
// Will return i2c_error
{
	bool err;

	err = (i2c_start(BMP180_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(BMP180_REG_CONTROL) == I2C_NACK); // address of control register
	if (!err) err = (i2c_write(BMP180_COMMAND_TEMPERATURE) == I2C_NACK);
	i2c_stop(); // close I2C bus
	return err;
} // bmp180_start_temperature()

bool bmp180_get_temperature(double *temperature)
// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires start_temperature() to have been called prior and > 5 msec. elapsed.
// temperature: external variable to hold result.
// Returns error: false if successful, true if I2C error.
{
	bool     err;
	uint16_t data = 0;
	double   a;
	
	err = bmp180_read_uint(BMP180_REG_RESULT,&data);

	//example from Bosch datasheet
	//tu = 27898;

	//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
	//tu = 0x69EC;
	if (!err)
	{
		a            = c5 * ((double)data - c6);
		*temperature = a + (mc / (a + md));
	} // if
	return err;
} // bmp180_get_temperature()

bool bmp180_start_pressure(uint8_t oversampling)
// Begin a pressure reading.
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
// Will return delay in ms to wait, or 0 if I2C error.
{
	bool    err;
	uint8_t data;

	switch (oversampling)
	{
		case 0: // needs 5 msec. delay
			data = BMP180_COMMAND_PRESSURE0; break;
		case 1: // needs 8 msec. delay
			data = BMP180_COMMAND_PRESSURE1; break;
		case 2: // needs 14 msec. delay
			data = BMP180_COMMAND_PRESSURE2; break;
		case 3: // needs 26 msec. delay
			data = BMP180_COMMAND_PRESSURE3; break;
		default:
			data = BMP180_COMMAND_PRESSURE0; break;
	} // switch
	err = (i2c_start(BMP180_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(BMP180_REG_CONTROL) == I2C_NACK); // address of control register
	if (!err) err = (i2c_write(data) == I2C_NACK);
	i2c_stop(); // close I2C bus
	return err;
} // bmp180_start_pressure()

bool bmp180_get_pressure(double *P, double temp)
// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.

// P: external variable to hold pressure.
// T: previously-calculated temperature.
// Returns: I2C error.
{
	bool err;
	uint8_t data[3];
	double  pu,s,x,y,z;
	
	err = bmp180_read_bytes(BMP180_REG_RESULT,data,3);
	if (!err) // good read, calculate pressure
	{
		pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);

		//example from Bosch datasheet
		//pu = 23843;
		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf, pu = 0x982FC0;	
		//pu = (0x98 * 256.0) + 0x2F + (0xC0/256.0);
		
		s  = temp - 25.0;
		x  = (x2 * pow(s,2)) + (x1 * s) + x0;
		y  = (yy2 * pow(s,2)) + (yy1 * s) + yy0;
		z  = (pu - x) / y;
		*P = (p2 * pow(z,2)) + (p1 * z) + p0;
	} // if
	return(err);
} // bmp180_get_pressure()

