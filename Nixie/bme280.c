/*==================================================================
  File Name    : bme280.c
  Function name: -
  Author       : Ronald / Emile
  ------------------------------------------------------------------
  Purpose : local implementation for bme280 library
  ================================================================== */ 
#include "bme280.h"
#include <inttypes.h>

// Calibration data
uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;
uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;
uint8_t  dig_H1;
int16_t  dig_H2;
uint8_t  dig_H3;
int16_t  dig_H4;
int16_t  dig_H5;
int8_t   dig_H6;
int32_t  t_fine;

/*------------------------------------------------------------------------
  Purpose  : This function writes one 8-bit value into the BME280
             and returns the result.
  Variables: reg: the i2c address of the register to write
             dat: the value to write to the bme280 register
  Returns  : true = error
  ------------------------------------------------------------------------*/
bool bme280_write8(uint8_t reg, uint8_t dat)
{
	bool    err = false;
	
	err = (i2c_start(BME280_I2C_ADDR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(reg) == I2C_NACK); // seconds register is first register to read
	if (!err) err = (i2c_write(dat) == I2C_NACK); // seconds register is first register to read
	i2c_stop();
	return err;
} // bme280_write8()

/*------------------------------------------------------------------------
  Purpose  : This function reads one 8-bit register from the BME280
             and returns the result.
  Variables: reg: the i2c address of the register to read
  Returns  : the 8-bit result
  ------------------------------------------------------------------------*/
uint8_t bme280_read8(uint8_t reg, bool *err)
{
	uint8_t x = 0;	
	
	// dig_T register lsb = 0x88, msb = 0x89
	*err = (i2c_start(BME280_I2C_ADDR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
	if (!*err) *err = (i2c_write(reg) == I2C_NACK); // seconds register is first register to read
	if (!*err) *err = (i2c_rep_start(BME280_I2C_ADDR | I2C_READ) == I2C_NACK);
	if (!*err) x   = (uint8_t)i2c_readNak();
	i2c_stop();
	return x;
} // bme280_read8()

/*------------------------------------------------------------------------
  Purpose  : This function reads one 16-bit register from the BME280
             and returns the result.
  Variables: reg: the i2c address of the register to read
  Returns  : the 16-bit result
  ------------------------------------------------------------------------*/
uint16_t bme280_read16(uint8_t reg, bool *err)
{
	uint8_t  msb_reg, lsb_reg;
	uint16_t retv;
	
	lsb_reg = bme280_read8(reg,err);
	msb_reg = bme280_read8(reg+1,err);
	retv    = (uint16_t)msb_reg << 8 | lsb_reg;
	return retv;
} // bme280_read16()

/*------------------------------------------------------------------------
  Purpose  : This function reads N  8-bit registers from the BME280
             and returns the result.
  Variables: reg: the first register address to read from
             len: the number of 8-bit registers to read
			 *p : pointer to struct where to store the registers read
			 *err: true = error
  Returns  : - 
  ------------------------------------------------------------------------*/
void bme280_readN(uint8_t reg, uint8_t len, uint8_t *p, bool *err)
{
	uint8_t  i = 0;
	
	*err = false;
	while ((i < len) && !*err)
	{
		p[i] = bme280_read8(reg+i,err);
		i++;
	} // while
} // bme280_readN()

int8_t get_calib_data()
{
	bool    err = false;
	uint8_t lsb, msb;
	
	dig_T1 = bme280_read16(0x88,&err);               // read dig_T1
	dig_T2 = (int16_t)bme280_read16(0x8A,&err);      // Read dig_T2
	dig_T3 = (int16_t)bme280_read16(0x8C,&err);      // Read dig_T3

	dig_P1 = bme280_read16(0x8E,&err);               // read dig_P1
	dig_P2 = (int16_t)bme280_read16(0x90,&err);      // Read dig_P2
	dig_P3 = (int16_t)bme280_read16(0x92,&err);      // Read dig_P3
	dig_P4 = (int16_t)bme280_read16(0x94,&err);      // Read dig_P4
	dig_P5 = (int16_t)bme280_read16(0x96,&err);      // Read dig_P5
	dig_P6 = (int16_t)bme280_read16(0x98,&err);      // Read dig_P6
	dig_P7 = (int16_t)bme280_read16(0x9A,&err);      // Read dig_P7
	dig_P8 = (int16_t)bme280_read16(0x9C,&err);      // Read dig_P8
	dig_P9 = (int16_t)bme280_read16(0x9E,&err);      // Read dig_P9
	
	dig_H1 = bme280_read8(0xA1,&err);                // Read dig_H1
	dig_H2 = (int16_t)bme280_read16(0xE1,&err);      // Read dig_H2
	dig_H3 = bme280_read8(0xE3,&err);                // Read dig_H3
	
	msb = bme280_read8(0xE4,&err);
	lsb = bme280_read8(0xE5,&err);
	dig_H4 = ((int16_t)msb << 4) | (lsb & 0x0F);
	msb = bme280_read8(0xE6,&err);
	dig_H5  = (int16_t)msb << 4;
	dig_H5 |= (lsb >> 4) & 0x0F;
	dig_H6 = (int8_t)bme280_read8(0xE7,&err);

#ifdef DEBUG_SENSORS
	char    s2[40];
	sprintf(s2,"dig_T1=0x%x\n",dig_T1); xputs(s2);
	sprintf(s2,"dig_T2=0x%x\n",dig_T2); xputs(s2);
	sprintf(s2,"dig_T3=0x%x\n",dig_T3); xputs(s2);
	sprintf(s2,"dig_P1=0x%x\n",dig_P1); xputs(s2);
	sprintf(s2,"dig_P2=0x%x\n",dig_P2); xputs(s2);
	sprintf(s2,"dig_P3=0x%x\n",dig_P3); xputs(s2);
	sprintf(s2,"dig_P4=0x%x\n",dig_P4); xputs(s2);
	sprintf(s2,"dig_P5=0x%x\n",dig_P5); xputs(s2);
	sprintf(s2,"dig_P6=0x%x\n",dig_P6); xputs(s2);
	sprintf(s2,"dig_P7=0x%x\n",dig_P7); xputs(s2);
	sprintf(s2,"dig_P8=0x%x\n",dig_P8); xputs(s2);
	sprintf(s2,"dig_P9=0x%x\n",dig_P9); xputs(s2);
	sprintf(s2,"dig_H1=0x%x\n",dig_H1); xputs(s2);
	sprintf(s2,"dig_H2=0x%x\n",dig_H2); xputs(s2);
	sprintf(s2,"dig_H3=0x%x\n",dig_H3); xputs(s2);
	sprintf(s2,"dig_H4=0x%x\n",dig_H4); xputs(s2);
	sprintf(s2,"dig_H5=0x%x\n",dig_H5); xputs(s2);
	sprintf(s2,"dig_H6=0x%x\n",dig_H6); xputs(s2);
#endif
	return err;
} // get_calib_data()

/*------------------------------------------------------------------------
  Purpose  : This functions initializes the bme280 device
  Variables: -
  Returns  : -
  ------------------------------------------------------------------------*/
void bme280_init(void)
{
	bool    err = false;
	uint8_t bme280_id;
	char    s2[30];

	// BME280 initialization. (BME280_REG_ID) 
	bme280_id = bme280_read8(BME280_CHIP_ID_ADDR,&err); // Read Chip-ID from bme280
	xputs("bme280_init(): ");
	if (err)  xputs("i2c-err\n");
	else      
	{
		xputs("ok\nchip-id: ");
		if (bme280_id == BME280_CHIP_ID) 
		{
			xputs("ok\n");
			/* Write the soft reset command in the sensor */
			err = bme280_write8(BME280_RESET_ADDR, BME280_RESET);
			/* As per data sheet, startup time is 2 ms. */
			delay_msec(2);
			if (!err) 
			{
				xputs("reset: ok\n");
				/* Read the calibration data */
				err = get_calib_data();
				xputs("calib: ");
				if (err) xputs("not ");
				xputs("ok\n");
			} // if
		} // if		
		else 
		{
			sprintf(s2,"error (0x%x)\n",bme280_id);
			xputs(s2);
		} // else
	} // else	
	
	// Settings and performance for indoor navigation (Data sheet recommendation settings)
	// Setting will be set while BME280 is in sleep mode
	//
	// Indoor Sensor mode:
	//  * Normal mode
	//  * Oversampling: Pressure x16, Temperature x2, Humidity x1
	//  * IIR filter: 16
	// Start to set BME280 register ctrl_hum (BME280_REG_CTRL_HUM)
	err = bme280_write8(BME280_CTRL_HUM_ADDR,BME280_OSRS_H);	        // Humidity oversampling x1
	
	// Set BME280 register config (BME280_REG_CONFIG).
	// Register has to be set in Sleep mode, before BME280_REG_CTRL_MEAS is set 
	err = bme280_write8(BME280_CONFIG_ADDR,BME280_T_SB | BME280_FILTER); // T_standby = 0.5 msec, IIR filter set to x16

	// Set BME280 register ctrl_meas (BME280_REG_CTRL_MEAS)
	err = bme280_write8(BME280_CTRL_MEAS_ADDR,BME280_OSRS_T | BME280_OSRS_P | BME280_MODE); // Set to Normal mode
} // bme280_init()

/*------------------------------------------------------------------------
  Purpose  : This task is called to read the BME280 temperature in E-2 °C.
  Variables: -
  Returns  : Compensated temperature in E-2 °C
  ------------------------------------------------------------------------*/
int32_t bme280_temperature(void)
{
	int32_t  adc_T; 
	int32_t  var1, var2;
	int32_t	 T;
	
	uint8_t msb_t, lsb_t, xlsb_t;
	char    s2[30];
	bool    err = false;
	
	msb_t  = bme280_read8(0xFA,&err); // Read msb_t
	lsb_t  = bme280_read8(0xFB,&err); // read lsb_t
	xlsb_t = bme280_read8(0xFC,&err); // read xlsb_t
    adc_T  = (xlsb_t >> 4) & 0x0000000F;
	adc_T |= ((uint32_t)msb_t << 12) | ((uint32_t)lsb_t << 4);
#ifdef DEBUG_SENSORS
	sprintf(s2,"msb_t = %x\n",msb_t);    xputs(s2);
	sprintf(s2,"lsb_t = %x\n",lsb_t);    xputs(s2);
	sprintf(s2,"xlsb_t = %x\n",xlsb_t);  xputs(s2);
	sprintf(s2,"adc_T = 0x%lx\n",adc_T); xputs(s2);
#endif	
	// Calculate Temperature in 32bit precision
    var1   = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	var2   = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
    T      = (t_fine * 5 + 128) >> 8;	
	//sprintf(s2,"T = %ld\n",T);
	//xputs(s2);
	return T;
} // bme280_temperature()

/*------------------------------------------------------------------------
  Purpose  : This task is called to read the BME280 temperature in Pascal.
  Variables: -
  Returns  : Compensated pressure in Pa.
  ------------------------------------------------------------------------*/
uint32_t bme280_pressure(void)
{
	int32_t  var1, var2;
	uint32_t p;
	int32_t  adc_P;
	uint8_t  msb_p, lsb_p, xlsb_p;
	char     s2[30];
	bool     err = false;
	
	msb_p  = bme280_read8(0xF7,&err); // Read msb_t
	lsb_p  = bme280_read8(0xF8,&err); // read lsb_t
	xlsb_p = bme280_read8(0xF9,&err); // read xlsb_t
    adc_P  = (xlsb_p >> 4) & 0x0000000F;
	adc_P |= ((uint32_t)msb_p << 12) | ((uint32_t)lsb_p << 4);
#ifdef DEBUG_SENSORS
	sprintf(s2,"msb_t = %x\n",msb_p);    xputs(s2);
	sprintf(s2,"lsb_t = %x\n",lsb_p);    xputs(s2);
	sprintf(s2,"xlsb_t = %x\n",xlsb_p);  xputs(s2);
	sprintf(s2,"adc_P = 0x%lx\n",adc_P); xputs(s2);
#endif	
	// Calculate Pressure in 32-bit precision
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
	var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
	var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if (p < 0x80000000)
	{
		p = (p << 1) / ((uint32_t)var1);
	}
	else
	{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
	p /= 10; // round to 1 decimal
	//sprintf(s2,"P = %ld\n",p);
	//xputs(s2);
	return p;
} // bme280_pressure()

/*------------------------------------------------------------------------
  Purpose  : This task is called to read the BME280 humidity in E-3 %.
  Variables: -
  Returns  : Compensated humidity in E-3 %
             Returns humidity in %RH as unsigned 32 bit integer in Q22.10 
			 format (22 integer and 10 fractional bits). 
			 Output value of “47445” represents 47445/1024 = 46.333 %RH
  ------------------------------------------------------------------------*/
uint32_t bme280_humidity(void)
{
	int32_t  adc_H; 
	uint32_t v_x1_u32r,H;
	uint8_t  msb_h, lsb_h;
	char     s2[30];
	bool     err = false;
	
	msb_h  = bme280_read8(0xFD,&err); // Read msb_h
	lsb_h  = bme280_read8(0xFE,&err); // read lsb_h
	adc_H  = ((uint32_t)msb_h << 8) | (uint32_t)lsb_h;
#ifdef DEBUG_SENSORS
	sprintf(s2,"msb_h = %x\n",msb_h);    xputs(s2);
	sprintf(s2,"lsb_h = %x\n",lsb_h);    xputs(s2);
	sprintf(s2,"adc_H = 0x%lx\n",adc_H); xputs(s2);
#endif	
	// Calculate Humidity in 32-bit precision
	v_x1_u32r = (t_fine - ((uint32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
                ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r *
				((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
				((int32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	H  = (uint32_t)(v_x1_u32r >> 12);
	H  = H * 125 >> 7; // * 1000 / 1024 = 125 / 128
	//sprintf(s2,"H = %ld\n",H);
	//xputs(s2);
	return H;} // bme280_humidity()