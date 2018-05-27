/*==================================================================
  File Name    : bme280.c
  Function name: -
  Author       : Ronald / Emile
  ------------------------------------------------------------------
  Purpose : local implementation for bme280 library
  ================================================================== */ 
#include "bme280.h"

// Reads BME280 registers in 8 bits
uint8_t bme280_read_reg(uint8_t reg)
{
	bool err;
	uint8_t x;	
	
	// dig_T register lsb = 0x88, msb = 0x89
	err = (i2c_start(BME280_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(reg) == I2C_NACK); // seconds register is first register to read
	if (!err) err = (i2c_rep_start(BME280_ADR | I2C_READ) == I2C_NACK);
	
	if (!err)
	{
		x  = (uint8_t)i2c_readNak();
		return x;
	} // if
	else
	{
		return 0;
	} // else
} // bme280_read_reg

/*------------------------------------------------------------------------
  Purpose  : This task is called by the Task-Scheduler every second.
             It reads the BME280 pressure and temperature.
  Variables: bme280_pressure, bme280_temperature
  Returns  : -
  ------------------------------------------------------------------------*/
void bme280_init(void)
{
	
	bool    err;
	uint8_t reg;
	char    s2[40];

	// BME280 initialization. (BME280_REG_ID) 
	err = (i2c_start(BME280_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(BME280_REG_ID) == I2C_NACK); // seconds register is first register to read
	if (!err) err = (i2c_rep_start(BME280_ADR | I2C_READ) == I2C_NACK);
	
	sprintf(s2,"Err = %01d\n",err);
	xputs(s2);
		
	if (!err)
	{
		reg  = (uint8_t)i2c_readNak();		
		sprintf(s2,"BME280_ID = 0x%02x\n",reg);
	}
	else sprintf(s2,"bme280_init() error\n");
	xputs(s2);
	
	//Perform RESET before setting registers
	//err = (i2c_start(BME280_ADR | I2C_WRITE) == I2C_NACK);			// generate I2C start + output address to I2C bus
	//if (!err) err = (i2c_write(BME280_REG_RESET) == I2C_NACK);		// write register address to write to
	//if (!err) err = (i2c_write(BME280_RESET) == I2C_NACK);			// write value into register
	//i2c_stop();														// close I2C bus		
	//
	//sprintf(s2,"BME280_RESET = 0x%01d\n",err);
	//xputs(s2);	
	
	// Settings and performance for indoor navigation (Data sheet recommendation settings)
	// Setting will be set while BME280 is in sleep mode
	//
	// Indoor Sensor mode:
	//  * Normal mode
	//  * Oversampling: Pressure x16, Temperature x2, Humidity x1
	//  * IIR filter: 16
		
	// Start to set BME280 register ctrl_hum (BME280_REG_CTRL_HUM)
	err = (i2c_start(BME280_ADR | I2C_WRITE) == I2C_NACK);			// generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(BME280_REG_CTRL_HUM) == I2C_NACK);   // write register address to write to
	if (!err) err = (i2c_write(BME280_OSRS_H) == I2C_NACK);         // write value into register
	i2c_stop();														// close I2C bus 

	sprintf(s2,"BME280_REG_CTRL_HUM = %01d\n",err);
	xputs(s2);
	
	
		//err = (i2c_start(BME280_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
		//if (!err) err = (i2c_write(BME280_REG_CTRL_HUM) == I2C_NACK); // seconds register is first register to read
		//if (!err) err = (i2c_rep_start(BME280_ADR | I2C_READ) == I2C_NACK);
		//
		//if (!err)
		//{
			//reg  = (uint8_t)i2c_readNak();
		//}
//
		//sprintf(s2,"BME280_REG_CTRL_HUM = 0x%02x\n",reg);
		//xputs(s2);	
	

	// Set BME280 register config (BME280_REG_CONFIG).
	//Register has to be set in Sleep mode, before BME280_REG_CTRL_MEAS is set 
	err = (i2c_start(BME280_ADR | I2C_WRITE) == I2C_NACK);						// generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(BME280_REG_CONFIG) == I2C_NACK);					// write register address to write to
	if (!err) err = (i2c_write(BME280_T_SB | BME280_FILTER) == I2C_NACK);       // write value into register
	i2c_stop();																	// close I2C bus
	
	sprintf(s2,"BME280_REG_CONFIG = %01d\n",err);
	xputs(s2);
	
		//err = (i2c_start(BME280_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
		//if (!err) err = (i2c_write(BME280_REG_CONFIG) == I2C_NACK); // seconds register is first register to read
		//if (!err) err = (i2c_rep_start(BME280_ADR | I2C_READ) == I2C_NACK);
		//
		//if (!err)
		//{
			//reg  = (uint8_t)i2c_readNak();
		//}
//
		//sprintf(s2,"BME280_REG_CONFIG = 0x%02x\n",reg);
		//xputs(s2);	
	

	// Set BME280 register ctrl_meas (BME280_REG_CTRL_MEAS)
	err = (i2c_start(BME280_ADR | I2C_WRITE) == I2C_NACK);										// generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(BME280_REG_CTRL_MEAS) == I2C_NACK);								// write register address to write to
	if (!err) err = (i2c_write(BME280_OSRS_T | BME280_OSRS_P | BME280_MODE) == I2C_NACK);       // write value into register
	i2c_stop();																					// close I2C bus	

		//err = (i2c_start(BME280_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
		//if (!err) err = (i2c_write(BME280_REG_CTRL_MEAS) == I2C_NACK); // seconds register is first register to read
		//if (!err) err = (i2c_rep_start(BME280_ADR | I2C_READ) == I2C_NACK);
		//
		//if (!err)
		//{
			//reg  = (uint8_t)i2c_readNak();
		//}
//
		//sprintf(s2,"BME280_REG_CTRL_MEAS = 0x%02x\n",reg);
		//xputs(s2);
//

	
	bme280_temperature();


} // bme280_init()

/*------------------------------------------------------------------------
  Purpose  : This task is called to read the BME280 temperature.
             
  Variables: bme280_pressure, bme280_temperature
  Returns  : Temperature
  ------------------------------------------------------------------------*/
void bme280_temperature(void)
{
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	int32_t  t_fine;
	uint32_t adc_T; 
	int32_t var1, var2;
	int32_t	T;
	
	uint8_t msb_reg, lsb_reg;
	uint8_t msb_t, lsb_t, xlsb_t;
	char    s2[30];
	
	// Read dig_T1
	lsb_reg = (bme280_read_reg(0x88));
	msb_reg = (bme280_read_reg(0x89));
	
	dig_T1 = (msb_reg << 8 | lsb_reg);		
		
	sprintf(s2,"dig_T1 = 0x%x\n",dig_T1);
	xputs(s2);
	
	
	// Read dig_T2
	lsb_reg = (bme280_read_reg(0x8A));
	msb_reg = (bme280_read_reg(0x8B));
		
	dig_T2 = (msb_reg << 8 | lsb_reg);
		
	sprintf(s2,"dig_T2 = 0x%x\n",dig_T2);
	xputs(s2);
	
	// Read dig_T3
	lsb_reg = (bme280_read_reg(0x8C));
	msb_reg = (bme280_read_reg(0x8D));
		
	dig_T3 = (msb_reg << 8 | lsb_reg);
		
	sprintf(s2,"dig_T3 = 0x%x\n",dig_T3);
	xputs(s2);
	
	//Read msb_t
	msb_t  = (bme280_read_reg(0xFA));
	sprintf(s2,"msb_t = %x\n",msb_t);
	xputs(s2);
	
	lsb_t  = (bme280_read_reg(0xFB));
	sprintf(s2,"lsb_t = %x\n",lsb_t);
	xputs(s2);
	
	xlsb_t = (bme280_read_reg(0xFC));
	sprintf(s2,"xlsb_t = %x\n",xlsb_t);
	xputs(s2);

	adc_T = (msb_t << 12 | lsb_t << 4 | xlsb_t >> 4); 
	
	sprintf(s2,"adc_T = %lx\n",adc_T);
	xputs(s2);
	
	//Calculate Temperature in 32bit precision
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	
	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	
	t_fine = var1 + var2;
    
    T = ((t_fine * 5 + 128) >> 8);	
	
		sprintf(s2,"Temperature = %ld\n",T);
		xputs(s2);
	
	//return T;
		
} //bme280_temperature


