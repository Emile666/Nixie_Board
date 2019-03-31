/*==================================================================
  File Name    : $Id: i2c.c,v 1.1 2016/05/07 09:37:27 Emile Exp $
  Function name: -
  Author       : Peter Fleury <pfleury@gmx.ch> http://jump.to/fleury
  ------------------------------------------------------------------
  Purpose : I2C master library using hardware TWI interface
  ------------------------------------------------------------------
  $Log: i2c.c,v $
  Revision 1.1  2016/05/07 09:37:27  Emile
  - Project restructure

  Revision 1.1.1.1  2016/05/07 08:50:25  Emile
  - Initial version for 1st time check-in to CVS.

  ================================================================== */ 
#include <inttypes.h>
#include <compat/twi.h>
#include <stdio.h>
#include "i2c.h"
#include "delay.h"         /* for delay_msec() */
#include "usart.h"

/*************************************************************************
 Initialization of the I2C bus interface. 
*************************************************************************/
void i2c_init(uint8_t clk)
{
  /* initialize TWI clock: TWPS = 0x01 => prescaler = 4 */
  
  if (clk == SCL_CLK_400KHZ) TWSR = 0x00; /* pre-scaler = 1 */
  else                       TWSR = 0x01; /* pre-scaler = 4 */
  TWBR = clk;  /* must be > 10 for stable operation */
} /* i2c_init() */

/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return:  I2C_ACK : device accessible
           I2C_NACK: failed to access device
*************************************************************************/
enum i2c_acks i2c_start(unsigned char address)
{
    uint8_t   twst;
    uint8_t   retries = 0;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while ((!(TWCR & (1<<TWINT))) && (retries < I2C_RETRIES))
	{
		delay_msec(1);
		retries++;
	} // while	

	// check value of TWI Status Register. Mask pre-scaler bits.
	twst = TW_STATUS & 0xF8;
	if (((twst != TW_START) && (twst != TW_REP_START)) || (retries >= I2C_RETRIES)) 
		return I2C_NACK;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask pre-scaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return I2C_NACK;

	return I2C_ACK;
} /* i2c_start() */

/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ACK polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;

    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask pre-scaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask pre-scaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	} // if
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     } // while
} /* i2c_start_wait() */

/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  I2C_ACK : device accessible
          I2C_NACK: failed to access device
*************************************************************************/
enum i2c_acks i2c_rep_start(unsigned char address)
{
    return i2c_start( address );
} /* i2c_rep_start() */

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));
} /* i2c_stop() */

/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transferred
  Return:   I2C_ACK : write successful 
            I2C_NACK: write failed
*************************************************************************/
enum i2c_acks i2c_write( unsigned char data )
{	
    uint8_t   twst;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask pre-scaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return I2C_NACK;
	return I2C_ACK;
} /* i2c_write() */

/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));    

    return TWDR;
} /* i2c_readAck() */

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	
    return TWDR;
} /* i2c_readNak() */

/*-----------------------------------------------------------------------------
  Purpose  : Scan all devices on the I2C bus
  Variables: -
 Returns  : -
  ---------------------------------------------------------------------------*/
void i2c_scan(void)
{
	char    s[50]; // needed for printing to serial terminal
	uint8_t x = 0;
	int     i;     // Leave this as an int!
	
	xputs("I2C-scan: ");
	for (i = 0x02; i < 0xff; i+=2)
	{
		if (i2c_start(i) == I2C_ACK)
		{
			sprintf(s,"0x%0x ",i);
		  	xputs(s);
			x++;
		} // if
		i2c_stop();
	} // for
	if (!x) 
	{
	  	xputs("none");
	} // else	
  	xputs("\n");
} // i2c_scan()

uint8_t	ds3231_decode(uint8_t value)
{
	uint8_t decoded = value & 0x7F;
	decoded = (decoded & 0x0F) + 10 * ((decoded & (0xF0)) >> 4);
	return decoded;
} // ds3231_decode()

uint8_t ds3231_decodeH(uint8_t value)
{
  if (value & 0x40) // 12 hour format
    value = (value & 0x0F) + ((value & 0x20) ? 10 : 0);
  else // 24 hour format
    value = (value & 0x0F) + (5 * ((value & 0x30) >> 3));
  return value;
} // ds3231_decodeH()

uint8_t	ds3231_decodeY(uint8_t value)
{
	uint8_t decoded = (value & 0x0F) + 10 * ((value & 0xF0) >> 4);
	return decoded;
} // ds3231_decodeY()

uint8_t ds3231_encode(uint8_t value)
{
	uint8_t encoded = ((value / 10) << 4) + (value % 10);
	return encoded;
} // ds3231_encode()

bool ds3231_read_register(uint8_t reg, uint8_t *value)
{
	bool err;

	err = (i2c_start(DS3231_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(reg) == I2C_NACK);          // write register address to read from
	if (!err) err = (i2c_rep_start(DS3231_ADR | I2C_READ) == I2C_NACK);
	if (!err) *value = i2c_readNak(); // Read register, generate I2C stop condition
	i2c_stop(); // close I2C bus
	return err;
} // ds3231_read_register()
 
bool ds3231_write_register(uint8_t reg, uint8_t value)
{
	bool err;

	err = (i2c_start(DS3231_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(reg) == I2C_NACK);          // write register address to write to
	if (!err) err = (i2c_write(value) == I2C_NACK);        // write value into register
	i2c_stop(); // close I2C bus
	return err;
} // ds3231_write_register()

bool ds3231_gettime(Time *p)
{
	bool err;
	uint8_t reg;

	err = (i2c_start(DS3231_ADR | I2C_WRITE) == I2C_NACK); // generate I2C start + output address to I2C bus
	if (!err) err = (i2c_write(REG_SEC) == I2C_NACK); // seconds register is first register to read
	if (!err) err = (i2c_rep_start(DS3231_ADR | I2C_READ) == I2C_NACK);
	if (!err)
	{
		reg     = i2c_readAck();      // Read SECONDS register
		p->sec  = ds3231_decode(reg);
		reg     = i2c_readAck();      // Read MINUTES register
		p->min  = ds3231_decode(reg);
		reg     = i2c_readAck();      // Read HOURS register
		p->hour = ds3231_decodeH(reg);
		p->dow  = i2c_readAck();      // Read DOW register
		reg     = i2c_readAck();      // Read DAY register
		p->date = ds3231_decode(reg);
		reg     = i2c_readAck();      // Read MONTH register
		p->mon  = ds3231_decode(reg);
		reg     = i2c_readNak();      // Read YEAR register, generate I2C stop condition
		p->year = 2000 + ds3231_decodeY(reg);
	} // if
	else 
	{   // in case of error
		p->sec = p->min  = p->hour = p->year = 0; 
		p->dow = p->date = p->mon  = 1;
	} // else	
	i2c_stop(); // close I2C bus
	return err;
} // ds3231_gettime()

void ds3231_settime(uint8_t hour, uint8_t min, uint8_t sec)
{
	if (((hour >= 0) && (hour < 24)) && ((min >= 0) && (min < 60)) && ((sec >= 0) && (sec < 60)))
	{
		ds3231_write_register(REG_HOUR, ds3231_encode(hour));
		ds3231_write_register(REG_MIN , ds3231_encode(min));
		ds3231_write_register(REG_SEC , ds3231_encode(sec));
	} // if	
} // ds3231_settime()

// 1=Monday, 2=Tuesday, 3=Wednesday, 4=Thursday, 5=Friday, 6=Saturday, 7=Sunday
uint8_t ds3231_calc_dow(uint8_t date, uint8_t mon, uint16_t year)
{
	uint32_t JND = date + ((153L * (mon + 12 * ((14 - mon) / 12) - 3) + 2) / 5)
	+ (365L * (year + 4800L - ((14 - mon) / 12)))
	+ ((year + 4800L - ((14 - mon) / 12)) / 4)
	- ((year + 4800L - ((14 - mon) / 12)) / 100)
	+ ((year + 4800L - ((14 - mon) / 12)) / 400)
	- 32044L;
	return (JND % 7);
} // ds3231_calc_dow()

void ds3231_setdate(uint8_t date, uint8_t mon, uint16_t year)
{
	uint8_t dow;
	
	if (((date > 0) && (date <= 31)) && ((mon > 0) && (mon <= 12)) && ((year >= 2000) && (year < 3000)))
	{
		dow = ds3231_calc_dow(date, mon, year);
		ds3231_write_register(REG_DOW,dow);
		year -= 2000;
		ds3231_write_register(REG_YEAR, ds3231_encode(year));
		ds3231_write_register(REG_MON , ds3231_encode(mon));
		ds3231_write_register(REG_DATE, ds3231_encode(date));
	} // if
} // ds3231_setdate()

void ds3231_setdow(uint8_t dow)
{
	if ((dow > 0) && (dow < 8))
		ds3231_write_register(REG_DOW, dow);
} // ds3231_setdow() 

// Returns the Temperature in a Q8.2 format
int16_t ds3231_gettemp(void)
{
	bool err;
	uint8_t msb,lsb;
	int16_t retv;
	
	err    = ds3231_read_register(REG_TEMPM, &msb);
	retv   = msb;
	retv <<= 2; // SHL 2
	if (!err) err = ds3231_read_register(REG_TEMPL, &lsb);
	retv  |= (lsb >> 6);
	if (retv & 0x0200)
	{   // sign-bit is set
		retv &= ~0x0200; // clear sign bit
		retv = -retv;    // 2-complement
	} // if
	if (!err) return retv;
	else      return 0;
} // ds3231_gettemp()
