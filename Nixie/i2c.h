/*==================================================================
  File Name    : $Id: i2c.h,v 1.1 2016/05/07 09:37:27 Emile Exp $
  Function name: -
  Author       : Peter Fleury <pfleury@gmx.ch> http://jump.to/fleury
  ------------------------------------------------------------------
  Purpose : This is the header-file for the I2C master interface (i2c.c)
  ------------------------------------------------------------------
  $Log: i2c.h,v $
  Revision 1.1  2016/05/07 09:37:27  Emile
  - Project restructure

  Revision 1.1.1.1  2016/05/07 08:50:25  Emile
  - Initial version for 1st time check-in to CVS.

  ================================================================== */ 
#ifndef _I2C_H
#define _I2C_H   1
#include <avr/io.h>
#include <stdbool.h>

//-------------------------------------------
// Set the prescaler (TWSR register) to 0x01, 
// so that the prescaler value equals 4.
// 100 kHz: TWBR = 18 (should be > 10)
//  10 kHz: TWBR = 198 (should be < 255)
//
// For 400 kHz: TWSR should be set to 0x00,
//              so that the prescaler equals 1!
//-------------------------------------------
#define SCL_CLK_10KHZ  (((F_CPU/10000)-16)/8)
#define SCL_CLK_20KHZ  (((F_CPU/20000)-16)/8)
#define SCL_CLK_30KHZ  (((F_CPU/30000)-16)/8)
#define SCL_CLK_40KHZ  (((F_CPU/40000)-16)/8)
#define SCL_CLK_50KHZ  (((F_CPU/50000)-16)/8)
#define SCL_CLK_60KHZ  (((F_CPU/60000)-16)/8)
#define SCL_CLK_70KHZ  (((F_CPU/70000)-16)/8)
#define SCL_CLK_80KHZ  (((F_CPU/80000)-16)/8)
#define SCL_CLK_90KHZ  (((F_CPU/90000)-16)/8)
#define SCL_CLK_100KHZ (((F_CPU/100000)-16)/8)
#define SCL_CLK_400KHZ (((F_CPU/400000)-16)/2)

#if SCL_CLK_100KHZ < 11
#error "SCL_CLK_100KHZ value too small"
#endif
#if SCL_CLK_10KHZ > 255
#error "SCL_CLK_10KHZ value too big"
#endif

#define LSPEED (0)
#define HSPEED (1)

/** defines the data direction (reading from I2C device) in i2c_start(), i2c_rep_start() */
#define I2C_READ    (1)
#define I2C_WRITE   (0)

#define FALSE (0)
#define TRUE  (!FALSE)
#define I2C_RETRIES (3)

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


// DS3231 DOW defines
#define MONDAY		1
#define TUESDAY		2
#define WEDNESDAY	3
#define THURSDAY	4
#define FRIDAY		5
#define SATURDAY	6
#define SUNDAY		7

#define SQW_RATE_1		0
#define SQW_RATE_1K		1
#define SQW_RATE_4K		2
#define SQW_RATE_8K		3

#define OUTPUT_SQW		0
#define OUTPUT_INT		1

typedef struct _Time
{
	
    uint8_t		hour;
	uint8_t		min;
	uint8_t		sec;
	uint8_t		date;
	uint8_t		mon;
	uint16_t	year;
	uint8_t		dow;
} Time;

#define DS3231_ADR  (0xD0)

#define REG_SEC		(0x00)
#define REG_MIN		(0x01)
#define REG_HOUR	(0x02)
#define REG_DOW		(0x03)
#define REG_DATE	(0x04)
#define REG_MON		(0x05)
#define REG_YEAR	(0x06)
#define REG_CON		(0x0e)
#define REG_STATUS	(0x0f)
#define REG_AGING	(0x10)
#define REG_TEMPM	(0x11)
#define REG_TEMPL	(0x12)

// I2C slave HW responds with an ACK (0) or an NACK (1)
enum i2c_acks {I2C_ACK, I2C_NACK};

void          i2c_init(uint8_t clk); // Initializes the I2C Interface. Needs to be called only once
void          i2c_stop(void); // Terminates the data transfer and releases the I2C bus
enum i2c_acks i2c_start(unsigned char addr); // Issues a start condition and sends address and transfer direction
enum i2c_acks i2c_rep_start(unsigned char addr); // Issues a repeated start condition and sends address and transfer direction
void          i2c_start_wait(unsigned char addr); // i2c_start() + If device is busy, use ack polling to wait until device ready
enum i2c_acks i2c_write(unsigned char data); // Send one byte to I2C device
unsigned char i2c_readAck(void); // read one byte from the I2C device, request more data from device
unsigned char i2c_readNak(void); // read one byte from the I2C device, read is followed by a stop condition
unsigned char i2c_read(unsigned char ack); // read one byte from the I2C device
void i2c_scan(void);

//  Implemented as a macro, which calls either i2c_readAck or i2c_readNak
#define i2c_read(ack)  (ack==I2C_ACK) ? i2c_readAck() : i2c_readNak(); 

// Function prototypes for DS3231
bool    ds3231_gettime(Time *p);
void    ds3231_settime(uint8_t hour, uint8_t min, uint8_t sec);
uint8_t ds3231_calc_dow(uint8_t date, uint8_t mon, uint16_t year);
void    ds3231_setdate(uint8_t date, uint8_t mon, uint16_t year);
void    ds3231_setdow(uint8_t dow);
int16_t ds3231_gettemp(void);

#endif
