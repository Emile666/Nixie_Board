//
//    FILE: dht.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.22
// PURPOSE: DHT Temperature & Humidity Sensor library for Arduino
//     URL: http://arduino.cc/playground/Main/DHTLib
//
// HISTORY:
// 0.1.22 Adapted by Emile for Atmel Studio and DHT22 only
// 0.1.21 replace delay with delayMicroseconds() + small fix
// 0.1.20 Reduce footprint by using uint8_t as error codes. (thanks to chaveiro)
// 0.1.19 masking error for DHT11 - FIXED (thanks Richard for noticing)
// 0.1.18 version 1.16/17 broke the DHT11 - FIXED
// 0.1.17 replaced micros() with adaptive loopcount
//        removed DHTLIB_INVALID_VALUE
//        added  DHTLIB_ERROR_CONNECT
//        added  DHTLIB_ERROR_ACK_L  DHTLIB_ERROR_ACK_H
// 0.1.16 masking unused bits (less errors); refactored bits[]
// 0.1.15 reduced # micros calls 2->1 in inner loop.
// 0.1.14 replace digital read with faster (~3x) code => more robust low MHz machines.
//
// 0.1.13 fix negative temperature
// 0.1.12 support DHT33 and DHT44 initial version
// 0.1.11 renamed DHTLIB_TIMEOUT
// 0.1.10 optimized faster WAKEUP + TIMEOUT
// 0.1.09 optimize size: timeout check + use of mask
// 0.1.08 added formula for timeout based upon clockspeed
// 0.1.07 added support for DHT21
// 0.1.06 minimize footprint (2012-12-27)
// 0.1.05 fixed negative temperature bug (thanks to Roseman)
// 0.1.04 improved readability of code using DHTLIB_OK in code
// 0.1.03 added error values for temp and humidity when read failed
// 0.1.02 added error codes
// 0.1.01 added support for Arduino 1.0, fixed typos (31/12/2011)
// 0.1.00 by Rob Tillaart (01/04/2011)
//
// inspired by DHT11 library
// Released to the public domain
//
#include <math.h>
#include <avr/io.h>
#include "dht22.h"
#include "delay.h"

uint8_t bits[5];  // buffer to receive data

int8_t dht22_read_sensor(void)
{
    uint8_t  mask      = 128; // bit-mask
    uint8_t  i, idx    = 0;   // loop counter, byte-number
    uint8_t  data      = 0;   // data-byte read from DHT22
    uint8_t  state     = 0;   // State of DHT22 pin
    uint8_t  pstate    = 0;   // Previous value of state
    uint16_t zeroLoop  = DHTLIB_TIMEOUT;
	uint16_t loopCount = DHTLIB_TIMEOUT * 2; // 200 usec. max.
    uint16_t delta     = 0;
	uint8_t  lzeroBits = 40 - DHTLIB_DHT_LEADING_ZEROS; // reverse counting...

    // REQUEST SAMPLE
	DDRB  |=  DHT22_PIN; // Set as output port
	PORTB &= ~DHT22_PIN; // write a 0 
	delay_msec(DHTLIB_DHT_WAKEUP); // delay 1 msec.
	PORTB |= DHT22_PIN;  // write a 1
	DDRB  &= ~DHT22_PIN; // set to input again     

    while ((PINB & DHT22_PIN))
    {
        if (--loopCount == 0) return DHTLIB_ERROR_CONNECT;
    } // while

    // GET ACKNOWLEDGE or TIMEOUT
    loopCount = DHTLIB_TIMEOUT;
    while (!(PINB & DHT22_PIN))  // T-rel
    {
        if (--loopCount == 0) return DHTLIB_ERROR_ACK_L;
    } // while

    loopCount = DHTLIB_TIMEOUT;
    while ((PINB & DHT22_PIN))  // T-reh
    {
        if (--loopCount == 0) return DHTLIB_ERROR_ACK_H;
    } // while

    loopCount = DHTLIB_TIMEOUT;
    // READ THE OUTPUT - 40 BITS => 5 BYTES
    for (i = 40; i != 0; )
    {
        state = (PINB & DHT22_PIN); // WAIT FOR FALLING EDGE
        if (!state && pstate)
        {   // Falling edge detected
            if (i > lzeroBits) // DHT22 first 6 bits are all zero (DHT11 only 1)
            {
                if (zeroLoop > loopCount) zeroLoop = loopCount; // min(zeroLoop, loopCount)
                delta = (DHTLIB_TIMEOUT - zeroLoop) / 4;
            } // if
            else if ( loopCount <= (zeroLoop - delta) ) // long -> one
            {
                data |= mask;
            } // else
            mask >>= 1;
            if (mask == 0)   // next byte
            {
                mask        = 128;
                bits[idx++] = data;
                data        = 0;
            } // if
            --i; // next bit
            loopCount = DHTLIB_TIMEOUT; // reset timeout flag
        } // if
        pstate = state; // remember previous state of DHT22 pin
        // Check timeout
        if (--loopCount == 0)
        {
            return DHTLIB_ERROR_TIMEOUT;
        } // if
    } // for
    DDRB  |= DHT22_PIN; // set as output again
	PORTB |= DHT22_PIN; // set output pin high
    return DHTLIB_OK;
} // dht22_read_sensor()

int8_t dht22_read(uint16_t *humidity, int16_t *temperature)
{
    int8_t result = dht22_read_sensor(); // Read values

    // these bits are always zero, masking them reduces errors.
    bits[0] &= 0x03;
    bits[2] &= 0x83;

    // CONVERT AND STORE
    *humidity    = ((bits[0] << 8) | bits[1]);
    *temperature = (((bits[2] & 0x7F) << 8) | bits[3]);
    if (bits[2] & 0x80)  // negative temperature
    {
        *temperature = -*temperature;
    } // if

    // TEST CHECKSUM
    uint8_t sum = bits[0] + bits[1] + bits[2] + bits[3];
    if (bits[4] != sum)
    {
        return DHTLIB_ERROR_CHECKSUM;
    } // if
    return result;
} // dht22_read()

// delta max = 0.6544 wrt dewPoint()
// 5x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
// humidity: E-1 %, Celsius: E-1 Celsius
int16_t dht22_dewpoint(uint16_t humidity, int16_t celsius)
{
	double a = 17.271;
	double b = 2377.0; // x 10
	double temp, td;
	
	temp = (a * celsius) / (b + celsius) + log(humidity/1000.0);
	td = (b * temp) / (a - temp); // x 10
	return (int16_t)td;
} // dht22_dewpoint()