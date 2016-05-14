 //
//    FILE: dht.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.22
// PURPOSE: DHT Temperature & Humidity Sensor library for Arduino
//     URL: http://arduino.cc/playground/Main/DHTLib
//
// HISTORY:
// see dht.cpp file
//
#ifndef dht22_h
#define dht22_h

#define DHT_LIB_VERSION "0.1.22"

#define DHTLIB_OK                   ( 0)
#define DHTLIB_ERROR_CHECKSUM       (-1)
#define DHTLIB_ERROR_TIMEOUT        (-2)
#define DHTLIB_ERROR_CONNECT        (-3)
#define DHTLIB_ERROR_ACK_L          (-4)
#define DHTLIB_ERROR_ACK_H          (-5)

#define DHTLIB_DHT11_WAKEUP         (18)
#define DHTLIB_DHT11_LEADING_ZEROS  (1)
#define DHTLIB_DHT_WAKEUP           (1)
#define DHTLIB_DHT_LEADING_ZEROS    (6)

// DHT22 input pin is PORTB bit 4
#define DHT22_PIN                   (0x10)
             
// max timeout is 100 usec.
// For a 16 Mhz proc 100 usec is 1600 clock cycles
// loops using DHTLIB_TIMEOUT use at least 4 clock cycli
// so 100 us takes max 400 loops
// so by dividing F_CPU by 40000 we "fail" as fast as possible
#define DHTLIB_TIMEOUT (F_CPU/40000)

int8_t  dht22_read(uint16_t *humidity, int16_t *temperature);
int16_t dht22_dewpoint(uint16_t humidity, int16_t celsius);

#endif
//
// END OF FILE
//