/*
  DHT22.cpp - DHT22 sensor library
  Developed by Ben Adams - 2011

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


Humidity and Temperature Sensor DHT22 info found at
http://www.sparkfun.com/products/10167

Version 0.5: 15 Jan 2012 by Craig Ringer
- Updated to build against Arduino 1.0
- Made accessors inline in the header so they can be optimized away

Version 0.4: 24-Jan-2011 by Ben Adams
Added return code constants to keywords.txt
Returns DHT_ERROR_CHECKSUM on check sum mismatch 

Version 0.3: 17-Jan-2011 by Ben Adams
This version reads data
Needs check sum code added at the end of readData

Version 0.2: 16-Jan-2011 by Ben Adams
Changed coding style to match other Arduino libraries.
This version will not read data either!

Version 0.1: 10-Jan-2011 by Ben Adams nethoncho AT gmail.com
First Version is a skeleton. This version will not read data!
Code adapted from the following sources:
The Arduino OneWire lib
http://sheepdogguides.com/arduino/ar3ne1humDHT11.htm

*/

#include "DHT22.h"
#include <Arduino.h>
#include <pins_arduino.h>

extern "C" {
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
}

#define DIRECT_READ(base, mask)		(((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)	((*(base+1)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)	((*(base+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)	((*(base+2)) &= ~(mask))
//#define DIRECT_WRITE_HIGH(base, mask)	((*(base+2)) |= (mask))

// This should be 40, but the sensor is adding an extra bit at the start
#define DHT22_DATA_BIT_COUNT (40+1)

DHT22::DHT22(uint8_t pin)
{
    _bitmask = digitalPinToBitMask(pin);
    _baseReg = portInputRegister(digitalPinToPort(pin));
    _lastReadTime = millis();
    _lastHumidity = DHT22_ERROR_VALUE;
    _lastTemperature = DHT22_ERROR_VALUE;
}

int wait_for_input(int timeout, int level) {
  uint8_t bitmask = _bitmask;
  volatile uint8_t *reg asm("r30") = _baseReg;
  int retryCount = 0;
  const int delayms = 2;
  while(!!DIRECT_READ(reg, bitmask) == !!level)
  {
    if (retryCount > timeout)
    {
      return -1;
    }
    delayMicroseconds(delayms);
    retryCount += delayms;
  }
  return retryCount / 2;
}

//
// Read the 40 bit data stream from the DHT 22
// Store the results in private member data to be read by public member functions
//
DHT22_ERROR_t DHT22::readData()
{
  uint8_t bitmask = _bitmask;
  volatile uint8_t *reg asm("r30") = _baseReg;
  uint8_t retryCount;
  uint8_t bitTimes[DHT22_DATA_BIT_COUNT];
  int currentHumidity;
  int currentTemperature;
  uint8_t checkSum, csPart1, csPart2, csPart3, csPart4;
  unsigned long currentTime;
  int i;

  currentHumidity = 0;
  currentTemperature = 0;
  checkSum = 0;
  currentTime = millis();
  for(i = 0; i < DHT22_DATA_BIT_COUNT; i++)
  {
    bitTimes[i] = 0;
  }

  if(currentTime - _lastReadTime < 2000)
  {
    // Caller needs to wait 2 seconds between each call to readData
    return DHT_ERROR_TOOQUICK;
  }

  // Pin needs to start HIGH, wait until it is HIGH with a timeout
  cli();
  DIRECT_MODE_INPUT(reg, bitmask);
  sei();
  if (wait_for_input(250, 0) < 0)
    return DHT_BUS_HUNG;
  // Send the activate pulse
  cli();
  DIRECT_WRITE_LOW(reg, bitmask);
  DIRECT_MODE_OUTPUT(reg, bitmask); // Output Low
  sei();
  delayMicroseconds(1100); // 1.1 ms
  cli();
  DIRECT_MODE_INPUT(reg, bitmask);	// Switch back to input so pin can float
  sei();
  // Find the start of the ACK Pulse
  if (wait_for_input(40, 0) < 0)
    return DHT_ERROR_NOT_PRESENT;
  // Find the end of the ACK Pulse
  if (wait_for_input(40, 1) < 0)
    return DHT_ERROR_ACK_TOO_LONG;
  // Read the 40 bit data stream
  for(i = 0; i < DHT22_DATA_BIT_COUNT; i++)
  {
    // Find the start of the sync pulse
    if (wait_for_input(50, 0) < 0)
      return DHT_ERROR_SYNC_TIMEOUT;
    // Measure the width of the data pulse
    bitTimes[i] = wait_for_input(80, 0);
    if (bitTimes[i] < 0)
      return DHT_ERROR_DATA_TIMEOUT;
  }
  // Now bitTimes have the number of retries (us *2)
  // that were needed to find the end of each data bit
  // Spec: 0 is 26 to 28 us
  // Spec: 1 is 70 us
  // bitTimes[x] <= 11 is a 0
  // bitTimes[x] >  11 is a 1
  // Note: the bits are offset by one from the data sheet, not sure why
  for(i = 0; i < 16; i++)
  {
    if(bitTimes[i + 1] > 11)
    {
      currentHumidity |= (1 << (15 - i));
    }
  }
  for(i = 0; i < 16; i++)
  {
    if(bitTimes[i + 17] > 11)
    {
      currentTemperature |= (1 << (15 - i));
    }
  }
  for(i = 0; i < 8; i++)
  {
    if(bitTimes[i + 33] > 11)
    {
      checkSum |= (1 << (7 - i));
    }
  }

  // verify the data against the checksum
  csPart1 = currentHumidity >> 8;
  csPart2 = currentHumidity & 0xFF;
  csPart3 = currentTemperature >> 8;
  csPart4 = currentTemperature & 0xFF;
  if(checkSum != ((csPart1 + csPart2 + csPart3 + csPart4) & 0xFF))
  {
    return DHT_ERROR_CHECKSUM;
  }

  // the checksum is valid - now we can update the readings
  _lastReadTime = currentTime;
  _lastHumidity = currentHumidity & 0x7FFF;
  if(currentTemperature & 0x8000)
  {
    // Below zero, non standard way of encoding negative numbers!
    // Convert to native negative format.
    _lastTemperature = -currentTemperature & 0x7FFF;
  }
  else
  {
    _lastTemperature = currentTemperature;
  }

  return DHT_ERROR_NONE;
}

//
// This is used when the millis clock rolls over to zero
//
void DHT22::clockReset()
{
  _lastReadTime = millis();
}
