/*
  This is a library for the Qwiic MicroPressure Sensor, which can read from 0 to 25 PSI.
  By: Alex Wende
  Date: July 2020 
  License: This code is public domain but you buy me a beer if you use this and 
  we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
 */
#include <math.h>
#include <iostream>

#ifdef WIN
#define __builtin_bswap32(x)  _byteswap_ushort(x)
#endif

#include "Qwiic_MicroPressure.h"

const char* register_names_Qwiic_MicroPressure[] = {
    "NDEF"	// 0x00
};

/* Constructor and sets default values.
   - (Optional) eoc_pin, End of Conversion indicator. Default: -1 (skip)
   - (Optional) rst_pin, Reset pin for MPR sensor. Default: -1 (skip)
   - minimumPSI, minimum range value of the sensor (in PSI). Default: 0
   - maximumPSI, maximum range value of the sensor (in pSI). Default: 25
*/
Qwiic_MicroPressure::Qwiic_MicroPressure(int8_t eoc_pin, int8_t rst_pin, uint8_t minimumPSI, uint8_t maximumPSI)
{
  _eoc = eoc_pin;
  _rst = rst_pin;
  _minPsi = minimumPSI;
  _maxPsi = maximumPSI;
}

/* Initialize hardware
  - deviceAddress, I2C address of the sensor. Default: 0x18
  - wirePort, sets the I2C bus used for communication. Default: Wire
  
  - Returns 0/1: 0: sensor not found, 1: sensor connected
*/
bool Qwiic_MicroPressure::begin(uint8_t deviceAddress)
{
  uint8_t error = setupI2CRPi(deviceAddress, register_names_Qwiic_MicroPressure);
  if(error > 0) 
    return true;
  else           
    return false;
}

/* Read the status byte of the sensor
  - Returns status byte
*/
uint8_t Qwiic_MicroPressure::readStatus(void)
{
  return (uint8_t)(readBytes() & 0xFF);
}

/* Read the Pressure Sensor Reading
 - (optional) Pressure_Units, can return various pressure units. Default: PSI
   Pressure Units available:
     - PSI: Pounds per Square Inch
	 - PA: Pascals
	 - KPA: Kilopascals
	 - TORR
	 - INHG: Inch of Mercury
	 - ATM: Atmospheres
	 - BAR
*/
double Qwiic_MicroPressure::readPressure(Pressure_Units units){
  uint8_t status = 0;
  uint8_t cmd[] = { 0xAA, 0, 0 };
  writeBytes(cmd, 3);
  //writeRegister(0xAA, 0, 2);
  // Wait for new pressure reading available
  if(_eoc != -1) // Use GPIO pin if defined
  {
//    while(!digitalRead(_eoc)) {
      sleep_ms(1);
//    }
    status = readStatus();
  }
  else // Check status byte if GPIO is not defined
  {
    do {
      sleep_ms(1);
      status = readStatus();
    }
    while ((status & BUSY_FLAG) && (status != 0xFF));
  }
  //check memory integrity and math saturation bits
  if((status & INTEGRITY_FLAG) || (status & MATH_SAT_FLAG))  {
    return NAN;
  }  
  //read 24-bit pressure, clear first, status byte 
  uint32_t reading = __builtin_bswap32(readBytes(4)) & 0x00FFFFFF; 
  //printf("reading %06x\n", reading);
  
  //convert from 24-bit to float psi value
  double pressure;
  pressure = (reading - OUTPUT_MIN) * (_maxPsi - _minPsi);
  pressure = (pressure / (OUTPUT_MAX - OUTPUT_MIN)) + _minPsi;

  if(units == PSI)       return pressure; //PSI
  else if(units == PA)   return pressure*6894.7573; //Pa (Pascal)
  else if(units == KPA)  return pressure*6.89476;   //kPa (kilopascal)
  else if(units == TORR) return pressure*51.7149;   //torr (mmHg)
  else if(units == INHG) return pressure*2.03602;   //inHg (inch of mercury)
  else if(units == ATM)  return pressure*0.06805;   //atm (atmosphere)
  else if(units == BAR)  return pressure*0.06895;   //bar
  else                   return pressure; //PSI
}
