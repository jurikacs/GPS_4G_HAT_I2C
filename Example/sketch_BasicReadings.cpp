#include <iostream>

#include "../Drivers/Qwiic_MicroPressure.h"

/*
 * Initialize Constructor
 * Optional parameters:
 *  - EOC_PIN: End Of Conversion (defualt: -1)
 *  - RST_PIN: Reset (defualt: -1)
 *  - MIN_PSI: Minimum Pressure (default: 0 PSI)
 *  - MAX_PSI: Maximum Pressure (default: 25 PSI)
 */
//SparkFun_MicroPressure mpr(EOC_PIN, RST_PIN, MIN_PSI, MAX_PSI);
Qwiic_MicroPressure mpr; // Use default values with reset and EOC pins unused


static void setup() {
  // Initalize UART, I2C bus, and connect to the micropressure sensor
  if(!mpr.begin(DEFAULT_ADDRESS))
  {
    printf("Cannot connect to MicroPressure sensor.\n");
  }
}

static void loop() {
  /* The micropressure sensor outputs pressure readings in pounds per square inch (PSI).
     Optionally, if you prefer pressure in another unit, the library can convert the
     pressure reading to: pascals, kilopascals, bar, torr, inches of murcury, and
     atmospheres.
   */
  printf("%6.2f PSI\n",  mpr.readPressure(PSI));     // 4
  printf("%6.0f Pa\n",   mpr.readPressure(PA));      // 1
  printf("%6.2f kPa\n",  mpr.readPressure(KPA));     // 4
  printf("%6.1f torr\n", mpr.readPressure(TORR));    // 3
  printf("%6.2f inHg\n", mpr.readPressure(INHG));    // 4
  printf("%6.2f atm\n",  mpr.readPressure(ATM));     // 6
  printf("%6.2f bar\n\n",mpr.readPressure(BAR));     // 6

  sleep_ms(2000);
}

void (*setup_BasicReadings)(void) = setup;
void (*loop_BasicReadings)(void) = loop;

