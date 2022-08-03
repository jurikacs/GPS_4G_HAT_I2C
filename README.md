**Repository GPS_4G_HAT_I2C structure**

demo_GPS_4G_HAT_I2C.cpp CLI monitor for I2C devices demonstration

 **Drivers**

C++ drivers for Finamon GPS 4G shield onboard and external I2C Qwiic devices.
- I2C HW support on Finamon GPS 4G shield:
  - i2c_RPi.cpp
  - i2c_RPi.h
- Accelerometer MC3419/MC3479
  - MC34X9.cpp
  - MC34X9.h
- SparkFun Qwiic Button Breakout board
  - Qwiic_Button.cpp
  - Qwiic_Button.h
  - Qwiic_Button_registers.h
- SparkFun Qwiic Micro Pressure Breakout board
  - Qwiic_MicroPressure.cpp
  - Qwiic_MicroPressure.h

**Examples**

Basic examples showing how to work with Finamon GPS 4G shield onboard and external I2C Qwiic devices using C++.
  - sketch_BasicReadings.cpp
  - sketch_LightWhenPressed.cpp
  - sketch_MC3479.cpp

**Prerequisites**

  git clone https://github.com/WiringPi/WiringPi.git
  
  cd ~/wiringPi
  ./build
