demo_i2c_GPS_4G_HAT : demo_GPS_4G_HAT_I2C.o\
	kbhit.o i2c_RPi.o\
	sketch_BasicReadings.o sketch_LightWhenPressed.o sketch_MC3479.o\
	MC34X9.o Qwiic_Button.o Qwiic_MicroPressure.o
	gcc -g -Wall -o demo_GPS_4G_HAT_I2C	demo_GPS_4G_HAT_I2C.o kbhit.o i2c_RPi.o sketch_BasicReadings.o sketch_LightWhenPressed.o sketch_MC3479.o MC34X9.o Qwiic_Button.o Qwiic_MicroPressure.o -lwiringPi -lstdc++


demo_GPS_4G_HAT_I2C.o : demo_GPS_4G_HAT_I2C.cpp
	gcc -g -Wall -c demo_GPS_4G_HAT_I2C.cpp

kbhit.o : Drivers/kbhit.cpp
	gcc -g -Wall -c Drivers/kbhit.cpp

i2c_RPi.o : Drivers/i2c_RPi.cpp
	gcc -g -Wall -c Drivers/i2c_RPi.cpp


sketch_BasicReadings.o : Example/sketch_BasicReadings.cpp
	gcc -g -Wall -c Example/sketch_BasicReadings.cpp

sketch_LightWhenPressed.o : Example/sketch_LightWhenPressed.cpp
	gcc -g -Wall -c Example/sketch_LightWhenPressed.cpp

sketch_MC3479.o : Example/sketch_MC3479.cpp
	gcc -g -Wall -c Example/sketch_MC3479.cpp


MC34X9.o : Drivers/MC34X9.cpp
	gcc -g -Wall -c Drivers/MC34X9.cpp

Qwiic_Button.o : Drivers/Qwiic_Button.cpp
	gcc -g -Wall -c Drivers/Qwiic_Button.cpp

Qwiic_MicroPressure.o : Drivers/Qwiic_MicroPressure.cpp
	gcc -g -Wall -c Drivers/Qwiic_MicroPressure.cpp
