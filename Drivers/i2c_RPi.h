#include <stdint.h>
#pragma once

typedef void (*fptr)(void);

#ifdef WIN32
#include <windows.h>
#elif __gnu_linux__ 
#include <unistd.h>
#endif // __gnu_linux__

void	sleep_ms(int milliseconds);

class i2c_RPi
{
public:
	int		 setupI2CRPi(uint8_t addr, const char ** reg_names_array = 0);
	uint32_t readBytes(uint8_t bytesToRead = 1);	// max 4 bytes
	int 	 writeBytes(uint8_t* data, uint8_t bytesToSend = 1);
	int		 readRegister(uint8_t address, uint8_t bytesToRead = 1);
	int		 writeRegister(uint8_t address, uint16_t value, uint8_t bytesToSend = 1);

	char** 	register_names;

private:
	uint8_t	i2cAddr;
	int		fd;
	
};
