#include <stdint.h>
#include <stdio.h>

#include "i2c_RPi.h"

#ifdef WIN
#include <Windows.h>

void sleep_ms(int milliseconds) {
    Sleep(milliseconds);
}

int i2c_RPi::setupI2CRPi(uint8_t addr, const char** reg_names_array) {
    printf("I2C bus setup\n");
    register_names = (char**)reg_names_array;
    i2cAddr = addr;
    return i2cAddr;
}

uint32_t i2c_RPi::readBytes(uint8_t bytesToRead) {
    printf("read %d byte(s)\n", bytesToRead);
    return bytesToRead;
}

int i2c_RPi::writeBytes(uint8_t* data, uint8_t bytesToSend) {
    int ret = -1;
    printf("write %d byte(s): 0x%06X  to  device 0x%02X\n", bytesToSend, (int)*data, i2cAddr);
    return bytesToSend;
    }

// Read data from one or more registers
//  address:      register address to read from
//  bytesToRead : number of bytes to read(1 or 2)
// @returns data: data that has been read

int i2c_RPi::readRegister(uint8_t address, uint8_t bytesToRead) {

    uint8_t data = address;
    printf("read  %d byte(s): 0x%02X from register %d \t%s\n", bytesToRead, data, address, register_names[address]);
    return data;
}

// Writes data to one or more registers
//  address:    register address to write to
//  data :      data to write
//bytesToSend : number of bytes to send(1 or 2)

int i2c_RPi::writeRegister(uint8_t address, uint16_t value, uint8_t bytesToSend) {
    printf("write %d byte(s): 0x%02X  to  register %d \t%s\n", bytesToSend, value, address, register_names[address]);
    return 0;
}

#elif __gnu_linux__ 

// time delay 
#include <unistd.h>

void sleep_ms(int milliseconds) {
    usleep(milliseconds * 1000);
}

// i2c communication
#include <sys/ioctl.h>
// if error, clone https://github.com/WiringPi/WiringPi.git
#include <wiringPiI2C.h> 

int fd;
bool debug_print_i2c = false;

int i2c_RPi::setupI2CRPi(uint8_t addr, const char ** reg_names_array) {
    register_names = (char**)reg_names_array;
    i2cAddr = addr;
    fd = wiringPiI2CSetup(i2cAddr);
    return fd;
}

uint32_t i2c_RPi::readBytes(uint8_t bytesToRead) {
    uint32_t word32  = 0;
    
    int ret = read(fd, (void*)&word32, bytesToRead);
    if(ret != bytesToRead)  {
        printf("ERROR in I2C read from device 0x%02X\n", i2cAddr);
    }
    if (debug_print_i2c)
        printf("read: 0x%X from device 0x%02X\n", word32, i2cAddr);
    return word32;
}

int i2c_RPi::writeBytes(uint8_t* data, uint8_t bytesToSend) {
    
    int ret = write(fd, (void*)data, bytesToSend);
    if (ret != bytesToSend) {
        printf("ERROR %d in I2C write to device 0x%02X\n", ret, i2cAddr);
    }
    if (debug_print_i2c) 
        printf("write %d byte(s): 0x%06X  to  device 0x%02X\n", bytesToSend, (int)*data, i2cAddr);
    return ret;
}

// Read data from one or more registers
//  address:        register address to read from
//  bytesToRead :   number of bytes to read(1 or 2)
// @returns data:   data that has been read

int i2c_RPi::readRegister(uint8_t address, uint8_t bytesToRead) {
    int data = -1;

    if (bytesToRead == 1) {
        data = wiringPiI2CReadReg8(fd, address);
        if (debug_print_i2c)
            printf("read byte:  0x%02X from register 0x%02X %s\n", data, address, register_names[address]);
    }
    else if (bytesToRead == 2) {
        data = wiringPiI2CReadReg16(fd, address);
        if (debug_print_i2c)
            printf("read two byte: 0x%04X from register 0x%02X %s\n", data, address, register_names[address]);
    }
    return data;
}

// Writes data to one or more registers
//  address:    register address to write to
//  data :      data to write
//bytesToSend : number of bytes to send(1 or 2)

int i2c_RPi::writeRegister(uint8_t address, uint16_t value, uint8_t bytesToSend) {
    int ret = -1;

    if (bytesToSend == 1) {
        ret = wiringPiI2CWriteReg8(fd, address, (uint8_t)(value & 0xff));
        if (debug_print_i2c)
            printf("write byte: 0x%02X  to  register 0x%02X %s \n", value, address, register_names[address]);
    }
    else if (bytesToSend == 2) {
        ret = wiringPiI2CWriteReg16(fd, address, value);
        if (debug_print_i2c)
            printf("write two byte: 0x%04X  to  register 0x%02X %s \n", value, address, register_names[address]);
    }
    if (ret < 0) {
        printf("ERROR %d in I2C write to register 0x%02X %s \n", ret, address, register_names[address]);
    }
    return ret;
}
#endif // __gnu_linux__




