/******************************************************************************
SparkFun_Qwiic_Button.cpp
SparkFun Qwiic Button/Switch Library Source File
Fischer Moseley @ SparkFun Electronics
Original Creation Date: July 24, 2019

This file implements the QwiicButton class, prototyped in SparkFun_Qwiic_Button.h

Development environment specifics:
	IDE: Arduino 1.8.9
	Hardware Platform: Arduino Uno/SparkFun Redboard
	Qwiic Button Version: 1.0.0
    Qwiic Switch Version: 1.0.0

This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/
#include <iostream>
#include "Qwiic_Button.h"

const char* register_names_QwiicButton[0x1E] = {
	"ID", 						// 0x00
    "FIRMWARE_MINOR", 			// 0x01
    "FIRMWARE_MAJOR", 			// 0x02
    "BUTTON_STATUS", 			// 0x03
    "INTERRUPT_CONFIG", 		// 0x04
    "BUTTON_DEBOUNCE_TIME", 	// 0x05
	"NDEF", 					// 0x06
    "PRESSED_QUEUE_STATUS", 	// 0x07	
    "PRESSED_QUEUE_FRONT", 		// 0x08
	"NDEF", 					// 0x09
 	"NDEF", 					// 0x0A
 	"NDEF", 					// 0x0B
    "PRESSED_QUEUE_BACK", 		// 0x0C
 	"NDEF", 					// 0x0D
 	"NDEF", 					// 0x0E
 	"NDEF", 					// 0x0F
    "CLICKED_QUEUE_STATUS", 	// 0x10
    "CLICKED_QUEUE_FRONT", 		// 0x11
 	"NDEF", 					// 0x12
 	"NDEF", 					// 0x13
 	"NDEF", 					// 0x14
    "CLICKED_QUEUE_BACK", 		// 0x15
 	"NDEF", 					// 0x16
 	"NDEF", 					// 0x17
 	"NDEF", 					// 0x18
    "LED_BRIGHTNESS", 			// 0x19
    "LED_PULSE_GRANULARITY",	// 0x1A
    "LED_PULSE_CYCLE_TIME",		// 0x1B
	"NDEF", 					// 0x1C
    "LED_PULSE_OFF_TIME",		// 0x1D
};

/*-------------------------------- Device Status ------------------------*/

bool QwiicButton::begin(uint8_t address)
{
    setupI2CRPi(address, register_names_QwiicButton);

    //return true if the device is connected and the device ID is what we expect
    return (isConnected() && checkDeviceID());
}

bool QwiicButton::isConnected()
{
    return true;
}

uint8_t QwiicButton::deviceID()
{
    return readSingleRegister(ID); //read and return the value in the ID register
}

bool QwiicButton::checkDeviceID()
{
    // return ((deviceID() == DEV_ID_SW) || (deviceID() == DEV_ID_BTN)); //Return true if the device ID matches either the button or the switch
    return (deviceID() == DEV_ID); //Return true if the device ID matches
}

uint8_t QwiicButton::getDeviceType()
{
    if (isConnected())
    { //only try to get the device ID if the device will acknowledge
        uint8_t id = deviceID();
        if (id == DEV_ID)
            return 1;
        // if (id == DEV_ID_SW)
        //     return 2;
    }
    return 0;
}

uint16_t QwiicButton::getFirmwareVersion()
{
    uint16_t version = (readSingleRegister(FIRMWARE_MAJOR)) << 8;
    version |= readSingleRegister(FIRMWARE_MINOR);
    return version;
}


/*------------------------------ Button Status ---------------------- */
bool QwiicButton::isPressed()
{
    statusRegisterBitField statusRegister;
    statusRegister.byteWrapped = readSingleRegister(BUTTON_STATUS);
    return statusRegister.isPressed;
}

bool QwiicButton::hasBeenClicked()
{
    statusRegisterBitField statusRegister;
    statusRegister.byteWrapped = readSingleRegister(BUTTON_STATUS);
    return statusRegister.hasBeenClicked;
}

uint16_t QwiicButton::getDebounceTime()
{
    return readDoubleRegister(BUTTON_DEBOUNCE_TIME);
}

uint8_t QwiicButton::setDebounceTime(uint16_t time)
{
    return writeDoubleRegisterWithReadback(BUTTON_DEBOUNCE_TIME, time);
}

/*------------------- Interrupt Status/Configuration ---------------- */
uint8_t QwiicButton::enablePressedInterrupt()
{
    interruptConfigBitField interruptConfigure;
    interruptConfigure.byteWrapped = readSingleRegister(INTERRUPT_CONFIG);
    interruptConfigure.pressedEnable = 1;
    return writeSingleRegisterWithReadback(INTERRUPT_CONFIG, interruptConfigure.byteWrapped);
}

uint8_t QwiicButton::disablePressedInterrupt()
{
    interruptConfigBitField interruptConfigure;
    interruptConfigure.byteWrapped = readSingleRegister(INTERRUPT_CONFIG);
    interruptConfigure.pressedEnable = 0;
    return writeSingleRegisterWithReadback(INTERRUPT_CONFIG, interruptConfigure.byteWrapped);
}

uint8_t QwiicButton::enableClickedInterrupt()
{
    interruptConfigBitField interruptConfigure;
    interruptConfigure.byteWrapped = readSingleRegister(INTERRUPT_CONFIG);
    interruptConfigure.clickedEnable = 1;
    return writeSingleRegisterWithReadback(INTERRUPT_CONFIG, interruptConfigure.byteWrapped);
}

uint8_t QwiicButton::disableClickedInterrupt()
{
    interruptConfigBitField interruptConfigure;
    interruptConfigure.byteWrapped = readSingleRegister(INTERRUPT_CONFIG);
    interruptConfigure.clickedEnable = 0;
    return writeSingleRegisterWithReadback(INTERRUPT_CONFIG, interruptConfigure.byteWrapped);
}

bool QwiicButton::available()
{
    statusRegisterBitField buttonStatus;
    buttonStatus.byteWrapped = readSingleRegister(BUTTON_STATUS);
    return buttonStatus.eventAvailable;
}

uint8_t QwiicButton::clearEventBits()
{
    statusRegisterBitField buttonStatus;
    buttonStatus.byteWrapped = readSingleRegister(BUTTON_STATUS);
    buttonStatus.isPressed = 0;
    buttonStatus.hasBeenClicked = 0;
    buttonStatus.eventAvailable = 0;
    return writeSingleRegisterWithReadback(BUTTON_STATUS, buttonStatus.byteWrapped);
}

uint8_t QwiicButton::resetInterruptConfig()
{
    interruptConfigBitField interruptConfigure;
    interruptConfigure.pressedEnable = 1;
    interruptConfigure.clickedEnable = 1;
    return writeSingleRegisterWithReadback(INTERRUPT_CONFIG, interruptConfigure.byteWrapped);
    statusRegisterBitField buttonStatus;
    buttonStatus.eventAvailable = 0;
    return writeSingleRegisterWithReadback(BUTTON_STATUS, buttonStatus.byteWrapped);
}

/*------------------------- Queue Manipulation ---------------------- */
//pressed queue manipulation
bool QwiicButton::isPressedQueueFull()
{
    queueStatusBitField pressedQueueStatus;
    pressedQueueStatus.byteWrapped = readSingleRegister(PRESSED_QUEUE_STATUS);
    return pressedQueueStatus.isFull;
}

bool QwiicButton::isPressedQueueEmpty()
{
    queueStatusBitField pressedQueueStatus;
    pressedQueueStatus.byteWrapped = readSingleRegister(PRESSED_QUEUE_STATUS);
    return pressedQueueStatus.isEmpty;
}

unsigned long QwiicButton::timeSinceLastPress()
{
    return readQuadRegister(PRESSED_QUEUE_FRONT);
}

unsigned long QwiicButton::timeSinceFirstPress()
{
    return readQuadRegister(PRESSED_QUEUE_BACK);
}

unsigned long QwiicButton::popPressedQueue()
{
    unsigned long tempData = timeSinceFirstPress(); //grab the oldest value on the queue

    queueStatusBitField pressedQueueStatus;
    pressedQueueStatus.byteWrapped = readSingleRegister(PRESSED_QUEUE_STATUS);
    pressedQueueStatus.popRequest = 1;
    writeSingleRegister(PRESSED_QUEUE_STATUS, pressedQueueStatus.byteWrapped); //remove the oldest value from the queue

    return tempData; //return the value we popped
}

//clicked queue manipulation
bool QwiicButton::isClickedQueueFull()
{
    queueStatusBitField clickedQueueStatus;
    clickedQueueStatus.byteWrapped = readSingleRegister(CLICKED_QUEUE_STATUS);
    return clickedQueueStatus.isFull;
}

bool QwiicButton::isClickedQueueEmpty()
{
    queueStatusBitField clickedQueueStatus;
    clickedQueueStatus.byteWrapped = readSingleRegister(CLICKED_QUEUE_STATUS);
    return clickedQueueStatus.isEmpty;
}

unsigned long QwiicButton::timeSinceLastClick()
{
    return readQuadRegister(CLICKED_QUEUE_FRONT);
}

unsigned long QwiicButton::timeSinceFirstClick()
{
    return readQuadRegister(CLICKED_QUEUE_BACK);
}

unsigned long QwiicButton::popClickedQueue()
{
    unsigned long tempData = timeSinceFirstClick();
    queueStatusBitField clickedQueueStatus;
    clickedQueueStatus.byteWrapped = readSingleRegister(CLICKED_QUEUE_STATUS);
    clickedQueueStatus.popRequest = 1;
    writeSingleRegister(CLICKED_QUEUE_STATUS, clickedQueueStatus.byteWrapped);
    return tempData;
}

/*------------------------ LED Configuration ------------------------ */
bool QwiicButton::LEDconfig(uint8_t brightness, uint16_t cycleTime, uint16_t offTime, uint8_t granularity)
{
    bool success = writeSingleRegister(LED_BRIGHTNESS, brightness);
    success &= writeSingleRegister(LED_PULSE_GRANULARITY, granularity);
    success &= writeDoubleRegister(LED_PULSE_CYCLE_TIME, cycleTime);
    success &= writeDoubleRegister(LED_PULSE_OFF_TIME, offTime);
    return success;
}

bool QwiicButton::LEDoff()
{
    return LEDconfig(0, 0, 0);
}

bool QwiicButton::LEDon(uint8_t brightness)
{
    return LEDconfig(brightness, 0, 0);
}

/*------------------------- Internal I2C Abstraction ---------------- */

uint8_t QwiicButton::readSingleRegister(Qwiic_Button_Register reg)
{
    return readRegister(reg);
}

uint16_t QwiicButton::readDoubleRegister(Qwiic_Button_Register reg)
{ //little endian
    return readRegister(reg, 2);
}

unsigned long QwiicButton::readQuadRegister(Qwiic_Button_Register reg)
{
    unsigned long ul_data = 0;
    ul_data = readRegister(reg+2, 2);
    ul_data <<= 16;
    ul_data += readRegister(reg, 2);
    return ul_data;
}

bool QwiicButton::writeSingleRegister(Qwiic_Button_Register reg, uint8_t data)
{
    if (writeRegister(reg, data) > 0)
        return true;
    return false;
}

bool QwiicButton::writeDoubleRegister(Qwiic_Button_Register reg, uint16_t data)
{
    if (writeRegister(reg, data, 2) > 0)
        return true;
    return false;
}

uint8_t QwiicButton::writeSingleRegisterWithReadback(Qwiic_Button_Register reg, uint8_t data)
{
    if (writeSingleRegister(reg, data))
        return 1;
    if (readSingleRegister(reg) != data)
        return 2;
    return 0;
}

uint8_t QwiicButton::writeDoubleRegisterWithReadback(Qwiic_Button_Register reg, uint16_t data)
{
    if (writeDoubleRegister(reg, data))
        return 1;
    if (readDoubleRegister(reg) != data)
        return 2;
    return 0;
}
