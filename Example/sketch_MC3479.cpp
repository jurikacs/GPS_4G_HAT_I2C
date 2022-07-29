#include <iostream>

#include "../Drivers/i2c_RPi.h"
#include "../Drivers/MC34X9.h"


// Chip Select & Address
uint8_t chipSelect = 0;
const uint8_t I2CAddress = 0x4c; // MC3479 I2C address

// Arduino interrupt pin
const int INTERRUPT_PIN = 8;

// *** FIFO control ***/
int FIFO_THRE_SIZE = 30;
// FIFO Interrupt
const bool enableFifoThrINT = false;
// For FIFO feature, enable FIFO interrupt will automatically enable FIFO feature
bool enableFIFO = false;

// *** Motion control ***/
// Enabling motion feature below also enables corresponded motion interrupt
const bool enableTILT = false;
const bool enableFLIP = false;
const bool enableANYM = true;
const bool enableSHAKE = false;
const bool enableTILT_35 = false;

// Determine if enable interrupt
const bool interruptEnabled = enableFifoThrINT || enableTILT || enableFLIP || enableANYM || enableSHAKE || enableTILT_35;


MC34X9 MC34X9_acc = MC34X9();

void sensorFIFO()
{
    //Enable FIFO and interrupt
    MC34X9_acc.stop();
    MC34X9_acc.SetSampleRate(MC34X9_SR_50Hz);
    MC34X9_acc.SetFIFOCtrl(MC34X9_FIFO_CTL_ENABLE, MC34X9_FIFO_MODE_WATERMARK, FIFO_THRE_SIZE);
    MC34X9_acc.SetFIFOINTCtrl(false, false, enableFifoThrINT); //Enable FIFO threshold interrupt
    MC34X9_acc.wake();
    printf("Sensor FIFO enable.\n");
}

void sensorMotion() {
    //Enable motion feature and motion interrupt
    MC34X9_acc.stop();
    MC34X9_acc.SetSampleRate(MC34X9_SR_DEFAULT_1000Hz);
    MC34X9_acc.SetMotionCtrl(enableTILT, enableFLIP, enableANYM, enableSHAKE, enableTILT_35);
    MC34X9_acc.SetINTCtrl(enableTILT, enableFLIP, enableANYM, enableSHAKE, enableTILT_35);
    MC34X9_acc.wake();
    printf("Sensor motion enable.\n");
}

void checkRange()
{
    switch (MC34X9_acc.GetRangeCtrl())
    {
    case MC34X9_RANGE_16G:
        printf("Range: +/- 16 g\n");
        break;
    case MC34X9_RANGE_12G:
        printf("Range: +/- 12 g\n");
        break;
    case MC34X9_RANGE_8G:
        printf("Range: +/- 8 g\n");
        break;
    case MC34X9_RANGE_4G:
        printf("Range: +/- 4 g\n");
        break;
    case MC34X9_RANGE_2G:
        printf("Range: +/- 2 g\n");
        break;
    default:
        printf("Range: +/- ?? g\n");
        break;
    }
}

void checkSamplingRate()
{
    printf("Low Power Mode SR\n");
    switch (MC34X9_acc.GetSampleRate())
    {
    case MC34X9_SR_25Hz:
        printf("Output Sampling Rate: 25 Hz\n");
        break;
    case MC34X9_SR_50Hz:
        printf("Output Sampling Rate: 50 Hz\n");
        break;
    case MC34X9_SR_62_5Hz:
        printf("Output Sampling Rate: 62.5 Hz\n");
        break;
    case MC34X9_SR_100Hz:
        printf("Output Sampling Rate: 100 Hz\n");
        break;
    case MC34X9_SR_125Hz:
        printf("Output Sampling Rate: 125 Hz\n");
        break;
    case MC34X9_SR_250Hz:
        printf("Output Sampling Rate: 250 Hz\n");
        break;
    case MC34X9_SR_500Hz:
        printf("Output Sampling Rate: 500 Hz\n");
        break;
    case MC34X9_SR_DEFAULT_1000Hz:
        printf("Output Sampling Rate: 1000 Hz\n");
        break;
    default:
        printf("Output Sampling Rate: ?? Hz\n");
        break;
    }
}

void readAndOutput() {
    // Read the raw sensor data count
    MC34X9_acc_t rawAccel = MC34X9_acc.readRawAccel();
    // Output Count
    printf("X: %6d \tY: %6d \tZ: %6d counts\n", rawAccel.XAxis, rawAccel.YAxis, rawAccel.ZAxis);
    printf("X: %2.3f \tY: %2.3f \tZ: %2.3f m/s^2\n\n",   rawAccel.XAxis_g, rawAccel.YAxis_g, rawAccel.ZAxis_g);
    return;
}

// Interrupt checker: read interrupt register and determine if interrupt happen
bool interruptChecker() {
    // Init interrupt table
    bool retCode = false;
    MC34X9_interrupt_event_t evt_mc34X9 = { 0 };
    MC34X9_fifo_interrupt_event_t fifo_evt_mc34X9 = { 0 };

    // Read interrupt table
    MC34X9_acc.FIFOINTHandler(&fifo_evt_mc34X9);
    MC34X9_acc.INTHandler(&evt_mc34X9);

    // Whether there is interrupt
    uint8_t* iter = (uint8_t*)&evt_mc34X9;
    for (unsigned int i = 0; i < sizeof(MC34X9_interrupt_event_t); i++, iter++) {
        if ((*iter) & 0x01)
            retCode = true;
    }
    iter = (uint8_t*)&fifo_evt_mc34X9;
    for (unsigned int i = 0; i < sizeof(MC34X9_fifo_interrupt_event_t); i++, iter++) {
        if ((*iter) & 0x01)
            retCode = true;
    }

    if (retCode)
        printf("Get interrupt: \n");
    if (enableFIFO) {
        if (fifo_evt_mc34X9.bFIFO_EMPTY) {
            printf("FIFO empty. \n");
        }
        if (fifo_evt_mc34X9.bFIFO_FULL) {
            printf("FIFO full. \n");
        }
        if (fifo_evt_mc34X9.bFIFO_THRESH) {
            printf("FIFO threshold. \n");
        }
    }
    if (evt_mc34X9.bTILT) {
        printf("Tilt. \n");
    }
    if (evt_mc34X9.bFLIP) {
        printf("Flip. \n");
    }
    if (evt_mc34X9.bANYM) {
        printf("Any Motion. \n");
    }
    if (evt_mc34X9.bSHAKE) {
        printf("Shake. \n");
    }
    if (evt_mc34X9.bTILT_35) {
        printf("Tilt 35. \n");
    }
    if (retCode)
        printf("\n");
    return retCode;
}

// Function for enabled interrupt mode
void interruptLoop() {
    if (interruptChecker()) {
        // When interrupt happen
        if (enableFIFO) {
            while (!(MC34X9_acc.IsFIFOEmpty())) {
                // Read and output all data in FIFO
                readAndOutput();
            }
        }
        else {
            readAndOutput();
        }
    }
    return;
}


static void setup(void)
{
    printf("accelerometer MC34X9:\n");
    // Init MC34X9 Object
    if (!MC34X9_acc.start(I2CAddress)) {
        // Invalid chip ID
        return;
    }
    // Check chip setup
    checkRange();
    checkSamplingRate();
    //Test read
    readAndOutput();
    // Enable feature & interrput
    enableFIFO = enableFIFO || enableFifoThrINT;
    if (enableFIFO) {
        sensorFIFO();
    }
    if (enableTILT || enableFLIP || enableANYM || enableSHAKE || enableTILT_35) {
        // Checker: These modes can only be enabled separately.
        int counter = 0;
        if (enableTILT)
            counter++;
        if (enableFLIP)
            counter++;
        if (enableANYM)
            counter++;
        if (enableSHAKE)
            counter++;
        if (enableTILT_35)
            counter++;
        if (counter > 1) {
            // Detected: These modes can only be enabled separately.
            printf("Error: Enable too many motion feature.\n");
            printf("Error: These modes can only be enabled separately.\n");
            // Block here
            while (true);
        }
        sensorMotion();
    }
//dump registers 
//for(int r = MC34X9_REG_DEV_STAT; r <= MC34X9_REG_TIMER_CTRL; r++)   
//  printf("%02X: %02X %s\n", r, MC34X9_acc.readRegister(r), MC34X9_acc.register_names[r]); 
}


static void loop(void)
{
    if (interruptEnabled) {
        interruptLoop();
        sleep_ms(1);
    }
    else {
        // without enable interrupt
        // read and output data periodically
        readAndOutput();
        sleep_ms(100);
    }
    return;
}

void (*setup_MC3479)(void) = setup;
void (*loop_MC3479)(void) = loop;

