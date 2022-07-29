#include <iostream>

#include "i2c_RPi.h"
#include "MC34X9.h"

#define MC34X9_CFG_MODE_DEFAULT           MC34X9_MODE_STANDBY
#define MC34X9_CFG_SAMPLE_RATE_DEFAULT    MC34X9_SR_DEFAULT_1000Hz
#define MC34X9_CFG_RANGE_DEFAULT          MC34X9_RANGE_8G

uint8_t CfgRange, CfgFifo;

const char* register_names_MC34X9[0x4B] = {
    "NDEF", 	// 0x00
    "NDEF", 	// 0x01
    "NDEF", 	// 0x02
    "NDEF", 	// 0x03
    "NDEF", 	// 0x04
    "DEV_STAT", // 0x05
    "INTR_CTRL",// 0x06
    "MODE", 	// 0x07
    "SR", 		// 0x08
    "MOTION_CTRL",// 0x09
    "FIFO_STAT",// 0x0A
    "FIFO_RD_P",// 0x0B
    "FIFO_WR_P",// 0x0C
    "XOUT_LSB", // 0x0D
    "XOUT_MSB", // 0x0E
    "YOUT_LSB", // 0x0F
    "YOUT_MSB", // 0x10
    "ZOUT_LSB", // 0x11
    "ZOUT_MSB", // 0x12
    "STATUS", 	// 0x13
    "INTR_STAT",// 0x14
    "NDEF", 	// 0x15
    "NDEF", 	// 0x16
    "NDEF", 	// 0x17
    "PROD", 	// 0x18
    "NDEF", 	// 0x19
    "NDEF", 	// 0x1A
    "NDEF", 	// 0x1B
    "NDEF", 	// 0x1C
    "NDEF", 	// 0x1D
    "NDEF", 	// 0x1E
    "NDEF", 	// 0x1F
    "RANGE_C", 	// 0x20
    "XOFFL", 	// 0x21
    "XOFFH", 	// 0x22
    "YOFFL", 	// 0x23
    "YOFFH", 	// 0x24
    "ZOFFL", 	// 0x25
    "ZOFFH", 	// 0x26
    "XGAIN", 	// 0x27
    "YGAIN", 	// 0x28
    "ZGAIN", 	// 0x29
    "NDEF", 	// 0x2A
    "NDEF", 	// 0x2B
    "NDEF", 	// 0x2C
    "FIFO_CTRL",// 0x2D
    "FIFO_TH", 	// 0x2E
    "FIFO_INTR",// 0x2F
    "FIFO_CTRL_SR2",// 0x30
    "COMM_CTRL",// 0x31
    "NDEF", 	// 0x32
    "GPIO_CTRL",// 0x33
    "NDEF", 	// 0x34
    "NDEF", 	// 0x35
    "NDEF", 	// 0x36
    "NDEF", 	// 0x37
    "NDEF", 	// 0x38
    "NDEF", 	// 0x39
    "NDEF", 	// 0x3A
    "NDEF", 	// 0x3B
    "NDEF", 	// 0x3C
    "NDEF", 	// 0x3D
    "NDEF", 	// 0x3E
    "NDEF", 	// 0x3F
    "TF_THRESH_LSB", 	// 0x40
    "TF_THRESH_MSB", 	// 0x41
    "TF_DB        ", 	// 0x42
    "AM_THRESH_LSB", 	// 0x43
    "AM_THRESH_MSB", 	// 0x44
    "AM_DB        ", 	// 0x45
    "SHK_THRESH_LSB", 		// 0x46
    "SHK_THRESH_MSB", 		// 0x47
    "PK_P2P_DUR_THRESH_LSB",// 0x48
    "PK_P2P_DUR_THRESH_MSB",// 0x49
    "TIMER_CTRL" 			// 0x4A
};

//Initialize the MC34X9 sensor and set as the default configuration
bool MC34X9::start(uint8_t i2c_addr) {
  setupI2CRPi(i2c_addr, register_names_MC34X9);
  //Init Reset
  reset();
  SetMode(MC34X9_MODE_STANDBY);
    /* Check I2C connection */
  uint8_t id = readRegister(MC34X9_REG_PROD);
  if (id != MC34X9_CHIP_ID)  {
    /* No MC34X9 detected ... return false */
    printf("No MC34X9 detected! Chip ID: %d\n", id);
    return false;
  }
    //Range: 8g
  SetRangeCtrl(MC34X9_CFG_RANGE_DEFAULT);
  //Sampling Rate: 50Hz by default
  SetSampleRate(MC34X9_CFG_SAMPLE_RATE_DEFAULT);
  //Mode: Active
  SetMode(MC34X9_MODE_CWAKE);
  sleep_ms(50);
  return true;
}

void MC34X9::wake() {
  //Set mode as wake
  SetMode(MC34X9_MODE_CWAKE);
}

void MC34X9::stop() {
  //Set mode as Sleep
  SetMode(MC34X9_MODE_STANDBY);
}

//Initial reset
void MC34X9::reset() {
  // Stand by mode
  writeRegister(MC34X9_REG_MODE, MC34X9_MODE_STANDBY);
  sleep_ms(10);
  // power-on-reset
  writeRegister(0x1c, 0x40);
  sleep_ms(50);
  // Disable interrupt
  writeRegister(0x06, 0x00);
  sleep_ms(10);
  // 1.00x Aanalog Gain
  writeRegister(0x2B, 0x00);
  sleep_ms(10);
  // DCM disable
  writeRegister(0x15, 0x00);
  sleep_ms(50);
}

//Set the operation mode
void MC34X9::SetMode(MC34X9_mode_t mode) {
  uint8_t value;

  value = readRegister(MC34X9_REG_MODE);
  value &= 0b11110000;
  value |= mode;
  writeRegister(MC34X9_REG_MODE, value);
}

//Set the range control
void MC34X9::SetRangeCtrl(MC34X9_range_t range) {
  uint8_t value;
  CfgRange = range;
  SetMode(MC34X9_MODE_STANDBY);
  value = readRegister(MC34X9_REG_RANGE_C);
  value &= 0b00000111;
  value |= (range << 4) & 0x70;
  writeRegister(MC34X9_REG_RANGE_C, value);
}

//Set the sampling rate
void MC34X9::SetSampleRate(MC34X9_sr_t sample_rate) {
  uint8_t value;
  SetMode(MC34X9_MODE_STANDBY);
  value = readRegister(MC34X9_REG_SR);
  value &= 0b00000000;
  value |= sample_rate;
  writeRegister(MC34X9_REG_SR, value);
}

// Set Motion feature
void MC34X9::SetMotionCtrl(bool tilt_ctrl,
                           bool flip_ctl,
                           bool anym_ctl,
                           bool shake_ctl,
                           bool tilt_35_ctl) {
  uint8_t CfgMotion = 0;

  if (tilt_ctrl || flip_ctl) {
    _M_DRV_MC34X6_SetTilt_Flip();
    CfgMotion |= (((tilt_ctrl || flip_ctl) & 0x01) << MC34X9_TILT_FEAT);
  }
  if (anym_ctl) {
    _M_DRV_MC34X6_SetAnym();
    CfgMotion |= ((anym_ctl & 0x01) << MC34X9_ANYM_FEAT);
  }
  if (shake_ctl) {
    _M_DRV_MC34X6_SetShake();
    // Also enable anyMotion feature
    CfgMotion |= ((shake_ctl & 0x01) << MC34X9_ANYM_FEAT) | ((shake_ctl & 0x01) << MC34X9_SHAKE_FEAT);
  }
  if (tilt_35_ctl) {
    _M_DRV_MC34X6_SetTilt35();
    // Also enable anyMotion feature
    CfgMotion |= ((tilt_35_ctl & 0x01) << MC34X9_ANYM_FEAT) | ((tilt_35_ctl & 0x01) << MC34X9_TILT35_FEAT);
  }
  writeRegister(MC34X9_REG_MOTION_CTRL, CfgMotion);
}

//Set FIFO feature
void MC34X9::SetFIFOCtrl(MC34X9_fifo_ctl_t fifo_ctl,
                         MC34X9_fifo_mode_t fifo_mode,
                         uint8_t fifo_thr) {
  if (fifo_thr > 31)  //maximum threshold
    fifo_thr = 31;

  SetMode(MC34X9_MODE_STANDBY);
  CfgFifo = (MC34X9_COMB_INT_ENABLE << 3) | ((fifo_ctl << 5) | (fifo_mode << 6)) ;
  writeRegister(MC34X9_REG_FIFO_CTRL, CfgFifo);

  uint8_t CfgFifoThr = fifo_thr;
  writeRegister(MC34X9_REG_FIFO_TH, CfgFifoThr);
}

void MC34X9::SetGerneralINTCtrl() {
  // Gerneral Interrupt setup
  uint8_t CfgGPIOINT = (((MC34X9_INTR_C_IAH_ACTIVE_LOW & 0x01) << 2) // int1
                        | ((MC34X9_INTR_C_IPP_MODE_OPEN_DRAIN & 0x01) << 3)// int1
                        | ((MC34X9_INTR_C_IAH_ACTIVE_LOW & 0x01) << 6)// int2
                        | ((MC34X9_INTR_C_IPP_MODE_OPEN_DRAIN & 0x01) << 7));// int2

  writeRegister(MC34X9_REG_GPIO_CTRL, CfgGPIOINT);
}

//Set interrupt control register
void MC34X9::SetINTCtrl(bool tilt_int_ctrl,
                        bool flip_int_ctl,
                        bool anym_int_ctl,
                        bool shake_int_ctl,
                        bool tilt_35_int_ctl) {
  SetMode(MC34X9_MODE_STANDBY);

  uint8_t CfgINT = (((tilt_int_ctrl & 0x01) << 0)
                    | ((flip_int_ctl & 0x01) << 1)
                    | ((anym_int_ctl & 0x01) << 2)
                    | ((shake_int_ctl & 0x01) << 3)
                    | ((tilt_35_int_ctl & 0x01) << 4)
                    | ((MC34X9_AUTO_CLR_ENABLE & 0x01) << 6));
  writeRegister(MC34X9_REG_INTR_CTRL, CfgINT);

  SetGerneralINTCtrl();
}

//Set FIFO interrupt control register
void MC34X9::SetFIFOINTCtrl(bool fifo_empty_int_ctl,
                            bool fifo_full_int_ctl,
                            bool fifo_thr_int_ctl) {
  SetMode(MC34X9_MODE_STANDBY);

  CfgFifo = CfgFifo
            | (((fifo_empty_int_ctl & 0x01) << 0)
            | ((fifo_full_int_ctl & 0x01) << 1)
            | ((fifo_thr_int_ctl & 0x01) << 2));

  writeRegister(MC34X9_REG_FIFO_CTRL, CfgFifo);
  SetGerneralINTCtrl();
}

//Interrupt handler (clear interrupt flag)
void MC34X9::INTHandler(MC34X9_interrupt_event_t *ptINT_Event) {
  uint8_t value;

  value = readRegister(MC34X9_REG_INTR_STAT);

  ptINT_Event->bTILT    = ((value >> 0) & 0x01);
  ptINT_Event->bFLIP    = ((value >> 1) & 0x01);
  ptINT_Event->bANYM    = ((value >> 2) & 0x01);
  ptINT_Event->bSHAKE   = ((value >> 3) & 0x01);
  ptINT_Event->bTILT_35 = ((value >> 4) & 0x01);

  value &= 0x40;
  writeRegister(MC34X9_REG_INTR_STAT, value);
}

//FIFO Interrupt handler (clear interrupt flag)
void MC34X9::FIFOINTHandler(MC34X9_fifo_interrupt_event_t *ptFIFO_INT_Event) {
  uint8_t value;

  value = readRegister(MC34X9_REG_FIFO_INTR);

  ptFIFO_INT_Event->bFIFO_EMPTY  = ((value >> 0) & 0x01);
  ptFIFO_INT_Event->bFIFO_FULL   = ((value >> 1) & 0x01);
  ptFIFO_INT_Event->bFIFO_THRESH = ((value >> 2) & 0x01);
}

//Get the range control
MC34X9_range_t MC34X9::GetRangeCtrl(void) {
  // Read the data format register to preserve bits
  uint8_t value;
  value = readRegister(MC34X9_REG_RANGE_C);
  printf("In GetRangeCtrl(): %d\n", value);
  value &= 0x70;
  return (MC34X9_range_t) (value >> 4);
}

//Get the output sampling rate
MC34X9_sr_t MC34X9::GetSampleRate(void) {
  // Read the data format register to preserve bits
  uint8_t value;
  value = readRegister(MC34X9_REG_SR);
  printf("In GetCWakeSampleRate(): %d\n", value);
  value &= 0b00011111;
  return (MC34X9_sr_t) (value);
}

//Is FIFO empty
bool MC34X9::IsFIFOEmpty(void) {
  // Read the data format register to preserve bits
  uint8_t value;
  value = readRegister(MC34X9_REG_FIFO_STAT);
  value &= 0x01;
  //Serial.println("FIFO_Status");
  //Serial.println(value, HEX);
  if (value ^ 0x01)
    return false;	//Not empty
  else {
    return true;  //Is empty
  }
}

//Read the raw counts and SI units measurement data
MC34X9_acc_t MC34X9::readRawAccel(void) {
  //{2g, 4g, 8g, 16g, 12g}
  float faRange[5] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f};
  // 16bit
  float faResolution = 32768.0f;

  AccRaw.XAxis = readRegister(MC34X9_REG_XOUT_LSB, 2);
  AccRaw.YAxis = readRegister(MC34X9_REG_YOUT_LSB, 2);
  AccRaw.ZAxis = readRegister(MC34X9_REG_ZOUT_LSB, 2);

  AccRaw.XAxis_g = (float) (AccRaw.XAxis) / faResolution * faRange[CfgRange];
  AccRaw.YAxis_g = (float) (AccRaw.YAxis) / faResolution * faRange[CfgRange];
  AccRaw.ZAxis_g = (float) (AccRaw.ZAxis) / faResolution * faRange[CfgRange];

  return AccRaw;
}

// ***MC34X9 dirver motion part***
void MC34X9::M_DRV_MC34X6_SetTFThreshold(uint16_t threshold) {
  uint8_t _bFTThr[2] = {0};

  _bFTThr[0] = (threshold & 0x00ff);
  _bFTThr[1] = ((threshold & 0x7f00) >> 8 );
  // set threshold
  writeRegister(MC34X9_REG_TF_THRESH_LSB, _bFTThr[0]);
  writeRegister(MC34X9_REG_TF_THRESH_MSB, _bFTThr[1]);
}

void MC34X9::M_DRV_MC34X6_SetTFDebounce(uint8_t debounce) {
  // set debounce
    writeRegister(MC34X9_REG_TF_DB, debounce);
}

void MC34X9::M_DRV_MC34X6_SetANYMThreshold(uint16_t threshold) {
  uint8_t _bANYMThr[2] = {0};

  _bANYMThr[0] = (threshold & 0x00ff);
  _bANYMThr[1] = ((threshold & 0x7f00) >> 8 );
  // set threshold
  writeRegister(MC34X9_REG_AM_THRESH_LSB, _bANYMThr[0]);
  writeRegister(MC34X9_REG_AM_THRESH_MSB, _bANYMThr[1]);
}

void MC34X9::M_DRV_MC34X6_SetANYMDebounce(uint8_t debounce) {
    writeRegister(MC34X9_REG_AM_DB, debounce);
}

void MC34X9::M_DRV_MC34X6_SetShakeThreshold(uint16_t threshold) {
  uint8_t _bSHKThr[2] = {0};

  _bSHKThr[0] = (threshold & 0x00ff);
  _bSHKThr[1] = ((threshold & 0xff00) >> 8 );

  // set threshold
  writeRegister(MC34X9_REG_SHK_THRESH_LSB, _bSHKThr[0]);
  writeRegister(MC34X9_REG_SHK_THRESH_MSB, _bSHKThr[1]);
}

void MC34X9::M_DRV_MC34X6_SetShake_P2P_DUR_THRESH(uint16_t threshold, uint8_t shakeCount) {

  uint8_t _bSHKP2PDuration[2] = {0};

  _bSHKP2PDuration[0] = (threshold & 0x00ff);
  _bSHKP2PDuration[1] = ((threshold & 0x0f00) >> 8);
  _bSHKP2PDuration[1] |= ((shakeCount & 0x7) << 4);

  // set peak to peak duration and count
  writeRegister(MC34X9_REG_PK_P2P_DUR_THRESH_LSB, _bSHKP2PDuration[0]);
  writeRegister(MC34X9_REG_PK_P2P_DUR_THRESH_MSB, _bSHKP2PDuration[1]);
}

void MC34X9::M_DRV_MC34X6_SetTILT35Threshold(uint16_t threshold) {
  M_DRV_MC34X6_SetTFThreshold(threshold);
}

void MC34X9::M_DRV_MC34X6_SetTILT35Timer(uint8_t timer) {
  uint8_t value;

  value = readRegister(MC34X9_REG_TIMER_CTRL);
  value &= 0b11111000;
  value |= MC34X9_TILT35_2p0;
  writeRegister(MC34X9_REG_TIMER_CTRL, timer);
}

// Tilt & Flip
void MC34X9::_M_DRV_MC34X6_SetTilt_Flip() {
  // set threshold
  M_DRV_MC34X6_SetTFThreshold(s_bCfgFTThr);
  // set debounce
  M_DRV_MC34X6_SetTFDebounce(s_bCfgFTDebounce);
  return;
}

// AnyMotion
void MC34X9::_M_DRV_MC34X6_SetAnym() {
  // set threshold
  M_DRV_MC34X6_SetANYMThreshold(s_bCfgANYMThr);
  // set debounce
  M_DRV_MC34X6_SetANYMDebounce(s_bCfgANYMDebounce);
  return;
}

// Shake
void MC34X9::_M_DRV_MC34X6_SetShake() {
  // Config anymotion
  _M_DRV_MC34X6_SetAnym();
  // Config shake
  // set threshold
  M_DRV_MC34X6_SetShakeThreshold(s_bCfgShakeThr);
  // set peak to peak duration and count
  M_DRV_MC34X6_SetShake_P2P_DUR_THRESH(s_bCfgShakeP2PDuration, s_bCfgShakeCount);
  return;
}

// Tilt 35
void MC34X9::_M_DRV_MC34X6_SetTilt35() {
  // Config anymotion
  _M_DRV_MC34X6_SetAnym();
  // Config Tilt35
  // set threshold
  M_DRV_MC34X6_SetTILT35Threshold(s_bCfgTILT35Thr);
  //set timer
  M_DRV_MC34X6_SetTILT35Timer(MC34X9_TILT35_2p0);
  return;
}

