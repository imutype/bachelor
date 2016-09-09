#include "MPU9150.h"

#include <Arduino.h>

// Copyright (c) 2014 Chris Pepper

//The Firmware upload code was borrowed/modified from the sparkfun repository, which included this copyright
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

MPU9150::MPU9150(uint8_t address)
    : i2c(0, address) {}

MPU9150::~MPU9150() {}

// https://store.invensense.com/Datasheets/invensense/RM-MPU-9150A-00-v3.0.pdf

uint8_t MPU9150::getDeviceID(){
    return i2c.getBits(MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH);
}

bool MPU9150::isReady(){
    return (getDeviceID() == (i2c.getAddress() >> 1));
}

void MPU9150::initialise(){
    reset();
    delay(20);

    sleep(false);
    clockSelect(MPU6050_CLOCK_PLL_XGYRO); //use the gyro clock as its more reliable
    setGyroFullScaleRange(MPU6050_GYRO_FS_250);
    setAccelFullScaleRange(MPU6050_ACCEL_FS_2);
    setStandbyAccX(true);
    setI2CMasterClock(MPU6050_CLOCK_DIV_400);
    setDigitalLowPassFilter(MPU6050_DLPF_BW_42);
    setSampleRateDivider(4);

    initialiseMagnetometer();

    setFifoReset(true);

    setTemperatureFifo(true);
    setAccelFifo(true);
    setGyroFifo(true);
    setSlave0Fifo(true);

    setInterruptDataReadyEnable(true);
    setEnableFifo(true);
}

void MPU9150::initialiseMagnetometer(){
    //set up slave 0 to read the magnetometor data
    setWaitForExternalSensor(true);
    //read data
    setI2cSlaveRW(0, true);
    setI2cSlaveAddress(0, 0x0C);
    setI2cSlaveRegister(0, 3);
    setI2cSlaveEnable(0, true);
    setI2cSlaveTransactionLength(0, 6);


    //set up slave 1 to request a new magnetometor reading by writing 0x01 to 0xA
    setI2cSlaveAddress(1, 0x0C);
    setI2cSlaveRegister(1, 0x0A);
    setI2cSlaveTransactionLength(1, 1);
    setI2cSlaveEnable(1, true);
    setI2cSlaveDataOut(1, 1);

    //configure update rates
    setI2cMasterDelay(4);
    setI2cSlaveDelay(0, true);
    setI2cSlaveDelay(1, true);

    //Enable the aux i2c bus with MPU9150 as master
    setI2cMasterEnable(true);
}

uint8_t MPU9150::initialiseDMP(){
    reset();
    delay(20);
    sleep(false);

    setMemoryBank(0x10, true, true);
    setMemoryStartAddress(0x06);
//    debug.printf("Hardware Version: %d\r\n", readMemoryByte());

    setMemoryBank(0);
    // check OTP bank valid
    uint8_t otpValid = getOTPBankValid();
    if (!otpValid) return ERR_OTP_INVALID;
//    debug.printf("optValid: %d\r\n", otpValid);

    //Enabling interrupt latch, clear on any read, AUX bypass enabled
    i2c.setByte(MPU6050_RA_INT_PIN_CFG, 0x32);

    if (writeMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE, 0 ,0, true)) {
 //       debug.printf("Success! DMP code written and verified.\r\n");
        if (writeDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {
//            debug.printf("Success! DMP configuration written and verified.\r\n");
            setIntDMPEnabled(true);
            setInterruptFifoOverflowEnable(true);
            setSampleRateDivider(4);
            clockSelect(MPU6050_CLOCK_PLL_XGYRO);
            setDigitalLowPassFilter(MPU6050_DLPF_BW_42);
            setGyroFullScaleRange(MPU6050_GYRO_FS_2000);

            setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);
            setDMPConfig1(0x03);
            setDMPConfig2(0x00);

            unsigned char *update_ptr = (unsigned char*)dmpUpdates;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);

            setFifoReset(true);

            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);

            i2c.setByte(MPU6050_RA_PWR_MGMT_2, 0x00);
            setInterruptAnyReadClear(true);
            setInterruptLatch(true);

            setI2cSlaveRW(0, true);
            setI2cSlaveAddress(0, 0x0C);
            setI2cSlaveRegister(0, 1);
            setI2cSlaveEnable(0, true);
            setI2cSlaveTransactionLength(0, 10);

            //set up slave 1 to request a new magnetometor reading by writing 0x01 to 0xA
            setI2cSlaveAddress(2, 0x0C);
            setI2cSlaveRegister(2, 0x0A);
            setI2cSlaveTransactionLength(2, 1);
            setI2cSlaveEnable(2, true);
            setI2cSlaveDataOut(2, 1);

            //configure update rates
            setI2cMasterDelay(4);
            setI2cSlaveDelay(0, true);
            setI2cSlaveDelay(2, true);

            //Enable the aux i2c bus with MPU9150 as master
            setI2cMasterEnable(true);

            i2c.setByte(MPU6050_RA_INT_PIN_CFG, 0x00);

            // enable I2C master mode and reset DMP/FIFO
            //DEBUG_PRINTLN(F("Enabling I2C master mode..."));
            i2c.setByte( MPU6050_RA_USER_CTRL, 0x20);
            //DEBUG_PRINTLN(F("Resetting FIFO..."));
            i2c.setByte(MPU6050_RA_USER_CTRL, 0x24);
            //DEBUG_PRINTLN(F("Rewriting I2C master mode enabled because...I don't know"));
            i2c.setByte(MPU6050_RA_USER_CTRL, 0x20);
            //DEBUG_PRINTLN(F("Enabling and resetting DMP/FIFO..."));
            i2c.setByte(MPU6050_RA_USER_CTRL, 0xE8);

            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);

            //read?
            update_ptr += update_ptr[2] + 3;
            //stalls?
            //readMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1]);


            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);
            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);

            int fifoCount = 0;
            while ((fifoCount = getFifoCount()) < 46);
            uint8_t buffer[128];
            getFifoBuffer((char *)buffer, fifoCount);
            getInterruptStatus();

            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);

            fifoCount = 0;
            while ((fifoCount = getFifoCount()) < 48);
            getFifoBuffer((char *)buffer, fifoCount);
            getInterruptStatus();
            fifoCount = 0;
            while ((fifoCount = getFifoCount()) < 48);
            getFifoBuffer((char *)buffer, fifoCount);
            getInterruptStatus();

            update_ptr += update_ptr[2] + 3;
            writeMemoryBlock(update_ptr + 3, update_ptr[2], update_ptr[0], update_ptr[1], true);

            setDMPEnabled(false);

 //           debug.printf("finished\r\n");

        }
    }


    return 0;
}

//PWR_MGMT_1 Control Register
//*****************************/
void MPU9150::reset(){
    i2c.setBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}

void MPU9150::sleep(bool state){
    i2c.setBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, state);
}

/*
cycle between sleep mode and waking up to take a single sample of data from
active sensors at a rate determined by LP_WAKE_CTRL (register 108).
*/
void MPU9150::cycleMode(bool state){
    i2c.setBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, state);
}
void MPU9150::disableTemperatureSensor(bool state){
    i2c.setBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, state);
}
void MPU9150::clockSelect(uint8_t clk){
    i2c.setBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, clk);
}

//PWR_MGMT_2 Control Register
//*****************************/
void MPU9150::setCycleWakeFrequency(uint8_t freq){
    i2c.setBits(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT, MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, freq);
}
void MPU9150::setStandbyAccX(bool value){
    i2c.setBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, value);
}
void MPU9150::setStandbyAccY(bool value){
    i2c.setBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, value);
}
void MPU9150::setStandbyAccZ(bool value){
    i2c.setBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, value);
}
void MPU9150::setStandbyGyroX( bool value){
    i2c.setBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, value);
}
void MPU9150::setStandbyGyroY( bool value){
    i2c.setBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, value);
}
void MPU9150::setStandbyGyroZ( bool value){
    i2c.setBit(MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, value);
}

//SMPRT_DIV  Sample Rate Divider
//*****************************/
void MPU9150::setSampleRateDivider(uint8_t value){
    i2c.setByte(MPU6050_RA_SMPLRT_DIV, value);
}

//CONFIG
void MPU9150::setExternalFrameSync(uint8_t value){
    i2c.setBits(MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, value);
}
void MPU9150::setDigitalLowPassFilter(uint8_t value){
    i2c.setBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, value);
}

//GYRO_CONFIG
void MPU9150::setGyroSelfTest(bool value){
    i2c.setBit(MPU6050_RA_GYRO_CONFIG, 7, value); //X
    i2c.setBit(MPU6050_RA_GYRO_CONFIG, 6, value); //Y
    i2c.setBit(MPU6050_RA_GYRO_CONFIG, 5, value); //Z
}

void MPU9150::setGyroFullScaleRange(uint8_t value){
    i2c.setBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, value);
}

//ACCEL_CONFIG
void MPU9150::setAccelSelfTest(bool value){
    i2c.setBit(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, value);
    i2c.setBit(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, value);
    i2c.setBit(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, value);
}
void MPU9150::setAccelFullScaleRange(uint8_t value){
    i2c.setBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT , MPU6050_ACONFIG_AFS_SEL_LENGTH, value);
}

//FIFO_EN
void MPU9150::setTemperatureFifo(bool value){
    i2c.setBit(MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, value);
}
void MPU9150::setGyroFifo(bool value){
    i2c.setBit(MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, value);
    i2c.setBit(MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, value);
    i2c.setBit(MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, value);
}
void MPU9150::setAccelFifo(bool value){
    i2c.setBit(MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, value);
}
void MPU9150::setSlave2Fifo(bool value){
    i2c.setBit(MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, value);
}
void MPU9150::setSlave1Fifo(bool value){
    i2c.setBit(MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, value);
}
void MPU9150::setSlave0Fifo(bool value){
    i2c.setBit(MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, value);
}

//I2C_MST_CTRL
void MPU9150::setMultiMaster(bool value){
    i2c.setBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, value);
}
void MPU9150::setWaitForExternalSensor(bool value){
    i2c.setBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, value);
}
void MPU9150::setSlave3Fifo(bool value){
    i2c.setBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT, value);
}
void MPU9150::setMasterStartStop(bool value){
    i2c.setBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, value);
}
void MPU9150::setI2CMasterClock(uint8_t value){
    i2c.setBits(MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_LENGTH, value);
}

//I2C slaves 0 to 3
//I2C_SLV0_ADDR
void MPU9150::setI2cSlaveRW(uint8_t slave_id, bool value){
    if(slave_id > 3)return;
    i2c.setBit(MPU6050_RA_I2C_SLV0_ADDR + (slave_id * 3), MPU6050_I2C_SLV_RW_BIT, value);
}
void MPU9150::setI2cSlaveAddress(uint8_t slave_id, uint8_t value){
    if(slave_id > 3)return;
    i2c.setBits(MPU6050_RA_I2C_SLV0_ADDR + (slave_id * 3), MPU6050_I2C_SLV_ADDR_BIT, MPU6050_I2C_SLV_ADDR_LENGTH, value);
}
//I2C_SLV0_REG,
void MPU9150::setI2cSlaveRegister(uint8_t slave_id, uint8_t value){
    if(slave_id > 3)return;
    i2c.setByte(MPU6050_RA_I2C_SLV0_REG + (slave_id * 3), value);
}
//I2C_SLV0_CTRL
void MPU9150::setI2cSlaveEnable(uint8_t slave_id, bool value){
    if(slave_id > 3)return;
    i2c.setBit(MPU6050_RA_I2C_SLV0_CTRL + (slave_id * 3), MPU6050_I2C_SLV_EN_BIT, value);
}
void MPU9150::setI2cSlaveByteSwap(uint8_t slave_id, bool value){
    if(slave_id > 3)return;
    i2c.setBit(MPU6050_RA_I2C_SLV0_CTRL + (slave_id * 3), MPU6050_I2C_SLV_BYTE_SW_BIT, value);
}
void MPU9150::setI2cSlaveRegDisable(uint8_t slave_id, bool value){
    if(slave_id > 3)return;
    i2c.setBit(MPU6050_RA_I2C_SLV0_CTRL + (slave_id * 3), MPU6050_I2C_SLV_REG_DIS_BIT, value);
}
void MPU9150::setI2cSlaveByteGrouping(uint8_t slave_id, bool value){
    if(slave_id > 3)return;
    i2c.setBit(MPU6050_RA_I2C_SLV0_CTRL + (slave_id * 3), MPU6050_I2C_SLV_GRP_BIT, value);
}
void MPU9150::setI2cSlaveTransactionLength(uint8_t slave_id, uint8_t value){
    if(slave_id > 3)return;
    i2c.setBits(MPU6050_RA_I2C_SLV0_CTRL + (slave_id * 3), MPU6050_I2C_SLV_LEN_BIT, MPU6050_I2C_SLV_LEN_LENGTH, value);
}
//I2C_SLV0_DO
void MPU9150::setI2cSlaveDataOut(uint8_t slave_id, uint8_t value){
    if(slave_id > 3)return;
    i2c.setByte(MPU6050_RA_I2C_SLV0_DO + slave_id, value);
}
//I2C_MST_DELAY_CTRL
void MPU9150::setI2cSlaveDelay(uint8_t slave_id, uint8_t value){
    i2c.setBit(MPU6050_RA_I2C_MST_DELAY_CTRL, slave_id, value);
}
void MPU9150::setI2cSlaveShadowDelay(uint8_t value){
    i2c.setBit(MPU6050_RA_I2C_MST_DELAY_CTRL, 7, value);
}

//I2C slave4
//I2C_SLV4_ADDR
void MPU9150::setI2cSlave4RW( bool value){
    i2c.setBit(MPU6050_RA_I2C_SLV4_ADDR, MPU6050_I2C_SLV4_RW_BIT, value);
}
void MPU9150::setI2cSlave4Address( uint8_t value){
    i2c.setBits(MPU6050_RA_I2C_SLV4_ADDR, MPU6050_I2C_SLV4_ADDR_BIT, MPU6050_I2C_SLV4_ADDR_LENGTH, value);
}
//I2C_SLV4_REG,
void MPU9150::setI2cSlave4Register(uint8_t value){
    i2c.setByte(MPU6050_RA_I2C_SLV4_REG, value);
}
//I2C_SLV4_DO
void MPU9150::setI2cSlave4DataOut(uint8_t value){
    i2c.setByte(MPU6050_RA_I2C_SLV4_DO, value);
}

//I2C_SLV4_CTRL
void MPU9150::setI2cSlave4Enable(bool value){
    i2c.setBit(MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT, value);
}

void MPU9150::setI2cSlave4IntEnable(bool value){
    i2c.setBit(MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT, value);
}

void MPU9150::setI2cSlave4RegDisable(bool value){
    i2c.setBit(MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_REG_DIS_BIT, value);
}

void MPU9150::setI2cMasterDelay(uint8_t value){
    i2c.setBits(MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT, MPU6050_I2C_SLV4_MST_DLY_LENGTH, value);
}

uint8_t MPU9150::getI2cSlave4Di(){
    return i2c.getByte(MPU6050_RA_I2C_SLV4_DI);
}

//I2C_MST_STATUS
bool MPU9150::setI2cPassthrough(){
    return i2c.getBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_PASS_THROUGH_BIT);
}
bool MPU9150::setI2cSlave4Done(){
    return i2c.getBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_DONE_BIT);
}
bool MPU9150::setI2cLostArbitration(){
    return i2c.getBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_LOST_ARB_BIT);
}
bool MPU9150::setI2cSlave0Nack(){
    return i2c.getBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV0_NACK_BIT);
}
bool MPU9150::setI2cSlave1Nack(){
    return i2c.getBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV1_NACK_BIT);
}
bool MPU9150::setI2cSlave2Nack(){
    return i2c.getBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV2_NACK_BIT);
}
bool MPU9150::setI2cSlave3Nack(){
    return i2c.getBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV3_NACK_BIT);
}
bool MPU9150::setI2cSlave4Nack(){
   return i2c.getBit(MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_NACK_BIT);
}

//INT_PIN_CFG
void MPU9150::setInterruptActiveLow(bool value){
    i2c.setBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, value);
}
void MPU9150::setInterruptOpenDrain(bool value){
    i2c.setBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, value);
}
void MPU9150::setInterruptLatch(bool value){
    i2c.setBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, value);
}
void MPU9150::setInterruptAnyReadClear(bool value){
    i2c.setBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, value);
}
void MPU9150::setFsyncInterruptActiveLow(bool value){
    i2c.setBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, value);
}
void MPU9150::setFsyncInterruptEnable(bool value){
    i2c.setBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, value);
}
void MPU9150::setI2cAuxBypassEnable(bool value){
    i2c.setBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, value);
}

//INT_ENABLE
void MPU9150::setInterruptFifoOverflowEnable(bool value){
    i2c.setBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, value);
}
void MPU9150::setInterruptMasterEnable(bool value){
    i2c.setBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT, value);
}
void MPU9150::setInterruptDataReadyEnable(bool value){
    i2c.setBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, value);
}

//INT_STATUS
bool MPU9150::getInterruptFifoOverflow(){
    return i2c.getBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_FIFO_OFLOW_BIT);
}
bool MPU9150::getInterruptMaster(){
    return i2c.getBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_I2C_MST_INT_BIT);
}
bool MPU9150::getInterruptDataReady(){
    return i2c.getBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DATA_RDY_BIT);
}

uint8_t MPU9150::getInterruptStatus(){
    return i2c.getByte(MPU6050_RA_INT_STATUS);
}

//SIGNAL_PATH_RESET
void MPU9150::resetGyroSignalPath(){
    i2c.setBit(MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_GYRO_RESET_BIT, true);
}
void MPU9150::resetAccelSignalPath(){
    i2c.setBit(MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_ACCEL_RESET_BIT, true);
}
void MPU9150::resetTempSignalPath(){
    i2c.setBit(MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_TEMP_RESET_BIT, true);
}

//USER_CTRL
void MPU9150::setEnableFifo(bool value){
    i2c.setBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, value);
}
void MPU9150::setI2cMasterEnable(bool value){
    i2c.setBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, value);
}
void MPU9150::setFifoReset(bool value){
    i2c.setBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, value);
}
void MPU9150::setI2cMasterReset(bool value){
    i2c.setBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, value);
}
void MPU9150::setFullSensorReset(bool value){
    i2c.setBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_SIG_COND_RESET_BIT, value);
}

//FIFO_COUNT_H and FIFO_COUNT_L
int16_t MPU9150::getFifoCount(){
    return i2c.getWord(MPU6050_RA_FIFO_COUNTH);
}

//FIFO_R_W
bool MPU9150::getFifoBuffer(char* buffer, int16_t length){
    return i2c.readBytes(MPU6050_RA_FIFO_R_W, buffer, length);
}

//UNDOCUMENTED (again reimplemention from sparkfun github) can't find any origional documentation
// XG_OFFS_TC
uint8_t MPU9150::getOTPBankValid() {
    return i2c.getBit(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT);
}

//INT_ENABLE
void MPU9150::setIntPLLReadyEnabled(bool value) {
    i2c.setBit( MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, value);
}
void MPU9150::setIntDMPEnabled(bool value) {
    i2c.setBit( MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, value);
}

// INT_STATUS
bool MPU9150::getIntPLLReadyStatus() {
    return i2c.getBit( MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_PLL_RDY_INT_BIT);
}
bool MPU9150::getIntDMPStatus() {
    return i2c.getBit( MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DMP_INT_BIT);
}

// USER_CTRL
bool MPU9150::getDMPEnabled() {
    return i2c.getBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT);
}
void MPU9150::setDMPEnabled(bool value) {
    i2c.setBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, value);
}
void MPU9150::resetDMP() {
    i2c.setBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);
}

// BANK_SEL
void MPU9150::setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank) {
    bank &= 0x1F;
    if (userBank){
        bank |= 0x20;
    }
    if (prefetchEnabled){
        bank |= 0x40;
    }
    i2c.setByte( MPU6050_RA_BANK_SEL, bank);
}

// MEM_START_ADDR
void MPU9150::setMemoryStartAddress(uint8_t address) {
    i2c.setByte(MPU6050_RA_MEM_START_ADDR, address);
}

// MEM_R_W
uint8_t MPU9150::readMemoryByte() {
    return i2c.getByte(MPU6050_RA_MEM_R_W);
}
void MPU9150::writeMemoryByte(uint8_t value) {
    i2c.setByte(MPU6050_RA_MEM_R_W, value);
}
void MPU9150::readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
    setMemoryBank(bank);
    setMemoryStartAddress(address);

    uint8_t chunkSize;
    for (uint16_t i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;
        //debug.printf("reading %d", chunkSize);
        // read the chunk of data as specified
        i2c.readBytes(MPU6050_RA_MEM_R_W, (char*)(data+i), chunkSize);
        //debug.printf("read");
        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            setMemoryBank(bank);
            setMemoryStartAddress(address);
        }
    }
}
bool MPU9150::writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify) {
    setMemoryBank(bank);
    setMemoryStartAddress(address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer = 0;
    uint8_t *progBuffer = 0;
    uint16_t i;

    if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        progBuffer = (uint8_t *)data + i;

        i2c.writeBytes(MPU6050_RA_MEM_R_W, (char*)progBuffer, chunkSize);


        // verify data if needed
        if (verify && verifyBuffer) {
            setMemoryBank(bank);
            setMemoryStartAddress(address);
            i2c.readBytes(MPU6050_RA_MEM_R_W, (char*)verifyBuffer, chunkSize);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                free(verifyBuffer);
                return false; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            setMemoryBank(bank);
            setMemoryStartAddress(address);
        }
    }
    if (verify) free(verifyBuffer);
    return true;
}
bool MPU9150::writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize) {
    uint8_t *progBuffer;
    uint8_t success, special;
    uint16_t i;

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    uint8_t bank, offset, length;
    for (i = 0; i < dataSize;) {
        bank = data[i++];
        offset = data[i++];
        length = data[i++];

        // write data or perform special action
        if (length > 0) {
            progBuffer = (uint8_t *)data + i;
            success = writeMemoryBlock(progBuffer, length, bank, offset, true);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            special = data[i++];

            if (special == 0x01) {
                // enable DMP-related interrupts
                //setIntZeroMotionEnabled(true);
                //setIntFIFOBufferOverflowEnabled(true);
                //setIntDMPEnabled(true);
                i2c.setByte(MPU6050_RA_INT_ENABLE, 0x32);  // single operation
                success = true;
            } else {
                // unknown special command
                success = false;
            }
        }

        if (!success) {
            return false;
        }
    }
    return true;
}
// DMP_CFG_1
uint8_t MPU9150::getDMPConfig1() {
   return i2c.getByte(MPU6050_RA_DMP_CFG_1);

}
void MPU9150::setDMPConfig1(uint8_t config) {
    i2c.setByte(MPU6050_RA_DMP_CFG_1, config);
}

// DMP_CFG_2
uint8_t MPU9150::getDMPConfig2() {
    return i2c.getByte(MPU6050_RA_DMP_CFG_2);

}
void MPU9150::setDMPConfig2(uint8_t config) {
    i2c.setByte(MPU6050_RA_DMP_CFG_2, config);
}
