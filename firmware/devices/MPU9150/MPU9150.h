#ifndef MPU9150_INCLUDE
#define MPU9150_INCLUDE

#include <Arduino.h>
#include <Wire.h>
#include "registers.h"
#include "dmpdata.h"
#include "../../communications/i2c.h"

#define ERR_OTP_INVALID 1

class MPU9150{
public:
    MPU9150(uint8_t address = 0x69);
    ~MPU9150();

    uint8_t getDeviceID();
    bool isReady();
    void initialise();
    void initialiseMagnetometer();
    uint8_t initialiseDMP();


    //PWR_MGMT_1 Control Register
    void reset();
    void sleep(bool state);
    void cycleMode(bool state);
    void disableTemperatureSensor(bool state);
    void clockSelect(uint8_t clk);

    //PWR_MGMT_2 Control Register
    void setCycleWakeFrequency(uint8_t value);
    void setStandbyAccX( bool value );
    void setStandbyAccY( bool value );
    void setStandbyAccZ( bool value );
    void setStandbyGyroX( bool value );
    void setStandbyGyroY( bool value );
    void setStandbyGyroZ( bool value );

    //SMPRT_DI Sample Rate Divider
    void setSampleRateDivider(uint8_t value);

    //CONFIG
    void setExternalFrameSync(uint8_t value);
    void setDigitalLowPassFilter(uint8_t value);

    //GYRO_CONFIG
    void setGyroSelfTest(bool value);
    void setGyroFullScaleRange(uint8_t value);

    //ACCEL_CONFIG
    void setAccelSelfTest(bool value);
    void setAccelFullScaleRange(uint8_t value);

    //FIFO_EN
    void setTemperatureFifo(bool value);
    void setGyroFifo(bool value);
    void setAccelFifo(bool value);
    void setSlave2Fifo(bool value);
    void setSlave1Fifo(bool value);
    void setSlave0Fifo(bool value);

    //I2C_MST_CTRL
    void setMultiMaster(bool value);
    void setWaitForExternalSensor(bool value);
    void setSlave3Fifo(bool value);
    void setMasterStartStop(bool value);
    void setI2CMasterClock(uint8_t value);

    //I2C_SLV0_ADDR
    void setI2cSlaveRW(uint8_t slave_id, bool value);
    void setI2cSlaveAddress(uint8_t slave_id, uint8_t value);
    //I2C_SLV0_REG,
    void setI2cSlaveRegister(uint8_t slave_id, uint8_t value);
    //I2C_SLV0_CTRL
    void setI2cSlaveEnable(uint8_t slave_id, bool value);
    void setI2cSlaveByteSwap(uint8_t slave_id, bool value);
    void setI2cSlaveRegDisable(uint8_t slave_id, bool value);
    void setI2cSlaveByteGrouping(uint8_t slave_id, bool value);
    void setI2cSlaveTransactionLength(uint8_t slave_id, uint8_t value);
    //I2C_SLV0_DO
    void setI2cSlaveDataOut(uint8_t slave_id, uint8_t value);
    //I2C_MST_DELAY_CTRL
    void setI2cSlaveDelay(uint8_t slave_id, uint8_t value);
    void setI2cSlaveShadowDelay(uint8_t value)    ;
    //Slave4 is different
    void setI2cSlave4RW( bool value);
    void setI2cSlave4Address( uint8_t value);
    void setI2cSlave4Register(uint8_t value);
    void setI2cSlave4DataOut(uint8_t value);
    void setI2cSlave4Enable(bool value);
    void setI2cSlave4IntEnable(bool value);
    void setI2cSlave4RegDisable(bool value);
    void setI2cMasterDelay(uint8_t value);
    uint8_t getI2cSlave4Di();

    //I2C_MST_STATUS
    bool setI2cPassthrough();
    bool setI2cSlave4Done();
    bool setI2cLostArbitration();
    bool setI2cSlave0Nack();
    bool setI2cSlave1Nack();
    bool setI2cSlave2Nack();
    bool setI2cSlave3Nack();
    bool setI2cSlave4Nack();

    //INT_PIN_CFG
    void setInterruptActiveLow(bool value);
    void setInterruptOpenDrain(bool value);
    void setInterruptLatch(bool value);
    void setInterruptAnyReadClear(bool value);
    void setFsyncInterruptActiveLow(bool value);
    void setFsyncInterruptEnable(bool value);
    void setI2cAuxBypassEnable(bool value);

    //INT_ENABLE
    void setInterruptFifoOverflowEnable(bool value);
    void setInterruptMasterEnable(bool value);
    void setInterruptDataReadyEnable(bool value);

    //INT_STATUS
    bool getInterruptFifoOverflow();
    bool getInterruptMaster();
    bool getInterruptDataReady();
    uint8_t getInterruptStatus();

    //SIGNAL_PATH_RESET
    void resetGyroSignalPath();
    void resetAccelSignalPath();
    void resetTempSignalPath();

    //USER_CTRL
    void setEnableFifo(bool value);
    void setI2cMasterEnable(bool value);
    void setFifoReset(bool value);
    void setI2cMasterReset(bool value);
    void setFullSensorReset(bool value);

    //FIFO_COUNT_H and FIFO_COUNT_L
    int16_t getFifoCount();

    //FIFO_R_W
    bool getFifoBuffer(char* buffer, int16_t length);

    //UNDOCUMENTED
    // XG_OFFS_TC
    uint8_t getOTPBankValid();

    //INT_ENABLE
    void setIntPLLReadyEnabled(bool value);
    void setIntDMPEnabled(bool value);

    // INT_STATUS
    bool getIntPLLReadyStatus();
    bool getIntDMPStatus();

    // USER_CTRL
    bool getDMPEnabled();
    void setDMPEnabled(bool value);
    void resetDMP();

    // BANK_SEL
    void setMemoryBank(uint8_t bank, bool prefetchEnabled=false, bool userBank=false);

    // MEM_START_ADDR
    void setMemoryStartAddress(uint8_t address);

    // MEM_R_W register
    uint8_t readMemoryByte();
    void writeMemoryByte(uint8_t value);
    void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
    bool writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank = 0, uint8_t address = 0, bool verify = false);
    bool writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);

    // DMP_CFG_1
    uint8_t getDMPConfig1();
    void setDMPConfig1(uint8_t config);

    // DMP_CFG_2
    uint8_t getDMPConfig2();
    void setDMPConfig2(uint8_t config);
private:
    I2cDevice i2c;
};

#endif
