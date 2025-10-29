#include  "HMS_BH17XX_DRIVER.h"

#if defined(HMS_BH17XX_LOGGER_ENABLED)
    #include "ChronoLog.h"
    ChronoLogger      bh17xxLogger("HMS_BH17XX",CHRONOLOG_LEVEL_DEBUG);
#endif
HMS_BH17XX::HMS_BH17XX() {
    // Constructor implementation
}
HMS_BH17XX::~HMS_BH17XX(){
    // Destructor implementation
}
#if defined(HMS_BH17XX_PLATFORM_ARDUINO)
        //here is the begin function for arduino
#elif defined(HMS_BH17XX_PLATFORM_ESP_IDF)
        //here is the begin function for esp-idf
#elif defined(HMS_BH17XX_PLATFORM_ZEPHYR)
        //here is the begin function for zephyr 
#elif defined(HMS_BH17XX_PLATFORM_STM32_HAL)

    HMS_BH17XX_StatusTypeDef HMS_BH17XX::begin(I2C_HandleTypeDef *hi2c, uint8_t addr, HMS_BH17XX_SensorType sensorType, HMS_BH17XX_Mode mode){
        if (hi2c == NULL){
            #ifdef HMS_BH17XX_LOGGER_ENABLED
                bh17xxLogger.error("I2C handle is NULL");
            #endif
            return HMS_BH17XX_ERROR;
        }
        Type = sensorType;
        measurementMode = mode;
        setDefaultValues();

        bh17xx_hi2c   = hi2c;
        deviceAddress = addr << 1;
            
        status = HAL_I2C_IsDeviceReady(bh17xx_hi2c, deviceAddress, 1, 100);
        if (status != HAL_OK) {
            #ifdef HMS_BH17XX_LOGGER_ENABLED
                bh17xxLogger.error("Device is not found at address 0x%02X", deviceAddress);
            #endif
                return HMS_BH17XX_NOT_FOUND;
            }
        #ifdef HMS_BH17XX_LOGGER_ENABLED
            bh17xxLogger.info("Device is found at address 0x%02X", deviceAddress);
        #endif
        return init();
    }

HMS_BH17XX_StatusTypeDef HMS_BH17XX::sendCommand(uint8_t command)
{
    status = HAL_I2C_Master_Transmit(bh17xx_hi2c, deviceAddress, &command, 1, 100);
    if(command == HMS_BH17XX_CMD_POWER_ON){
        if(status == HAL_OK){
            #ifdef HMS_BH17XX_DEBUG_ENABLED
                bh17xxLogger.debug("Power ON command sent successfully");
            #endif
            return HMS_BH17XX_OK;
        }
    }
    if (status != HAL_OK) {
    #ifdef HMS_BH17XX_DEBUG_ENABLED
        bh17xxLogger.error("Failed to send command 0x%02X", command);
    #endif
        return HMS_BH17XX_ERROR;
    }

    #ifdef HMS_BH17XX_DEBUG_ENABLED
    bh17xxLogger.debug("Command 0x%02X sent successfully", command);
    #endif
    return HMS_BH17XX_OK;
}
HMS_BH17XX_StatusTypeDef HMS_BH17XX::init() {
    if(sendCommand(powerOnCommand) != HMS_BH17XX_OK){
        return HMS_BH17XX_ERROR;
    }
    bh17Delay(200);
    if(writeMTReg(120) != HMS_BH17XX_OK){
        return HMS_BH17XX_ERROR;
    }
    bh17Delay(200);
    if(sendCommand(measurementCommand) == HMS_BH17XX_OK){     // here mode is set by the defaultvalues function
        bh17Delay(200);
        readLightLevel();
    }
    return HMS_BH17XX_OK;
}

float HMS_BH17XX::readLightLevel()
{
    // 2 bytes read from sensor
    status = HAL_I2C_Master_Receive(bh17xx_hi2c, deviceAddress, readData, sizeof(readData), HAL_MAX_DELAY);

    if (status == HAL_OK)
    {
        rawData = (readData[0] << 8) | readData[1];
        lux = (float)rawData / resolutionFactor; 
        return lux;
    }
    else
    {
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("Failed to read light level");
        #endif
        return -1;
    }
}
#endif

void HMS_BH17XX::bh17Delay(uint32_t ms) {
    #if defined(HMS_BH17XX_PLATFORM_ARDUINO)
        delay(ms);
    #elif defined(HMS_BH17XX_PLATFORM_ESP_IDF)
        vTaskDelay(ms / portTICK_PERIOD_MS);
    #elif defined(HMS_BH17XX_PLATFORM_ZEPHYR)
        k_msleep(ms);
    #elif defined(HMS_BH17XX_PLATFORM_STM32_HAL)
        HAL_Delay(ms);
    #endif
}

void HMS_BH17XX::setDefaultValues(){
    switch (Type) {
        case HMS_BH17XX_SENSOR_BH1750:
            resolutionFactor = HMS_BH1750_Resolution_Factor;
            break;
        case HMS_BH17XX_SENSOR_BH1721:
            resolutionFactor = HMS_BH1721_Resolution_Factor;
            break;
        case HMS_BH17XX_SENSOR_BH1715:
            resolutionFactor = HMS_BH1715_Resolution_Factor;
            break;
    }
    switch(measurementMode)
    {
        case CONTINUOUS_HIGH_RES_MODE:
            measurementCommand = HMS_BH17XX_CMD_CONT_H_RES_MODE;
            break;
        case CONTINUOUS_HIGH_RES_MODE_2:
            measurementCommand = HMS_BH17XX_CMD_CONT_H_RES_MODE2;
            break;
        case CONTINUOUS_LOW_RES_MODE:
            measurementCommand = HMS_BH17XX_CMD_CONT_L_RES_MODE;
            break;
        // one time
        case ONE_TIME_HIGH_RES_MODE:
            measurementCommand = HMS_BH17XX_CMD_ONE_TIME_H_RES_MODE2;
            break;
        case ONE_TIME_HIGH_RES_MODE_2:
            measurementCommand = HMS_BH17XX_CMD_ONE_TIME_H_RES_MODE;
            break;
        case ONE_TIME_LOW_RES_MODE:
            measurementCommand = HMS_BH17XX_CMD_ONE_TIME_L_RES_MODE;
            break;
        default:
            measurementCommand = HMS_BH17XX_CMD_CONT_H_RES_MODE2;
            break;
    }
    powerDownCommand = HMS_BH17XX_CMD_POWER_DOWN;
    powerOnCommand   = HMS_BH17XX_CMD_POWER_ON;
}
void HMS_BH17XX::setMode(HMS_BH17XX_Mode mode){
    measurementMode = mode;
}

void HMS_BH17XX::sendOneTimeLowResolution(){
    if (sendCommand(HMS_BH17XX_CMD_ONE_TIME_L_RES_MODE) == HMS_BH17XX_OK){
        bh17Delay(200);
    }else{
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("Failed to send one time low resolution command");
        #endif
    }
}
void HMS_BH17XX::sendOneTimeHighResolution(){
    if (sendCommand(HMS_BH17XX_CMD_ONE_TIME_H_RES_MODE) == HMS_BH17XX_OK){
        bh17Delay(200);
    }else{
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("Failed to send one time high resolution command");
        #endif
    }
}
void HMS_BH17XX::sendOneTimeHighResolution2(){
    if (sendCommand(HMS_BH17XX_CMD_ONE_TIME_H_RES_MODE2) == HMS_BH17XX_OK){
        bh17Delay(200);
    }else{
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("Failed to send one time high resolution 2 command");
        #endif
    }
}
void HMS_BH17XX::sendContinuousLowResolution(){
    if(sendCommand(HMS_BH17XX_CMD_CONT_L_RES_MODE) == HMS_BH17XX_OK){
        bh17Delay(200);
    }else{
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("Failed to send continuous low resolution command");
        #endif
    }
}
void HMS_BH17XX::sendContinuousHighResolution(){
    if(sendCommand(HMS_BH17XX_CMD_CONT_H_RES_MODE) == HMS_BH17XX_OK){
        bh17Delay(200);
    }else{
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("Failed to send continuous high resolution command");
        #endif
    }
}
void HMS_BH17XX::sendContinuousHighResolution2(){
    if(sendCommand(HMS_BH17XX_CMD_CONT_H_RES_MODE2) == HMS_BH17XX_OK){
        bh17Delay(200);
    }else{
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("Failed to send continuous high resolution 2 command");
        #endif
    }
}
void HMS_BH17XX::sendPowerDown(){
    if(sendCommand(powerDownCommand) == HMS_BH17XX_OK){
        bh17Delay(200);
    }else{
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("Failed to send power down command");
        #endif
    }
}
void HMS_BH17XX::sendPowerOn(){
    if(sendCommand(powerOnCommand) == HMS_BH17XX_OK){
        bh17Delay(200);
    }else{
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("Failed to send power on command");
        #endif
    }
}
HMS_BH17XX_StatusTypeDef HMS_BH17XX::setMTreg(uint8_t mtreg){
    if (mtreg < HMS_BH17XX_MTREG_MIN || mtreg > HMS_BH17XX_MTREG_MAX){
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("MTreg value out of range (31-254): %d", mtreg);
        #endif
        return HMS_BH17XX_ERROR;
    }
    MeasurementTime = mtreg;
    return writeMTReg(mtreg);
}
HMS_BH17XX_StatusTypeDef  HMS_BH17XX::writeMTReg(uint8_t MTreg){
    uint8_t highBitCommand = 0x40 | (MTreg >> 5);    // Set high bits (bits 5-7)
    uint8_t lowBitCommand  = 0x60 | (MTreg & 0x1F); // Set low bits (bits 0-4)

    if (sendCommand(highBitCommand) != HMS_BH17XX_OK) {
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("Failed to send high bits of MTreg command");
        #endif
        return HMS_BH17XX_ERROR;
    }
    bh17Delay(10); // Short delay between commands

    if (sendCommand(lowBitCommand) != HMS_BH17XX_OK) {
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("Failed to send low bits of MTreg command");
        #endif
        return HMS_BH17XX_ERROR;
    }
    bh17Delay(10); // Short delay after commands

    #ifdef HMS_BH17XX_DEBUG_ENABLED
        bh17xxLogger.debug("MTreg set to %d", MTreg);
    #endif
    return HMS_BH17XX_OK;
}
void HMS_BH17XX::reset(){
    if(sendCommand(HMS_BH17XX_CMD_RESET) == HMS_BH17XX_OK){
        bh17Delay(200);
    }else{
        #ifdef HMS_BH17XX_DEBUG_ENABLED
            bh17xxLogger.error("Failed to send reset command");
        #endif
    }
}
void HMS_BH17XX::setResolutionFactor(float RF){
    resolutionFactor = RF;
}

float HMS_BH17XX::ReadSensor(){ 
    const int numSamples = 10;
    float sumLux = 0;
    for (int i = 0; i < numSamples; i++)
        {
        if (sendCommand(HMS_BH17XX_CMD_ONE_TIME_H_RES_MODE2) == HMS_BH17XX_OK)
        {
            bh17Delay(200); 
            float currentLux = readLightLevel();

            if (currentLux >= 0)
            {
                sumLux += currentLux;
            }
            else
            {
                #ifdef HMS_BH17XX_DEBUG_ENABLED
                    bh17xxLogger->error("Reading %d failed", i);
                #endif
            }
        }
        else
        {
            #ifdef HMS_BH17XX_DEBUG_ENABLED
                bh17xxLogger->error("Failed to send command on iteration %d", i);
            #endif
        }

        bh17Delay(50); 
    }

    lux = sumLux / numSamples;;
    #ifdef HMS_BH17XX_LOGGER_ENABLED
        bh17xxLogger->info("Average Light Level after %d samples: %.2f lux", numSamples, lux);
    #endif
    return lux;
}