   /*
 ====================================================================================================
 * File:        HMS_BH17XX_DRIVER.h
 * Author:      Hamas Saeed
 * Version:     Rev_1.0.0
 * Date:        Oct 10 2025
 * Brief:       This Package Provide BH17XX Driver Library for Cross Platform (STM/ESP/nRF)
 * 
 ====================================================================================================
 * License: 
 * MIT License
 * 
 * Copyright (c) 2025 Hamas Saeed
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * For any inquiries, contact Hamas Saeed at hamasaeed@gmail.com
 *
 ====================================================================================================
 */

#ifndef HMS_BH17XX_DRIVER_H
#define HMS_BH17XX_DRIVER_H

#if defined(ARDUINO)                                                                                       // Platform detection
  #define HMS_BH17XX_PLATFORM_ARDUINO
#elif defined(ESP_PLATFORM)
  #define HMS_BH17XX_PLATFORM_ESP_IDF
#elif defined(__ZEPHYR__)
  #define HMS_BH17XX_PLATFORM_ZEPHYR
#elif defined(__STM32__)
  #define HMS_BH17XX_PLATFORM_STM32_HAL
#endif

#if defined(HMS_BH17XX_PLATFORM_ARDUINO)
  #include <Wire.h>
  #include <Arduino.h>
#elif defined(HMS_BH17XX_PLATFORM_ESP_IDF)
#elif defined(HMS_BH17XX_PLATFORM_ZEPHYR)
  #include <stdio.h>
  #include <zephyr/device.h>
  #include <zephyr/drivers/i2c.h>
#elif defined(HMS_BH17XX_PLATFORM_STM32_HAL)
  #include "main.h"
#endif
#include   "HMS_BH17XX_Config.h"

#if defined(HMS_BH17XX_DEBUG_ENABLED) && (HMS_BH17XX_DEBUG_ENABLED == 1)
    #define HMS_BH17XX_LOGGER_ENABLED
#endif
typedef enum {
    HMS_BH17XX_OK       = 0x00,
    HMS_BH17XX_ERROR    = 0x01,
    HMS_BH17XX_BUSY     = 0x02,
    HMS_BH17XX_TIMEOUT  = 0x03,
    HMS_BH17XX_NOT_FOUND= 0x04
} HMS_BH17XX_StatusTypeDef;

typedef enum {
    HMS_BH17XX_SENSOR_BH1750,
    HMS_BH17XX_SENSOR_BH1721,
    HMS_BH17XX_SENSOR_BH1715
} HMS_BH17XX_SensorType;

typedef enum {
    UNCONFIGURED  = 0,                  // same as Power Down  
    CONTINUOUS_HIGH_RES_MODE,          // Measurement at 1 lux resolution. Measurement time is approx 120ms.
    CONTINUOUS_HIGH_RES_MODE_2,       // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
    CONTINUOUS_LOW_RES_MODE,         // Measurement at 4 lux resolution. Measurement time is approx 16ms.
    ONE_TIME_HIGH_RES_MODE,         // Measurement at 1 lux resolution. Measurement time is approx 120ms.
    ONE_TIME_HIGH_RES_MODE_2,      // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
    ONE_TIME_LOW_RES_MODE         // Measurement at 4 lux resolution. Measurement time is approx 16ms.
} HMS_BH17XX_Mode;



class HMS_BH17XX {
    public:
        HMS_BH17XX();
        ~HMS_BH17XX();
        #if defined(HMS_BH17XX_PLATFORM_ARDUINO)
            //here is the begin function for arduino
        #elif defined(HMS_BH17XX_PLATFORM_ESP_IDF)
            //here is the begin function for esp-idf
        #elif defined(HMS_BH17XX_PLATFORM_ZEPHYR)
            //here is the begin function for zephyr 
        #elif defined(HMS_BH17XX_PLATFORM_STM32_HAL)
             HMS_BH17XX_StatusTypeDef begin(I2C_HandleTypeDef *hi2c = NULL, uint8_t addr = HMS_BH17XX_DEVICE_ADDRESS,HMS_BH17XX_SensorType sensorType = HMS_BH17XX_SENSOR_BH1750, HMS_BH17XX_Mode mode = CONTINUOUS_HIGH_RES_MODE_2);
        #endif

  float                               readLightLevel();
  HMS_BH17XX_StatusTypeDef            sendCommand(uint8_t command);
  void                                setMode(HMS_BH17XX_Mode mode);
  void                                sendOneTimeLowResolution();        
  void                                sendOneTimeHighResolution();       
  void                                sendOneTimeHighResolution2();
  void                                sendContinuousLowResolution();
  void                                sendContinuousHighResolution();
  void                                sendContinuousHighResolution2();
  void                                sendPowerOn();
  void                                sendPowerDown();
  void                                reset();
  void                                setResolutionFactor(float RF = 1.2f);
  HMS_BH17XX_StatusTypeDef            writeMTReg(uint8_t MTreg);
  HMS_BH17XX_StatusTypeDef            setMTreg(uint8_t mtreg = HMS_BH17XX_MTREG_DEFAULT);
  uint8_t                             getDeviceAddress()    const                           { return deviceAddress;   }
  HMS_BH17XX_Mode                     getMode()             const                           { return measurementMode; }
  uint8_t                             getMeasurmentTime()   const                           { return MeasurementTime; }
  float                               getResolutionFactor() const                           { return resolutionFactor;}
  
private:

  uint16_t                  rawData;
  uint8_t                   readData[2];
  uint8_t                   deviceAddress;
  uint8_t                   measurementCommand;
  uint8_t                   powerOnCommand;
  uint8_t                   powerDownCommand;
  uint8_t                   MeasurementTime;
  float                     lux;
  float                     resolutionFactor;
  HMS_BH17XX_SensorType     Type;
  HMS_BH17XX_Mode           measurementMode;

  #if defined(HMS_BH17XX_PLATFORM_ARDUINO)
    TwoWire *bh17xx_wire = NULL;
  #elif defined(HMS_BH17XX_PLATFORM_ESP_IDF)
    i2c_port_t bh17xx_i2c_port;
  #elif defined(HMS_BH17XX_PLATFORM_ZEPHYR)
    struct device *bh17xx_i2c_dev;
  #elif defined(HMS_BH17XX_PLATFORM_STM32_HAL)
    I2C_HandleTypeDef *bh17xx_hi2c;
    HAL_StatusTypeDef  status;
  #endif
  
  HMS_BH17XX_StatusTypeDef init();
  void                     setDefaultValues();
  void                     bh17Delay(uint32_t ms);
};


#endif // HMS_BH17XX_DRIVER_H