   /*
 ====================================================================================================
 * File:        HMS_BH17XX_DRIVER.h
 * Author:      Hamas Saeed
 * Version:     Rev_1.0.0
 * Date:        Sep 30 2025
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
#elif defined(STM32F0) || defined(STM32F1) || defined(STM32F3) || defined(STM32F4) || \
      defined(STM32F7) || defined(STM32G0) || defined(STM32G4) || defined(STM32H7) || \
      defined(STM32L0) || defined(STM32L1) || defined(STM32L4) || defined(STM32L5) || \
      defined(STM32WB) || defined(STM32WL)
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

#endif // HMS_BH17XX_DRIVER_H

typedef enum {
    HMS_BH17XX_OK       = 0x00,
    HMS_BH17XX_ERROR    = 0x01,
    HMS_BH17XX_BUSY     = 0x02,
    HMS_BH17XX_TIMEOUT  = 0x03,
    HMS_BH17XX_NOT_FOUND= 0x04
} HMS_BH17XX_StatusTypeDef;

typedef enum Mode {
    UNCONFIGURED                = 0,            // same as Power Down  
    CONTINUOUS_HIGH_RES_MODE    = 0x10,         // Measurement at 1 lux resolution. Measurement time is approx 120ms.
    CONTINUOUS_HIGH_RES_MODE_2  = 0x11,         // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
    CONTINUOUS_LOW_RES_MODE     = 0x13,         // Measurement at 4 lux resolution. Measurement time is approx 16ms.
    ONE_TIME_HIGH_RES_MODE      = 0x20,         // Measurement at 1 lux resolution. Measurement time is approx 120ms.
    ONE_TIME_HIGH_RES_MODE_2    = 0x21,         // Measurement at 0.5 lux resolution. Measurement time is approx 120ms.
    ONE_TIME_LOW_RES_MODE       = 0x23          // Measurement at 4 lux resolution. Measurement time is approx 16ms.
} HMS_BH17XX_Mode;



class HMS_BH17XX {
    public:
        HMS_BH17XX(byte addr = 0x23);
  bool begin(
    Mode mode = CONTINUOUS_HIGH_RES_MODE, byte addr = 0x23,
    TwoWire* i2c = nullptr);
  bool configure(Mode mode);
  bool setMTreg(byte MTreg);
  bool measurementReady(bool maxWait = false);
  float readLightLevel();

private:
  byte BH1750_I2CADDR;
  byte BH1750_MTreg = (byte)BH1750_DEFAULT_MTREG;
  // Correction factor used to calculate lux. Typical value is 1.2 but can
  // range from 0.96 to 1.44. See the data sheet (p.2, Measurement Accuracy)
  // for more information.
  const float BH1750_CONV_FACTOR = 1.2;
  Mode BH1750_MODE = UNCONFIGURED;
  TwoWire* I2C;
  unsigned long lastReadTimestamp;
};

#endif // HMS_BH17XX_DRIVER_H