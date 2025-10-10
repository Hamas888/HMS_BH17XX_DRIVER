/*
  ====================================================================================================
  * File:        HMS_BH17XX_Config.h
  * Author:      Hamas Saeed
  * Version:     Rev_1.0.0
  * Date:        Oct 10 2025
  * Brief:       This Package Provide BH17XX Driver Selection
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
#ifndef _HMS_BH17XX_CONFIG_H_
#define _HMS_BH17XX_CONFIG_H_

/*
  ┌─────────────────────────────────────────────────────────────────────┐
  │ Note:     Enable only if ChronoLog is included                      │
  │ Requires: ChronoLog library → https://github.com/Hamas888/ChronoLog │
  └─────────────────────────────────────────────────────────────────────┘
*/
#define HMS_BH17XX_DEBUG_ENABLED              1                           // Enable debug messages (1=enabled, 0=disabled)



/*
  ┌─────────────────────────────────────────────────────────────────────┐
  │ Supported Sensor Types                                              │
  │                                                                     │
  │ This driver provides support for the following BH17XX family:       │
  │   • BH1750  → Standard ambient light sensor                         │
  │   • BH1721  → High accuracy variant                                 │
  │   • BH1715  → Alternative variant                                   │
  │                                                                     │
  │ Select sensor type in your application code.                        │
  └─────────────────────────────────────────────────────────────────────┘
*/



#define HMS_BH17XX_DEVICE_NAME                   "BH17XX"                     // Device Name
#define HMS_BH17XX_DEVICE_ADDRESS                 0x23                       // BH17XX Default  I2C Address
#define HMS_BH17XX_DEVICE_ID                      0x23                        // BH17XX Device ID
// ===================== Power Mode =====================
#define HMS_BH17XX_CMD_POWER_DOWN                 0x00
#define HMS_BH17XX_CMD_POWER_ON                   0x01
#define HMS_BH17XX_CMD_RESET                      0x07
// ===================== Continuous Mode =====================
#define HMS_BH17XX_CMD_CONT_H_RES_MODE            0x10   // 1 lx, High Resoultion
#define HMS_BH17XX_CMD_CONT_H_RES_MODE2           0x11   // 0.5 lx, High Resoultion
#define HMS_BH17XX_CMD_CONT_L_RES_MODE            0x13   // 4 lx, Low Reoultion

// ===================== One-Time Mode =====================
#define HMS_BH17XX_CMD_ONE_TIME_H_RES_MODE        0x20   // 1 lx, High Resoultion
#define HMS_BH17XX_CMD_ONE_TIME_H_RES_MODE2       0x21   // 0.5 lx, High Resoultion
#define HMS_BH17XX_CMD_ONE_TIME_L_RES_MODE        0x23   // 4 lx, High Resoultion

// ===================== Resolution Factor =====================
#define HMS_BH1750_Resolution_Factor              1.2f   
#define HMS_BH1721_Resolution_Factor              1.15f
#define HMS_BH1715_Resolution_Factor              1.25f
// ===================== Measurement Time Register =====================
#define HMS_BH17XX_MTREG_DEFAULT                  69        // Default Measurement Time Register Value  
#define HMS_BH17XX_MTREG_MIN                      31        // Minimum Measurement Time Register Value
#define HMS_BH17XX_MTREG_MAX                      254       // Maximum Measurement Time Register Value


#endif