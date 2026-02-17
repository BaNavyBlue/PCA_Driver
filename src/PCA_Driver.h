/*
 * The MIT License (MIT)

Copyright (c) 2015 Jetsonhacks
Modified 2025 Antone A. Bajor

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _JHPWMPCA9685_H
#define _JHPWMPCA9685_H

#include <stdint.h>
#include <cstddef>

// ==========================================================
// Platform Selection
// ==========================================================

// If compiling on Arduino IDE, ARDUINO is automatically defined
#ifdef ARDUINO
    #define PCA9685_ARDUINO
#else
    #define PCA9685_LINUX
#endif


// ==========================================================
// Platform Includes
// ==========================================================

#ifdef PCA9685_LINUX
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#endif

#ifdef PCA9685_ARDUINO
    #include <Arduino.h>
    #include <Wire.h>
#endif


const uint16_t TWO_POW_TWELVE = 4096;

class PCA9685
{
private:

#ifdef PCA9685_LINUX
    uint8_t kI2CBus;
    int kI2CFileDescriptor;
#endif

    uint8_t kI2CAddress;
    int error;

    double cpuClock_ = 25000000.0;
    bool useExternClk_ = false;
    uint8_t prescale_;
    float frequency_ = 50.0f;
    double period_slice_;
    int pwmOnBias_ = 0;

    void setPeriodSlice();

public:

#ifdef PCA9685_LINUX
    PCA9685(uint8_t bus, uint8_t address);
    PCA9685(uint8_t bus, uint8_t address, double externalClock);
    uint8_t get_kI2CBus();
    void set_kI2CBus(uint8_t bus);
    int get_kI2Address();
#endif

#ifdef PCA9685_ARDUINO
    PCA9685(uint8_t address);
    PCA9685(uint8_t address, double externalClock);
#endif

    ~PCA9685();

    bool openPCA9685();
    void closePCA9685();

    uint8_t getPrescale();
    float getFrequency();
    int getError();
    double getPeriodSlice();
    double getClock();

    void reset();

    // The percent error for 1ms on at a given input frequency ends up following a 1/x type curve.
    // The Bias can shift which grouping of frequencies is closer to 0% Error. From initial tests
    // Lower frequenncies have slightly positive Error which quickly become more negative and 
    // settle toward a horizontal assymptote.
    void setPWMBias(int bias);
    int getPWMBias();

    // Sets the frequency of the PWM signal
    // Frequency is ranged between 40 and 1000 Hz
    void setPWMFrequency(float frequency);
    void setExtClk();
    int stopPCA();

    void setPWM(uint8_t channel, uint16_t onValue, uint16_t offValue);
    void setAllPWM(uint16_t onValue, uint16_t offValue);

    uint8_t readByte(uint8_t readRegister);
    void writeByte(uint8_t writeRegister, uint8_t writeValue);

    int sleepPCA();
    int wakePCA();
};


// Register definitions from Table 7.3 NXP Semiconductors
// Product Data Sheet, Rev. 4 - 16 April 2015
#define PCA9685_MODE1            0x00
#define PCA9685_MODE2            0x01
#define PCA9685_SUBADR1          0x02
#define PCA9685_SUBADR2          0x03
#define PCA9685_SUBADR3          0x04
#define PCA9685_ALLCALLADR       0x05

#define MODE1_SLEEP              0x10  /**< Low power mode. Oscillator off */
#define MODE1_AI                 0x20  /**< Auto-Increment enabled */
#define MODE1_EXTCLK             0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART            0x80  /**< Restart enabled */
// PWM High Low States
#define PCA9685_LED0_ON_L        0x06
#define PCA9685_LED0_ON_H        0x07
#define PCA9685_LED0_OFF_L       0x08
#define PCA9685_LED0_OFF_H       0x09

#define PCA9685_LED1_ON_L        0x0A
#define PCA9685_LED1_ON_H        0x0B
#define PCA9685_LED1_OFF_L       0x0C
#define PCA9685_LED1_OFF_H       0x0D

#define PCA9685_LED2_ON_L        0x0E
#define PCA9685_LED2_ON_H        0x0F
#define PCA9685_LED2_OFF_L       0x10
#define PCA9685_LED2_OFF_H       0x11

#define PCA9685_LED3_ON_L        0x12
#define PCA9685_LED3_ON_H        0x13
#define PCA9685_LED3_OFF_L       0x14
#define PCA9685_LED3_OFF_H       0x15

#define PCA9685_LED4_ON_L        0x16
#define PCA9685_LED4_ON_H        0x17
#define PCA9685_LED4_OFF_L       0x18
#define PCA9685_LED4_OFF_H       0x19

#define PCA9685_LED5_ON_L        0x1A
#define PCA9685_LED5_ON_H        0x1B
#define PCA9685_LED5_OFF_L       0x1C
#define PCA9685_LED5_OFF_H       0x1D

#define PCA9685_LED6_ON_L        0x1E
#define PCA9685_LED6_ON_H        0x1F
#define PCA9685_LED6_OFF_L       0x20
#define PCA9685_LED6_OFF_H       0x21

#define PCA9685_LED7_ON_L        0x22
#define PCA9685_LED7_ON_H        0x23
#define PCA9685_LED7_OFF_L       0x24
#define PCA9685_LED7_OFF_H       0x25

#define PCA9685_LED8_ON_L        0x26
#define PCA9685_LED8_ON_H        0x27
#define PCA9685_LED8_OFF_L       0x28
#define PCA9685_LED8_OFF_H       0x29

#define PCA9685_LED9_ON_L        0x2A
#define PCA9685_LED9_ON_H        0x2B
#define PCA9685_LED9_OFF_L       0x2C
#define PCA9685_LED9_OFF_H       0x2D

#define PCA9685_LED10_ON_L       0x2E
#define PCA9685_LED10_ON_H       0x2F
#define PCA9685_LED10_OFF_L      0x30
#define PCA9685_LED10_OFF_H      0x31

#define PCA9685_LED11_ON_L       0x32
#define PCA9685_LED11_ON_H       0x33
#define PCA9685_LED11_OFF_L      0x34
#define PCA9685_LED11_OFF_H      0x35

#define PCA9685_LED12_ON_L       0x36
#define PCA9685_LED12_ON_H       0x37
#define PCA9685_LED12_OFF_L      0x38
#define PCA9685_LED12_OFF_H      0x39

#define PCA9685_LED13_ON_L       0x3A
#define PCA9685_LED13_ON_H       0x3B
#define PCA9685_LED13_OFF_L      0x3C
#define PCA9685_LED13_OFF_H      0x3D

#define PCA9685_LED14_ON_L       0x3E
#define PCA9685_LED14_ON_H       0x3F
#define PCA9685_LED14_OFF_L      0x40
#define PCA9685_LED14_OFF_H      0x41

#define PCA9685_LED15_ON_L       0x42
#define PCA9685_LED15_ON_H       0x43
#define PCA9685_LED15_OFF_L      0x44
#define PCA9685_LED15_OFF_H      0x45

#define PCA9685_ALL_LED_ON_L     0xFA
#define PCA9685_ALL_LED_ON_H     0xFB
#define PCA9685_ALL_LED_OFF_L    0xFC
#define PCA9685_ALL_LED_OFF_H    0xFD
#define PCA9685_PRE_SCALE        0xFE
//#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */

// Register Bits
#define PCA9685_ALLCALL          0x01
#define PCA9685_OUTDRV           0x04
#define PCA9685_RESTART          0x80
#define PCA9685_SLEEP            0x10
#define PCA9685_INVERT           0x10



#endif
