// -----------------------------------------------------------------------------
// ADS1X15-based Energy Monitor Sensor over I2C
// Copyright (C) 2017-2018 by Xose Pérez <xose dot perez at gmail dot com>
// -----------------------------------------------------------------------------

#if SENSOR_SUPPORT && EMON_ADS1X15_SUPPORT

#pragma once

#include "Arduino.h"
#include "EmonSensor.h"

//In the ADS 1115 the 16 bit config register is structured in the following way:

#define ADS1X15_CHANNELS (4)

#define ADS1X15_CHIP_ADS1015 (0)
#define ADS1X15_CHIP_ADS1115 (1)

#define ADS1X15_RESOLUTION (16)

#define ADS1015_CONVERSIONDELAY (1)
#define ADS1115_CONVERSIONDELAY (8)

#define ADS1015_BIT_SHIFT (4)
#define ADS1115_BIT_SHIFT (0)

#define ADS1X15_REG_POINTER_MASK (0x03)
#define ADS1X15_REG_POINTER_CONVERT (0x00)
#define ADS1X15_REG_POINTER_CONFIG (0x01)
#define ADS1X15_REG_POINTER_LOWTHRESH (0x02)
#define ADS1X15_REG_POINTER_HITHRESH (0x03)

/*
    BIT 15

    Operational status or single-shot conversion start
    This bit determines the operational status of the device. OS can only be written
    when in power-down state and has no effect when a conversion is ongoing.
    When writing:
        0 : No effect
        1 : Start a single conversion (when in power-down state)
    When reading:
        0 : Device is currently performing a conversion
        1 : Device is not currently performing a conversion
*/

#define ADS1X15_REG_CONFIG_OS_MASK (0x8000)    //14
#define ADS1X15_REG_CONFIG_OS_SINGLE (0x8000)  // Write: Set to start a single-conversion
#define ADS1X15_REG_CONFIG_OS_BUSY (0x0000)    // Read: Bit = 0 when conversion is in progress
#define ADS1X15_REG_CONFIG_OS_NOTBUSY (0x8000) // Read: Bit = 1 when device is not performing a conversion

/*
    BIT 14-12

    Input multiplexer configuration (ADS1115 only)
    These bits configure the input multiplexer. These bits serve no function on the
    ADS1113 and ADS1114.
        000 : AINP = AIN0 and AINN = AIN1 (default)
        001 : AINP = AIN0 and AINN = AIN3
        010 : AINP = AIN1 and AINN = AIN3
        011 : AINP = AIN2 and AINN = AIN3
        100 : AINP = AIN0 and AINN = GND
        101 : AINP = AIN1 and AINN = GND
        110 : AINP = AIN2 and AINN = GND
        111 : AINP = AIN3 and AINN = GND

*/

#define ADS1X15_REG_CONFIG_MUX_MASK (0x7000)
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_1 (0x0000) // Differential P = AIN0, N = AIN1 (default)
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_3 (0x1000) // Differential P = AIN0, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_1_3 (0x2000) // Differential P = AIN1, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_2_3 (0x3000) // Differential P = AIN2, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000) // Single-ended AIN0
#define ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000) // Single-ended AIN1
#define ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000) // Single-ended AIN2
#define ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000) // Single-ended AIN3

/*

    BIT 11-9

    Programmable gain amplifier configuration
    These bits set the FSR of the programmable gain amplifier. These bits serve no
    function on the ADS1113.
        000 : FSR = ±6.144 V
        001 : FSR = ±4.096 V
        010 : FSR = ±2.048 V (default)
        011 : FSR = ±1.024 V
        100 : FSR = ±0.512 V
        101 : FSR = ±0.256 V
        110 : FSR = ±0.256 V
        111 : FSR = ±0.256 V

*/

#define ADS1X15_REG_CONFIG_PGA_MASK (0x0E00)
#define ADS1X15_REG_CONFIG_PGA_6_144V (0x0000) // +/-6.144V range = Gain 2/3
#define ADS1X15_REG_CONFIG_PGA_4_096V (0x0200) // +/-4.096V range = Gain 1
#define ADS1X15_REG_CONFIG_PGA_2_048V (0x0400) // +/-2.048V range = Gain 2 (default)
#define ADS1X15_REG_CONFIG_PGA_1_024V (0x0600) // +/-1.024V range = Gain 4
#define ADS1X15_REG_CONFIG_PGA_0_512V (0x0800) // +/-0.512V range = Gain 8
#define ADS1X15_REG_CONFIG_PGA_0_256V (0x0A00) // +/-0.256V range = Gain 16

/*
    BIT 8

    Device operating mode
    This bit controls the operating mode.
        0 : Continuous-conversion mode
        1 : Single-shot mode or power-down state (default)

*/

#define ADS1X15_REG_CONFIG_MODE_MASK (0x0100)
#define ADS1X15_REG_CONFIG_MODE_CONTIN (0x0000) // Continuous conversion mode
#define ADS1X15_REG_CONFIG_MODE_SINGLE (0x0100) // Power-down single-shot mode (default)

/*
    BIT 7-5

    Data rate
    These bits control the data rate setting.
        000 : 8 SPS
        001 : 16 SPS
        010 : 32 SPS
        011 : 64 SPS
        100 : 128 SPS (default)
        101 : 250 SPS
        110 : 475 SPS
        111 : 860 SPS

*/

#define ADS1X15_REG_CONFIG_DR_MASK (0x00E0)
#define ADS1015_REG_CONFIG_DR_128SPS (0x0000)  // 128 samples per second
#define ADS1015_REG_CONFIG_DR_250SPS (0x0020)  // 250 samples per second
#define ADS1015_REG_CONFIG_DR_490SPS (0x0040)  // 490 samples per second
#define ADS1015_REG_CONFIG_DR_920SPS (0x0060)  // 920 samples per second
#define ADS1015_REG_CONFIG_DR_1600SPS (0x0080) // 1600 samples per second (default)
#define ADS1015_REG_CONFIG_DR_2400SPS (0x00A0) // 2400 samples per second
#define ADS1015_REG_CONFIG_DR_3300SPS (0x00C0) // 3300 samples per second
#define ADS1115_REG_CONFIG_DR_8SPS (0x0000)    // 8 samples per second
#define ADS1115_REG_CONFIG_DR_16SPS (0x0020)   // 16 samples per second
#define ADS1115_REG_CONFIG_DR_32SPS (0x0040)   // 32 samples per second
#define ADS1115_REG_CONFIG_DR_64SPS (0x0060)   // 64 samples per second
#define ADS1115_REG_CONFIG_DR_128SPS (0x0080)  // 128 samples per second (default)
#define ADS1115_REG_CONFIG_DR_250SPS (0x00A0)  // 250 samples per second
#define ADS1115_REG_CONFIG_DR_475SPS (0x00C0)  // 475 samples per second
#define ADS1115_REG_CONFIG_DR_860SPS (0x00E0)  // 860 samples per second

/*
    BIT 4

    Comparator mode (ADS1114 and ADS1115 only)
    This bit configures the comparator operating mode. This bit serves no function on
    the ADS1113.
        0 : Traditional comparator (default)
        1 : Window comparator

*/

#define ADS1X15_REG_CONFIG_CMODE_MASK (0x0010)
#define ADS1X15_REG_CONFIG_CMODE_TRAD (0x0000)   // Traditional comparator with hysteresis (default)
#define ADS1X15_REG_CONFIG_CMODE_WINDOW (0x0010) // Window comparator

/*
    BIT 3
   
    Comparator polarity (ADS1114 and ADS1115 only)
    This bit controls the polarity of the ALERT/RDY pin. This bit serves no function on
    the ADS1113.
        0 : Active low (default)
        1 : Active high

*/

#define ADS1X15_REG_CONFIG_CPOL_MASK (0x0008)
#define ADS1X15_REG_CONFIG_CPOL_ACTVLOW (0x0000) // ALERT/RDY pin is low when active (default)
#define ADS1X15_REG_CONFIG_CPOL_ACTVHI (0x0008)  // ALERT/RDY pin is high when active

/*
    BIT 2

    Latching comparator (ADS1114 and ADS1115 only)
    This bit controls whether the ALERT/RDY pin latches after being asserted or
    clears after conversions are within the margin of the upper and lower threshold
    values. This bit serves no function on the ADS1113.
        0 : Nonlatching comparator . The ALERT/RDY pin does not latch when asserted (default).
        1 : Latching comparator. The asserted ALERT/RDY pin remains latched until conversion data are read by the master or an appropriate SMBus alert response
            is sent by the master. The device responds with its addre

*/

#define ADS1X15_REG_CONFIG_CLAT_MASK (0x0004)   // Determines if ALERT/RDY pin latches once asserted
#define ADS1X15_REG_CONFIG_CLAT_NONLAT (0x0000) // Non-latching comparator (default)
#define ADS1X15_REG_CONFIG_CLAT_LATCH (0x0004)  // Latching comparator


/*

    BIT 1
    Comparator queue and disable (ADS1114 and ADS1115 only)
    These bits perform two functions. When set to 11, the comparator is disabled and
    the ALERT/RDY pin is set to a high-impedance state. When set to any other
    value, the ALERT/RDY pin and the comparator function are enabled, and the set
    value determines the number of successive conversions exceeding the upper or
    lower threshold required before asserting the ALERT/RDY pin. These bits serve
    no function on the ADS1113.
        00 : Assert after one conversion
        01 : Assert after two conversions
        10 : Assert after four conversions
        11 : Disable comparator and set ALERT/RDY pin to high-impedance (default)

*/

#define ADS1X15_REG_CONFIG_CQUE_MASK (0x0003)
#define ADS1X15_REG_CONFIG_CQUE_1CONV (0x0000) // Assert ALERT/RDY after one conversions
#define ADS1X15_REG_CONFIG_CQUE_2CONV (0x0001) // Assert ALERT/RDY after two conversions
#define ADS1X15_REG_CONFIG_CQUE_4CONV (0x0002) // Assert ALERT/RDY after four conversions
#define ADS1X15_REG_CONFIG_CQUE_NONE (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)

class EmonADS1X15Sensor : public EmonSensor
{

  public:
    // ---------------------------------------------------------------------
    // Public
    // ---------------------------------------------------------------------

    EmonADS1X15Sensor() : EmonSensor()
    {
        _channels = ADS1X15_CHANNELS;
        _sensor_id = SENSOR_EMON_ADS1X15_ID;
        init();
    }

    // ---------------------------------------------------------------------

    void setType(unsigned char type)
    {
        if (_type == type)
            return;
        _type = type;
        _dirty = true;
    }

    void setMask(unsigned char mask)
    {
        if (_mask == mask)
            return;
        _mask = mask;
        _dirty = true;
    }

    void setGain(unsigned int gain)
    {
        if (_gain == gain)
            return;
        _gain = gain;
        _dirty = true;
    }

    // ---------------------------------------------------------------------

    unsigned char getType()
    {
        return _type;
    }

    unsigned char getMask()
    {
        return _mask;
    }

    unsigned char getGain()
    {
        return _gain;
    }

    // ---------------------------------------------------------------------
    // Sensor API
    // ---------------------------------------------------------------------

    // Initialization method, must be idempotent
    void begin()
    {

        if (!_dirty)
            return;

        // Discover
        unsigned char addresses[] = {0x48, 0x49, 0x4A, 0x4B};
        _address = _begin_i2c(_address, sizeof(addresses), addresses);
        if (_address == 0)
            return;

        // Calculate ports
        _ports = 0;
        unsigned char mask = _mask;
        while (mask)
        {
            if (mask & 0x01)
                ++_ports;
            mask = mask >> 1;
        }
        _count = _ports * _magnitudes;

        // Bit depth
        _resolution = ADS1X15_RESOLUTION;

        // Reference based on gain
        if (_gain == ADS1X15_REG_CONFIG_PGA_6_144V)
            _reference = 12.288;
        if (_gain == ADS1X15_REG_CONFIG_PGA_4_096V)
            _reference = 8.192;
        if (_gain == ADS1X15_REG_CONFIG_PGA_2_048V)
            _reference = 4.096;
        if (_gain == ADS1X15_REG_CONFIG_PGA_1_024V)
            _reference = 2.048;
        if (_gain == ADS1X15_REG_CONFIG_PGA_0_512V)
            _reference = 1.024;
        if (_gain == ADS1X15_REG_CONFIG_PGA_0_256V)
            _reference = 0.512;

        // Call the parent class method
        EmonSensor::begin();

        // warmup all channels
        warmup();
    }

    // Descriptive name of the sensor
    String description()
    {
        char buffer[30];
        snprintf(buffer, sizeof(buffer), "EMON @ ADS1%d15 @ I2C (0x%02X)", _type == ADS1X15_CHIP_ADS1015 ? 0 : 1, _address);
        return String(buffer);
    }

    // Descriptive name of the slot # index
    String slot(unsigned char index)
    {
        char buffer[35];
        unsigned char channel = getChannel(index % _ports);
        snprintf(buffer, sizeof(buffer), "EMON @ ADS1%d15 (A%d) @ I2C (0x%02X)", _type == ADS1X15_CHIP_ADS1015 ? 0 : 1, channel, _address);
        return String(buffer);
    }

    // Address of the sensor (it could be the GPIO or I2C address)
    String address(unsigned char index)
    {
        char buffer[10];
        unsigned char channel = getChannel(index % _ports);
        snprintf(buffer, sizeof(buffer), "0x%02X:%u", _address, channel);
        return String(buffer);
    }

    // Type for slot # index
    unsigned char type(unsigned char index)
    {
        unsigned char magnitude = index / _ports;
        unsigned char i = 0;
#if EMON_REPORT_CURRENT
        if (magnitude == i++)
            return MAGNITUDE_CURRENT;
#endif
#if EMON_REPORT_POWER
        if (magnitude == i++)
            return MAGNITUDE_POWER_APPARENT;
#endif
#if EMON_REPORT_ENERGY
        if (magnitude == i)
            return MAGNITUDE_ENERGY;
#endif
        return MAGNITUDE_NONE;
    }

    void pre()
    {
        static unsigned long last = 0;
        for (unsigned char port = 0; port < _ports; port++)
        {
            unsigned char channel = getChannel(port);
            _current[port] = getCurrent(channel);
#if EMON_REPORT_ENERGY
            _energy[port] += (_current[port] * _voltage * (millis() - last) / 1000);
#endif
        }
        last = millis();
        _error = SENSOR_ERROR_OK;
    }

    // Current value for slot # index
    double value(unsigned char index)
    {
        unsigned char port = index % _ports;
        unsigned char magnitude = index / _ports;
        unsigned char i = 0;
#if EMON_REPORT_CURRENT
        if (magnitude == i++)
            return _current[port];
#endif
#if EMON_REPORT_POWER
        if (magnitude == i++)
            return _current[port] * _voltage;
#endif
#if EMON_REPORT_ENERGY
        if (magnitude == i)
            return _energy[port];
#endif
        return 0;
    }

  protected:
    //----------------------------------------------------------------------
    // Protected
    //----------------------------------------------------------------------

    unsigned char getChannel(unsigned char port)
    {
        unsigned char count = 0;
        unsigned char bit = 1;
        for (unsigned char channel = 0; channel < ADS1X15_CHANNELS; channel++)
        {
            if ((_mask & bit) == bit)
            {
                if (count == port)
                    return channel;
                ++count;
            }
            bit <<= 1;
        }
        return 0;
    }

    void warmup()
    {
        for (unsigned char port = 0; port < _ports; port++)
        {
            unsigned char channel = getChannel(port);
            _pivot[channel] = _adc_counts >> 1;
            getCurrent(channel);
        }
    }

    //----------------------------------------------------------------------
    // I2C
    //----------------------------------------------------------------------

    void setConfigRegistry(unsigned char channel, bool continuous, bool start)
    {

        // Start with default values
        uint16_t config = 0;

        config |= ADS1X15_REG_CONFIG_MUX_DIFF_0_1;
        config |= ADS1X15_REG_CONFIG_PGA_2_048V; //gain
        config |= ADS1X15_REG_CONFIG_MODE_SINGLE;
        config |= ADS1X15_REG_CONFIG_CMODE_WINDOW;

        if (start)
        {
            config |= ADS1X15_REG_CONFIG_OS_SINGLE; // Start a single-conversion (0x8000)
        }

        if (continuous)
        {
            //config |= ADS1X15_REG_CONFIG_MODE_CONTIN;   // Continuous mode (default) (0x0000)
        }
        else
        {
            config |= ADS1X15_REG_CONFIG_MODE_SINGLE; // Single-shot mode (0x0100)
        }
        //config |= ((channel + 4) << 12); // Set single-ended input channel (0x4000 - 0x7000)

        // Write config register to the ADC
        i2c_write_uint16(_address, ADS1X15_REG_POINTER_CONFIG, config);
    }

    double getCurrent(unsigned char channel)
    {

        // Force stop by setting single mode and back to continuous
        static unsigned char previous = 9;
        if (previous != channel)
        {
            setConfigRegistry(channel, true, false);
            setConfigRegistry(channel, false, false);
            setConfigRegistry(channel, false, true);
            nice_delay(10);
            readADC(channel);
            previous = channel;
        }
        setConfigRegistry(channel, true, true);

        return read(channel);
    }

    unsigned int readADC(unsigned char channel)
    {
        (void)channel;
        unsigned int value = i2c_read_uint16(_address, ADS1X15_REG_POINTER_CONVERT);
        if (_type = ADS1X15_CHIP_ADS1015)
            value >>= ADS1015_BIT_SHIFT;
        delayMicroseconds(500);
        return value;
    }

    unsigned char _type = ADS1X15_CHIP_ADS1115;
    unsigned char _mask = 0x0F;
    unsigned int _gain = ADS1X15_REG_CONFIG_PGA_2_048V;
    unsigned char _ports;
};

#endif // SENSOR_SUPPORT && EMON_ADS1X15_SUPPORT
