/*!
 * @file Adafruit_FXOS8700.h
 *
 * This is part of Adafruit's FXOS8700 driver for the Arduino platform.  It is
 * designed specifically to work with the Adafruit FXOS8700 breakout:
 * https://www.adafruit.com/products/3463
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
/** \file Adafruit_FXOS8700.h */
#ifndef __FXOS8700_H__
#define __FXOS8700_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    /** 7-bit I2C address for this sensor */
    #define FXOS8700_ADDRESS           (0x1F)     // 0011111
    /** Device ID for this sensor (used as sanity check during init) */
    #define FXOS8700_ID                (0xC7)     // 1100 0111
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    /*!
        Raw register addresses used to communicate with the sensor.
    */
    typedef enum
    {
      FXOS8700_REGISTER_STATUS          = 0x00, /**< 0x00 */
      FXOS8700_REGISTER_OUT_X_MSB       = 0x01, /**< 0x01 */
      FXOS8700_REGISTER_OUT_X_LSB       = 0x02, /**< 0x02 */
      FXOS8700_REGISTER_OUT_Y_MSB       = 0x03, /**< 0x03 */
      FXOS8700_REGISTER_OUT_Y_LSB       = 0x04, /**< 0x04 */
      FXOS8700_REGISTER_OUT_Z_MSB       = 0x05, /**< 0x05 */
      FXOS8700_REGISTER_OUT_Z_LSB       = 0x06, /**< 0x06 */
      FXOS8700_REGISTER_WHO_AM_I        = 0x0D, /**< 0x0D (default value = 0b11000111, read only) */
      FXOS8700_REGISTER_XYZ_DATA_CFG    = 0x0E, /**< 0x0E */
      FXOS8700_REGISTER_CTRL_REG1       = 0x2A, /**< 0x2A (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_CTRL_REG2       = 0x2B, /**< 0x2B (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_CTRL_REG3       = 0x2C, /**< 0x2C (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_CTRL_REG4       = 0x2D, /**< 0x2D (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_CTRL_REG5       = 0x2E, /**< 0x2E (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_MSTATUS         = 0x32, /**< 0x32 */
      FXOS8700_REGISTER_MOUT_X_MSB      = 0x33, /**< 0x33 */
      FXOS8700_REGISTER_MOUT_X_LSB      = 0x34, /**< 0x34 */
      FXOS8700_REGISTER_MOUT_Y_MSB      = 0x35, /**< 0x35 */
      FXOS8700_REGISTER_MOUT_Y_LSB      = 0x36, /**< 0x36 */
      FXOS8700_REGISTER_MOUT_Z_MSB      = 0x37, /**< 0x37 */
      FXOS8700_REGISTER_MOUT_Z_LSB      = 0x38, /**< 0x38 */
      FXOS8700_REGISTER_MCTRL_REG1      = 0x5B, /**< 0x5B (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_MCTRL_REG2      = 0x5C, /**< 0x5C (default value = 0b00000000, read/write) */
      FXOS8700_REGISTER_MCTRL_REG3      = 0x5D, /**< 0x5D (default value = 0b00000000, read/write) */
    } fxos8700Registers_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
    /*!
        Range settings for the accelerometer sensor.
    */
    typedef enum
    {
      ACCEL_RANGE_2G                    = 0x00, /**< +/- 2g range */
      ACCEL_RANGE_4G                    = 0x01, /**< +/- 4g range */
      ACCEL_RANGE_8G                    = 0x02  /**< +/- 8g range */
    } fxos8700AccelRange_t;
/*=========================================================================*/

/*=========================================================================
    RAW GYROSCOPE DATA TYPE
    -----------------------------------------------------------------------*/
    /*!
        @brief  Raw (integer) values from the gyroscope sensor.
    */
    typedef struct
    {
      int16_t x;    /**< Raw int16_t value from the x axis */
      int16_t y;    /**< Raw int16_t value from the y axis */
      int16_t z;    /**< Raw int16_t value from the z axis */
    } fxos8700RawData_t;
/*=========================================================================*/

/**************************************************************************/
/*!
    @brief  Unified sensor driver for the Adafruit FXOS8700 breakout.
*/
/**************************************************************************/
class Adafruit_FXOS8700 : public Adafruit_Sensor
{
  public:
    Adafruit_FXOS8700(int32_t accelSensorID = -1, int32_t magSensorID = -1);

    bool begin           ( fxos8700AccelRange_t rng = ACCEL_RANGE_2G );
    bool getEvent        ( sensors_event_t* accel );
    void getSensor       ( sensor_t* accel );
    bool getEvent        ( sensors_event_t* accel, sensors_event_t* mag );
    void getSensor       ( sensor_t* accel, sensor_t* mag );
    void standby         ( boolean standby );

    /*! Raw accelerometer values from last sucsessful sensor read */
    fxos8700RawData_t accel_raw;
    /*! Raw magnetometer values from last successful sensor read */
    fxos8700RawData_t mag_raw;

  private:
    void        write8  ( byte reg, byte value );
    byte        read8   ( byte reg );

    fxos8700AccelRange_t _range;
    int32_t              _accelSensorID;
    int32_t              _magSensorID;
};

#endif
