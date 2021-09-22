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

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
/** 7-bit I2C address for this sensor */
// #define FXOS8700_ADDRESS (0x1F) // 0011111
/** Device ID for this sensor (used as sanity check during init) */
#define FXOS8700_ID (0xC7) // 1100 0111
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
/*!
    Raw register addresses used to communicate with the sensor.
*/
typedef enum {
  FXOS8700_REGISTER_STATUS = 0x00,    /**< 0x00 */
  FXOS8700_REGISTER_OUT_X_MSB = 0x01, /**< 0x01 */
  FXOS8700_REGISTER_OUT_X_LSB = 0x02, /**< 0x02 */
  FXOS8700_REGISTER_OUT_Y_MSB = 0x03, /**< 0x03 */
  FXOS8700_REGISTER_OUT_Y_LSB = 0x04, /**< 0x04 */
  FXOS8700_REGISTER_OUT_Z_MSB = 0x05, /**< 0x05 */
  FXOS8700_REGISTER_OUT_Z_LSB = 0x06, /**< 0x06 */
  FXOS8600_REGISTER_SYSMOD = 0x0B,    /**< 0x0B */
  FXOS8700_REGISTER_WHO_AM_I =
      0x0D, /**< 0x0D (default value = 0b11000111, read only) */
  FXOS8700_REGISTER_XYZ_DATA_CFG = 0x0E, /**< 0x0E */
  FXOS8700_REGISTER_CTRL_REG1 =
      0x2A, /**< 0x2A (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG2 =
      0x2B, /**< 0x2B (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG3 =
      0x2C, /**< 0x2C (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG4 =
      0x2D, /**< 0x2D (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_CTRL_REG5 =
      0x2E, /**< 0x2E (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_MSTATUS = 0x32,    /**< 0x32 */
  FXOS8700_REGISTER_MOUT_X_MSB = 0x33, /**< 0x33 */
  FXOS8700_REGISTER_MOUT_X_LSB = 0x34, /**< 0x34 */
  FXOS8700_REGISTER_MOUT_Y_MSB = 0x35, /**< 0x35 */
  FXOS8700_REGISTER_MOUT_Y_LSB = 0x36, /**< 0x36 */
  FXOS8700_REGISTER_MOUT_Z_MSB = 0x37, /**< 0x37 */
  FXOS8700_REGISTER_MOUT_Z_LSB = 0x38, /**< 0x38 */
  FXOS8700_REGISTER_MCTRL_REG1 =
      0x5B, /**< 0x5B (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_MCTRL_REG2 =
      0x5C, /**< 0x5C (default value = 0b00000000, read/write) */
  FXOS8700_REGISTER_MCTRL_REG3 =
      0x5D, /**< 0x5D (default value = 0b00000000, read/write) */
} fxos8700Registers_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SENSOR MODE SETTINGS
    -----------------------------------------------------------------------*/
/*!
    System status for the overall FXOS8700 sensor. Gets the current mode as
    the 2bit sysmod[1:0] setting to ensure proper mode configuration
*/
typedef enum {
  STANDBY = 0x00, /**< sysmod[1:0]. Standby status */
  WAKE = 0x01,    /**< sysmod[1:0]. Wake status */
  SLEEP = 0x02    /**< sysmod[1:0]. Sleep status */
} fxos8700SystemStatus_t;

/*!
    Sensor mode seetings for the overall FXOS8700 sensor. Sets the sensor in
    accelerometer-only, magnerometer-only, or hybrid modes. Sent to
    FXOS8700_REGISTER_MCTRL_REG1
*/
typedef enum {
  ACCEL_ONLY_MODE = 0b00, /**< m_hms[1:0] = 0b00. Accel-only mode */
  MAG_ONLY_MODE = 0b01,   /**< m_hms[1:0] = 0b01. Mag-only mode */
  HYBRID_MODE = 0b11      /**< m_hms[1:0] = 0b11. Hybrid mode */
} fxos8700SensorMode_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
/*!
    Output Data Rate (ODR) key for the overall FXOS8700 sensor. Called by
    user for convenient variable name matching
*/
typedef enum {
  ODR_800HZ,    /**< 800Hz, only available in accel/mag-only modes */
  ODR_400HZ,    /**< 400Hz, available in all modes */
  ODR_200HZ,    /**< 200Hz, available in all modes */
  ODR_100HZ,    /**< 100Hz, available in all modes */
  ODR_50HZ,     /**< 50Hz, available in all modes */
  ODR_25HZ,     /**< 25Hz, only available in hybrid mode */
  ODR_12_5HZ,   /**< 12.5Hz, only available in accel/mag-only modes */
  ODR_6_25HZ,   /**< 6.25Hz, available in all modes */
  ODR_3_125HZ,  /**< 3.125Hz, only available in hybrid mode */
  ODR_1_5625HZ, /**< 3.125Hz, only available in accel/mag-only modes */
  ODR_0_7813HZ, /**< 0.7813Hz, only available in hybrid mode */
} fxos8700ODR_t;

/*!
    Output Data Rates (ODRs) available for accel/mag-only modes, type array
    of user set fxos8700ODR_t
*/
const fxos8700ODR_t ACCEL_MAG_ONLY_AVAILABLE_ODRs[] = {
    ODR_800HZ,    /**< 800Hz, only available in accel/mag-only modes */
    ODR_400HZ,    /**< 400Hz, available in all modes */
    ODR_200HZ,    /**< 200Hz, available in all modes */
    ODR_100HZ,    /**< 100Hz, available in all modes */
    ODR_50HZ,     /**< 50Hz, available in all modes */
    ODR_12_5HZ,   /**< 12.5Hz, only available in accel/mag-only modes */
    ODR_6_25HZ,   /**< 6.25Hz, available in all modes */
    ODR_1_5625HZ, /**< 3.125Hz, only available in accel/mag-only modes */
};

/*!
    Output Data Rates (ODRs) available for hybrid mode, type array of user
    set fxos8700ODR_t
*/
const fxos8700ODR_t HYBRID_AVAILABLE_ODRs[] = {
    ODR_400HZ,    /**< 400Hz, available in all modes */
    ODR_200HZ,    /**< 200Hz, available in all modes */
    ODR_100HZ,    /**< 100Hz, available in all modes */
    ODR_50HZ,     /**< 50Hz, available in all modes */
    ODR_25HZ,     /**< 25Hz, only available in hybrid mode */
    ODR_6_25HZ,   /**< 6.25Hz, available in all modes */
    ODR_3_125HZ,  /**< 3.125Hz, only available in hybrid mode */
    ODR_0_7813HZ, /**< 0.7813Hz, only available in hybrid mode */
};

/*!
    Output Data Rate (ODR) settings to write to the dr[2:0] bits in
    CTRL_REG_1, array type to pass the index from available accel/mag-only
    and hyrbrid modes
*/
const uint16_t ODR_drBits[] = {
    0x00, /**< dr=0b000. 800Hz accel/mag-only modes, 400Hz hyrbid mode */
    0x08, /**< dr=0b001. 400Hz accel/mag-only modes, 200Hz hyrbid mode */
    0x10, /**< dr=0b010. 200Hz accel/mag-only modes, 100Hz hyrbid mode */
    0x18, /**< dr=0b011. 100Hz accel/mag-only modes, 50Hz hyrbid mode */
    0x20, /**< dr=0b100. 50Hz accel/mag-only modes, 25Hz hyrbid mode */
    0x28, /**< dr=0b101. 12.5Hz accel/mag-only modes, 6.25Hz hyrbid mode */
    0x30, /**< dr=0b110. 6.25Hz accel/mag-only modes, 3.125Hz hyrbid mode */
    0x38, /**< dr=0b111. 1.5625Hz accel/mag-only modes, 0.7813Hz hyrbid mode */
};
/*=========================================================================*/

/*=========================================================================
    OPTIONAL RANGE SETTINGS
    -----------------------------------------------------------------------*/
/*!
    Range settings for the accelerometer sensor.
*/
typedef enum {
  ACCEL_RANGE_2G = 0x00, /**< +/- 2g range */
  ACCEL_RANGE_4G = 0x01, /**< +/- 4g range */
  ACCEL_RANGE_8G = 0x02  /**< +/- 8g range */
} fxos8700AccelRange_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL MAGNETOMETER OVERSAMPLING SETTINGS
    -----------------------------------------------------------------------*/
/*!
    Range settings for the accelerometer sensor.
*/
typedef enum {
  MAG_OSR_0, /**< Mag oversampling ratio = 0 */
  MAG_OSR_1, /**< Mag oversampling ratio = 1 */
  MAG_OSR_2, /**< Mag oversampling ratio = 2 */
  MAG_OSR_3, /**< Mag oversampling ratio = 3 */
  MAG_OSR_4, /**< Mag oversampling ratio = 4 */
  MAG_OSR_5, /**< Mag oversampling ratio = 5 */
  MAG_OSR_6, /**< Mag oversampling ratio = 6 */
  MAG_OSR_7, /**< Mag oversampling ratio = 7 */
} fxos8700MagOSR_t;
/*=========================================================================*/

/*=========================================================================
    RAW 3DOF SENSOR DATA TYPE
    -----------------------------------------------------------------------*/
/*!
    @brief  Raw (integer) values from a 3dof sensor.
*/
typedef struct {
  int16_t x; /**< Raw int16_t value from the x axis */
  int16_t y; /**< Raw int16_t value from the y axis */
  int16_t z; /**< Raw int16_t value from the z axis */
} fxos8700RawData_t;
/*=========================================================================*/

class Adafruit_FXOS8700;

/** Adafruit Unified Sensor interface for accelerometer component of FXOS8700 */
class Adafruit_FXOS8700_Accelerometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the FXOS8700 class */
  Adafruit_FXOS8700_Accelerometer(Adafruit_FXOS8700 *parent) {
    _theFXOS8700 = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 8701;
  Adafruit_FXOS8700 *_theFXOS8700 = NULL;
};

/** Adafruit Unified Sensor interface for magnetometer component of FXOS8700 */
class Adafruit_FXOS8700_Magnetometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the magnetometer
     sensor
      @param parent A pointer to the FXOS8700 class */
  Adafruit_FXOS8700_Magnetometer(Adafruit_FXOS8700 *parent) {
    _theFXOS8700 = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 8702;
  Adafruit_FXOS8700 *_theFXOS8700 = NULL;
};

/**************************************************************************/
/*!
    @brief  Unified sensor driver for the Adafruit FXOS8700 breakout.
*/
/**************************************************************************/
class Adafruit_FXOS8700 : public Adafruit_Sensor {
public:
  Adafruit_FXOS8700(int32_t accelSensorID = -1, int32_t magSensorID = -1);
  ~Adafruit_FXOS8700();
  bool begin(uint8_t addr = 0x1F, TwoWire *wire = &Wire);

  bool getEvent(sensors_event_t *accel);
  void getSensor(sensor_t *singleSensorEvent);
  bool getEvent(sensors_event_t *accel, sensors_event_t *mag);
  void getSensor(sensor_t *accel, sensor_t *mag);
  void standby(boolean standby);

  /*! Raw accelerometer values from last sucsessful sensor read */
  fxos8700RawData_t accel_raw;
  /*! Raw magnetometer values from last successful sensor read */
  fxos8700RawData_t mag_raw;

  Adafruit_Sensor *getMagnetometerSensor(void);
  Adafruit_Sensor *getAccelerometerSensor(void);

  Adafruit_FXOS8700_Accelerometer *accel_sensor =
      NULL; ///< Accelerometer data object
  Adafruit_FXOS8700_Magnetometer *mag_sensor = NULL; ///< Mag data object

  void setSensorMode(fxos8700SensorMode_t mode);
  fxos8700SensorMode_t getSensorMode();

  void setAccelRange(fxos8700AccelRange_t range);
  fxos8700AccelRange_t getAccelRange();

  void setOutputDataRate(fxos8700ODR_t rate);
  fxos8700ODR_t getOutputDataRate();

  void setMagOversamplingRatio(fxos8700MagOSR_t ratio);
  fxos8700MagOSR_t getMagOversamplingRatio();

protected:
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface

private:
  bool initialize();
  fxos8700SensorMode_t _mode = HYBRID_MODE;
  fxos8700AccelRange_t _range = ACCEL_RANGE_2G;
  fxos8700ODR_t _rate = ODR_100HZ;
  fxos8700MagOSR_t _ratio = MAG_OSR_7;
  int32_t _accelSensorID;
  int32_t _magSensorID;
};

#endif
