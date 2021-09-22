/*!
 * @file Adafruit_FXOS8700.cpp
 *
 * @mainpage Adafruit FXOS8700 accel/mag sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's FXOS8700 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit FXOS8700 breakout: https://www.adafruit.com/products/3463
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a
 * href="https://github.com/adafruit/Adafruit_Sensor"> Adafruit_Sensor</a> being
 * present on your system. Please make sure you have installed the latest
 * version before using this library.
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#include "Adafruit_FXOS8700.h"
#include <limits.h>

/** Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000244mg) */
#define ACCEL_MG_LSB_2G (0.000244F)
/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000488mg) */
#define ACCEL_MG_LSB_4G (0.000488F)
/** Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000976mg) */
#define ACCEL_MG_LSB_8G (0.000976F)
/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB (0.1F)

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Initializes the hardware to a default state.

    @return True if the device was successfully initialized, otherwise false.
*/
/**************************************************************************/
bool Adafruit_FXOS8700::initialize() {
  Adafruit_BusIO_Register CTRL_REG1(i2c_dev, FXOS8700_REGISTER_CTRL_REG1);
  Adafruit_BusIO_Register CTRL_REG2(i2c_dev, FXOS8700_REGISTER_CTRL_REG2);
  Adafruit_BusIO_RegisterBits lnoise_bit(&CTRL_REG1, 1, 2);
  Adafruit_BusIO_RegisterBits mods_bits(&CTRL_REG2, 2, 0);

  /* Set the full scale range of the accelerometer */
  setAccelRange(ACCEL_RANGE_2G);

  /* Low Noise & High accelerometer OSR resolution */
  standby(true);
  lnoise_bit.write(0x01);
  mods_bits.write(0x02);
  standby(false);

  /* Set in hybrid mode, jumps to reg 0x33 after reading 0x06 */
  setSensorMode(HYBRID_MODE);
  /* Set the output data rate to 100Hz, default */
  setOutputDataRate(ODR_100HZ);

  /* Configure the magnetometer */
  /* Highest Over Sampling Ratio = 7 > (Over Sampling Rate = 16 @ 100Hz ODR)*/
  setMagOversamplingRatio(MAG_OSR_7);

  /* Clear the raw sensor data */
  accel_raw.x = 0;
  accel_raw.y = 0;
  accel_raw.z = 0;
  mag_raw.x = 0;
  mag_raw.y = 0;
  mag_raw.z = 0;

  return true;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_FXOS8700 class, including assigning
            a unique ID to the accel and magnetometer for logging purposes.

    @param accelSensorID The unique ID to associate with the accelerometer.
    @param magSensorID The unique ID to associate with the magnetometer.
*/
/**************************************************************************/
Adafruit_FXOS8700::Adafruit_FXOS8700(int32_t accelSensorID,
                                     int32_t magSensorID) {
  _accelSensorID = accelSensorID;
  _magSensorID = magSensorID;

  accel_sensor = new Adafruit_FXOS8700_Accelerometer(this);
  mag_sensor = new Adafruit_FXOS8700_Magnetometer(this);
}

/***************************************************************************
 DESTRUCTOR
 ***************************************************************************/
Adafruit_FXOS8700::~Adafruit_FXOS8700() {
  if (i2c_dev)
    delete i2c_dev;
  if (accel_sensor)
    delete accel_sensor;
  if (mag_sensor)
    delete mag_sensor;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Initializes the hardware.

    @param  addr I2C address for the device.
    @param  wire Pointer to Wire instance

    @return True if the device was successfully initialized, otherwise false.
*/
/**************************************************************************/
bool Adafruit_FXOS8700::begin(uint8_t addr, TwoWire *wire) {
  if (i2c_dev)
    delete i2c_dev;
  i2c_dev = new Adafruit_I2CDevice(addr, wire);
  if (!i2c_dev->begin())
    return false;

  Adafruit_BusIO_Register WHO_AM_I(i2c_dev, FXOS8700_REGISTER_WHO_AM_I);
  if (WHO_AM_I.read() != FXOS8700_ID)
    return false;

  return initialize();
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor events.

            This function reads from both the accelerometer and the
            magnetometer in one call, and is a deviation from the standard
            Adafruit_Sensor API, but is provided as a convenience since most
            AHRS algorithms require sensor samples to be as close in time as
            possible.

    @param    accelEvent
              A reference to the sensors_event_t instances where the
              accelerometer data should be written.
    @param    magEvent
              A reference to the sensors_event_t instances where the
              magnetometer data should be written.

    @return True if the event read was successful, otherwise false.
*/
/**************************************************************************/
bool Adafruit_FXOS8700::getEvent(sensors_event_t *accelEvent,
                                 sensors_event_t *magEvent) {

  /* Read 13 bytes from the sensor */
  uint8_t buffer[13];
  buffer[0] = FXOS8700_REGISTER_STATUS;
  i2c_dev->write_then_read(buffer, 1, buffer, 13);

  uint32_t const timestamp = millis();

  if (accelEvent) {
    /* Clear the event */
    memset(accelEvent, 0, sizeof(sensors_event_t));

    /* Clear the raw data placeholder */
    accel_raw.x = 0;
    accel_raw.y = 0;
    accel_raw.z = 0;

    /* Set the static metadata */
    accelEvent->version = sizeof(sensors_event_t);
    accelEvent->sensor_id = _accelSensorID;
    accelEvent->type = SENSOR_TYPE_ACCELEROMETER;

    /* Set the timestamps */
    accelEvent->timestamp = timestamp;

    /* Shift values to create properly formed integers */
    /* Note, accel data is 14-bit and left-aligned, so we shift two bit right */
    accelEvent->acceleration.x = (int16_t)((buffer[1] << 8) | buffer[2]) >> 2;
    accelEvent->acceleration.y = (int16_t)((buffer[3] << 8) | buffer[4]) >> 2;
    accelEvent->acceleration.z = (int16_t)((buffer[5] << 8) | buffer[6]) >> 2;

    /* Assign raw values in case someone needs them */
    accel_raw.x = accelEvent->acceleration.x;
    accel_raw.y = accelEvent->acceleration.y;
    accel_raw.z = accelEvent->acceleration.z;

    /* Convert accel values to m/s^2 */
    switch (_range) {
    case (ACCEL_RANGE_2G):
      accelEvent->acceleration.x *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
      accelEvent->acceleration.y *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
      accelEvent->acceleration.z *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
      break;
    case (ACCEL_RANGE_4G):
      accelEvent->acceleration.x *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
      accelEvent->acceleration.y *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
      accelEvent->acceleration.z *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
      break;
    case (ACCEL_RANGE_8G):
      accelEvent->acceleration.x *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
      accelEvent->acceleration.y *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
      accelEvent->acceleration.z *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
      break;
    }
  }

  if (magEvent) {
    memset(magEvent, 0, sizeof(sensors_event_t));

    mag_raw.x = 0;
    mag_raw.y = 0;
    mag_raw.z = 0;

    magEvent->version = sizeof(sensors_event_t);
    magEvent->sensor_id = _magSensorID;
    magEvent->type = SENSOR_TYPE_MAGNETIC_FIELD;

    magEvent->timestamp = timestamp;

    magEvent->magnetic.x = (int16_t)((buffer[7] << 8) | buffer[8]);
    magEvent->magnetic.y = (int16_t)((buffer[9] << 8) | buffer[10]);
    magEvent->magnetic.z = (int16_t)((buffer[11] << 8) | buffer[12]);

    mag_raw.x = magEvent->magnetic.x;
    mag_raw.y = magEvent->magnetic.y;
    mag_raw.z = magEvent->magnetic.z;

    /* Convert mag values to uTesla */
    magEvent->magnetic.x *= MAG_UT_LSB;
    magEvent->magnetic.y *= MAG_UT_LSB;
    magEvent->magnetic.z *= MAG_UT_LSB;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets sensor_t data for both the accel and mag in one operation.

    @param    accelSensor
              A reference to the sensor_t instances where the
              accelerometer sensor info should be written.
    @param    magSensor
              A reference to the sensor_t instances where the
              magnetometer sensor info should be written.
*/
/**************************************************************************/
void Adafruit_FXOS8700::getSensor(sensor_t *accelSensor, sensor_t *magSensor) {
  /* Clear the sensor_t object */
  memset(accelSensor, 0, sizeof(sensor_t));
  memset(magSensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(accelSensor->name, "FXOS8700", sizeof(accelSensor->name) - 1);
  accelSensor->name[sizeof(accelSensor->name) - 1] = 0;
  accelSensor->version = 1;
  accelSensor->sensor_id = _accelSensorID;
  accelSensor->type = SENSOR_TYPE_ACCELEROMETER;
  accelSensor->min_delay = 0.01F; // 100Hz
  switch (_range) {
  case (ACCEL_RANGE_2G):
    accelSensor->max_value = 2.0F * SENSORS_GRAVITY_STANDARD;
    accelSensor->min_value = -1.999F * SENSORS_GRAVITY_STANDARD;
    accelSensor->resolution = ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
    break;
  case (ACCEL_RANGE_4G):
    accelSensor->max_value = 4.0F * SENSORS_GRAVITY_STANDARD;
    accelSensor->min_value = -3.998F * SENSORS_GRAVITY_STANDARD;
    accelSensor->resolution = ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
    break;
  case (ACCEL_RANGE_8G):
    accelSensor->max_value = 8.0F * SENSORS_GRAVITY_STANDARD;
    accelSensor->min_value = -7.996F * SENSORS_GRAVITY_STANDARD;
    accelSensor->resolution = ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
    break;
  }

  strncpy(magSensor->name, "FXOS8700", sizeof(magSensor->name) - 1);
  magSensor->name[sizeof(magSensor->name) - 1] = 0;
  magSensor->version = 1;
  magSensor->sensor_id = _magSensorID;
  magSensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  magSensor->min_delay = 0.01F; // 100Hz
  magSensor->max_value = 1200.0F;
  magSensor->min_value = -1200.0F;
  magSensor->resolution = 0.1F;
}

/**************************************************************************/
/*!
    @brief    Reads a single sensor event from the accelerometer.

    @attention

    This function exists to keep Adafruit_Sensor happy since we
    need a single sensor in the canonical .getEvent call. The
    non-standard .getEvent call with two parameters should
    generally be used with this sensor.

    @param    singleSensorEvent
              A reference to the sensors_event_t instances where the
              accelerometer or magnetometer data should be written.

    @return True if the event read was successful, otherwise false.
*/
/**************************************************************************/
bool Adafruit_FXOS8700::getEvent(sensors_event_t *singleSensorEvent) {
  sensors_event_t dummy;
  switch (_mode) {
  case ACCEL_ONLY_MODE:
    return getEvent(singleSensorEvent, &dummy);
  case MAG_ONLY_MODE:
    return getEvent(&dummy, singleSensorEvent);
  default:
    return false;
  }
}

/**************************************************************************/
/*!
    @brief   Gets sensor details about the accelerometer.

    @attention

    This function exists to keep Adafruit_Sensor happy since we
    need a single sensor in the canonical .getSensor call. The
    non-standard .getSensor call with two parameters should
    generally be used with this sensor.

    @param   accelSensor
             A reference to the sensor_t instances where the
             accelerometer sensor info should be written.
*/
/**************************************************************************/
void Adafruit_FXOS8700::getSensor(sensor_t *accelSensor) {
  sensor_t accel;

  return getSensor(accelSensor, &accel);
}

/**************************************************************************/
/*!
    @brief  Puts device into/out of standby mode

    @param standby Set this to a non-zero value to enter standy mode.
*/
/**************************************************************************/
void Adafruit_FXOS8700::standby(boolean standby) {
  Adafruit_BusIO_Register CTRL_REG1(i2c_dev, FXOS8700_REGISTER_CTRL_REG1);
  Adafruit_BusIO_Register SYS_MOD(i2c_dev, FXOS8600_REGISTER_SYSMOD);
  Adafruit_BusIO_RegisterBits standby_bit(&CTRL_REG1, 1, 0);
  Adafruit_BusIO_RegisterBits sysmode(&SYS_MOD, 2, 0);

  if (standby) {
    standby_bit.write(0);
    while (sysmode.read() != STANDBY) {
      delay(10);
    }
  } else {
    standby_bit.write(1);
    while (sysmode.read() == STANDBY) {
      delay(10);
    }
  }
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the accelerometer
    sensor component
    @return Adafruit_Sensor pointer to accelerometer sensor
 */
Adafruit_Sensor *Adafruit_FXOS8700::getAccelerometerSensor(void) {
  return accel_sensor;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the mag sensor component
    @return Adafruit_Sensor pointer to mag sensor
 */
Adafruit_Sensor *Adafruit_FXOS8700::getMagnetometerSensor(void) {
  return mag_sensor;
}

/**************************************************************************/
/*!
    @brief  Set the sensor mode to hybrid, or accel/mag-only modes

    @param mode The sensor mode to set.
*/
/**************************************************************************/
void Adafruit_FXOS8700::setSensorMode(fxos8700SensorMode_t mode) {
  Adafruit_BusIO_Register MCTRL_REG1(i2c_dev, FXOS8700_REGISTER_MCTRL_REG1);
  Adafruit_BusIO_Register MCTRL_REG2(i2c_dev, FXOS8700_REGISTER_MCTRL_REG2);
  Adafruit_BusIO_RegisterBits fs_bits_mreg1(&MCTRL_REG1, 2, 0);
  Adafruit_BusIO_RegisterBits fs_bit_mreg2(&MCTRL_REG2, 1, 5);

  standby(true);
  fs_bits_mreg1.write(mode);
  fs_bit_mreg2.write(mode == HYBRID_MODE ? 1 : 0);
  standby(false);

  _mode = mode;
}

/**************************************************************************/
/*!
    @brief  Get the sensor mode (either hybrid, or accel/mag-only).

    @return The sensor mode.
*/
/**************************************************************************/
fxos8700SensorMode_t Adafruit_FXOS8700::getSensorMode() { return _mode; }

/**************************************************************************/
/*!
    @brief  Set the accelerometer full scale range.

    @attention

    This function resets the lnoise bit to low if a 8G accelerometer range
    is requested. Otherwise, the active lnoise bit won't allow the sensor
    to measure past 4G.

    @param range The accelerometer full scale range to set.
*/
/**************************************************************************/
void Adafruit_FXOS8700::setAccelRange(fxos8700AccelRange_t range) {
  Adafruit_BusIO_Register CTRL_REG1(i2c_dev, FXOS8700_REGISTER_CTRL_REG1);
  Adafruit_BusIO_Register XYZ_DATA_CFG(i2c_dev, FXOS8700_REGISTER_XYZ_DATA_CFG);
  Adafruit_BusIO_RegisterBits lnoise_bit(&CTRL_REG1, 1, 2);
  Adafruit_BusIO_RegisterBits fs_bits(&XYZ_DATA_CFG, 2, 0);

  standby(true);
  fs_bits.write(range);
  if (range == ACCEL_RANGE_8G)
    lnoise_bit.write(0x00);
  standby(false);

  _range = range;
}

/**************************************************************************/
/*!
    @brief  Get the accelerometer full scale range.

    @return The accelerometer full scale range.
*/
/**************************************************************************/
fxos8700AccelRange_t Adafruit_FXOS8700::getAccelRange() { return _range; }

/**************************************************************************/
/*!
    @brief  Set the FXOS8700's ODR from any sensor mode.

    @param rate The FXOS8700's ODR.
*/
/**************************************************************************/
void Adafruit_FXOS8700::setOutputDataRate(fxos8700ODR_t rate) {
  Adafruit_BusIO_Register CTRL_REG1(i2c_dev, FXOS8700_REGISTER_CTRL_REG1);
  Adafruit_BusIO_RegisterBits fs_bits(&CTRL_REG1, 3, 3);
  bool isRateInMode = false;
  uint8_t odr;

  if (_mode == HYBRID_MODE) {
    // test if rate param belongs in the available hybrid ODR mode options
    for (int i = 0; i < 8; i++) {
      if (rate == HYBRID_AVAILABLE_ODRs[i]) {
        odr = ODR_drBits[i];
        isRateInMode = true;
        break;
      }
    }
  } else {
    // test if rate param belongs in the available accel/mag-only ODR mode
    // options
    for (int i = 0; i < 8; i++) {
      if (rate == ACCEL_MAG_ONLY_AVAILABLE_ODRs[i]) {
        odr = ODR_drBits[i];
        isRateInMode = true;
        break;
      }
    }
  }

  if (!isRateInMode) {
    // requested rate can't be set in current sensor mode, so return
    // existing rate without setting
    return;
  }

  standby(true);
  CTRL_REG1.write(odr);
  standby(false);

  _rate = rate;
}

/**************************************************************************/
/*!
    @brief  Get the FXOS8700's Hybrid ODR.

    @return The FXOS8700's Hybrid ODR.
*/
/**************************************************************************/
fxos8700ODR_t Adafruit_FXOS8700::getOutputDataRate() { return _rate; }

/**************************************************************************/
/*!
    @brief  Set the magnetometer oversampling ratio (OSR)

    @param ratio The magnetometer OSR to set.
*/
/**************************************************************************/
void Adafruit_FXOS8700::setMagOversamplingRatio(fxos8700MagOSR_t ratio) {
  Adafruit_BusIO_Register MCTRL_REG1(i2c_dev, FXOS8700_REGISTER_MCTRL_REG1);
  Adafruit_BusIO_RegisterBits fs_bits_mreg1(&MCTRL_REG1, 3, 2);

  standby(true);
  fs_bits_mreg1.write(ratio);
  standby(false);

  _ratio = ratio;
}

/**************************************************************************/
/*!
    @brief  Get the magnetometer oversampling ratio (OSR).

    @return The magnetometer OSR.
*/
/**************************************************************************/
fxos8700MagOSR_t Adafruit_FXOS8700::getMagOversamplingRatio() { return _ratio; }

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the FXOS8700's magnetic sensor
*/
/**************************************************************************/
void Adafruit_FXOS8700_Magnetometer::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "FXOS8700_M", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->min_value = -1200;
  sensor->max_value = +1200;
  sensor->resolution = 0;
}

/**************************************************************************/
/*!
    @brief  Gets the magnetometer as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool Adafruit_FXOS8700_Magnetometer::getEvent(sensors_event_t *event) {
  return _theFXOS8700->getEvent(NULL, event);
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the FXOS8700's accelerometer
*/
/**************************************************************************/
void Adafruit_FXOS8700_Accelerometer::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "FXOS8700_A", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->min_value = -78.4532F; /*  -8g = 78.4532 m/s^2  */
  sensor->max_value = 78.4532F;  /* 8g = 78.4532 m/s^2  */
  sensor->resolution = 0.061;    /* 0.061 mg/LSB at +-2g */
}

/**************************************************************************/
/*!
    @brief  Gets the accelerometer as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool Adafruit_FXOS8700_Accelerometer::getEvent(sensors_event_t *event) {
  return _theFXOS8700->getEvent(event, NULL);
}
