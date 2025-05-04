/**
 * @file ads1115.cpp
 * @brief Implementation of ADS1115 driver for Raspberry Pi using pigpio I2C interface.
 *
 * Provides methods to configure and read from the ADS1115 16-bit ADC over I2C.
 * Supports single-ended and differential measurements, configurable gain, data rate,
 * comparator alerts, and both single-shot and continuous conversion modes.
 *
 * Dependencies:
 *   - pigpio for I2C (i2cOpen, i2cReadWordData, i2cWriteWordData)
 *   - unistd.h for usleep()
 *   - ctime for timeout logic
 *
 * Language: C++
 * Platform: Raspberry Pi
 * Author: Mateusz Synak
 * Created: 04.05.2025
 * License: MIT
 */

#include "ads1115.h"
#include <iostream>
#include <cstdint>
//#include <iomanip>
#include <pigpio.h>
#include <unistd.h>
#include <cstdio>
#include <ctime>
//#include <cerrno>

/**
 * @brief Constructs the ADS1115 object and opens the I2C connection.
 *
 * @param i2cBus       I2C bus number (e.g., 1 for /dev/i2c-1).
 * @param i2cAddress   7-bit I2C address (use one of ADS1115_ADDRESS_GND.._SCL defines).
 * @param dr           Data rate (use ADS1115_8SPS, 16SPS, 32SPS, 64SPS, 128SPS, 250SPS, 475SPS, or 860SPS).
 * @param gain         PGA gain (use ADS1115_GAIN_6P144V, _4P096V, _2P048V, _1P024V, _0P512V, or _0P256V).
 */
ADS1115::ADS1115(int i2cBus, uint8_t i2cAddress, uint8_t dr, uint8_t gain)
  : address(i2cAddress),
    dataRate(dr),
    gainSetting(gain),
    muxSetting(ADS1115_MUX_AIN0_AIN1_DIFF),
    modeSetting(ADS1115_SINGLE_CONVERSION),
    compMode(ADS1115_COMP_MODE_HYSTERESIS),
    compPol(ADS1115_COMP_POL_ACTIVE_LOW),
    compLat(ADS1115_COMP_LAT_NON_LATCHING),
    compQue(ADS1115_COMP_DISABLED)
    {
      handle = i2cOpen(i2cBus, i2cAddress, 0);
      if (handle < 0) {
        fprintf(stderr, "[ADS1115] Failed to open I2C bus %d at address 0x%02X\n", i2cBus, i2cAddress);
      }
    }
 /**
  * @brief Destroys the ADS1115 object and closes the I2C connection if open.
  */
ADS1115::~ADS1115() {
  if (handle >= 0) {
    i2cClose(handle);
  }
}

/**
 * @brief Checks if the ADS1115 driver is initialized and I2C handle is valid.
 *
 * @return true if initialized; false otherwise.
 */
bool ADS1115::isClassInitialized() const {
    return handle >= 0;
}

/**
 * @brief Initializes ADS1115 registers based on stored configuration.
 *
 * Configures data rate, gain, multiplexer, mode, and comparator settings.
 */
void ADS1115::init() {
  setDataRate(dataRate);
  setGain(ADS1115_GAIN_2P048V);
  setMultiplexer(ADS1115_MUX_AIN0_AIN1_DIFF);
  setMode(ADS1115_SINGLE_CONVERSION);
  setComparatorMode(ADS1115_COMP_MODE_HYSTERESIS);
  setComparatorPolarity(ADS1115_COMP_POL_ACTIVE_LOW);
  setComparatorLatchEnabled(ADS1115_COMP_LAT_NON_LATCHING);
  setComparatorQueueMode(ADS1115_COMP_DISABLED);
}

/**
 * @brief Performs a self-test by reading the CONFIG register.
 *
 * @return true if CONFIG register contains valid bits; false on error or invalid data.
 */
bool ADS1115::selfTest() {
  uint16_t cfg;
  if (!readRegister(ADS1115_REG_CONFIG, cfg)) return false;
  return cfg != 0x0000 && cfg != 0xFFFF;
}

/**
 * @brief Performs a single-shot conversion and polls until complete, returning raw value.
 *
 * This function performs a single ADC conversion on the selected input channel or differential pair.
 * The function blocks execution until the conversion is complete and then reads the raw ADC result.
 *
 * @param mux     Input multiplexer setting (0..7).
 *                It is recommended to use the following #define constants for clarity:
 *                  - ADS1115_MUX_AIN0_AIN1_DIFF (0x00) – default
 *                  - ADS1115_MUX_AIN0_AIN3_DIFF (0x01)
 *                  - ADS1115_MUX_AIN1_AIN3_DIFF (0x02)
 *                  - ADS1115_MUX_AIN2_AIN3_DIFF (0x03)
 *                  - ADS1115_MUX_AIN0_GND_SINGLE (0x04)
 *                  - ADS1115_MUX_AIN1_GND_SINGLE (0x05)
 *                  - ADS1115_MUX_AIN2_GND_SINGLE (0x06)
 *                  - ADS1115_MUX_AIN3_GND_SINGLE (0x07)
 *
 * @param result  Reference to variable where the unsigned raw ADC result will be stored.
 * @return true on success; false on invalid mux value or I2C communication failure.
 */
bool ADS1115::performSingleConversionPoll(uint8_t mux, uint16_t &result) {
  if(mux > 7) return false;
  if(!performSingleConversion(mux))   return false;
  if(!waitForConversion())            return false;
  if (!readRaw(result))               return false;
  return true;
}

/**
 * @brief Performs a single-shot differential conversion and polls until complete.
 *
 * This function performs a single differential ADC conversion on the selected input pair.
 * It blocks until the conversion is complete, then reads the signed raw ADC result.
 *
 * @param mux     Differential multiplexer setting (0..3).
 *                It is recommended to use the following #define constants for clarity:
 *                  - ADS1115_MUX_AIN0_AIN1_DIFF (0x00)
 *                  - ADS1115_MUX_AIN0_AIN3_DIFF (0x01)
 *                  - ADS1115_MUX_AIN1_AIN3_DIFF (0x02)
 *                  - ADS1115_MUX_AIN2_AIN3_DIFF (0x03)
 *                Using values outside this range will result in failure.
 *
 * @param result  Reference to variable where the signed raw ADC result will be stored.
 * @return true on success; false on invalid mux value or I2C communication failure.
 */
bool ADS1115::performSingleConversionDiffPoll(uint8_t mux, int16_t &result) {
  if(mux > 3) return false;
  if(!performSingleConversion(mux))   return false;
  if(!waitForConversion())            return false;
  if (!readRawDiff(result))           return false;
  return true;
}

/**
 * @brief Performs a single-shot conversion and polls until complete, returning voltage.
 *
 * This function performs a single ADC conversion in single-ended or differential mode
 * depending on the MUX setting. It blocks until the conversion is complete and then
 * returns the measured voltage as a float value in volts.
 *
 * @param mux     Input multiplexer setting (0..7).
 *                It is recommended to use the following #define constants for clarity:
 *                  - ADS1115_MUX_AIN0_AIN1_DIFF   (0x00)
 *                  - ADS1115_MUX_AIN0_AIN3_DIFF   (0x01)
 *                  - ADS1115_MUX_AIN1_AIN3_DIFF   (0x02)
 *                  - ADS1115_MUX_AIN2_AIN3_DIFF   (0x03)
 *                  - ADS1115_MUX_AIN0_GND_SINGLE  (0x04)
 *                  - ADS1115_MUX_AIN1_GND_SINGLE  (0x05)
 *                  - ADS1115_MUX_AIN2_GND_SINGLE  (0x06)
 *                  - ADS1115_MUX_AIN3_GND_SINGLE  (0x07)
 *                Using values outside this range will result in failure.
 *
 * @param result  Reference to a float variable where the measured voltage (in volts) will be stored.
 * @return true on success; false on invalid mux value or I2C communication failure.
 */

bool ADS1115::performSingleConversionPollVoltage(uint8_t mux, float &result) {
  if(mux > 7) return false;
  if(!performSingleConversion(mux))   return false;
  if(!waitForConversion())            return false;
  if (!readVoltage(result))           return false;
  return true;
}

/**
 * @brief Initiates a single-shot conversion on the selected input channel.
 *
 * This function configures the ADS1115 to perform a single-shot conversion
 * on the selected input using the multiplexer (MUX) setting. It does not wait
 * for the conversion to complete — it only starts the process. Use
 * waitForConversion() to wait until the conversion finishes.
 *
 * @param mux     Input multiplexer setting (0..7).
 *                Recommended to use predefined constants:
 *                  - ADS1115_MUX_AIN0_AIN1_DIFF   (0x00)
 *                  - ADS1115_MUX_AIN0_AIN3_DIFF   (0x01)
 *                  - ADS1115_MUX_AIN1_AIN3_DIFF   (0x02)
 *                  - ADS1115_MUX_AIN2_AIN3_DIFF   (0x03)
 *                  - ADS1115_MUX_AIN0_GND_SINGLE  (0x04)
 *                  - ADS1115_MUX_AIN1_GND_SINGLE  (0x05)
 *                  - ADS1115_MUX_AIN2_GND_SINGLE  (0x06)
 *                  - ADS1115_MUX_AIN3_GND_SINGLE  (0x07)
 *                Any other value will cause the function to fail.
 *
 * @return true if the conversion was successfully started; false if an error occurred.
 */
bool ADS1115::performSingleConversion(uint8_t mux) {
  if(mux > 7) return false;
  if(muxSetting != mux) {
      if(!setMultiplexer(mux)) return false;
  }
  if(!setMode(ADS1115_SINGLE_CONVERSION)) return false;
  uint16_t cfg;
  if(!readRegister(ADS1115_REG_CONFIG, cfg)) return false;
  cfg |= (1 << 15);
  if(!writeRegister(ADS1115_REG_CONFIG, cfg)) return false;

  readConfig(cfg);
  return true;
}

/**
 * @brief Checks if a conversion is complete by reading OS bit.
 *
 * @param ready  Reference to boolean set true if conversion ready.
 * @return true on I2C success; false on error.
 */
bool ADS1115::isConversionComplete(bool &ready) {
  uint16_t cfg;
  if(!readRegister(ADS1115_REG_CONFIG, cfg)) return false;
  ready = ((cfg >> 15) & 0x01) == ADS1115_OS_READY;
  return true;
}

/**
 * @brief Blocks until conversion completes or timeout expires.
 *
 * @return true if conversion ready; false on timeout or error.
 */
bool ADS1115::waitForConversion() {
  clock_t start = clock();
  bool ready = false;
  while(true) {
    if (!isConversionComplete(ready)) return false;
    if (ready) return true;
    if (timeoutExpired(start, CONVERSION_TIMEOUT_MS)) return false;
    usleep(100);
  }
}

/**
 * @brief Starts continuous conversion mode on the selected input channel.
 *
 * This function configures the ADS1115 to continuously perform conversions
 * on the specified input channel or differential pair using the MUX setting.
 * It does not read or return the results — use readRaw() or readVoltage()
 * in a loop to obtain the samples.
 *
 * @param mux   Input multiplexer setting (0..7).
 *              Recommended to use predefined constants:
 *                - ADS1115_MUX_AIN0_AIN1_DIFF   (0x00)
 *                - ADS1115_MUX_AIN0_AIN3_DIFF   (0x01)
 *                - ADS1115_MUX_AIN1_AIN3_DIFF   (0x02)
 *                - ADS1115_MUX_AIN2_AIN3_DIFF   (0x03)
 *                - ADS1115_MUX_AIN0_GND_SINGLE  (0x04)
 *                - ADS1115_MUX_AIN1_GND_SINGLE  (0x05)
 *                - ADS1115_MUX_AIN2_GND_SINGLE  (0x06)
 *                - ADS1115_MUX_AIN3_GND_SINGLE  (0x07)
 *
 * @return true if continuous mode was successfully started; false if an error occurred.
 */

bool ADS1115::startContinuousConversion(uint8_t mux) {
  if(mux > 7) return false;
  if(muxSetting != mux) {
      if(!setMultiplexer(mux)) return false;
  }
  if(!setMode(ADS1115_CONTINUOUS_CONVERSION)) return false;
  return true;
}

/**
 * @brief Stops continuous conversion and returns to single-shot mode.
 *
 * @return true on success; false on error.
 */
bool ADS1115::stopContinuousConversion() {
  return setMode(ADS1115_SINGLE_CONVERSION);
}

/**
 * @brief Reads raw conversion value (unsigned).
 *
 * @param result  Reference to store raw ADC result.
 * @return true on success; false on error.
 */
bool ADS1115::readRaw(uint16_t &result) {
  return readRegister(ADS1115_REG_CONVERSION, result);
}

/**
 * @brief Reads raw conversion value for differential mode (signed).
 *
 * @param result  Reference to store signed raw ADC result.
 * @return true on success; false on error.
 */
bool ADS1115::readRawDiff(int16_t &result) {
  uint16_t raw_u;
  if (!readRegister(ADS1115_REG_CONVERSION, raw_u)) return false;
  result = static_cast<int16_t>(raw_u);
  return true;
}

/**
 * @brief Reads conversion and converts to voltage, handling signed for differential.
 *
 * @param result  Reference to store voltage in volts.
 * @return true on success; false on error.
 */
bool ADS1115::readVoltage(float &result) {
  uint16_t raw_u;
  if(!readRegister(ADS1115_REG_CONVERSION, raw_u)) return false;
  bool differential = (muxSetting < 4);
  int16_t raw_signed = differential ? static_cast<int16_t>(raw_u) : 0;
  float lsb_mv =
      gainSetting == ADS1115_GAIN_6P144V ? (6.144f  / 32768.0f * 1000.0f) :
      gainSetting == ADS1115_GAIN_4P096V ? (4.096f  / 32768.0f * 1000.0f) :
      gainSetting == ADS1115_GAIN_2P048V ? (2.048f  / 32768.0f * 1000.0f) :
      gainSetting == ADS1115_GAIN_1P024V ? (1.024f  / 32768.0f * 1000.0f) :
      gainSetting == ADS1115_GAIN_0P512V ? (0.512f  / 32768.0f * 1000.0f) :
                                         (0.256f  / 32768.0f * 1000.0f);

  if(differential) result = (raw_signed * lsb_mv) / 1000.0f;
  else result = (raw_u * lsb_mv) / 1000.0f;
  return true;
}

/**
 * @brief Sets comparator alert thresholds (raw values).
 *
 * @param lowThreshold   Lower threshold raw ADC counts.
 * @param highThreshold  Upper threshold raw ADC counts.
 * @return true on success; false on error.
 */
bool ADS1115::enableAlert(uint16_t lowThreshold, uint16_t highThreshold) {
  if(!writeRegister(ADS1115_REG_LO_THRESH, lowThreshold))   return false;
  if(!writeRegister(ADS1115_REG_HI_THRESH, highThreshold))  return false;
  return true;
}

/**
 * @brief Sets comparator alert thresholds (voltage values).
 *
 * @param lowThreshold   Lower threshold in volts.
 * @param highThreshold  Upper threshold in volts.
 * @return true on success; false on error.
 */
bool ADS1115::enableAlertVoltage(float lowThreshold, float highThreshold) {
  uint16_t low  = uint16_t((lowThreshold  * 1000.0f) / (
      gainSetting == ADS1115_GAIN_6P144V ? ADS1115_MV_6P144 :
      gainSetting == ADS1115_GAIN_4P096V ? ADS1115_MV_4P096 :
      gainSetting == ADS1115_GAIN_2P048V ? ADS1115_MV_2P048 :
      gainSetting == ADS1115_GAIN_1P024V ? ADS1115_MV_1P024 :
      gainSetting == ADS1115_GAIN_0P512V ? ADS1115_MV_0P512 :
                                          ADS1115_MV_0P256
  ) );
  uint16_t high = uint16_t((highThreshold * 1000.0f) / (
      gainSetting == ADS1115_GAIN_6P144V ? ADS1115_MV_6P144 :
      gainSetting == ADS1115_GAIN_4P096V ? ADS1115_MV_4P096 :
      gainSetting == ADS1115_GAIN_2P048V ? ADS1115_MV_2P048 :
      gainSetting == ADS1115_GAIN_1P024V ? ADS1115_MV_1P024 :
      gainSetting == ADS1115_GAIN_0P512V ? ADS1115_MV_0P512 :
                                          ADS1115_MV_0P256
  ) );
  return enableAlert(low, high);
}

/**
 * @brief Reads the alert threshold values (low and high) from the ADS1115.
 *
 * @param low    Reference to store the low threshold value.
 * @param high   Reference to store the high threshold value.
 * @return true on success; false on error.
 *
 * This function reads the low and high threshold registers (LO_THRESH and HI_THRESH)
 * from the ADS1115. These thresholds are used for comparison to the current ADC value,
 * typically in the context of alert functionality, where an alert is triggered if the
 * ADC value goes beyond the specified thresholds.
 */
bool ADS1115::getAlertThresholds(uint16_t &low, uint16_t &high) {
  if(!readRegister(ADS1115_REG_LO_THRESH, low)) return false;
  if(!readRegister(ADS1115_REG_HI_THRESH, high)) return false;
  return true;
}

/**
 * @brief Reads the alert threshold values (low and high) in voltage from the ADS1115.
 *
 * @param low    Reference to store the low threshold value in voltage (volts).
 * @param high   Reference to store the high threshold value in voltage (volts).
 * @return true on success; false on error.
 *
 * This function reads the raw low and high threshold values from the ADS1115
 * using the `getAlertThresholds()` function and then converts them into voltage
 * values. The conversion is based on the current gain setting (configured in the
 * `gainSetting` variable). The resulting voltage values are stored in the `low`
 * and `high` parameters passed to the function.
 *
 * The conversion depends on the reference voltage range set by the `gainSetting`:
 * - ADS1115_GAIN_6P144V: 6.144V range
 * - ADS1115_GAIN_4P096V: 4.096V range
 * - ADS1115_GAIN_2P048V: 2.048V range
 * - ADS1115_GAIN_1P024V: 1.024V range
 * - ADS1115_GAIN_0P512V: 0.512V range
 * - ADS1115_GAIN_0P256V: 0.256V range
 */
bool ADS1115::getAlertThresholdsVoltage(float &low, float &high) {
  uint16_t rawLow, rawHigh;
  if(!getAlertThresholds(rawLow, rawHigh)) return false;
  low  = rawLow  * (
      gainSetting == ADS1115_GAIN_6P144V ? ADS1115_MV_6P144 :
      gainSetting == ADS1115_GAIN_4P096V ? ADS1115_MV_4P096 :
      gainSetting == ADS1115_GAIN_2P048V ? ADS1115_MV_2P048 :
      gainSetting == ADS1115_GAIN_1P024V ? ADS1115_MV_1P024 :
      gainSetting == ADS1115_GAIN_0P512V ? ADS1115_MV_0P512 :
                                          ADS1115_MV_0P256
  ) / 1000.0f;
  high = rawHigh * (
      gainSetting == ADS1115_GAIN_6P144V ? ADS1115_MV_6P144 :
      gainSetting == ADS1115_GAIN_4P096V ? ADS1115_MV_4P096 :
      gainSetting == ADS1115_GAIN_2P048V ? ADS1115_MV_2P048 :
      gainSetting == ADS1115_GAIN_1P024V ? ADS1115_MV_1P024 :
      gainSetting == ADS1115_GAIN_0P512V ? ADS1115_MV_0P512 :
                                          ADS1115_MV_0P256
  ) / 1000.0f;
  return true;
}

/**
 * @brief Clears the alert flag by reading the conversion register.
 *
 * @return true on success; false on error.
 *
 * This function clears the alert flag in the ADS1115 by reading the conversion
 * register (`ADS1115_REG_CONVERSION`). A read from this register clears the
 * alert condition, allowing the device to resume normal operation.
 */
bool ADS1115::clearAlertFlag() {
  uint16_t dummy;
  return readRegister(ADS1115_REG_CONVERSION, dummy);
}

/**
 * @brief Sets the alert latch mode for the comparator.
 *
 * @param mode   The latch mode to set:
 *               - ADS1115_COMP_LAT_NON_LATCHING (0) for non-latched mode (default),
 *               - ADS1115_COMP_LAT_LATCHING (1) for latched mode.
 * @return true on success; false on error.
 *
 * This function configures the alert latch mode for the comparator in the
 * ADS1115. If mode is set to ADS1115_COMP_LAT_NON_LATCHING (0), the comparator
 * output is not latched (i.e., it automatically resets when the condition clears).
 * If mode is set to ADS1115_COMP_LAT_LATCHING (1), the comparator output is latched
 * until explicitly cleared.
 */
bool ADS1115::setAlertLatchMode(uint8_t mode) {
  if(mode > 1) return false;
  return setComparatorLatchEnabled(mode);
}

/**
 * @brief Sets the polarity of the alert pin for the comparator.
 *
 * @param polarity  The polarity to set:
 *                  - ADS1115_COMP_POL_ACTIVE_LOW (0) for active low (default),
 *                  - ADS1115_COMP_POL_ACTIVE_HIGH (1) for active high.
 * @return true on success; false on error.
 *
 * This function sets the polarity of the alert pin for the comparator in the
 * ADS1115. If polarity is set to ADS1115_COMP_POL_ACTIVE_LOW (0), the alert pin
 * will be active low (logic low when the comparator condition is met). If polarity
 * is set to ADS1115_COMP_POL_ACTIVE_HIGH (1), the alert pin will be active high.
 */
bool ADS1115::setAlertPinPolarity(uint8_t polarity) {
  if(polarity > 1) return false;
  return setComparatorPolarity(polarity);
}

/**
 * @brief Sets the comparator alert mode for the ADS1115.
 *
 * @param alertMode  The alert mode to set:
 *                   - ADS1115_COMP_TRIGGER_AFTER_1 (0) for triggering after 1 conversion,
 *                   - ADS1115_COMP_TRIGGER_AFTER_2 (1) for triggering after 2 conversions,
 *                   - ADS1115_COMP_TRIGGER_AFTER_4 (2) for triggering after 4 conversions,
 *                   - ADS1115_COMP_DISABLED (3) to disable the comparator alert (default).
 * @return true on success; false on error.
 *
 * This function configures the comparator alert mode. The alert pin will behave according
 * to the mode set, where the alert can trigger after a specified number of conversions
 * or be disabled completely.
 */
bool ADS1115::setAlertMode(bool alertMode) {
  if(alertMode > 3) return false;
  return setComparatorQueueMode(alertMode);
}

/**
 * @brief Disables the comparator alert by setting the queue mode to "disabled".
 *
 * @return true on success; false on error.
 *
 * This function disables the alert functionality on the ADS1115 by setting the comparator
 * queue mode to `ADS1115_COMP_DISABLED`, which stops the comparator from triggering the alert pin.
 */
bool ADS1115::disableAlert() {
  return setComparatorQueueMode(ADS1115_COMP_DISABLED);
}

/**
 * @brief Reads a 16-bit register from the ADS1115 via I2C.
 *
 * ADS1115 returns MSB first, so bytes are swapped before returning.
 *
 * @param reg   Register pointer (ADS1115_REG_CONVERSION, CONFIG, etc.).
 * @param out   Reference to store the read 16-bit value.
 * @return true on success; false on I2C error.
 */
bool ADS1115::readRegister(uint8_t reg, uint16_t& out) {
  int32_t raw = i2cReadWordData(handle, reg);
  if(raw < 0) {
    std::cerr << "Error reading register 0x" << std::hex << int(reg)
              << ": I2C read failed with error code " << raw << std::dec << std::endl;
    return false;
  }
  out = uint16_t((raw & 0xFF) << 8) | uint16_t((raw >> 8) & 0xFF);
  return true;
}

/**
 * @brief Writes a 16-bit value to an ADS1115 register via I2C.
 *
 * @param reg     Register pointer to write (CONFIG, threshold registers, etc.).
 * @param value   16-bit value to write.
 * @return true on success; false on I2C error.
 */
bool ADS1115::writeRegister(uint8_t reg, uint16_t value) {
  int result = i2cWriteWordData(handle, reg, value);
  if(result != 0) {
    std::cerr << "Error writing register 0x" << std::hex << int(reg)
              << ": I2C write failed with error code " << result << std::dec << std::endl;
    return false;
  }
  return true;
}

/**
 * @brief Reads and parses the configuration register of the ADS1115.
 *
 * @param cfg  The 16-bit value read from the configuration register.
 *
 * This function takes the 16-bit configuration value `cfg` and extracts individual settings
 * for the ADS1115's various configuration parameters:
 * - Comparator queue mode (`compQue`)
 * - Comparator latch mode (`compLat`)
 * - Comparator polarity (`compPol`)
 * - Comparator mode (`compMode`)
 * - Operation mode (`modeSetting`)
 * - Data rate (`dataRate`)
 * - Gain setting (`gainSetting`)
 * - Multiplexer input setting (`muxSetting`)
 *
 * The bits are extracted from the provided `cfg` value and assigned to the corresponding class member variables.
 */
void ADS1115::readConfig(uint16_t cfg) {
  compQue         =  cfg        & 0x03;
  compLat         = (cfg >> 2)  & 0x01;
  compPol         = (cfg >> 3)  & 0x01;
  compMode        = (cfg >> 4)  & 0x01;
  modeSetting     = (cfg >> 8)  & 0x01;
  dataRate        = (cfg >> 5)  & 0x07;
  gainSetting     = (cfg >> 9)  & 0x07;
  muxSetting      = (cfg >> 12) & 0x07;
}

/**
 * @brief Sets the ADS1115 data rate.
 *
 * @param rate   Data rate define: ADS1115_8SPS..ADS1115_860SPS.
 *               - ADS1115_8SPS    : 8 Samples Per Second (SPS)
 *               - ADS1115_16SPS   : 16 Samples Per Second (SPS)
 *               - ADS1115_32SPS   : 32 Samples Per Second (SPS)
 *               - ADS1115_64SPS   : 64 Samples Per Second (SPS)
 *               - ADS1115_128SPS  : 128 Samples Per Second (SPS) [default]
 *               - ADS1115_250SPS  : 250 Samples Per Second (SPS)
 *               - ADS1115_475SPS  : 475 Samples Per Second (SPS)
 *               - ADS1115_860SPS  : 860 Samples Per Second (SPS)
 *
 * @return true on success; false on read/write error.
 *
 * This function sets the data rate (sample rate) for the ADS1115. The rate parameter
 * defines the number of samples per second (SPS). It updates the configuration register
 * with the new data rate by masking and shifting the appropriate bits. The configuration
 * is then written back to the ADS1115.
 */
bool ADS1115::setDataRate(uint8_t rate) {
  if(rate > 7) return false;
  uint16_t cfg;
  if (!readRegister(ADS1115_REG_CONFIG, cfg)) return false;
  cfg &= ~(7 << 5);
  cfg |= (uint16_t(rate) << 5);
  if (!writeRegister(ADS1115_REG_CONFIG, cfg)) return false;
  readConfig(cfg);
  return true;
}

/**
 * @brief Sets the PGA gain of the ADS1115.
 *
 * @param gain   Gain define: ADS1115_GAIN_6P144V..ADS1115_GAIN_0P256V.
 *               - ADS1115_GAIN_6P144V  : ±6.144V
 *               - ADS1115_GAIN_4P096V  : ±4.096V
 *               - ADS1115_GAIN_2P048V  : ±2.048V [default]
 *               - ADS1115_GAIN_1P024V  : ±1.024V
 *               - ADS1115_GAIN_0P512V  : ±0.512V
 *               - ADS1115_GAIN_0P256V  : ±0.256V
 *
 * @return true on success; false on read/write error.
 *
 * This function sets the Programmable Gain Amplifier (PGA) gain for the ADS1115.
 * The gain setting adjusts the input voltage range that the ADC can measure.
 * The `gain` parameter should be one of the predefined values that correspond
 * to the available voltage ranges.
 *
 * **Important**: If the ADS1115 is operating in continuous conversion mode,
 * the change in gain will not take effect immediately. In such cases,
 * you need to temporarily switch to single-shot mode and then return to
 * continuous mode for the new gain setting to take effect.
 * This function does not automatically handle that change, as blocking the program
 * might interfere with other operations (e.g., GUI handling), so the user must
 * manually stop and resume continuous conversion.
 */
bool ADS1115::setGain(uint8_t gain) {
  if(gain > 7) return false;
  uint16_t cfg;
  if(!readRegister(ADS1115_REG_CONFIG, cfg)) return false;
  cfg &= ~(7 << 9);
  cfg |= (uint16_t(gain) << 9);
  if(!writeRegister(ADS1115_REG_CONFIG, cfg)) return false;
  readConfig(cfg);
  return true;
}

/**
 * @brief Sets the input multiplexer (MUX) for the ADS1115.
 *
 * @param mux   Multiplexer setting define:
 *              - ADS1115_MUX_AIN0_AIN1_DIFF    : Differential input between AIN0 and AIN1 [default]
 *              - ADS1115_MUX_AIN0_AIN3_DIFF    : Differential input between AIN0 and AIN3
 *              - ADS1115_MUX_AIN1_AIN3_DIFF    : Differential input between AIN1 and AIN3
 *              - ADS1115_MUX_AIN2_AIN3_DIFF    : Differential input between AIN2 and AIN3
 *              - ADS1115_MUX_AIN0_GND_SINGLE   : Single-ended input from AIN0 and GND
 *              - ADS1115_MUX_AIN1_GND_SINGLE   : Single-ended input from AIN1 and GND
 *              - ADS1115_MUX_AIN2_GND_SINGLE   : Single-ended input from AIN2 and GND
 *              - ADS1115_MUX_AIN3_GND_SINGLE   : Single-ended input from AIN3 and GND
 *
 * @return true on success; false on read/write error.
 *
 * This function sets the input multiplexer for the ADS1115, which determines
 * which pair of inputs (differential or single-ended) will be used for
 * analog-to-digital conversion.
 *
 * **Important**: If the ADS1115 is operating in continuous conversion mode,
 * the change in multiplexer setting will not take effect immediately. In such
 * cases, you need to temporarily switch to single-shot mode and then return
 * to continuous mode for the new multiplexer setting to take effect.
 * This function does not automatically handle that change, as blocking the
 * program might interfere with other operations (e.g., GUI handling), so the
 * user must manually stop and resume continuous conversion.
 */
bool ADS1115::setMultiplexer(uint8_t mux) {
  if(mux > 7) return false;
  uint16_t cfg;
  if(!readRegister(ADS1115_REG_CONFIG, cfg)) return false;
  cfg &= ~(7 << 12);
  cfg |= (uint16_t(mux) << 12);
  if(!writeRegister(ADS1115_REG_CONFIG, cfg)) return false;
  readConfig(cfg);
  return true;
}

/**
 * @brief Sets the operation mode of the ADS1115.
 *
 * @param mode   Operation mode define:
 *               - ADS1115_CONTINUOUS_CONVERSION : Continuous conversion mode
 *               - ADS1115_SINGLE_CONVERSION    : Single-shot conversion mode [default]
 *
 * @return true on success; false on read/write error.
 *
 * This function sets the operation mode of the ADS1115. In continuous conversion
 * mode, the device continuously performs analog-to-digital conversions. In
 * single-shot mode, the conversion is triggered once and then the device stops
 * converting until the next trigger.
 */
bool ADS1115::setMode(uint8_t mode) {
  if(mode > 1) return false;
  uint16_t cfg;
  if (!readRegister(ADS1115_REG_CONFIG, cfg)) return false;
  cfg &= ~(1 << 8);
  cfg |= (uint16_t(mode) << 8);
  if(!writeRegister(ADS1115_REG_CONFIG, cfg)) return false;
  readConfig(cfg);
  return true;
}

/**
 * @brief Sets the comparator mode of the ADS1115.
 *
 * @param mode   Comparator mode define:
 *               - ADS1115_COMP_MODE_HYSTERESIS : Hysteresis mode [default]
 *               - ADS1115_COMP_MODE_WINDOW     : Window mode
 *
 * @return true on success; false on read/write error.
 *
 * This function sets the comparator mode for the ADS1115. In hysteresis mode, the
 * comparator output will remain in the same state until the input crosses a certain
 * threshold with the specified hysteresis. In window mode, the comparator output is
 * active when the input is within a defined window, and inactive when the input falls
 * outside of this range.
 */
bool ADS1115::setComparatorMode(uint8_t mode) {
  if(mode > 1) return false;
  uint16_t cfg;
  if (!readRegister(ADS1115_REG_CONFIG, cfg)) return false;
  cfg &= ~(1 << 4);
  cfg |= (uint16_t(mode) << 4);
  if (!writeRegister(ADS1115_REG_CONFIG, cfg)) return false;
  readConfig(cfg);
  return true;
}

/**
 * @brief Sets the comparator polarity of the ADS1115.
 *
 * @param polarity   Comparator polarity define:
 *                   - ADS1115_COMP_POL_ACTIVE_LOW   : Active low [default]
 *                   - ADS1115_COMP_POL_ACTIVE_HIGH  : Active high
 *
 * @return true on success; false on read/write error.
 *
 * This function sets the polarity of the comparator output. When the polarity is
 * set to active low, the comparator output will be low when the condition is met.
 * When set to active high, the comparator output will be high when the condition is met.
 */
bool ADS1115::setComparatorPolarity(uint8_t polarity) {
  if(polarity > 1) return false;
  uint16_t cfg;
  if (!readRegister(ADS1115_REG_CONFIG, cfg)) return false;
  cfg &= ~(1 << 3);
  cfg |= (uint16_t(polarity) << 3);
  if (!writeRegister(ADS1115_REG_CONFIG, cfg)) return false;
  readConfig(cfg);
  return true;
}

/**
 * @brief Sets the comparator latch mode of the ADS1115.
 *
 * @param enabled   Comparator latch mode:
 *                  - ADS1115_COMP_LAT_NON_LATCHING  : Non-latching mode [default]
 *                  - ADS1115_COMP_LAT_LATCHING     : Latching mode
 *
 * @return true on success; false on read/write error.
 *
 * In non-latching mode, the comparator output changes immediately when the condition is met.
 * In latching mode, the comparator output will remain in its current state (either high or low)
 * until the latch is cleared. Latching is useful when you want to hold the comparator output
 * for some period, even if the condition is no longer true.
 */
bool ADS1115::setComparatorLatchEnabled(uint8_t enabled) {
  if(enabled > 1) return false;
  uint16_t cfg;
  if (!readRegister(ADS1115_REG_CONFIG, cfg)) return false;
  cfg &= ~(1 << 2);
  cfg |= (uint16_t(enabled) << 2);
  if (!writeRegister(ADS1115_REG_CONFIG, cfg)) return false;
  readConfig(cfg);
  return true;
}

/**
 * @brief Sets the comparator queue mode of the ADS1115.
 *
 * @param mode   Comparator queue mode:
 *               - ADS1115_COMP_TRIGGER_AFTER_1  : Trigger comparator after 1 conversion.
 *               - ADS1115_COMP_TRIGGER_AFTER_2  : Trigger comparator after 2 conversions.
 *               - ADS1115_COMP_TRIGGER_AFTER_4  : Trigger comparator after 4 conversions.
 *               - ADS1115_COMP_DISABLED         : Disable comparator (default).
 *
 * @return true on success; false on read/write error.
 *
 * The comparator queue mode defines how many conversions must be completed before the comparator
 * output is triggered.
 * - If the mode is set to trigger after 1, the comparator will trigger after each conversion.
 * - If set to trigger after 2 or 4, the comparator will only trigger after 2 or 4 consecutive conversions.
 * - If set to "Disabled", the comparator will not trigger at all.
 */
bool ADS1115::setComparatorQueueMode(uint8_t mode) {
  if(mode > 3) return false;
  uint16_t cfg;
  if (!readRegister(ADS1115_REG_CONFIG, cfg)) return false;
  cfg &= ~0x03;
  cfg |= (uint16_t(mode) & 0x03);
  if (!writeRegister(ADS1115_REG_CONFIG, cfg)) return false;
  readConfig(cfg);
  return true;
}

/**
 * @brief Checks if a timeout period has elapsed since the given clock start.
 *
 * @param start  clock() value at operation start.
 * @param ms     Timeout threshold in milliseconds.
 * @return true if elapsed >= ms; false otherwise.
 */
bool ADS1115::timeoutExpired(clock_t start, unsigned int ms) {
    return ((clock() - start) * 1000 / CLOCKS_PER_SEC) >= ms;
}
