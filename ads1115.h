/**
 * @file ads1115.h
 * @brief C++ library for ADS1115 Analog-to-Digital Converter
 *
 * This library provides an interface for the ADS1115 16-bit ADC using the I2C bus.
 * It is designed to run on Raspberry Pi and is written in C++.
 * The library supports single-ended and differential input modes, configurable gain,
 * data rate, alert functionality, and both single-shot and continuous conversion modes.
 *
 * Supported features:
 * - All input multiplexer configurations
 * - Adjustable gain (FSR)
 * - Adjustable data rate (SPS)
 * - Polling for conversion complete
 * - Alert/ready pin configuration
 * - Conversion result in raw or millivolts
 *
 * Target Platform: Raspberry Pi
 * Language: C++
 *
 * Author: Mateusz Synak
 * Created: 04.05.2025
 * License: MIT
 */


#ifndef ADS1115_H
#define ADS1115_H

#include <cstdint>
#include <ctime>

// I2C address options based on ADDR pin connection
#define ADS1115_ADDRESS_GND             0x48
#define ADS1115_ADDRESS_VDD             0x49
#define ADS1115_ADDRESS_SDA             0x4A
#define ADS1115_ADDRESS_SCL             0x4B

// Programmable Gain Amplifier (PGA) settings
#define ADS1115_GAIN_6P144V             0x00
#define ADS1115_GAIN_4P096V             0x01
#define ADS1115_GAIN_2P048V             0x02  //default
#define ADS1115_GAIN_1P024V             0x03
#define ADS1115_GAIN_0P512V             0x04
#define ADS1115_GAIN_0P256V             0x05

// Data rate
#define ADS1115_8SPS                    0x00
#define ADS1115_16SPS                   0x01
#define ADS1115_32SPS                   0x02
#define ADS1115_64SPS                   0x03
#define ADS1115_128SPS                  0x04  //default
#define ADS1115_250SPS                  0x05
#define ADS1115_475SPS                  0x06
#define ADS1115_860SPS                  0x07

// Multiplexer (MUX) configuration for differential and single-ended inputs
#define ADS1115_MUX_AIN0_AIN1_DIFF      0x00 //default
#define ADS1115_MUX_AIN0_AIN3_DIFF      0x01
#define ADS1115_MUX_AIN1_AIN3_DIFF      0x02
#define ADS1115_MUX_AIN2_AIN3_DIFF      0x03
#define ADS1115_MUX_AIN0_GND_SINGLE     0x04
#define ADS1115_MUX_AIN1_GND_SINGLE     0x05
#define ADS1115_MUX_AIN2_GND_SINGLE     0x06
#define ADS1115_MUX_AIN3_GND_SINGLE     0x07

// Operational modes
#define ADS1115_CONTINUOUS_CONVERSION   0x00
#define ADS1115_SINGLE_CONVERSION       0x01 //default

// Comparator modes and polarity
#define ADS1115_COMP_MODE_HYSTERESIS    0x00 //default
#define ADS1115_COMP_MODE_WINDOW        0x01
#define ADS1115_COMP_POL_ACTIVE_LOW     0x00 //default
#define ADS1115_COMP_POL_ACTIVE_HIGH    0x01
#define ADS1115_COMP_LAT_NON_LATCHING   0x00 //default
#define ADS1115_COMP_LAT_LATCHING       0x01

// Comparator queue and disable bits
#define ADS1115_COMP_TRIGGER_AFTER_1    0x00
#define ADS1115_COMP_TRIGGER_AFTER_2    0x01
#define ADS1115_COMP_TRIGGER_AFTER_4    0x02
#define ADS1115_COMP_DISABLED           0x03 //default

// Operational status bits
#define ADS1115_OS_BUSY                 0x00
#define ADS1115_OS_READY                0x01
#define ADS1115_START_SINGLE_CONV       0x01

// Conversion factors: mV per LSB (depends on PGA setting)
#define ADS1115_MV_6P144                0.187500  // 6.144V / 32768 * 1000
#define ADS1115_MV_4P096                0.125000  // 4.096V / 32768 * 1000
#define ADS1115_MV_2P048                0.062500  // 2.048V / 32768 * 1000 (default)
#define ADS1115_MV_1P024                0.031250  // 1.024V / 32768 * 1000
#define ADS1115_MV_0P512                0.015625  // 0.512V / 32768 * 1000
#define ADS1115_MV_0P256                0.007813  // 0.256V / 32768 * 1000

// Register pointers
#define ADS1115_REG_CONVERSION          0x00
#define ADS1115_REG_CONFIG              0x01
#define ADS1115_REG_LO_THRESH           0x02
#define ADS1115_REG_HI_THRESH           0x03

// ADS1115 class for Raspberry Pi I2C interface
class ADS1115 {
public:
  // Constructor: specify I2C bus, device address, data rate, and gain
  ADS1115(int i2cBus, uint8_t i2cAddress, uint8_t dr, uint8_t gain);
  ~ADS1115();

  bool isClassInitialized() const;                                              // Returns true if initialization completed successfully
  void init();                                                                  // Initialize the ADS1115 with the provided settings
  bool selfTest();                                                              // Perform a self-test by reading the config register and verifying defaults
  bool performSingleConversionPoll(uint8_t mux, uint16_t &result);              // Perform a single conversion in polling mode and return raw ADC value
  bool performSingleConversionDiffPoll(uint8_t mux, int16_t &result);           // Perform a single differential conversion in polling mode and return signed raw value
  bool performSingleConversionPollVoltage(uint8_t mux, float &result);          // Perform a single conversion and return the voltage in volts
  bool performSingleConversion(uint8_t mux);                                    // Start a conversion without reading the result (for non-blocking use)
  bool isConversionComplete(bool &ready);                                       // Check if conversion is complete; sets 'ready' to true if done
  bool waitForConversion();                                                     // Block until conversion completes or times out
  bool startContinuousConversion(uint8_t mux);                                  // Enter continuous conversion mode on the specified MUX setting
  bool stopContinuousConversion();                                              // Stop continuous conversion and power down
  bool readRaw(uint16_t &result);                                               // Read raw ADC result (unsigned)
  bool readRawDiff(int16_t &result);                                            // Read raw ADC result for differential mode (signed)
  bool readVoltage(float &result);                                              // Read converted voltage (handles both single-ended and differential)
  bool enableAlert(uint16_t lowThreshold, uint16_t highThreshold);              // Configure comparator alert thresholds (raw values)
  bool enableAlertVoltage(float lowThreshold, float highThreshold);             // Configure comparator alert thresholds (voltages in V)
  bool getAlertThresholds(uint16_t &low, uint16_t &high);                       // Read comparator threshold registers (raw values)
  bool getAlertThresholdsVoltage(float &low, float &high);                      // Read comparator threshold registers (voltages in V)
  bool clearAlertFlag();                                                        // Clear the comparator alert flag
  bool setAlertLatchMode(uint8_t mode);                                         // Set comparator latch mode (latching vs non-latching)
  bool setAlertPinPolarity(uint8_t polarity);                                   // Set ALERT/RDY pin polarity (active high vs active low)
  bool setAlertMode(bool alertMode);                                            // Enable or disable comparator mode
  bool disableAlert();                                                          // Disable comparator completely

private:
  static constexpr unsigned int CONVERSION_TIMEOUT_MS = 100;                    // Timeout for conversion in ms
  int handle;             // I2C bus handle
  uint8_t address;        // I2C device address
  uint8_t gainSetting;    // PGA setting
  uint8_t dataRate;       // Data rate
  uint8_t muxSetting;     // Input multiplexer setting
  uint8_t modeSetting;    // Conversion mode
  uint8_t compMode;       // Comparator mode
  uint8_t compPol;        // Comparator polarity
  uint8_t compLat;        // Comparator latching
  uint8_t compQue;        // Comparator queue or disable

  bool readRegister(uint8_t reg, uint16_t& out);          // Read a 16-bit register from the ADS1115
  bool writeRegister(uint8_t reg, uint16_t value);        // Write a 16-bit value to an ADS1115 register
  void readConfig(uint16_t cfg);                          // Parse configuration bits (optional helper)
  bool setDataRate(uint8_t rate);                         // Setters for individual configuration fields
  bool setGain(uint8_t gain);
  bool setMultiplexer(uint8_t mux);
  bool setMode(uint8_t mode);
  bool setComparatorMode(uint8_t mode);
  bool setComparatorPolarity(uint8_t polarity);
  bool setComparatorLatchEnabled(uint8_t enabled);
  bool setComparatorQueueMode(uint8_t mode);
  bool timeoutExpired(clock_t start, unsigned int ms);    // Check if the timeout has expired since 'start'
};

#endif // ADS1115_H
