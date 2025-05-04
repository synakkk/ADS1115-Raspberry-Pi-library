# ADS1115 - RaspberryPi Library
A library for the Raspberry Pi enabling easy integration with the ADS1115 Analog-to-Digital Converter (ADC). This library allows users to easily connect and configure the ADS1115 for reading analog voltages via the I2C interface.

## 🌟Features:
- **Single-shot & continuous conversion**  
- **Configurable PGA gain** (±6.144V … ±0.256V)  
- **Adjustable data rate** (8 SPS … 860 SPS)  
- **Differential or single-ended inputs**  
- **Comparator & alert thresholds**  
- **Read results as raw 16-bit or float (volts)**  

## ⚙️ Requirements
- Raspberry Pi with I²C enabled  
- Linux (e.g. Raspberry Pi OS / Raspberry Pi OS Lite)  
- `pigpio` library for I²C (install with `sudo apt install pigpio`)  
- ADS1115 wired to I²C pins (SDA, SCL, VCC, GND) 



**Ensure I2C is enabled on your Raspberry Pi. You can do this via the raspi-config tool:**
```bash
sudo raspi-config
# → Interface Options → I2C → Enable
```

Connect the ADS1115 via I2C and configure parameters using the methods provided by the library.

## 📘 Usage Example
```cpp
#include <iostream>
#include <unistd.h>
#include <pigpio.h>
#include <csignal>
#include <atomic>
#include "ads1115.h"

#define I2C_BUS 1
#define ADS_ADDR ADS1115_ADDRESS_GND

static std::atomic<bool> keepRunning{true};

void handleSigint(int) {
  keepRunning = false;
}

int main() {
  // Catch Ctrl+C
  std::signal(SIGINT, handleSigint);

  if(gpioInitialise() < 0) {
    std::cerr << "[ERROR] pigpio initialization failed" << std::endl;
    return 1;
  }

  // Create ADS1115 object: bus, address, data rate, gain
  ADS1115 adc(I2C_BUS, ADS_ADDR, ADS1115_32SPS, ADS1115_GAIN_1P024V);
  if(!adc.isClassInitialized()) {
      std::cerr << "[ERROR] Failed to initialize ADS1115" << std::endl;
      return 1;
  }
  // Write initial configuration
  adc.init();
  usleep(2000);

  float voltage = 0;

  while(keepRunning) {
    // Single-shot voltage read on AIN0 vs GND
    if (!adc.performSingleConversionPollVoltage(ADS1115_MUX_AIN0_GND_SINGLE, voltage)) {
        std::cerr << "[ERROR] Voltage read failed" << std::endl;
        keepRunning = false;
    }
    else std::cout << "Voltage: " << voltage << " V" << std::endl;
    sleep(1);
  }
  gpioTerminate();
  return 0;
}
```

🛠 Compilation & Startup

```bash
g++ -v main.cpp ads1115.cpp -o ads1115_example -lpigpio -lpthread
sudo ./ads1115_example
```
