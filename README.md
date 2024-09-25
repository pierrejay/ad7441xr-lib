# AD7441XR Arduino Library

This Arduino library provides an interface for the AD7441XR chip from Analog Devices. It is based on the original C library provided by Analog Devices but has been adapted and enhanced for the Arduino environment.

## Features

This version is easier to use compared to the original AD library:
- User-friendly methods for accessing chip parameters and converting real ADC/DAC values & units
- Automatic ADC polling & status update is done in the background, by calling the loop() method in the user program's main loop for non-blocking operation (especially effective when combined with continuous ADC reading mode)
- ADC function & conversion mode changes are fully implemented as recommended in the chip's datasheet
- MUX register settings have been modified in output modes (voltage or current) to provide real output voltage/current feedback
- The 200K pull-down resistor is automatically enabled depending on the ADC function, as recommended in the chip's datasheet

## Usage

Include the library in your sketch:

```cpp
#include <ad7441xr.h>
```

Initialize the AD7441XR object:

```cpp
AD7441XR ad7441xr(CS_PIN, SPI, AD74413R);
```

In your `setup()` function, initialize the chip:

```cpp
ad7441xr.begin();
```

In your `loop()` function, call the `poll()` method to automatically read ADC values:

```cpp
ad7441xr.poll();
```

## Public Methods

- `AD7441XR(int cs, SPIClass &spi, enum ad7441xr_chip_id id, int rstPin = -1, int alertPin = -1)`: Constructor
- `int begin()`: Initialize the AD7441XR chip
- `int poll()`: Fetch ADC value and status bits
- `int enableChannel(int ch, bool enable)`: Enable ADC channel
- `int isEnabled(int ch)`: Get enabled channel state
- `int setChannelFunc(int ch, enum ad7441xr_op_mode func)`: Set channel function
- `int getChannelFunc(int ch)`: Get channel function
- `int setAdcMode(enum ad7441xr_conv_seq mode)`: Set ADC conversion mode (idle, single, continuous, off)
- `int requestAdc()`: Request a new ADC conversion (only works in single mode)
- `long getAlerts()`: Get alerts bits
- `ad7441xr_alert_info* getAlertList()`: Get an array of alert info objects
- `int isAdcBusy()`: Check if ADC is busy
- `float getAdc(int ch)`: Get latest real ADC value
- `long getAdcRaw(int ch)`: Get latest raw ADC value
- `float getSingleAdc(int ch)`: Get one-shot ADC value
- `int getAdcUnit(int ch)`: Get ADC unit (0, V, mA, ohm)
- `int setDac(int ch, float val)`: Set DAC value
- `float getDac(int ch)`: Get real DAC value
- `long getDacRaw(int ch)`: Get raw DAC code
- `int getDi(int ch)`: Get DI status
- `int setDiThreshold(int ch, int mv)`: Set DI threshold
- `int getTemp(int ch)`: Get chip temperature
- `ad7441xr_cfg getCfg()`: Get configuration structure
- `int loop()`: Monitor ALERT pin
- `int softReset()`: Perform a soft reset

## License

This library is released under the [MIT License](LICENSE).

## Contributing

Contributions to this library are welcome. Please submit pull requests or open issues on the GitHub repository.

## Credits

This library is based on the original C library provided by Analog Devices.