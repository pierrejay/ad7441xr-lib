# AD7441XR Arduino Library

This Arduino library provides an interface for the AD74412R & AD74413R chips from Analog Devices (does not support HART functionality on AD74413R). It is based on the original C library provided by Analog Devices but has been adapted and enhanced for the Arduino environment.

## Features

This version is easier to use compared to the original AD library:
- User-friendly methods for accessing chip parameters and converting real ADC/DAC values & units
- Automatic ADC polling & status update is done in the background, by calling the poll() method in the user program's main loop for non-blocking operation (especially effective when combined with continuous ADC reading mode)
- ADC function & conversion mode changes are fully implemented as recommended in the chip's datasheet
- MUX register settings have been modified in output modes (voltage or current) to provide real output voltage/current feedback
- The 200K pull-down resistor is automatically enabled depending on the ADC function, as recommended in the chip's datasheet

## Usage

Include the library in your sketch, make sure you're using SPI as well:

```cpp
#include <ad7441xr.h>
#include <SPI.h>
```

Initialize the AD7441XR object:

```cpp
AD7441XR ad7441xr(CS_PIN, SPI, AD74412R);
```

In your `setup()` function, initialize SPI and the chip:

```cpp
SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
digitalWrite(AD7441XR_CS_PIN, HIGH);
ad7441xr.begin();
```

In your `loop()` function, call the `poll()` method to automatically update ADC values:

```cpp
ad7441xr.poll();
```

You can call several methods to enable & set channels, either in `loop()` or in `setup()`. 
```cpp
// Enable channels A & B
swio.enableChannel(AD7441XR_CH_A, true);
swio.enableChannel(AD7441XR_CH_B, true);

// Set channel functions to AI and AO
swio.setChannelFunc(AD7441XR_CH_A, AD7441XR_VOLTAGE_IN);
swio.setChannelFunc(AD7441XR_CH_B, AD7441XR_VOLTAGE_OUT);

// Start ADC continuous reading (put in setup())
swio.setAdcMode(AD7441XR_START_CONT);

// Set DAC to 3.14V on channel B = index 1
swio.setDac(1, 3.14);
```


If your `poll()` function is executed frequently, and the chip is set in continuous ADC reading mode, ADC values can be accessed at any point of the code with get methods :

```cpp
float adcValue = swio.getAdc(i);    // ADC value
int adcUnit = swio.getAdcUnit(i);   // ADC unit code (1=V, 2=mA...)
```

Refer to the `ad7441xr.h` file to get the complete list of public methods included in the library.

## License

This library is released under the [MIT License](LICENSE).

## Contributing

Contributions to this library are welcome. Please submit pull requests or open issues on the GitHub repository.

## Credits

This library is based on the original C library provided by Analog Devices.