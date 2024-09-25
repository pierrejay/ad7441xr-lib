# AD7441XR Arduino Library

This Arduino library provides an interface for the AD74412R & AD74413R chips from Analog Devices (does not support HART functionality on AD74413R). It is based on the original C library provided by Analog Devices but has been adapted and enhanced for the Arduino environment.

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

## License

This library is released under the [MIT License](LICENSE).

## Contributing

Contributions to this library are welcome. Please submit pull requests or open issues on the GitHub repository.

## Credits

This library is based on the original C library provided by Analog Devices.