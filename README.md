
# mt9v034-i2c

Rust no_std driver for the 
ON Semiconductor MT9V034 image sensor

This is a low-cost global shutter CMOS image sensor useful for robotics
and machine vision applications.

This driver specifically interfaces with the sensor's
two-wire (i2c) programming interface, which allows you to set
various camera configuration parameters.  This driver does not obtain
pixel data from the camera.  For that you will need a driver that 
pulls data from the camera's parallel data lines.  
(Example in the [px4flow_bsp crate](https://crates.io/crates/px4flow_bsp).)

## License

BSD-3-Clause: see LICENSE file

## Status

- [x] Access to basic configuration registers
- [x] Support for register locking (to prevent accidental changes due to i2c noise)
- [x] [Example for cortex-m](https://github.com/tstellanova/px4flow_bsp)

