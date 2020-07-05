
# mt9v034-i2c

Rust no_std driver for the 
ON Semiconductor MT9V034 image sensor

This is a low-cost global shutter CMOS image sensor useful for robotics
and machine vision applications.

This driver specifically interfaces with the sensor's
two-wire (i2c) programming interface, which allows you to set
various camera configuration parameters.  This driver does not obtain
pixel data from the camera.  For that you will need a driver that 
pulls data from the camera's parallel data lines.  (Example forthcoming in another crate.)

## License

BSD-3-Clause: see LICENSE file

## Status

- [ ] Access to basic configuration registers
- [ ] Support for register locking (to prevent accidental changes due to i2c noise)
- [ ] Example for cortex-m

