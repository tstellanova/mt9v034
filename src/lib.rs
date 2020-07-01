
/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/
#![no_std]

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

//! Driver for MT9V034 image sensor

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE, PinE> {
    /// Sensor communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),

    /// The sensor is not responding
    SensorUnresponsive,
}


pub struct ParallelPixelPins<DIN, COUT> {

}

const NUM_DOUT_PINS: usize = 10;
/// This imaging sensor has multiple interfaces:
/// - i2c for configuration registers (i2c)
/// - parallel pixel data out (dout)
/// - pixel out sync (vsync, hsync, pix clock)
pub struct Mt9v034<DOUT, I2C> {
    dout: [DOUT; NUM_DOUT_PINS ],
    i2c: I2C,
}

impl<DOUT, I2C> Mt9v034<DOUT, I2C>
where
    I2C: embedded_hal::blocking::i2c::Write<Error = CommE>
    + embedded_hal::blocking::i2c::Read<Error = CommE>
    + embedded_hal::blocking::i2c::WriteRead<Error = CommE>,
{
    fn new(dout: [DOUT; NUM_DOUT_PINS], i2c: I2C) -> Self {
        Self {
            dout,
            i2c
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
