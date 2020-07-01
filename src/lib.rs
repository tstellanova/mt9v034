
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

    /// The sensor did not respond in a timely manner
    Timeout,

}

/// The number of parallel pixel data lines
const NUM_DIN_PINS: usize = 10;


/// Contains the main parallel data interface configuration
pub struct ParallelPixelPins<DIN, COUT> {
    /// The pixel data pins
    din: [DIN; NUM_DIN_PINS ],
    /// pixel data clock
    pix_clk: COUT,
    /// horizontal sync / LINE_VALID
    hsync: COUT,
    /// vertical sync / FRAME_VALID
    vsync: COUT,
}


/// This imaging sensor has multiple interfaces:
/// - i2c for configuration registers (i2c)
/// - parallel pixel data out (dout)
/// - pixel out sync (vsync, hsync, pix clock)
pub struct Mt9v034<DIN, COUT, I2C> {
    ppix: ParallelPixelPins<DIN, COUT>,
    i2c: I2C,
}

impl<DIN, COUT, I2C, CommE> Mt9v034<DIN, COUT, I2C>
where
    I2C: embedded_hal::blocking::i2c::Write<Error = CommE>
    + embedded_hal::blocking::i2c::Read<Error = CommE>
    + embedded_hal::blocking::i2c::WriteRead<Error = CommE>,
    COUT: embedded_hal::digital::OutputPin,

{
    fn new(ppx: ParallelPixelPins<DIN, COUT>, i2c: I2C) -> Self {
        Self {
            ppix,
            i2c
        }
    }

    pub fn init(&mut self) -> &mut Self {
        //TODO configure reserved registers per Rev G data sheet table 8
        self
    }

    pub fn read_reg_u8(&mut self, reg: u16) -> Result<u8, crate::Error<(),()>>   {
        unimplemented!()
    }

    pub fn read_reg_u16(&mut self, reg: u16) -> Result<u16, crate::Error<(),()>> {
        // read upper u8
        let mut val: u16 = (self.read_reg_u8(reg) << 8) as u16;
        // read lower u8
        val = val | self.read_reg_u8(FOLLOW_UP_ADDRESS)? as u16;
        Ok(result)
    }

    pub fn write_reg_u8(&mut self, reg: u16, data: u8) -> Result<(), crate::Error<(),()>>   {
        unimplemented!()
    }

    pub fn write_reg_u16(&mut self, reg: u16, data: u16) -> Result<(), crate::Error<(),()>> {
        // write upper u8
        self.write_reg_u8(reg, (data >> 8) as u8)?;
        // write lower u8
        self.write_reg_u8(FOLLOW_UP_ADDRESS, (data & 0xFF) as u8)?;
        Ok(())
    }
}

const FOLLOW_UP_ADDRESS: u16 = 0xF0;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
