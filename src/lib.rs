/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/
#![no_std]

//! Driver for MT9V034 image sensor

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

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
    din: [DIN; NUM_DIN_PINS],
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
    fn new(ppix: ParallelPixelPins<DIN, COUT>, i2c: I2C) -> Self {
        Self { ppix, i2c }
    }

    pub fn init(&mut self) -> &mut Self {
        //TODO configure reserved registers per Rev G data sheet table 8
        self
    }

    pub fn read_reg_u8(&mut self, reg: u8) -> Result<u8, crate::Error<(), ()>> {
        let cmd_buf = [reg];
        let mut recv_buf = [0u8];
        self.i2c
            .write_read(I2C_READ_ADDRESS, &cmd_buf, &mut recv_buf)
            .map_err(Error::Comm)?;

        Ok(recv_buf[0])
    }

    pub fn read_reg_u16(
        &mut self,
        reg: u8,
    ) -> Result<u16, crate::Error<(), ()>> {
        // read upper u8
        let mut val: u16 = (self.read_reg_u8(reg) << 8) as u16;
        // read lower u8
        val = val | self.read_reg_u8(FOLLOW_UP_ADDRESS)? as u16;
        Ok(val)
    }

    pub fn write_reg_u8(
        &mut self,
        reg: u8,
        val: u8,
    ) -> Result<(), crate::Error<(), ()>> {
        let write_buf = [reg, val];
        self.i2c
            .write(I2C_WRITE_ADDRESS, &write_buf)
            .map_err(Error::Comm)?;
        Ok(())
    }

    pub fn write_reg_u16(
        &mut self,
        reg: u8,
        data: u16,
    ) -> Result<(), crate::Error<(), ()>> {
        // write upper u8
        self.write_reg_u8(reg, (data >> 8) as u8)?;
        // write lower u8
        self.write_reg_u8(FOLLOW_UP_ADDRESS, (data & 0xFF) as u8)?;
        Ok(())
    }
}

const I2C_WRITE_ADDRESS: u8 = 0xB8;
const I2C_READ_ADDRESS: u8 = 0xB9;
/// Used for reading and writing a second byte on registers
const FOLLOW_UP_ADDRESS: u8 = 0xF0;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
