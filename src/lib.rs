/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/
#![no_std]

//! Configuration driver for the ON Semiconductor MT9V034 image sensor
//! This imaging sensor has multiple interfaces:
//! - Two-wire i2c for configuration registers (i2c)
//! - parallel pixel data out (dout)
//! - pixel out sync (vsync, hsync, pix clock)
//! This driver is concerned only with the i2c interface

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

pub const DEFAULT_I2C_ADDRESS: u8 = I2C_WRITE_ADDRESS;

/// Main driver struct
pub struct Mt9v034<I2C> {
    address: u8,
    i2c: I2C,
}

impl<I2C, CommE> Mt9v034<I2C>
where
    I2C: embedded_hal::blocking::i2c::Write<Error = CommE>
        + embedded_hal::blocking::i2c::Read<Error = CommE>
        + embedded_hal::blocking::i2c::WriteRead<Error = CommE>,
{
    /// Create a new instance with an i2c address:
    /// May use DEFAULT_I2C_ADDRESS if in doubt.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { address, i2c }
    }

    /// Second-stage configuration
    pub fn init(&mut self) -> &mut Self {
        //TODO configure reserved registers per Rev G data sheet table 8
        self
    }

    /// Read a u8 from an 8-bit address
    pub fn read_reg_u8(&mut self, reg: u8) -> Result<u8, crate::Error<(), ()>> {
        let cmd_buf = [reg];
        let mut recv_buf = [0u8];
        self.i2c
            .write_read(I2C_READ_ADDRESS, &cmd_buf, &mut recv_buf)
            .map_err(Error::Comm)?;

        Ok(recv_buf[0])
    }

    /// Read a u16 from an 8-bit address
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

    /// Write a u8 to an 8-bit address
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

    /// Write a u16 to an 8-bit address
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

// TODO addd support for alternate addresses
const I2C_WRITE_ADDRESS: u8 = 0xB8;
const I2C_READ_ADDRESS: u8 = 0xB9;
/// Used for reading and writing a second byte on registers
const FOLLOW_UP_ADDRESS: u8 = 0xF0;

// Array format: Wide-VGA, Active 752 H x 480 V
pub const MAX_FRAME_HEIGHT: u16 = 480;
pub const MAX_FRAME_WIDTH: u16 = 752;

#[repr(u8)]
pub enum GeneralRegisters {
    Version = 0x00,
    Control = 0x07,
    SoftReset = 0x0c,
    HdrEnable = 0x0f,
    AdcResCtrl = 0x1c,
    RowNoiseCorrCtrl = 0x70,
    DigitalTest = 0x7f,
    TiledDigitalGain = 0x80,

    AgcAecDesiredBin = 0xa5,
    AecUpdate = 0xa6,
    AecLowpass = 0xa8,
    AgcUpdate = 0xa9,
    AgcLowpass = 0xaa,
    MaxGain = 0xab,
    /// Minimum coarse shutter width
    MinExposure = 0xac,
    /// Maximum coarse shutter width
    MaxExposure = 0xad,
    AecAgcEnable = 0xaf,
    AgcAecPixelCount = 0xb0,
}

#[repr(u8)]
pub enum ContextARegisters {
    ColumnStart = 0x01,
    //TODO fill in the rest
}

#[repr(u8)]
pub enum ContextBRegisters {
    ColumnStart = 0xc9,
    //TODO fill in the rest
}
