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
pub enum Error<CommE> {
    /// Sensor communication error
    Comm(CommE),

    /// The sensor did not respond in a timely manner
    Timeout,
}

pub const DEFAULT_I2C_ADDRESS: u8 = I2C_WRITE_ADDRESS;

/// Main driver struct
pub struct Mt9v034<I2C> {
    base_address: u8,
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
        Self {
            base_address: address,
            i2c,
        }
    }

    pub fn default(i2c: I2C) -> Self {
        Self::new(i2c, DEFAULT_I2C_ADDRESS)
    }

    /// With this sensor, the user may switch between two full register sets (listed in Table 7)
    /// by writing to a context switch change bit in register 0x07.
    /// This context switch will change all registers (no shadowing)
    /// at the frame start time and have the new values apply
    /// to the immediate next exposure and readout time (frame n+1),
    /// except for shutter width and V1-V4 control,
    /// which will take effect for next exposure but will show up in the n+2 image.
    pub fn set_context(
        &mut self,
        context: ParamContext,
    ) -> Result<(), crate::Error<CommE>> {
        self.write_reg_u16(GeneralRegisters::Control as u8, context as u16)
    }

    /// Second-stage configuration
    pub fn setup(&mut self) -> Result<(), crate::Error<CommE>> {
        #[cfg(feature = "rttdebug")]
        rprintln!("mt9v034-i2c setup start");

        //self.simple_probe()?;
        let _version = self.read_reg_u8(GeneralRegisters::ChipVersion as u8)?;
        self.write_reg_u8(GeneralRegisters::SoftReset as u8, 0b11)?;

        //TODO configure reserved registers per Rev G data sheet table 8

        #[cfg(feature = "rttdebug")]
        rprintln!("mt9v034-i2c setup done");
        Ok(())
    }

    pub fn simple_probe(&mut self) -> Result<(), crate::Error<CommE>> {
        let mut recv_buf = [0u8];
        self.i2c
            .read(self.base_address, &mut recv_buf)
            .map_err(Error::Comm)?;
        Ok(())
    }

    /// Read a u8 from an 8-bit address
    pub fn read_reg_u8(&mut self, reg: u8) -> Result<u8, crate::Error<CommE>> {
        // behaves similarly to SCCB serial bus
        let cmd_buf = [reg];
        let mut recv_buf = [0u8];
        self.i2c
            .write(self.base_address, &cmd_buf)
            .map_err(Error::Comm)?;
        self.i2c
            .read(self.base_address, &mut recv_buf)
            .map_err(Error::Comm)?;

        Ok(recv_buf[0])
    }

    /// Read a u16 from an 8-bit address
    pub fn read_reg_u16(
        &mut self,
        reg: u8,
    ) -> Result<u16, crate::Error<CommE>> {
        let upper = (self.read_reg_u8(reg)? as u16) << 8;
        let lower = self.read_reg_u8(FOLLOW_UP_ADDRESS)? as u16;
        Ok(upper | lower)
    }

    /// Write a u8 to an 8-bit address
    pub fn write_reg_u8(
        &mut self,
        reg: u8,
        val: u8,
    ) -> Result<(), crate::Error<CommE>> {
        let write_buf = [reg, val];
        self.i2c
            .write(self.base_address, &write_buf)
            .map_err(Error::Comm)?;
        Ok(())
    }

    /// Write a u16 to an 8-bit address
    pub fn write_reg_u16(
        &mut self,
        reg: u8,
        data: u16,
    ) -> Result<(), crate::Error<CommE>> {
        // write upper u8
        self.write_reg_u8(reg, (data >> 8) as u8)?;
        // write lower u8
        self.write_reg_u8(FOLLOW_UP_ADDRESS, (data & 0xFF) as u8)?;
        Ok(())
    }
}

//TODO add support for register locking:
// If the unique pattern 0xDEAD is written to R0xFE ,
// any subsequent i2c writes to any register _other than R0xFE_ is not committed.
// Subsequently writing the unique pattern 0xBEEF to the R0xFE will unlock registers.

// TODO add support for alternate addresses
//  The sensor has four possible IDs:
//  (0x90, 0x98, 0xB0 and 0xB8) determined by the S_CTRL_ADR0 and S_CTRL_ADR1 input pins.

const I2C_WRITE_ADDRESS: u8 = 0xB8;

/// Used for reading and writing a second byte on registers: aka "Byte-Wise Address register"
const FOLLOW_UP_ADDRESS: u8 = 0xF0;

// Array format: Wide-VGA, Active 752 H x 480 V
pub const MAX_FRAME_HEIGHT: u16 = 480;
pub const MAX_FRAME_WIDTH: u16 = 752;

#[repr(u8)]
pub enum GeneralRegisters {
    ChipVersion = 0x00,
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

/// Allows switching quickly between two separate configuration contexts
#[repr(u16)]
pub enum ParamContext {
    ContextA = 0x0188,
    ContextB = 0x8188,
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
