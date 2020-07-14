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

/// Camera i2c address configuration for {S_CTRL_ADR1, S_CTRL_ADR0} inputs
/// (see Table 6 "address modes" in rev. 7 datasheet)

pub const CAM_PERIPH_ADDRESS_00: u8 = 0x90 >> 1;
pub const CAM_PERIPH_ADDRESS_01: u8 = 0x98 >> 1;
pub const CAM_PERIPH_ADDRESS_10: u8 = 0xB0 >> 1;
pub const CAM_PERIPH_ADDRESS_11: u8 = 0xB8 >> 1;

/// The camera i2c address for the PX4FLOW board (both v1.3 and v2.3)
pub const PX4FLOW_CAM_ADDRESS: u8 = CAM_PERIPH_ADDRESS_11;

/// The camera i2c address for the Arducam breakout board ("UC-396 Rev. A")
pub const ARDUCAM_BREAKOUT_ADDRESS: u8 = CAM_PERIPH_ADDRESS_00;

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
    /// Create a new instance with an i2c address
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            base_address: address,
            i2c,
        }
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
        self.write_general_reg(GeneralRegister::Control, context as u16)
    }

    /// Second-stage configuration
    pub fn setup(&mut self) -> Result<(), crate::Error<CommE>> {
        #[cfg(feature = "rttdebug")]
        rprintln!("mt9v034-i2c setup start 0x{:x}", self.base_address);

        // probe the device: version should match 0x1324 ?
        let _version = self.read_reg_u16(GeneralRegister::ChipVersion as u8)?;

        // configure settings that apply to all contexts
        self.set_general_defaults()?;
        self.set_context_b_defaults()?;
        self.set_context_a_defaults()?;

        //default to Context A
        self.set_context(ParamContext::ContextA)?;

        // restart image collection
        self.write_reg_u8(GeneralRegister::SoftReset as u8, 0b11)?;

        let _verify_version = self.read_reg_u16(GeneralRegister::ChipVersion as u8)?;
        #[cfg(feature = "rttdebug")]
        rprintln!("mt9v034-i2c setup done, vers: 0x{:x}",_verify_version);

        Ok(())
    }

    //TODO add config methods for setting frame dimensions for context A and context B

    fn write_general_reg(
        &mut self,
        reg: GeneralRegister,
        data: u16,
    ) -> Result<(), crate::Error<CommE>> {
        self.write_reg_u16(reg as u8, data)?;
        Ok(())
    }

    fn write_context_a_reg(
        &mut self,
        reg: ContextARegister,
        data: u16,
    ) -> Result<(), crate::Error<CommE>> {
        self.write_reg_u16(reg as u8, data)?;
        Ok(())
    }

    fn write_context_b_reg(
        &mut self,
        reg: ContextBRegister,
        data: u16,
    ) -> Result<(), crate::Error<CommE>> {
        self.write_reg_u16(reg as u8, data)?;
        Ok(())
    }

    /// Set some general configuration defaults
    pub fn set_general_defaults(&mut self) -> Result<(), crate::Error<CommE>> {
        self.write_general_reg(GeneralRegister::RowNoiseCorrCtrl, 0x0101)?; //default noise correction
        self.write_general_reg(GeneralRegister::AecAgcEnable, 0x0011)?; //enable both AEC and AGC
        self.write_general_reg(GeneralRegister::HdrEnable, 0x0001)?; // enable HDR
        self.write_general_reg(GeneralRegister::MinExposure, 0x0001)?;
        self.write_general_reg(GeneralRegister::MaxExposure, 0x1F4)?;

        self.write_general_reg(GeneralRegister::AgcMaxGain, 0x0010)?;
        self.write_general_reg(GeneralRegister::AgcAecPixelCount, 64 * 64)?; // use all pixels
        self.write_general_reg(GeneralRegister::AgcAecDesiredBin, 20)?; //desired luminance
        self.write_general_reg(GeneralRegister::AdcResCtrl, 0x0303)?; // 12 bit ADC

        self.write_general_reg(GeneralRegister::AecUpdate, 0x02)?;
        self.write_general_reg(GeneralRegister::AecLowpass, 0x01)?;
        self.write_general_reg(GeneralRegister::AgcUpdate, 0x02)?;
        self.write_general_reg(GeneralRegister::AgcLowpass, 0x02)?;

        Ok(())
    }

    /// Set configuration defaults for Context B
    pub fn set_context_b_defaults(
        &mut self,
    ) -> Result<(), crate::Error<CommE>> {
        //TODO calculate frame/line sizes
        self.write_context_b_reg(ContextBRegister::WindowWidth, 0)?;
        self.write_context_b_reg(ContextBRegister::WindowHeight, 0)?;
        self.write_context_b_reg(ContextBRegister::HBlanking, 0)?;
        self.write_context_b_reg(ContextBRegister::VBlanking, 0)?;
        self.write_context_b_reg(ContextBRegister::ReadMode, 0x305)?; // row bin 2 col bin 4 enable, (9:8)
        self.write_context_b_reg(ContextBRegister::ColumnStart, 0)?;
        self.write_context_b_reg(ContextBRegister::RowStart, 0)?;
        self.write_context_b_reg(ContextBRegister::CoarseShutter1, 443)?;
        self.write_context_b_reg(ContextBRegister::CoarseShutter2, 473)?;
        self.write_context_b_reg(ContextBRegister::CoarseShutterCtrl, 0x0164)?;
        self.write_context_b_reg(ContextBRegister::CoarseShutterTotal, 480)?;

        Ok(())
    }

    /// Set configuration defaults for Context A
    pub fn set_context_a_defaults(
        &mut self,
    ) -> Result<(), crate::Error<CommE>> {
        //TODO calculate frame/line sizes from passed parameters
        const FLOW_IMG_HEIGHT: u16 = 64;
        const FLOW_IMG_WIDTH: u16 = 64;
        const IMG_W: u16 = FLOW_IMG_WIDTH * 4;
        const IMG_H: u16 = FLOW_IMG_HEIGHT * 4;
        const MIN_H_BLANK: u16 = 91;//min horizontal blanking for "column bin 4 mode"
        // Per datasheet:
        // "The minimum total row time is 704 columns (horizontal width + horizontal blanking).
        // The minimum horizontal blanking is 61 for normal mode, 71 for column bin 2 mode,
        // and 91 for column bin 4 mode. When the window width is set below 643,
        // horizontal blanking must be increased.
        // In binning mode, the minimum row time is R0x04+R0x05 = 704."
        // Note for horiz blanking: 709 is minimum value without distortions
        // Note for vert blanking: 10 the first value without dark line image errors
        const H_BLANK: u16 = 425 + MIN_H_BLANK;
        const V_BLANK: u16 = 10;
        const MAX_FRAME_WIDTH: u16 = 752;
        const MAX_FRAME_HEIGHT: u16 = 480;
        const MIN_COL_START: u16 = 1;
        const MIN_ROW_START: u16 = 4;
        const COL_START: u16 = (MAX_FRAME_WIDTH - IMG_W)/2 + MIN_COL_START;
        const ROW_START: u16 = (MAX_FRAME_HEIGHT - IMG_H)/ 2 + MIN_ROW_START;

        self.write_context_a_reg(ContextARegister::WindowWidth, IMG_W)?;
        self.write_context_a_reg(ContextARegister::WindowHeight, IMG_H)?;
        self.write_context_a_reg(ContextARegister::HBlanking, H_BLANK)?;
        self.write_context_a_reg(ContextARegister::VBlanking, V_BLANK)?;
        self.write_context_a_reg(ContextARegister::ReadMode, 0x30A)?; // row + col bin 4 enable, (9:8) default
        self.write_context_a_reg(ContextARegister::ColumnStart, COL_START)?;
        self.write_context_a_reg(ContextARegister::RowStart, ROW_START)?;
        self.write_context_a_reg(ContextARegister::CoarseShutter1, 443)?;
        self.write_context_a_reg(ContextARegister::CoarseShutter2, 473)?;
        self.write_context_a_reg(ContextARegister::CoarseShutterCtrl, 0x0164)?;
        self.write_context_a_reg(ContextARegister::CoarseShutterTotal, 480)?;

        Ok(())
    }

    //TODO enable/disable test pattern with write to reg 0x7f

    /// Set a test pattern to test pixel flow from the camera
    pub fn enable_pixel_test_pattern(&mut self, enable: bool, pattern: u16)
        -> Result<(), crate::Error<CommE>>
    {
        if enable {
            self.write_general_reg(GeneralRegister::TestPattern, pattern)?;
            //disable row noise correction as well (pass through test pixels)
            self.write_general_reg(GeneralRegister::RowNoiseCorrCtrl, 0x0000)?;
        }
        else {
            // clear the test pattern
            self.write_general_reg(GeneralRegister::TestPattern, 0x0000)?;
            //enable default noise correction
            self.write_general_reg(GeneralRegister::RowNoiseCorrCtrl, 0x0101)?;
        }
        Ok(())
    }

    /// Write-protect (or unprotect) all writable registers.
    /// If you enable register protection, then any subsequent writes to registers
    /// (except the write-protection register itself)
    /// will not be committed.
    pub fn protect_all_registers(
        &mut self,
        protect: bool,
    ) -> Result<(), crate::Error<CommE>> {
        self.write_general_reg(
            GeneralRegister::RegisterLock,
            if protect { 0xDEAD } else { 0xBEEF },
        )
    }

    /// Read a u8 from an 8-bit address
    pub fn read_reg_u8(&mut self, reg: u8) -> Result<u8, crate::Error<CommE>> {
        // behaves similarly to SCCB serial bus
        let cmd_buf = [reg];
        let mut recv_buf = [0u8];
        self.i2c
            .write_read(self.base_address, &cmd_buf, &mut recv_buf)
            .map_err(Error::Comm)?;

        Ok(recv_buf[0])
    }

    /// Read a u16 from an 8-bit address
    pub fn read_reg_u16(
        &mut self,
        reg: u8,
    ) -> Result<u16, crate::Error<CommE>> {
        //TODO can we replace this with a single two-byte read?
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

/// Used for reading and writing a second byte on registers: aka "Byte-Wise Address register"
const FOLLOW_UP_ADDRESS: u8 = 0xF0;

// Array format: Wide-VGA, Active 752 H x 480 V

/// maximum image frame height (pixels)
pub const MAX_FRAME_HEIGHT: u16 = 480;
/// maximum image frame width (pixels)
pub const MAX_FRAME_WIDTH: u16 = 752;

#[repr(u8)]
pub enum GeneralRegister {
    ChipVersion = 0x00,
    /// Control register: used for eg switching config contexts
    Control = 0x07,
    /// Soft Reset of Logic
    SoftReset = 0x0c,
    /// High Dynamic Range enable
    HdrEnable = 0x0f,
    /// ADC Resolution Control
    AdcResCtrl = 0x1c,
    /// Row Noise Correction Control 1
    RowNoiseCorrCtrl = 0x70,
    /// Test pattern storage
    TestPattern = 0x7f,
    /// Tiled digital gain
    TiledDigitalGain = 0x80,
    /// Desired luminance
    AgcAecDesiredBin = 0xa5,
    /// Exposure skip (number of frames to skip between changes in AEC, 0..15)
    AecUpdate = 0xa6,
    /// AEC Lowpass filter (0..2)
    AecLowpass = 0xa8,
    /// Gain skip (number of frames to skip between changes in AGC, 0-15)
    AgcUpdate = 0xa9,
    /// AGC Lowpass filter (0..2)
    AgcLowpass = 0xaa,
    /// AGC Max Gain
    AgcMaxGain = 0xab,
    /// Minimum coarse shutter width
    MinExposure = 0xac,
    /// Maximum coarse shutter width
    MaxExposure = 0xad,
    /// AEC/AGC Enable
    AecAgcEnable = 0xaf,
    /// Histogram pixel count
    AgcAecPixelCount = 0xb0,

    /// Register locking (either All/RW or just RO)
    RegisterLock = 0xfe,
}

/// Allows switching quickly between two separate configuration contexts
#[repr(u16)]
pub enum ParamContext {
    ContextA = 0x0188,
    ContextB = 0x8188,
}

#[repr(u8)]
pub enum ContextARegister {
    ColumnStart = 0x01,
    RowStart = 0x02,
    WindowHeight = 0x03,
    WindowWidth = 0x04,
    /// Horizontal Blanking
    HBlanking = 0x05,
    /// Vertical Blanking
    VBlanking = 0x06,
    /// Coarse Shutter Width 1
    CoarseShutter1 = 0x08,
    /// Coarse Shutter Width 2
    CoarseShutter2 = 0x09,
    /// Coarse Shutter Width Control
    CoarseShutterCtrl = 0x0A,
    /// Coarse Shutter Width Total
    CoarseShutterTotal = 0x0B,
    ReadMode = 0x0D,
    V1Ctrl = 0x31,
    V2Ctrl = 0x32,
    V3Ctrl = 0x33,
    V4Ctrl = 0x34,
    /// Analog Gain Control
    AnalogGainCtrl = 0x35,
    /// Fine Shutter Width 1
    FineShutter1 = 0xD3,
    /// Fine Shutter Width 2
    FineShutter2 = 0xD4,
    /// Fine Shutter Width Total
    FineShutterTotal = 0xD5,
}

#[repr(u8)]
pub enum ContextBRegister {
    ColumnStart = 0xC9,
    RowStart = 0xCA,
    WindowHeight = 0xCB,
    WindowWidth = 0xCC,
    HBlanking = 0xCD,
    VBlanking = 0xCE,
    CoarseShutter1 = 0xCF,
    CoarseShutter2 = 0xD0,
    CoarseShutterCtrl = 0xD1,
    CoarseShutterTotal = 0xD2,
    ReadMode = 0x0E,
    V1Ctrl = 0x39,
    V2Ctrl = 0x3A,
    V3Ctrl = 0x3B,
    V4Ctrl = 0x3C,
    AnalogGainCtrl = 0x36,
    FineShutter1 = 0xD6,
    FineShutter2 = 0xD7,
    FineShutterTotal = 0xD8,
}
