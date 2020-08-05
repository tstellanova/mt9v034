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
    win_width_a: u16,
    win_height_a: u16,
    win_width_b: u16,
    win_height_b: u16,
    col_bin_factor_a: BinningFactor,
    row_bin_factor_a: BinningFactor,
    col_bin_factor_b: BinningFactor,
    row_bin_factor_b: BinningFactor,
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
            win_width_a: 0,
            win_height_a: 0,
            win_width_b: 0,
            win_height_b: 0,
            col_bin_factor_a: BinningFactor::None,
            row_bin_factor_a: BinningFactor::None,
            col_bin_factor_b: BinningFactor::None,
            row_bin_factor_b: BinningFactor::None,
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

        // setup_with_dimensions
        self.set_context_b_default_dimensions()?;
        self.set_context_b_shutter_defaults()?;
        self.set_context_a_default_dimensions()?;
        self.set_context_a_shutter_defaults()?;

        // configure settings that apply to all contexts
        // 64x64 is the default image size for Context A (small square flow)
        self.set_general_defaults(4096)?;

        //Select Context A
        self.set_context(ParamContext::ContextA)?;

        // restart image collection
        self.write_general_reg(GeneralRegister::SoftReset, 0x01)?;

        let _verify_version =
            self.read_reg_u16(GeneralRegister::ChipVersion as u8)?;
        #[cfg(feature = "rttdebug")]
        rprintln!("mt9v034-i2c setup done, vers: 0x{:x}", _verify_version);

        Ok(())
    }

    /// Setup with configurable Context A and Context B dimensions.
    /// Use this method instead `setup` if you know the exact dimensions
    /// and binning you need.
    /// - `win_width` and `win_height` are the window dimensions in pixels that will be collected
    /// - `col_bin` is the column binning and `row_bin` is the row binning that describes how
    /// many rows or columns are combined from the window into single result pixels
    /// - `default_context` sets which configuration context (A or B) is set as the
    /// initial active context
    ///
    pub fn setup_with_dimensions(
        &mut self,
        win_width_a: u16,
        win_height_a: u16,
        col_bin_a: BinningFactor,
        row_bin_a: BinningFactor,
        win_width_b: u16,
        win_height_b: u16,
        col_bin_b: BinningFactor,
        row_bin_b: BinningFactor,
        default_context: ParamContext,
    ) -> Result<(), crate::Error<CommE>> {
        #[cfg(feature = "rttdebug")]
        rprintln!("mt9v034-i2c setup start 0x{:x}", self.base_address);

        // probe the device: version should match 0x1324 ?
        // with blocking i2c, the program will hang here if the device can't
        // be contacted
        let _version = self.read_reg_u16(GeneralRegister::ChipVersion as u8)?;

        self.set_context_dimensions(
            ParamContext::ContextB,
            win_height_b,
            win_width_b,
            col_bin_b,
            row_bin_b,
        )?;

        self.set_context_b_shutter_defaults()?;
        self.set_context_dimensions(
            ParamContext::ContextA,
            win_height_a,
            win_width_a,
            col_bin_a,
            row_bin_a,
        )?;
        self.set_context_a_shutter_defaults()?;

        let max_pixels = match default_context {
            ParamContext::ContextA => {
                (self.win_height_a / self.row_bin_factor_a as u16)
                    * (self.win_width_a / self.col_bin_factor_a as u16)
            }
            ParamContext::ContextB => {
                (self.win_height_b / self.row_bin_factor_b as u16)
                    * (self.win_width_b / self.col_bin_factor_b as u16)
            }
        };
        // configure settings that apply to all contexts
        self.set_general_defaults(max_pixels as u32)?;

        // set an initial context
        self.set_context(default_context)?;

        // restart image collection after changing dimensions
        self.write_general_reg(GeneralRegister::SoftReset, 0x01)?;

        let _verify_version =
            self.read_reg_u16(GeneralRegister::ChipVersion as u8)?;
        #[cfg(feature = "rttdebug")]
        rprintln!("mt9v034-i2c setup done, vers: 0x{:x}", _verify_version);

        Ok(())
    }

    fn write_general_reg(
        &mut self,
        reg: GeneralRegister,
        data: u16,
    ) -> Result<(), crate::Error<CommE>> {
        self.write_reg_u16(reg as u8, data)
    }

    fn write_context_a_reg(
        &mut self,
        reg: ContextARegister,
        data: u16,
    ) -> Result<(), crate::Error<CommE>> {
        self.write_reg_u16(reg as u8, data)
    }

    fn write_context_b_reg(
        &mut self,
        reg: ContextBRegister,
        data: u16,
    ) -> Result<(), crate::Error<CommE>> {
        self.write_reg_u16(reg as u8, data)
    }

    /// Set just the maximum pixels to be used for adjusting automatic gain control
    /// Note this the _output_ pixel count, ie the pixels post-binning
    pub fn set_agc_pixel_count(
        &mut self,
        max_pixels: u32,
    ) -> Result<(), crate::Error<CommE>> {
        let agc_pixels: u16 = if max_pixels > 65535 {
            65535
        } else {
            max_pixels as u16
        };
        self.write_general_reg(GeneralRegister::AgcAecPixelCount, agc_pixels)
    }

    /// Set some general configuration defaults
    /// - `max_pixel_count` is the maximum output pixels that will be used in the default context
    pub fn set_general_defaults(
        &mut self,
        max_pixel_count: u32,
    ) -> Result<(), crate::Error<CommE>> {
        self.write_reg_u8(GeneralRegister::RowNoiseConstant as u8, 0x00)?;

        // reserved register recommendations from:
        // "Table 8. RECOMMENDED REGISTER SETTINGS AND PERFORMANCE IMPACT (RESERVED REGISTERS)"
        self.write_reg_u16(0x13, 0x2D2E)?; // reg 0x13 = 0x2d32 (11570)
        self.write_reg_u16(0x20, 0x03C7)?; // reg 0x20 = 0x1c1 (449)
        self.write_reg_u16(0x24, 0x001B)?; // reg 0x24 = 0x10 (16)
        self.write_reg_u16(0x2B, 0x0003)?; // reg 0x2B = 0x4 (4)
        self.write_reg_u16(0x2F, 0x0003)?; // reg 0x2F = 0x4 (4)

        // disable any test pattern by default
        self.write_general_reg(GeneralRegister::TestPattern, 0x0000)?;

        self.write_general_reg(GeneralRegister::RowNoiseCorrCtrl, 0x0101)?; //default noise correction
        self.write_general_reg(GeneralRegister::AecAgcEnable, 0x0011)?; //enable both AEC and AGC
        self.write_general_reg(GeneralRegister::HdrEnable, 0x0001)?; // enable HDR
        self.write_general_reg(GeneralRegister::MinExposure, 0x0001)?;
        self.write_general_reg(GeneralRegister::MaxExposure, 0x1F4)?;

        self.write_general_reg(GeneralRegister::AgcMaxGain, 0x0010)?;
        self.set_agc_pixel_count(max_pixel_count)?;
        self.write_general_reg(GeneralRegister::AgcAecDesiredBin, 20)?; //desired luminance
        self.write_general_reg(GeneralRegister::AdcResCtrl, 0x0303)?; // 12 bit ADC

        self.write_general_reg(GeneralRegister::AecUpdate, 0x02)?;
        self.write_general_reg(GeneralRegister::AecLowpass, 0x01)?;
        self.write_general_reg(GeneralRegister::AgcUpdate, 0x02)?;
        self.write_general_reg(GeneralRegister::AgcLowpass, 0x02)?;

        Ok(())
    }

    /// Set default image capture dimensions for Context B
    pub fn set_context_b_default_dimensions(
        &mut self,
    ) -> Result<(), crate::Error<CommE>> {
        //TODO calculate frame/line sizes
        const VIDEO_IMG_HEIGHT: u16 = 480 / 2;
        const VIDEO_IMG_WIDTH: u16 = 752 / 2;
        const COLUMN_BINNING: BinningFactor = BinningFactor::Two;
        const ROW_BINNING: BinningFactor = BinningFactor::Two;
        const WINDOW_W: u16 = VIDEO_IMG_WIDTH * 2;
        const WINDOW_H: u16 = VIDEO_IMG_HEIGHT * 2;

        self.set_context_dimensions(
            ParamContext::ContextB,
            WINDOW_H,
            WINDOW_W,
            COLUMN_BINNING,
            ROW_BINNING,
        )
    }

    /// Set default image capture dimensions for Context A
    pub fn set_context_a_default_dimensions(
        &mut self,
    ) -> Result<(), crate::Error<CommE>> {
        const FLOW_IMG_HEIGHT: u16 = 64;
        const FLOW_IMG_WIDTH: u16 = 64;
        const COLUMN_BINNING: BinningFactor = BinningFactor::Four;
        const ROW_BINNING: BinningFactor = BinningFactor::Four;
        const WINDOW_W: u16 = FLOW_IMG_WIDTH * 4;
        const WINDOW_H: u16 = FLOW_IMG_HEIGHT * 4;

        self.set_context_dimensions(
            ParamContext::ContextA,
            WINDOW_H,
            WINDOW_W,
            COLUMN_BINNING,
            ROW_BINNING,
        )
    }

    /// Configure image capture dimensions for the given context
    /// - `context` the configuration context (A or B) to configure
    /// - `window_h` and `window_w`: dimensions of the pixel window to collect pixels from
    /// - `column_binning` binning to apply to the window's columns
    /// - `row_binning` binning to apply to the window's rows
    pub fn set_context_dimensions(
        &mut self,
        context: ParamContext,
        window_h: u16,
        window_w: u16,
        col_bin_factor: BinningFactor,
        row_bin_factor: BinningFactor,
    ) -> Result<(), crate::Error<CommE>> {
        // Per datasheet:
        // "The minimum total row time is 704 columns (horizontal width + horizontal blanking).
        // The minimum horizontal blanking is:
        // - 61 for normal mode,
        // - 71 for column bin 2 mode,
        // - 91 for column bin 4 mode.
        // When the window width is set below 643,  horizontal blanking must be increased.
        // In binning mode, the minimum row time is R0x04+R0x05 = 704."

        let min_h_blank: u16 = match col_bin_factor {
            BinningFactor::None => 61,
            BinningFactor::Two => 71,
            BinningFactor::Four => 91,
        };

        let col_binning = binning_factor_to_selector(col_bin_factor);
        let row_binning = binning_factor_to_selector(row_bin_factor);

        // Note for vert blanking: 10 the first value without dark line image errors

        //TODO calculate  V_BLANK and H_BLANK based on parameter inputs
        let h_blank: u16 = 425 + min_h_blank;
        const V_BLANK: u16 = 10;

        const MIN_COL_START: u16 = 1;
        const MIN_ROW_START: u16 = 4; //TODO verify "dark rows"
                                      // center the window horizontally
        let col_start: u16 = (MAX_FRAME_WIDTH - window_w) / 2 + MIN_COL_START;
        // center the window vertically
        let row_start: u16 = (MAX_FRAME_HEIGHT - window_h) / 2 + MIN_ROW_START;

        //s/b 0x30A with both bin 4:
        // 0x300 is the default value for 9:8 on ReadMode
        let read_mode: u16 =
            0x300 | ((col_binning as u16) << 2) | (row_binning as u16);

        match context {
            ParamContext::ContextA => {
                self.win_width_a = window_w;
                self.win_height_a = window_h;
                self.col_bin_factor_a = col_bin_factor;
                self.row_bin_factor_a = row_bin_factor;
                self.write_context_a_reg(
                    ContextARegister::WindowWidth,
                    window_w,
                )?;
                self.write_context_a_reg(
                    ContextARegister::WindowHeight,
                    window_h,
                )?;
                self.write_context_a_reg(ContextARegister::HBlanking, h_blank)?;
                self.write_context_a_reg(ContextARegister::VBlanking, V_BLANK)?;
                self.write_context_a_reg(
                    ContextARegister::ReadMode,
                    read_mode,
                )?;
                self.write_context_a_reg(
                    ContextARegister::ColumnStart,
                    col_start,
                )?;
                self.write_context_a_reg(
                    ContextARegister::RowStart,
                    row_start,
                )?;
            }
            ParamContext::ContextB => {
                self.win_width_b = window_w;
                self.win_height_b = window_h;
                self.col_bin_factor_b = col_bin_factor;
                self.row_bin_factor_b = row_bin_factor;
                self.write_context_b_reg(
                    ContextBRegister::WindowWidth,
                    window_w,
                )?;
                self.write_context_b_reg(
                    ContextBRegister::WindowHeight,
                    window_h,
                )?;
                self.write_context_b_reg(ContextBRegister::HBlanking, h_blank)?;
                self.write_context_b_reg(ContextBRegister::VBlanking, V_BLANK)?;
                self.write_context_b_reg(
                    ContextBRegister::ReadMode,
                    read_mode,
                )?;
                self.write_context_b_reg(
                    ContextBRegister::ColumnStart,
                    col_start,
                )?;
                self.write_context_b_reg(
                    ContextBRegister::RowStart,
                    row_start,
                )?;
            }
        }

        Ok(())
    }

    /// Set default shutter values for Context A
    pub fn set_context_a_shutter_defaults(
        &mut self,
    ) -> Result<(), crate::Error<CommE>> {
        //TODO allow passing shutter control parameters?
        // by default we activate HDR
        self.write_context_a_reg(ContextARegister::CoarseShutter1, 443)?; //default value
        self.write_context_a_reg(ContextARegister::CoarseShutter2, 473)?; //default value
        self.write_context_a_reg(ContextARegister::CoarseShutterCtrl, 0x0164)?; //default value
        self.write_context_a_reg(ContextARegister::CoarseShutterTotal, 0x01E0)?; //default value
        Ok(())
    }

    /// Set default shutter values for Context B
    pub fn set_context_b_shutter_defaults(
        &mut self,
    ) -> Result<(), crate::Error<CommE>> {
        //TODO allow passing shutter control parameters?
        // by default we activate HDR
        self.write_context_b_reg(ContextBRegister::CoarseShutter1, 443)?; //default value
        self.write_context_b_reg(ContextBRegister::CoarseShutter2, 473)?; //default value
        self.write_context_b_reg(ContextBRegister::CoarseShutterCtrl, 0x0164)?; //default value
        self.write_context_b_reg(ContextBRegister::CoarseShutterTotal, 0x01E0)?; //default value
        Ok(())
    }

    #[cfg(feature = "rttdebug")]
    pub fn dump_context_a_settings(
        &mut self,
    ) -> Result<(), crate::Error<CommE>> {
        rprintln!("-- Context A settings:");
        self.dump_register_setting(ContextARegister::WindowWidth as u8)?;
        self.dump_register_setting(ContextARegister::WindowHeight as u8)?;
        self.dump_register_setting(ContextARegister::HBlanking as u8)?;
        self.dump_register_setting(ContextARegister::VBlanking as u8)?;
        self.dump_register_setting(ContextARegister::ReadMode as u8)?;
        self.dump_register_setting(ContextARegister::ColumnStart as u8)?;
        self.dump_register_setting(ContextARegister::RowStart as u8)?;
        self.dump_register_setting(ContextARegister::CoarseShutter1 as u8)?;
        self.dump_register_setting(ContextARegister::CoarseShutter2 as u8)?;
        self.dump_register_setting(ContextARegister::CoarseShutterCtrl as u8)?;
        self.dump_register_setting(ContextARegister::CoarseShutterTotal as u8)?;
        Ok(())
    }

    #[cfg(feature = "rttdebug")]
    pub fn dump_context_b_settings(
        &mut self,
    ) -> Result<(), crate::Error<CommE>> {
        rprintln!("-- Context B settings:");
        self.dump_register_setting(ContextBRegister::WindowWidth as u8)?;
        self.dump_register_setting(ContextBRegister::WindowHeight as u8)?;
        self.dump_register_setting(ContextBRegister::HBlanking as u8)?;
        self.dump_register_setting(ContextBRegister::VBlanking as u8)?;
        self.dump_register_setting(ContextBRegister::ReadMode as u8)?;
        self.dump_register_setting(ContextBRegister::ColumnStart as u8)?;
        self.dump_register_setting(ContextBRegister::RowStart as u8)?;
        self.dump_register_setting(ContextBRegister::CoarseShutter1 as u8)?;
        self.dump_register_setting(ContextBRegister::CoarseShutter2 as u8)?;
        self.dump_register_setting(ContextBRegister::CoarseShutterCtrl as u8)?;
        self.dump_register_setting(ContextBRegister::CoarseShutterTotal as u8)?;
        Ok(())
    }

    #[cfg(feature = "rttdebug")]
    pub fn dump_general_settings(&mut self) -> Result<(), crate::Error<CommE>> {
        rprintln!("-- General settings:");
        self.dump_register_setting(GeneralRegister::Control as u8)?;
        self.dump_register_setting(GeneralRegister::RowNoiseConstant as u8)?;

        // reserved register recommendation
        self.dump_register_setting(0x13)?;
        self.dump_register_setting(0x20)?;
        self.dump_register_setting(0x24)?;
        self.dump_register_setting(0x2B)?;
        self.dump_register_setting(0x2F)?;

        self.dump_register_setting(GeneralRegister::RowNoiseCorrCtrl as u8)?; //default noise correction
        self.dump_register_setting(GeneralRegister::TestPattern as u8)?; //Test pattern

        self.dump_register_setting(GeneralRegister::AecAgcEnable as u8)?; //enable both AEC and AGC
        self.dump_register_setting(GeneralRegister::HdrEnable as u8)?; // enable HDR
        self.dump_register_setting(GeneralRegister::MinExposure as u8)?;
        self.dump_register_setting(GeneralRegister::MaxExposure as u8)?;

        self.dump_register_setting(GeneralRegister::AgcMaxGain as u8)?;
        self.dump_register_setting(GeneralRegister::AgcAecPixelCount as u8)?; // use all pixels
        self.dump_register_setting(GeneralRegister::AgcAecDesiredBin as u8)?; //desired luminance
        self.dump_register_setting(GeneralRegister::AdcResCtrl as u8)?; // 12 bit ADC

        self.dump_register_setting(GeneralRegister::AecUpdate as u8)?;
        self.dump_register_setting(GeneralRegister::AecLowpass as u8)?;
        self.dump_register_setting(GeneralRegister::AgcUpdate as u8)?;
        self.dump_register_setting(GeneralRegister::AgcLowpass as u8)?;
        Ok(())
    }

    #[cfg(feature = "rttdebug")]
    pub fn dump_register_setting(
        &mut self,
        reg: u8,
    ) -> Result<(), crate::Error<CommE>> {
        let val = self.read_reg_u16(reg)?;
        rprintln!("0x{:X} = 0x{:x} {}", reg, val, val);
        Ok(())
    }

    /// Set a test pattern to test pixel data transfer from the camera
    pub fn enable_pixel_test_pattern(
        &mut self,
        enable: bool,
        pattern: PixelTestPattern,
    ) -> Result<(), crate::Error<CommE>> {
        if enable {
            self.write_general_reg(
                GeneralRegister::TestPattern,
                (pattern as u16) | 0x2000,
            )?;
            //disable row noise correction as well (pass through test pixels)
            self.write_general_reg(GeneralRegister::RowNoiseCorrCtrl, 0x0000)?;
        } else {
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
    /// Row Noise Constant
    RowNoiseConstant = 0x72,
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

/// Pixel test patterns for verifying correct pixel transfer from the camera
#[repr(u16)]
pub enum PixelTestPattern {
    None = 0x0000,
    VerticalShade = 0x0800,
    HorizontalShade = 0x1000,
    DiagonalShade = 0x1800,
}

#[repr(u16)]
#[derive(Copy, Clone, Debug)]
pub enum BinningFactor {
    /// No binning (full resolution)
    None = 1,
    /// Binning 2: combine two adjacent pixels
    Two = 2,
    /// Binning 4: combine four adjacent pixels
    Four = 4,
}

/// Values sent to the mt9v034 to select binning
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
enum BinningSelector {
    /// No binning (full resolution)
    None = 0b00,
    /// Binning 2: combine two adjacent pixels
    Two = 0b01,
    /// Binning 4: combine four adjacent pixels
    Four = 0b10,
}

/// Convert actual binning factor ( {1,2,4} ) to binning selector
fn binning_factor_to_selector(factor: BinningFactor) -> BinningSelector {
    match factor {
        BinningFactor::None => BinningSelector::None,
        BinningFactor::Two => BinningSelector::Two,
        BinningFactor::Four => BinningSelector::Four,
    }
}
