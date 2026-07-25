//! JTAG protocol helpers using MPSSE.
//!
//! Provides high-level JTAG TAP operations using the FTDI MPSSE engine.
//! Supports the standard JTAG signals: TCK, TDI, TDO, and TMS.
//!
//! # Pin Mapping
//!
//! | FTDI Pin | JTAG Signal | ADBUS Bit |
//! |----------|-------------|-----------|
//! | SK       | TCK         | 0         |
//! | DO       | TDI         | 1         |
//! | DI       | TDO         | 2         |
//! | GPIOL0   | TMS         | 3         |
//!
//! TRST# (optional) can be assigned to any available GPIO pin (ADBUS4-7 or
//! ACBUS0-7) and managed via `MpsseContext::set_gpio_low()` /
//! `set_gpio_high()`.
//!
//! # JTAG TAP State Machine
//!
//! The module provides primitives for navigating the JTAG Test Access Port
//! (TAP) state machine:
//!
//! - [`JtagBus::reset`] — Force TAP to Test-Logic-Reset via 5 TMS=1 clocks
//! - [`JtagBus::goto_shift_ir`] — Navigate from Run-Test/Idle to Shift-IR
//! - [`JtagBus::goto_shift_dr`] — Navigate from Run-Test/Idle to Shift-DR
//! - [`JtagBus::shift_bits`] — Shift data through TDI/TDO in Shift-IR/DR
//! - [`JtagBus::goto_idle`] — Return to Run-Test/Idle from Exit1
//!
//! # Example
//!
//! ```no_run
//! use ftdi_nusb::{FtdiDevice, mpsse::{MpsseContext, jtag::JtagBus}};
//!
//! let mut dev = FtdiDevice::open(0x0403, 0x6014)?; // FT232H
//! let mut mpsse = MpsseContext::init(&mut dev, 1_000_000)?; // 1 MHz TCK
//! let mut jtag = JtagBus::new(&mut mpsse, &mut dev)?;
//!
//! // Reset TAP to known state
//! jtag.reset(&mut dev)?;
//!
//! // Read IDCODE (first DR after reset is IDCODE on most devices)
//! jtag.goto_shift_dr(&mut dev)?;
//! let idcode = jtag.shift_bits(&mpsse, &mut dev, &[0; 4], 32, true)?;
//! jtag.goto_idle(&mut dev)?;
//! # Ok::<(), ftdi_nusb::Error>(())
//! ```

use maybe_async::maybe_async;

use crate::constants::mpsse;
use crate::context::FtdiDevice;
use crate::error::{Error, Result};

use super::MpsseContext;

/// JTAG TAP state (simplified — not all states tracked).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TapState {
    /// Test-Logic-Reset.
    Reset,
    /// Run-Test/Idle.
    Idle,
    /// Shift-DR state.
    ShiftDr,
    /// Shift-IR state.
    ShiftIr,
    /// Exit1-DR state.
    Exit1Dr,
    /// Exit1-IR state.
    Exit1Ir,
    /// Unknown / not tracked.
    Unknown,
}

/// JTAG bus instance using MPSSE.
///
/// Manages JTAG pin configuration and TAP state tracking.
#[derive(Debug, Clone)]
pub struct JtagBus {
    /// TMS pin bit mask in the low GPIO byte (default: ADBUS3 = 0x08).
    tms_pin: u8,
    /// Low-byte GPIO direction mask.
    dir_mask: u8,
    /// Current tracked TAP state.
    state: TapState,
}

impl JtagBus {
    /// Initialize JTAG mode on the MPSSE.
    ///
    /// Configures pins for JTAG operation:
    /// - ADBUS0 (SK) = TCK output
    /// - ADBUS1 (DO) = TDI output
    /// - ADBUS2 (DI) = TDO input
    /// - ADBUS3 (GPIOL0) = TMS output
    ///
    /// TCK and TMS are driven low initially. The TAP state is set to Unknown;
    /// call [`reset`](Self::reset) to bring it to a known state.
    #[maybe_async]
    pub async fn new(ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<Self> {
        let tms_pin = 0x08; // ADBUS3

        // Direction: TCK(0)=out, TDI(1)=out, TDO(2)=in, TMS(3)=out
        let dir_mask = 0x0B; // bits 0,1,3 = output

        // Initial state: TCK=0, TDI=0, TMS=0
        ctx.set_gpio_low(dev, 0x00, dir_mask).await?;

        Ok(Self {
            tms_pin,
            dir_mask,
            state: TapState::Unknown,
        })
    }

    /// Get the current tracked TAP state.
    pub fn state(&self) -> TapState {
        self.state
    }

    /// Reset the TAP state machine by clocking TMS=1 for 5 cycles.
    ///
    /// This forces the TAP into Test-Logic-Reset regardless of its
    /// current state. Then clocks once with TMS=0 to enter Run-Test/Idle.
    #[maybe_async]
    pub async fn reset(&mut self, dev: &mut FtdiDevice) -> Result<()> {
        // WRITE_TMS: clock TMS bits out. The MPSSE command is:
        //   0x4B length_minus_1 data_byte
        // where data bits are clocked LSB first on TMS, TDI is held at bit 7.
        let mut cmd = Vec::with_capacity(16);

        // 5 clocks with TMS=1 (bits: 0b11111 = 0x1F, length=4 meaning 5 bits)
        // TDI held at 0 (bit 7 of data byte = 0)
        cmd.push(mpsse::WRITE_TMS | mpsse::WRITE_NEG | mpsse::BITMODE | mpsse::LSB);
        cmd.push(4); // 5 bits (0-indexed)
        cmd.push(0x1F); // TMS=1,1,1,1,1 (LSB first), TDI=0 (bit 7)

        // 1 clock with TMS=0 to go to Run-Test/Idle
        cmd.push(mpsse::WRITE_TMS | mpsse::WRITE_NEG | mpsse::BITMODE | mpsse::LSB);
        cmd.push(0); // 1 bit
        cmd.push(0x00); // TMS=0

        dev.write_all(&cmd).await?;
        self.state = TapState::Idle;
        Ok(())
    }

    /// Navigate from Run-Test/Idle to Shift-DR.
    ///
    /// TMS sequence: 1, 0, 0 (Select-DR-Scan -> Capture-DR -> Shift-DR).
    #[maybe_async]
    pub async fn goto_shift_dr(&mut self, dev: &mut FtdiDevice) -> Result<()> {
        let mut cmd = Vec::with_capacity(4);
        // 3 bits: TMS = 1,0,0 -> LSB first = 0b001 = 0x01
        cmd.push(mpsse::WRITE_TMS | mpsse::WRITE_NEG | mpsse::BITMODE | mpsse::LSB);
        cmd.push(2); // 3 bits
        cmd.push(0x01); // TMS: bit0=1, bit1=0, bit2=0

        dev.write_all(&cmd).await?;
        self.state = TapState::ShiftDr;
        Ok(())
    }

    /// Navigate from Run-Test/Idle to Shift-IR.
    ///
    /// TMS sequence: 1, 1, 0, 0 (Select-DR-Scan -> Select-IR-Scan ->
    /// Capture-IR -> Shift-IR).
    #[maybe_async]
    pub async fn goto_shift_ir(&mut self, dev: &mut FtdiDevice) -> Result<()> {
        let mut cmd = Vec::with_capacity(4);
        // 4 bits: TMS = 1,1,0,0 -> LSB first = 0b0011 = 0x03
        cmd.push(mpsse::WRITE_TMS | mpsse::WRITE_NEG | mpsse::BITMODE | mpsse::LSB);
        cmd.push(3); // 4 bits
        cmd.push(0x03); // TMS: bit0=1, bit1=1, bit2=0, bit3=0

        dev.write_all(&cmd).await?;
        self.state = TapState::ShiftIr;
        Ok(())
    }

    /// Navigate from Exit1-DR/Exit1-IR to Run-Test/Idle.
    ///
    /// TMS sequence: 1, 0 (Update-DR/IR -> Run-Test/Idle).
    #[maybe_async]
    pub async fn goto_idle(&mut self, dev: &mut FtdiDevice) -> Result<()> {
        let mut cmd = Vec::with_capacity(4);
        // 2 bits: TMS = 1,0 -> LSB first = 0b01 = 0x01
        cmd.push(mpsse::WRITE_TMS | mpsse::WRITE_NEG | mpsse::BITMODE | mpsse::LSB);
        cmd.push(1); // 2 bits
        cmd.push(0x01); // TMS: bit0=1, bit1=0

        dev.write_all(&cmd).await?;
        self.state = TapState::Idle;
        Ok(())
    }

    /// Clock idle cycles in Run-Test/Idle (TMS=0).
    ///
    /// Useful for devices that require a certain number of TCK cycles
    /// in Run-Test/Idle for internal processing.
    #[maybe_async]
    pub async fn idle_clocks(&self, dev: &mut FtdiDevice, count: u32) -> Result<()> {
        if count == 0 {
            return Ok(());
        }

        let mut cmd = Vec::new();

        // Use CLK_BITS for small counts, CLK_BYTES for larger
        let mut remaining = count;
        while remaining > 0 {
            if remaining >= 8 {
                let bytes = (remaining / 8).min(0x10000) as u16;
                cmd.push(mpsse::CLK_BYTES);
                cmd.push((bytes - 1) as u8);
                cmd.push(((bytes - 1) >> 8) as u8);
                remaining -= bytes as u32 * 8;
            } else {
                cmd.push(mpsse::CLK_BITS);
                cmd.push((remaining - 1) as u8);
                remaining = 0;
            }
        }

        dev.write_all(&cmd).await
    }

    /// Maximum bytes per single MPSSE transfer command (2-byte length field,
    /// encoding len-1). Same limit as the SPI module.
    const MAX_MPSSE_TRANSFER: usize = 65536;

    /// Shift data through TDI/TDO while in Shift-DR or Shift-IR.
    ///
    /// Clocks `bit_count` bits through the scan chain. Data is shifted
    /// LSB first. The last bit is clocked with TMS=1 to exit to Exit1-DR/IR
    /// if `exit_shift` is true (this is the standard JTAG behavior).
    ///
    /// `tdi_data` provides the bits to clock out on TDI. If it has fewer
    /// bits than `bit_count`, the remaining bits are zero-padded.
    ///
    /// Large shifts (>65536 bytes) are automatically split into multiple
    /// MPSSE commands.
    ///
    /// Returns the captured TDO data as a byte vector. The first `bit_count`
    /// bits are valid (LSB first within each byte).
    #[maybe_async]
    pub async fn shift_bits(
        &mut self,
        _ctx: &MpsseContext,
        dev: &mut FtdiDevice,
        tdi_data: &[u8],
        bit_count: usize,
        exit_shift: bool,
    ) -> Result<Vec<u8>> {
        if bit_count == 0 {
            return Ok(Vec::new());
        }

        let byte_count = bit_count.div_ceil(8);
        let mut tdi = vec![0u8; byte_count];
        let copy_len = tdi_data.len().min(byte_count);
        tdi[..copy_len].copy_from_slice(&tdi_data[..copy_len]);

        let mut cmd = Vec::with_capacity(16 + byte_count);

        // Determine how many bits to shift with normal TMS=0 and how many
        // with TMS=1 (the final bit, if exiting).
        let normal_bits = if exit_shift { bit_count - 1 } else { bit_count };

        // Shift full bytes (chunked to MAX_MPSSE_TRANSFER)
        let full_bytes = normal_bits / 8;
        let remaining_bits = normal_bits % 8;
        let rw_cmd = mpsse::DO_WRITE | mpsse::DO_READ | mpsse::WRITE_NEG | mpsse::LSB;

        {
            let mut offset = 0;
            while offset < full_bytes {
                let chunk = (full_bytes - offset).min(Self::MAX_MPSSE_TRANSFER);
                let len_field = (chunk - 1) as u16;
                cmd.push(rw_cmd);
                cmd.push(len_field as u8);
                cmd.push((len_field >> 8) as u8);
                cmd.extend_from_slice(&tdi[offset..offset + chunk]);
                offset += chunk;
            }
        }

        // Shift remaining bits (not including the exit bit)
        if remaining_bits > 0 {
            cmd.push(rw_cmd | mpsse::BITMODE);
            cmd.push((remaining_bits - 1) as u8);
            cmd.push(tdi[full_bytes]);
        }

        // Exit bit: clock last bit with TMS=1
        if exit_shift {
            let last_bit_byte_idx = (bit_count - 1) / 8;
            let last_bit_bit_idx = (bit_count - 1) % 8;
            let last_tdi_bit = (tdi[last_bit_byte_idx] >> last_bit_bit_idx) & 1;

            // WRITE_TMS with DO_READ: TMS=1, TDI=last_bit
            cmd.push(
                mpsse::WRITE_TMS | mpsse::DO_READ | mpsse::WRITE_NEG | mpsse::BITMODE | mpsse::LSB,
            );
            cmd.push(0); // 1 bit
                         // bit 0 = TMS value (1 = exit), bit 7 = TDI value
            cmd.push(0x01 | (last_tdi_bit << 7));

            self.state = match self.state {
                TapState::ShiftDr => TapState::Exit1Dr,
                TapState::ShiftIr => TapState::Exit1Ir,
                _ => TapState::Unknown,
            };
        }

        cmd.push(mpsse::SEND_IMMEDIATE);
        dev.write_all(&cmd).await?;

        // Calculate expected response bytes
        let mut expected = full_bytes; // one byte per full byte shifted
        if remaining_bits > 0 {
            expected += 1; // bit-mode read returns 1 byte
        }
        if exit_shift {
            expected += 1; // TMS+read returns 1 byte
        }

        // Read response — error on short read (don't silently return partial data)
        let response = Self::read_exact(dev, expected).await?;

        // Reassemble TDO bits into a contiguous byte vector
        let mut tdo = vec![0u8; byte_count];
        let mut tdo_bit = 0usize;
        let mut resp_idx = 0;

        // Full bytes come back as-is
        if full_bytes > 0 {
            tdo[..full_bytes].copy_from_slice(&response[..full_bytes]);
            tdo_bit += full_bytes * 8;
            resp_idx += full_bytes;
        }

        // Remaining bits: returned right-aligned in one byte
        if remaining_bits > 0 {
            let bits_byte = response[resp_idx];
            // The MPSSE returns bit-mode reads right-shifted so the MSB of
            // the result is in bit (8 - remaining_bits).
            let shifted = bits_byte >> (8 - remaining_bits);
            for bit in 0..remaining_bits {
                if shifted & (1 << bit) != 0 {
                    tdo[tdo_bit / 8] |= 1 << (tdo_bit % 8);
                }
                tdo_bit += 1;
            }
            resp_idx += 1;
        }

        // Exit bit: WRITE_TMS + DO_READ returns the sampled TDO in bit 7
        // of the response byte (per FTDI AN-108).
        if exit_shift {
            let tms_byte = response[resp_idx];
            if tms_byte & 0x80 != 0 {
                tdo[tdo_bit / 8] |= 1 << (tdo_bit % 8);
            }
        }

        Ok(tdo)
    }

    /// Read exactly `len` bytes from the device, returning an error on short reads.
    #[maybe_async]
    async fn read_exact(dev: &mut FtdiDevice, len: usize) -> Result<Vec<u8>> {
        let mut buf = vec![0u8; len];
        let mut offset = 0;
        while offset < len {
            let n = dev.read_data(&mut buf[offset..]).await?;
            if n == 0 {
                return Err(Error::InvalidArgument(
                    "JTAG read returned fewer bytes than expected",
                ));
            }
            offset += n;
        }
        Ok(buf)
    }

    /// Write an IR value and return to Run-Test/Idle.
    ///
    /// Convenience method: navigates to Shift-IR, shifts the instruction,
    /// exits, and returns to Idle.
    #[maybe_async]
    pub async fn write_ir(
        &mut self,
        ctx: &MpsseContext,
        dev: &mut FtdiDevice,
        ir_data: &[u8],
        ir_len: usize,
    ) -> Result<Vec<u8>> {
        self.goto_shift_ir(dev).await?;
        let tdo = self.shift_bits(ctx, dev, ir_data, ir_len, true).await?;
        self.goto_idle(dev).await?;
        Ok(tdo)
    }

    /// Shift data through DR and return to Run-Test/Idle.
    ///
    /// Convenience method: navigates to Shift-DR, shifts the data,
    /// exits, and returns to Idle.
    #[maybe_async]
    pub async fn shift_dr(
        &mut self,
        ctx: &MpsseContext,
        dev: &mut FtdiDevice,
        dr_data: &[u8],
        dr_len: usize,
    ) -> Result<Vec<u8>> {
        self.goto_shift_dr(dev).await?;
        let tdo = self.shift_bits(ctx, dev, dr_data, dr_len, true).await?;
        self.goto_idle(dev).await?;
        Ok(tdo)
    }

    /// Get the TMS pin mask.
    pub fn tms_pin(&self) -> u8 {
        self.tms_pin
    }

    /// Get the direction mask.
    pub fn dir_mask(&self) -> u8 {
        self.dir_mask
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tap_state_initial_is_unknown() {
        let bus = JtagBus {
            tms_pin: 0x08,
            dir_mask: 0x0B,
            state: TapState::Unknown,
        };
        assert_eq!(bus.state(), TapState::Unknown);
    }

    #[test]
    fn tms_reset_sequence() {
        // Verify the TMS command for reset: 5 bits of TMS=1
        let cmd_byte = mpsse::WRITE_TMS | mpsse::WRITE_NEG | mpsse::BITMODE | mpsse::LSB;
        assert_eq!(cmd_byte, 0x4B);

        // Data: 0x1F = 0b00011111, 5 bits all TMS=1
        let data: u8 = 0x1F;
        for bit in 0..5 {
            assert_eq!((data >> bit) & 1, 1, "bit {} should be TMS=1", bit);
        }
    }

    #[test]
    fn tms_goto_shift_dr_sequence() {
        // TMS sequence from Idle to Shift-DR: 1, 0, 0
        // LSB first encoding: bit0=1, bit1=0, bit2=0 = 0x01
        let data: u8 = 0x01;
        assert_eq!(data & 1, 1); // Select-DR-Scan
        assert_eq!((data >> 1) & 1, 0); // Capture-DR
        assert_eq!((data >> 2) & 1, 0); // Shift-DR
    }

    #[test]
    fn tms_goto_shift_ir_sequence() {
        // TMS sequence from Idle to Shift-IR: 1, 1, 0, 0
        // LSB first encoding: bit0=1, bit1=1, bit2=0, bit3=0 = 0x03
        let data: u8 = 0x03;
        assert_eq!(data & 1, 1); // Select-DR-Scan
        assert_eq!((data >> 1) & 1, 1); // Select-IR-Scan
        assert_eq!((data >> 2) & 1, 0); // Capture-IR
        assert_eq!((data >> 3) & 1, 0); // Shift-IR
    }

    #[test]
    fn tms_goto_idle_sequence() {
        // TMS from Exit1 to Idle: 1, 0
        // LSB first: bit0=1, bit1=0 = 0x01
        let data: u8 = 0x01;
        assert_eq!(data & 1, 1); // Update-DR/IR
        assert_eq!((data >> 1) & 1, 0); // Run-Test/Idle
    }

    #[test]
    fn jtag_pin_mapping() {
        let bus = JtagBus {
            tms_pin: 0x08,
            dir_mask: 0x0B,
            state: TapState::Unknown,
        };
        // TCK=bit0, TDI=bit1, TDO=bit2 (input), TMS=bit3
        assert_eq!(bus.tms_pin(), 0x08);
        assert_eq!(bus.dir_mask(), 0x0B);
        // Direction: bits 0,1,3 are outputs (0b1011 = 0x0B)
        assert_eq!(bus.dir_mask() & 0x01, 0x01); // TCK = output
        assert_eq!(bus.dir_mask() & 0x02, 0x02); // TDI = output
        assert_eq!(bus.dir_mask() & 0x04, 0x00); // TDO = input
        assert_eq!(bus.dir_mask() & 0x08, 0x08); // TMS = output
    }

    #[test]
    fn shift_command_byte_values() {
        // Read+Write bytes, LSB first, clock out on negative edge
        let rw_cmd = mpsse::DO_WRITE | mpsse::DO_READ | mpsse::WRITE_NEG | mpsse::LSB;
        assert_eq!(rw_cmd, 0x39); // 0x10 | 0x20 | 0x01 | 0x08

        // Bit-mode variant
        let rw_bit_cmd = rw_cmd | mpsse::BITMODE;
        assert_eq!(rw_bit_cmd, 0x3B);
    }

    #[test]
    fn tms_with_read_command() {
        // WRITE_TMS + DO_READ + WRITE_NEG + BITMODE + LSB
        let cmd =
            mpsse::WRITE_TMS | mpsse::DO_READ | mpsse::WRITE_NEG | mpsse::BITMODE | mpsse::LSB;
        assert_eq!(cmd, 0x6B); // 0x40 | 0x20 | 0x01 | 0x02 | 0x08
    }
}
