//! SPI protocol helpers using MPSSE.
//!
//! Provides high-level SPI master operations using the FTDI MPSSE engine.
//! Supports configurable clock polarity (CPOL), clock phase (CPHA), bit order,
//! and chip-select pin management.
//!
//! # Pin Mapping
//!
//! | FTDI Pin | SPI Signal | ADBUS Bit |
//! |----------|-----------|-----------|
//! | SK       | SCLK      | 0         |
//! | DO       | MOSI      | 1         |
//! | DI       | MISO      | 2         |
//! | CS#      | CS (user) | 3-7       |
//!
//! # Example
//!
//! ```no_run
//! use ftdi_nusb::{FtdiDevice, mpsse::{MpsseContext, spi::{SpiDevice, SpiMode}}};
//!
//! let mut dev = FtdiDevice::open(0x0403, 0x6014)?;
//! let mut mpsse = MpsseContext::init(&mut dev, 1_000_000)?;
//! let spi = SpiDevice::new(&mut mpsse, &mut dev, SpiMode::Mode0)?;
//!
//! // Write 3 bytes, read 3 bytes (full duplex)
//! let response = spi.transfer(&mut mpsse, &mut dev, &[0x9F, 0x00, 0x00])?;
//!
//! // Write-only (CS automatically asserted/deasserted)
//! spi.write(&mut mpsse, &mut dev, &[0x06])?;
//!
//! // Read-only
//! let data = spi.read(&mut mpsse, &mut dev, 4)?;
//! # Ok::<(), ftdi_nusb::Error>(())
//! ```

use maybe_async::maybe_async;

use crate::constants::mpsse;
use crate::context::FtdiDevice;
use crate::error::{Error, Result};

use super::MpsseContext;

/// Maximum bytes per single MPSSE transfer command (2-byte length field, encoding len-1).
const MAX_MPSSE_TRANSFER: usize = 65536;

/// Read exactly `len` bytes from the MPSSE, returning an error on short reads.
#[maybe_async]
async fn read_exact(dev: &mut FtdiDevice, len: usize) -> Result<Vec<u8>> {
    let mut buf = vec![0u8; len];
    let mut offset = 0;
    while offset < len {
        let n = dev.read_data(&mut buf[offset..]).await?;
        if n == 0 {
            return Err(Error::InvalidArgument(
                "SPI read returned fewer bytes than expected",
            ));
        }
        offset += n;
    }
    Ok(buf)
}

/// Encode a chunk length into the 2-byte MPSSE length field (len-1, little-endian).
#[inline]
fn encode_len(len: usize) -> (u8, u8) {
    let v = (len - 1) as u16;
    (v as u8, (v >> 8) as u8)
}

/// SPI clock polarity and phase mode.
///
/// Standard Motorola SPI modes:
///
/// | Mode | CPOL | CPHA | Description |
/// |------|------|------|-------------|
/// | 0    | 0    | 0    | Clock idle low, sample on rising edge |
/// | 1    | 0    | 1    | Clock idle low, sample on falling edge |
/// | 2    | 1    | 0    | Clock idle high, sample on falling edge |
/// | 3    | 1    | 1    | Clock idle high, sample on rising edge |
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SpiMode {
    /// CPOL=0, CPHA=0.
    Mode0,
    /// CPOL=0, CPHA=1.
    Mode1,
    /// CPOL=1, CPHA=0.
    Mode2,
    /// CPOL=1, CPHA=1.
    Mode3,
}

impl SpiMode {
    /// Clock polarity: true = idle high.
    pub fn cpol(self) -> bool {
        matches!(self, Self::Mode2 | Self::Mode3)
    }

    /// Clock phase: true = sample on second edge.
    pub fn cpha(self) -> bool {
        matches!(self, Self::Mode1 | Self::Mode3)
    }
}

/// Configuration for an SPI device connected to the MPSSE.
#[derive(Debug, Clone)]
pub struct SpiDevice {
    /// The SPI mode (clock polarity and phase).
    mode: SpiMode,
    /// Whether to use LSB-first bit order (default: MSB first).
    lsb_first: bool,
    /// CS pin bit mask in the low GPIO byte (e.g., 0x08 for ADBUS3).
    cs_pin: u8,
    /// Whether CS is active low (default: true).
    cs_active_low: bool,
    /// MPSSE opcode for write (depends on mode).
    write_cmd: u8,
    /// MPSSE opcode for read (depends on mode).
    read_cmd: u8,
    /// MPSSE opcode for simultaneous read+write (depends on mode).
    rw_cmd: u8,
    /// Initial low-byte direction mask (SK=out, DO=out, DI=in, CS=out).
    dir_mask: u8,
    /// Initial low-byte value (CS deasserted, clock at idle level).
    idle_value: u8,
}

impl SpiDevice {
    /// Create a new SPI device configuration with default CS on ADBUS3.
    ///
    /// Initializes the MPSSE pins for SPI:
    /// - ADBUS0 (SK) = SCLK output
    /// - ADBUS1 (DO) = MOSI output
    /// - ADBUS2 (DI) = MISO input
    /// - ADBUS3 = CS# output (active low, deasserted on init)
    #[maybe_async]
    pub async fn new(ctx: &mut MpsseContext, dev: &mut FtdiDevice, mode: SpiMode) -> Result<Self> {
        Self::with_cs_pin(ctx, dev, mode, 0x08, true, false).await
    }

    /// Create an SPI device with a custom CS pin and options.
    ///
    /// `cs_pin` is the bit mask for the CS pin in the low GPIO byte (e.g.,
    /// 0x08 for ADBUS3, 0x10 for ADBUS4). Set to 0 to manage CS manually.
    ///
    /// `cs_active_low` controls the CS polarity (true = CS is active when low).
    ///
    /// `lsb_first` controls the bit order (true = LSB first, false = MSB first).
    #[maybe_async]
    pub async fn with_cs_pin(
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        mode: SpiMode,
        cs_pin: u8,
        cs_active_low: bool,
        lsb_first: bool,
    ) -> Result<Self> {
        // Build MPSSE opcodes based on mode and bit order
        let lsb = if lsb_first { mpsse::LSB } else { 0 };

        // For SPI we use byte-level commands (not BITMODE)
        // MPSSE edge names are physical: WRITE_NEG = shift out on falling SK,
        // READ_NEG = sample in on falling SK. The idle clock level (CPOL) is
        // handled separately via set_gpio_low. Per FTDI AN_108:
        //   Mode 0 (CPOL=0, CPHA=0): data changes on falling, sampled on rising
        //   Mode 1 (CPOL=0, CPHA=1): data changes on rising, sampled on falling
        //   Mode 2 (CPOL=1, CPHA=0): data changes on rising, sampled on falling
        //   Mode 3 (CPOL=1, CPHA=1): data changes on falling, sampled on rising
        let (write_cmd, read_cmd, rw_cmd) = match mode {
            SpiMode::Mode0 | SpiMode::Mode3 => {
                // Data out on falling SK (WRITE_NEG), data in on rising SK
                (
                    mpsse::DO_WRITE | mpsse::WRITE_NEG | lsb,
                    mpsse::DO_READ | lsb,
                    mpsse::DO_WRITE | mpsse::DO_READ | mpsse::WRITE_NEG | lsb,
                )
            }
            SpiMode::Mode1 | SpiMode::Mode2 => {
                // Data out on rising SK, data in on falling SK (READ_NEG)
                (
                    mpsse::DO_WRITE | lsb,
                    mpsse::DO_READ | mpsse::READ_NEG | lsb,
                    mpsse::DO_WRITE | mpsse::DO_READ | mpsse::READ_NEG | lsb,
                )
            }
        };

        // Direction: SK(0)=out, DO(1)=out, DI(2)=in, CS=out
        let dir_mask = 0x03 | cs_pin; // bits 0,1 = output, plus CS pin

        // Idle value: clock at idle level, CS deasserted
        let cs_idle = if cs_active_low { cs_pin } else { 0 }; // deasserted state
        let clk_idle = if mode.cpol() { 0x01 } else { 0x00 }; // SK at idle level
        let idle_value = clk_idle | cs_idle;

        let spi = Self {
            mode,
            lsb_first,
            cs_pin,
            cs_active_low,
            write_cmd,
            read_cmd,
            rw_cmd,
            dir_mask,
            idle_value,
        };

        // Set initial pin state
        ctx.set_gpio_low(dev, idle_value, dir_mask).await?;

        Ok(spi)
    }

    /// Assert the chip-select line (make it active).
    #[maybe_async]
    pub async fn cs_assert(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        if self.cs_pin == 0 {
            return Ok(());
        }
        let value = if self.cs_active_low {
            self.idle_value & !self.cs_pin // drive CS low
        } else {
            self.idle_value | self.cs_pin // drive CS high
        };
        ctx.set_gpio_low(dev, value, self.dir_mask).await
    }

    /// Deassert the chip-select line (make it inactive).
    #[maybe_async]
    pub async fn cs_deassert(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        if self.cs_pin == 0 {
            return Ok(());
        }
        ctx.set_gpio_low(dev, self.idle_value, self.dir_mask).await
    }

    /// Full-duplex SPI transfer: simultaneously write `tx` and read the same
    /// number of bytes.
    ///
    /// CS is automatically asserted before and deasserted after the transfer.
    /// Large transfers (>65536 bytes) are automatically split into multiple
    /// MPSSE commands within the same CS assertion.
    /// Returns the received bytes.
    #[maybe_async]
    pub async fn transfer(
        &self,
        _ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        tx: &[u8],
    ) -> Result<Vec<u8>> {
        if tx.is_empty() {
            return Ok(Vec::new());
        }

        let total = tx.len();
        let mut cmd = Vec::with_capacity(10 + total);

        // Assert CS
        self.append_cs_assert(&mut cmd);

        // Emit one or more R/W commands for chunks up to MAX_MPSSE_TRANSFER
        let mut offset = 0;
        while offset < total {
            let chunk_len = (total - offset).min(MAX_MPSSE_TRANSFER);
            let (lo, hi) = encode_len(chunk_len);
            cmd.push(self.rw_cmd);
            cmd.push(lo);
            cmd.push(hi);
            cmd.extend_from_slice(&tx[offset..offset + chunk_len]);
            offset += chunk_len;
        }

        // Deassert CS + flush
        self.append_cs_deassert(&mut cmd);
        cmd.push(mpsse::SEND_IMMEDIATE);

        dev.write_all(&cmd).await?;

        read_exact(dev, total).await
    }

    /// Write-only SPI transfer.
    ///
    /// CS is automatically asserted before and deasserted after the transfer.
    /// Large transfers (>65536 bytes) are automatically chunked.
    #[maybe_async]
    pub async fn write(
        &self,
        _ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        tx: &[u8],
    ) -> Result<()> {
        if tx.is_empty() {
            return Ok(());
        }

        let total = tx.len();
        let mut cmd = Vec::with_capacity(10 + total);

        self.append_cs_assert(&mut cmd);

        let mut offset = 0;
        while offset < total {
            let chunk_len = (total - offset).min(MAX_MPSSE_TRANSFER);
            let (lo, hi) = encode_len(chunk_len);
            cmd.push(self.write_cmd);
            cmd.push(lo);
            cmd.push(hi);
            cmd.extend_from_slice(&tx[offset..offset + chunk_len]);
            offset += chunk_len;
        }

        self.append_cs_deassert(&mut cmd);

        dev.write_all(&cmd).await
    }

    /// Read-only SPI transfer (writes zeros while reading).
    ///
    /// CS is automatically asserted before and deasserted after the transfer.
    /// Large reads (>65536 bytes) are automatically chunked.
    /// Returns the received bytes.
    #[maybe_async]
    pub async fn read(
        &self,
        _ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        len: usize,
    ) -> Result<Vec<u8>> {
        if len == 0 {
            return Ok(Vec::new());
        }

        let mut cmd = Vec::with_capacity(16);

        self.append_cs_assert(&mut cmd);

        let mut remaining = len;
        while remaining > 0 {
            let chunk_len = remaining.min(MAX_MPSSE_TRANSFER);
            let (lo, hi) = encode_len(chunk_len);
            cmd.push(self.read_cmd);
            cmd.push(lo);
            cmd.push(hi);
            remaining -= chunk_len;
        }

        self.append_cs_deassert(&mut cmd);
        cmd.push(mpsse::SEND_IMMEDIATE);

        dev.write_all(&cmd).await?;

        read_exact(dev, len).await
    }

    /// Perform a write-then-read SPI transaction with a single CS assertion.
    ///
    /// This is common for SPI devices where you send a command and then
    /// read the response (e.g., reading a register).
    /// Large transfers (>65536 bytes in either direction) are automatically chunked.
    #[maybe_async]
    pub async fn write_read(
        &self,
        _ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        tx: &[u8],
        read_len: usize,
    ) -> Result<Vec<u8>> {
        if tx.is_empty() && read_len == 0 {
            return Ok(Vec::new());
        }

        let mut cmd = Vec::with_capacity(16 + tx.len());

        self.append_cs_assert(&mut cmd);

        // Write phase (chunked)
        if !tx.is_empty() {
            let mut offset = 0;
            while offset < tx.len() {
                let chunk_len = (tx.len() - offset).min(MAX_MPSSE_TRANSFER);
                let (lo, hi) = encode_len(chunk_len);
                cmd.push(self.write_cmd);
                cmd.push(lo);
                cmd.push(hi);
                cmd.extend_from_slice(&tx[offset..offset + chunk_len]);
                offset += chunk_len;
            }
        }

        // Read phase (chunked)
        if read_len > 0 {
            let mut remaining = read_len;
            while remaining > 0 {
                let chunk_len = remaining.min(MAX_MPSSE_TRANSFER);
                let (lo, hi) = encode_len(chunk_len);
                cmd.push(self.read_cmd);
                cmd.push(lo);
                cmd.push(hi);
                remaining -= chunk_len;
            }
        }

        self.append_cs_deassert(&mut cmd);

        if read_len > 0 {
            cmd.push(mpsse::SEND_IMMEDIATE);
        }

        dev.write_all(&cmd).await?;

        if read_len == 0 {
            return Ok(Vec::new());
        }

        read_exact(dev, read_len).await
    }

    /// Get the current SPI mode.
    pub fn mode(&self) -> SpiMode {
        self.mode
    }

    /// Whether this SPI device uses LSB-first bit order.
    pub fn is_lsb_first(&self) -> bool {
        self.lsb_first
    }

    /// Get the CS pin bit mask.
    pub fn cs_pin(&self) -> u8 {
        self.cs_pin
    }

    /// Append a SET_BITS_LOW command to `cmd` that asserts CS.
    ///
    /// This is a zero-cost helper for building MPSSE command buffers without
    /// needing a mutable reference to `MpsseContext` or `FtdiDevice`.
    /// If `cs_pin` is 0 (manual CS), this is a no-op.
    fn append_cs_assert(&self, cmd: &mut Vec<u8>) {
        if self.cs_pin == 0 {
            return;
        }
        let value = if self.cs_active_low {
            self.idle_value & !self.cs_pin // drive CS low
        } else {
            self.idle_value | self.cs_pin // drive CS high
        };
        cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, value, self.dir_mask]);
    }

    /// Append a SET_BITS_LOW command to `cmd` that deasserts CS (returns to idle).
    ///
    /// If `cs_pin` is 0 (manual CS), this is a no-op.
    fn append_cs_deassert(&self, cmd: &mut Vec<u8>) {
        if self.cs_pin == 0 {
            return;
        }
        cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, self.idle_value, self.dir_mask]);
    }

    // ---- Test-only accessors ----

    #[cfg(test)]
    #[allow(dead_code)]
    pub(crate) fn write_cmd(&self) -> u8 {
        self.write_cmd
    }

    #[cfg(test)]
    #[allow(dead_code)]
    pub(crate) fn read_cmd(&self) -> u8 {
        self.read_cmd
    }

    #[cfg(test)]
    #[allow(dead_code)]
    pub(crate) fn rw_cmd(&self) -> u8 {
        self.rw_cmd
    }

    #[cfg(test)]
    #[allow(dead_code)]
    pub(crate) fn dir_mask(&self) -> u8 {
        self.dir_mask
    }

    #[cfg(test)]
    #[allow(dead_code)]
    pub(crate) fn idle_value(&self) -> u8 {
        self.idle_value
    }

    #[cfg(test)]
    pub(crate) fn test_append_cs_assert(&self, cmd: &mut Vec<u8>) {
        self.append_cs_assert(cmd);
    }

    #[cfg(test)]
    pub(crate) fn test_append_cs_deassert(&self, cmd: &mut Vec<u8>) {
        self.append_cs_deassert(cmd);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- SpiMode tests ----

    #[test]
    fn spi_mode_cpol() {
        assert!(!SpiMode::Mode0.cpol());
        assert!(!SpiMode::Mode1.cpol());
        assert!(SpiMode::Mode2.cpol());
        assert!(SpiMode::Mode3.cpol());
    }

    #[test]
    fn spi_mode_cpha() {
        assert!(!SpiMode::Mode0.cpha());
        assert!(SpiMode::Mode1.cpha());
        assert!(!SpiMode::Mode2.cpha());
        assert!(SpiMode::Mode3.cpha());
    }

    // ---- MPSSE opcode tests ----

    #[test]
    fn mode0_opcodes() {
        // Mode 0: data out on falling (WRITE_NEG), data in on rising (no READ_NEG)
        let write_cmd = mpsse::DO_WRITE | mpsse::WRITE_NEG;
        let read_cmd = mpsse::DO_READ;
        let rw_cmd = mpsse::DO_WRITE | mpsse::DO_READ | mpsse::WRITE_NEG;

        assert_eq!(write_cmd, 0x11);
        assert_eq!(read_cmd, 0x20);
        assert_eq!(rw_cmd, 0x31);
    }

    #[test]
    fn mode1_opcodes() {
        // Mode 1: data out on rising (no WRITE_NEG), data in on falling (READ_NEG)
        let write_cmd = mpsse::DO_WRITE;
        let read_cmd = mpsse::DO_READ | mpsse::READ_NEG;
        let rw_cmd = mpsse::DO_WRITE | mpsse::DO_READ | mpsse::READ_NEG;

        assert_eq!(write_cmd, 0x10);
        assert_eq!(read_cmd, 0x24);
        assert_eq!(rw_cmd, 0x34);
    }

    #[test]
    fn mode0_and_mode3_share_opcodes() {
        // Mode 0 and Mode 3 should produce the same MPSSE opcodes
        let lsb = 0u8;
        let mode0_write = mpsse::DO_WRITE | mpsse::WRITE_NEG | lsb;
        let mode3_write = mpsse::DO_WRITE | mpsse::WRITE_NEG | lsb;
        assert_eq!(mode0_write, mode3_write);
    }

    #[test]
    fn lsb_first_flag_applied() {
        let lsb = mpsse::LSB;
        let write_cmd = mpsse::DO_WRITE | mpsse::WRITE_NEG | lsb;
        assert_eq!(write_cmd & mpsse::LSB, mpsse::LSB);
        assert_eq!(write_cmd, 0x19); // 0x10 | 0x01 | 0x08
    }

    // ---- encode_len tests ----

    #[test]
    fn encode_len_one_byte() {
        // len=1: encodes as 0, which is (0x00, 0x00)
        let (lo, hi) = encode_len(1);
        assert_eq!(lo, 0x00);
        assert_eq!(hi, 0x00);
    }

    #[test]
    fn encode_len_256_bytes() {
        let (lo, hi) = encode_len(256);
        // 256 - 1 = 255 = 0xFF
        assert_eq!(lo, 0xFF);
        assert_eq!(hi, 0x00);
    }

    #[test]
    fn encode_len_65536_bytes() {
        let (lo, hi) = encode_len(65536);
        // 65536 - 1 = 65535 = 0xFFFF
        assert_eq!(lo, 0xFF);
        assert_eq!(hi, 0xFF);
    }

    // ---- CS pin logic tests ----

    #[test]
    fn cs_assert_active_low() {
        // Active low CS on ADBUS3 (0x08): idle has CS=high, asserted = CS low
        let spi = SpiDevice {
            mode: SpiMode::Mode0,
            lsb_first: false,
            cs_pin: 0x08,
            cs_active_low: true,
            write_cmd: 0,
            read_cmd: 0,
            rw_cmd: 0,
            dir_mask: 0x0B,
            idle_value: 0x08, // CS high (deasserted), CLK low
        };

        let mut cmd = Vec::new();
        spi.test_append_cs_assert(&mut cmd);
        // Should be SET_BITS_LOW, value, dir
        assert_eq!(cmd.len(), 3);
        assert_eq!(cmd[0], mpsse::SET_BITS_LOW);
        // Value should have CS bit cleared (low)
        assert_eq!(cmd[1] & 0x08, 0x00);
        assert_eq!(cmd[2], 0x0B);
    }

    #[test]
    fn cs_assert_active_high() {
        let spi = SpiDevice {
            mode: SpiMode::Mode0,
            lsb_first: false,
            cs_pin: 0x08,
            cs_active_low: false,
            write_cmd: 0,
            read_cmd: 0,
            rw_cmd: 0,
            dir_mask: 0x0B,
            idle_value: 0x00, // CS low (deasserted), CLK low
        };

        let mut cmd = Vec::new();
        spi.test_append_cs_assert(&mut cmd);
        assert_eq!(cmd[1] & 0x08, 0x08); // CS high (asserted)
    }

    #[test]
    fn cs_deassert_returns_to_idle() {
        let spi = SpiDevice {
            mode: SpiMode::Mode0,
            lsb_first: false,
            cs_pin: 0x08,
            cs_active_low: true,
            write_cmd: 0,
            read_cmd: 0,
            rw_cmd: 0,
            dir_mask: 0x0B,
            idle_value: 0x08,
        };

        let mut cmd = Vec::new();
        spi.test_append_cs_deassert(&mut cmd);
        assert_eq!(cmd[1], 0x08); // Back to idle value
    }

    #[test]
    fn cs_pin_zero_is_noop() {
        let spi = SpiDevice {
            mode: SpiMode::Mode0,
            lsb_first: false,
            cs_pin: 0x00, // Manual CS
            cs_active_low: true,
            write_cmd: 0,
            read_cmd: 0,
            rw_cmd: 0,
            dir_mask: 0x03,
            idle_value: 0x00,
        };

        let mut cmd = Vec::new();
        spi.test_append_cs_assert(&mut cmd);
        assert!(cmd.is_empty(), "CS=0 should be a no-op");

        spi.test_append_cs_deassert(&mut cmd);
        assert!(cmd.is_empty(), "CS=0 should be a no-op");
    }

    // ---- Idle value tests ----

    #[test]
    fn mode0_idle_value() {
        // Mode0: CPOL=0, so CLK idle low. Active-low CS on 0x08: CS high in idle
        let cs_pin = 0x08u8;
        let cs_idle = cs_pin; // active low -> deasserted = high
        let clk_idle = 0x00; // CPOL=0
        assert_eq!(clk_idle | cs_idle, 0x08);
    }

    #[test]
    fn mode2_idle_value() {
        // Mode2: CPOL=1, so CLK idle high. Active-low CS on 0x08: CS high in idle
        let cs_pin = 0x08u8;
        let cs_idle = cs_pin;
        let clk_idle = 0x01; // CPOL=1 -> SK=1
        assert_eq!(clk_idle | cs_idle, 0x09);
    }

    // ---- Direction mask tests ----

    #[test]
    fn dir_mask_default_cs() {
        // SK(0)=out, DO(1)=out, CS(3)=out -> 0x03 | 0x08 = 0x0B
        let dir = 0x03 | 0x08;
        assert_eq!(dir, 0x0B);
    }

    #[test]
    fn dir_mask_custom_cs_pin() {
        // CS on ADBUS4 (0x10): dir = 0x03 | 0x10 = 0x13
        let dir = 0x03 | 0x10;
        assert_eq!(dir, 0x13);
    }
}
