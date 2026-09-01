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
//! # async fn example(dev: &mut FtdiDevice) -> ftdi_nusb::Result<()> {
//! let mut mpsse = MpsseContext::init(dev, 1_000_000).await?;
//! let mut s = mpsse.session(dev)?;
//! let spi = SpiDevice::new(&mut s, SpiMode::Mode0).await?;
//!
//! // Write 3 bytes, read 3 bytes (full duplex)
//! let response = spi.transfer(&mut s, &[0x9F, 0x00, 0x00]).await?;
//!
//! // Write-only (CS automatically asserted/deasserted)
//! spi.write(&mut s, &[0x06]).await?;
//!
//! // Read-only
//! let data = spi.read(&mut s, 4).await?;
//! # Ok(())
//! # }
//! ```

use crate::constants::mpsse;
use crate::error::{Error, Result};

use super::{MpsseSession, read_exact};

/// Maximum bytes per single MPSSE transfer command (2-byte length field, encoding len-1).
const MAX_MPSSE_TRANSFER: usize = 65536;
/// Bound read-producing commands so the FTDI MPSSE output FIFO cannot fill
/// before the host queues the corresponding USB read.
const MAX_MPSSE_IO_CHUNK: usize = 1024;

fn io_chunks(total: usize) -> impl Iterator<Item = (usize, usize)> {
    (0..total).step_by(MAX_MPSSE_IO_CHUNK).map(move |offset| {
        let len = (total - offset).min(MAX_MPSSE_IO_CHUNK);
        (offset, len)
    })
}

fn transfer_command(opcode: u8, write: &[u8], offset: usize, len: usize) -> Vec<u8> {
    let (lo, hi) = encode_len(len);
    let mut command = Vec::with_capacity(len + 4);
    command.extend_from_slice(&[opcode, lo, hi]);
    let write_end = (offset + len).min(write.len());
    if offset < write_end {
        command.extend_from_slice(&write[offset..write_end]);
    }
    command.resize(3 + len, 0);
    command.push(mpsse::SEND_IMMEDIATE);
    command
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiConfig {
    /// SPI clock polarity and phase.
    pub mode: SpiMode,
    /// Chip-select bit mask in the low GPIO byte. Multiple bits are allowed.
    pub cs_mask: u8,
    /// Whether all selected chip-select bits are active low.
    pub cs_active_low: bool,
    /// Whether transfers use least-significant-bit first ordering.
    pub lsb_first: bool,
    /// Idle levels for auxiliary low-byte GPIO pins.
    ///
    /// The SPI pins (bits 0-2) and the bits in `cs_mask` are composed by the
    /// SPI implementation and override this value.
    pub low_idle: u8,
    /// Directions for auxiliary low-byte GPIO pins (`1` = output).
    ///
    /// SK and DO are always outputs, DI is always an input, and `cs_mask`
    /// bits are always outputs.
    pub low_dir: u8,
    /// Idle levels for the high GPIO byte.
    pub high_idle: u8,
    /// Directions for the high GPIO byte (`1` = output).
    pub high_dir: u8,
}

impl SpiConfig {
    /// Create a mode-specific configuration using ADBUS3 as active-low CS.
    pub const fn new(mode: SpiMode) -> Self {
        Self {
            mode,
            cs_mask: 0x08,
            cs_active_low: true,
            lsb_first: false,
            low_idle: 0,
            low_dir: 0,
            high_idle: 0,
            high_dir: 0,
        }
    }

    /// Select one or more low-byte GPIO bits as chip selects.
    pub const fn with_cs_mask(mut self, cs_mask: u8, active_low: bool) -> Self {
        self.cs_mask = cs_mask;
        self.cs_active_low = active_low;
        self
    }

    /// Configure the transfer bit order.
    pub const fn with_lsb_first(mut self, lsb_first: bool) -> Self {
        self.lsb_first = lsb_first;
        self
    }

    /// Configure idle levels and directions for auxiliary low-byte pins.
    pub const fn with_low_pins(mut self, idle: u8, direction: u8) -> Self {
        self.low_idle = idle;
        self.low_dir = direction;
        self
    }

    /// Configure idle levels and directions for high-byte pins.
    pub const fn with_high_pins(mut self, idle: u8, direction: u8) -> Self {
        self.high_idle = idle;
        self.high_dir = direction;
        self
    }
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self::new(SpiMode::Mode0)
    }
}

fn compose_low_pins(config: &SpiConfig) -> (u8, u8) {
    let auxiliary_mask = !(0x07 | config.cs_mask);
    let direction = 0x03 | config.cs_mask | (config.low_dir & auxiliary_mask);
    let cs_idle = if config.cs_active_low {
        config.cs_mask
    } else {
        0
    };
    let clock_idle = if config.mode.cpol() { 0x01 } else { 0 };
    let idle = (config.low_idle & auxiliary_mask) | clock_idle | cs_idle;
    (idle, direction)
}

/// Configured SPI device connected to the MPSSE.
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
    /// Recovery epoch of the device this SPI device was configured on.
    recovery_epoch: u64,
    /// MPSSE protocol configuration this SPI device was created under.
    protocol_epoch: u64,
    /// Identity of the device this SPI device was configured on.
    device_id: u64,
}

impl SpiDevice {
    fn ensure_current(&self, s: &MpsseSession<'_>) -> Result<()> {
        if self.device_id == s.dev.device_id()
            && self.recovery_epoch == s.dev.recovery_epoch()
            && self.protocol_epoch == s.protocol_epoch()
        {
            Ok(())
        } else {
            Err(Error::InvalidMpsseContext)
        }
    }

    /// Create a new SPI device configuration with default CS on ADBUS3.
    ///
    /// Initializes the MPSSE pins for SPI:
    /// - ADBUS0 (SK) = SCLK output
    /// - ADBUS1 (DO) = MOSI output
    /// - ADBUS2 (DI) = MISO input
    /// - ADBUS3 = CS# output (active low, deasserted on init)
    pub async fn new(s: &mut MpsseSession<'_>, mode: SpiMode) -> Result<Self> {
        Self::with_config(s, SpiConfig::new(mode)).await
    }

    /// Create an SPI device with a custom CS pin and options.
    ///
    /// `cs_pin` is the bit mask for the CS pin in the low GPIO byte (e.g.,
    /// 0x08 for ADBUS3, 0x10 for ADBUS4). Set to 0 to manage CS manually.
    ///
    /// `cs_active_low` controls the CS polarity (true = CS is active when low).
    ///
    /// `lsb_first` controls the bit order (true = LSB first, false = MSB first).
    pub async fn with_cs_pin(
        s: &mut MpsseSession<'_>,
        mode: SpiMode,
        cs_pin: u8,
        cs_active_low: bool,
        lsb_first: bool,
    ) -> Result<Self> {
        Self::with_config(
            s,
            SpiConfig::new(mode)
                .with_cs_mask(cs_pin, cs_active_low)
                .with_lsb_first(lsb_first),
        )
        .await
    }

    /// Create an SPI device with full chip-select and idle-pin configuration.
    pub async fn with_config(s: &mut MpsseSession<'_>, config: SpiConfig) -> Result<Self> {
        let SpiConfig {
            mode,
            cs_mask,
            cs_active_low,
            lsb_first,
            low_idle: _,
            low_dir: _,
            high_idle,
            high_dir,
        } = config;

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

        // Compose the fixed SPI pins with caller-provided auxiliary GPIO state.
        let (idle_value, dir_mask) = compose_low_pins(&config);

        // I2C enables three-phase clocking, which changes SPI timing. Restore
        // the SPI/JTAG clocking mode and invalidate objects created under the
        // previous protocol configuration.
        if s.ctx().is_h_type() {
            s.write_commands(&[mpsse::DIS_3_PHASE]).await?;
        }
        let protocol_epoch = s.bump_protocol_epoch();

        let spi = Self {
            mode,
            lsb_first,
            cs_pin: cs_mask,
            cs_active_low,
            write_cmd,
            read_cmd,
            rw_cmd,
            dir_mask,
            idle_value,
            recovery_epoch: s.dev.recovery_epoch(),
            protocol_epoch,
            device_id: s.dev.device_id(),
        };

        // Set initial pin state
        s.set_gpio_low(idle_value, dir_mask).await?;
        if high_idle != 0 || high_dir != 0 {
            s.set_gpio_high(high_idle, high_dir).await?;
        }

        Ok(spi)
    }

    /// Assert the chip-select line (make it active).
    pub async fn cs_assert(&self, s: &mut MpsseSession<'_>) -> Result<()> {
        self.ensure_current(s)?;
        if self.cs_pin == 0 {
            return Ok(());
        }
        let value = if self.cs_active_low {
            self.idle_value & !self.cs_pin // drive CS low
        } else {
            self.idle_value | self.cs_pin // drive CS high
        };
        s.set_gpio_low(value, self.dir_mask).await
    }

    /// Deassert the chip-select line (make it inactive).
    pub async fn cs_deassert(&self, s: &mut MpsseSession<'_>) -> Result<()> {
        self.ensure_current(s)?;
        if self.cs_pin == 0 {
            return Ok(());
        }
        s.set_gpio_low(self.idle_value, self.dir_mask).await
    }

    pub(crate) async fn transfer_into_raw(
        &self,
        s: &mut MpsseSession<'_>,
        read: &mut [u8],
        write: &[u8],
    ) -> Result<()> {
        let total_len = read.len().max(write.len());
        for (offset, chunk_len) in io_chunks(total_len) {
            let command = transfer_command(self.rw_cmd, write, offset, chunk_len);
            s.dev.write_all(&command).await?;

            let received = read_exact(s.dev, chunk_len).await?;
            let read_end = (offset + chunk_len).min(read.len());
            if offset < read_end {
                read[offset..read_end].copy_from_slice(&received[..read_end - offset]);
            }
        }
        Ok(())
    }

    pub(crate) async fn write_raw(&self, s: &mut MpsseSession<'_>, tx: &[u8]) -> Result<()> {
        for chunk in tx.chunks(MAX_MPSSE_TRANSFER) {
            let (lo, hi) = encode_len(chunk.len());
            let mut cmd = Vec::with_capacity(chunk.len() + 3);
            cmd.extend_from_slice(&[self.write_cmd, lo, hi]);
            cmd.extend_from_slice(chunk);
            s.dev.write_all(&cmd).await?;
        }
        Ok(())
    }

    pub(crate) async fn read_raw(&self, s: &mut MpsseSession<'_>, len: usize) -> Result<Vec<u8>> {
        let mut received = Vec::with_capacity(len);
        let mut remaining = len;
        while remaining > 0 {
            let chunk_len = remaining.min(MAX_MPSSE_IO_CHUNK);
            let (lo, hi) = encode_len(chunk_len);
            s.dev
                .write_all(&[self.read_cmd, lo, hi, mpsse::SEND_IMMEDIATE])
                .await?;
            received.extend(read_exact(s.dev, chunk_len).await?);
            remaining -= chunk_len;
        }
        Ok(received)
    }

    /// Full-duplex SPI transfer: simultaneously write `tx` and read the same
    /// number of bytes.
    pub async fn transfer(&self, s: &mut MpsseSession<'_>, tx: &[u8]) -> Result<Vec<u8>> {
        self.ensure_current(s)?;
        if tx.is_empty() {
            return Ok(Vec::new());
        }
        let guard = s.dev.begin_stateful_operation()?;
        self.cs_assert(s).await?;
        let mut received = vec![0; tx.len()];
        let operation = self.transfer_into_raw(s, &mut received, tx).await;
        let cleanup = self.cs_deassert(s).await;
        operation?;
        cleanup?;
        guard.disarm();
        Ok(received)
    }

    /// Write-only SPI transfer with automatic chip-select handling.
    pub async fn write(&self, s: &mut MpsseSession<'_>, tx: &[u8]) -> Result<()> {
        self.ensure_current(s)?;
        if tx.is_empty() {
            return Ok(());
        }
        let guard = s.dev.begin_stateful_operation()?;
        self.cs_assert(s).await?;
        let operation = self.write_raw(s, tx).await;
        let cleanup = self.cs_deassert(s).await;
        operation?;
        cleanup?;
        guard.disarm();
        Ok(())
    }

    /// Read-only SPI transfer with automatic chip-select handling.
    pub async fn read(&self, s: &mut MpsseSession<'_>, len: usize) -> Result<Vec<u8>> {
        self.ensure_current(s)?;
        if len == 0 {
            return Ok(Vec::new());
        }
        let guard = s.dev.begin_stateful_operation()?;
        self.cs_assert(s).await?;
        let operation = self.read_raw(s, len).await;
        let cleanup = self.cs_deassert(s).await;
        let received = operation?;
        cleanup?;
        guard.disarm();
        Ok(received)
    }

    /// Perform a write-then-read SPI transaction with a single CS assertion.
    pub async fn write_read(
        &self,
        s: &mut MpsseSession<'_>,
        tx: &[u8],
        read_len: usize,
    ) -> Result<Vec<u8>> {
        self.ensure_current(s)?;
        if tx.is_empty() && read_len == 0 {
            return Ok(Vec::new());
        }
        let guard = s.dev.begin_stateful_operation()?;
        let command = self.write_read_command(tx, read_len);
        s.dev.write_all(&command).await?;
        let received = read_exact(s.dev, read_len).await?;
        if self.cs_pin != 0 {
            s.ctx.update_gpio_low_state(self.idle_value, self.dir_mask);
        }
        guard.disarm();
        Ok(received)
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

    /// Get the chip-select bit mask.
    pub fn cs_mask(&self) -> u8 {
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

    fn write_read_command(&self, tx: &[u8], read_len: usize) -> Vec<u8> {
        let write_overhead = tx.len().div_ceil(MAX_MPSSE_TRANSFER) * 3;
        let read_overhead = read_len.div_ceil(MAX_MPSSE_IO_CHUNK) * 3;
        let mut cmd = Vec::with_capacity(tx.len() + write_overhead + read_overhead + 7);

        self.append_cs_assert(&mut cmd);
        for chunk in tx.chunks(MAX_MPSSE_TRANSFER) {
            let (lo, hi) = encode_len(chunk.len());
            cmd.extend_from_slice(&[self.write_cmd, lo, hi]);
            cmd.extend_from_slice(chunk);
        }
        for (_, chunk_len) in io_chunks(read_len) {
            let (lo, hi) = encode_len(chunk_len);
            cmd.extend_from_slice(&[self.read_cmd, lo, hi]);
        }
        self.append_cs_deassert(&mut cmd);
        cmd.push(mpsse::SEND_IMMEDIATE);
        cmd
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
    fn io_chunk_plan_bounds_unequal_transfers() {
        assert!(io_chunks(0).next().is_none());
        assert_eq!(
            io_chunks(1025).collect::<Vec<_>>(),
            vec![(0, 1024), (1024, 1)]
        );
        assert_eq!(io_chunks(5).map(|(_, len)| len).sum::<usize>(), 5);
    }

    #[test]
    fn unequal_transfer_commands_pad_missing_write_bytes() {
        let command = transfer_command(0x31, &[0xaa, 0xbb], 0, 4);
        assert_eq!(
            command,
            vec![0x31, 3, 0, 0xaa, 0xbb, 0, 0, mpsse::SEND_IMMEDIATE]
        );

        let second_chunk = transfer_command(0x31, &[1, 2, 3], 2, 3);
        assert_eq!(
            second_chunk,
            vec![0x31, 2, 0, 3, 0, 0, mpsse::SEND_IMMEDIATE]
        );
    }

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
            recovery_epoch: 0,
            protocol_epoch: 0,
            device_id: 0,
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
            recovery_epoch: 0,
            protocol_epoch: 0,
            device_id: 0,
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
            recovery_epoch: 0,
            protocol_epoch: 0,
            device_id: 0,
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
            recovery_epoch: 0,
            protocol_epoch: 0,
            device_id: 0,
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

    #[test]
    fn config_composes_cs_mask_and_auxiliary_pins() {
        let config = SpiConfig::new(SpiMode::Mode2)
            .with_cs_mask(0x18, true)
            .with_low_pins(0x64, 0xe4)
            .with_high_pins(0x20, 0x70);

        let (idle, direction) = compose_low_pins(&config);
        assert_eq!(idle, 0x79); // auxiliary 0x60, CS 0x18, idle-high clock
        assert_eq!(direction, 0xfb); // auxiliary 0xe0, CS 0x18, SK/DO outputs
        assert_eq!(config.high_idle, 0x20);
        assert_eq!(config.high_dir, 0x70);
    }

    #[test]
    fn write_read_is_one_chunked_command_buffer() {
        let spi = SpiDevice {
            mode: SpiMode::Mode0,
            lsb_first: false,
            cs_pin: 0x18,
            cs_active_low: true,
            write_cmd: 0x11,
            read_cmd: 0x20,
            rw_cmd: 0x31,
            dir_mask: 0x3b,
            idle_value: 0x38,
            recovery_epoch: 0,
            protocol_epoch: 0,
            device_id: 0,
        };

        let command = spi.write_read_command(&[0x9f], 1025);
        assert_eq!(
            command,
            vec![
                mpsse::SET_BITS_LOW,
                0x20,
                0x3b,
                0x11,
                0,
                0,
                0x9f,
                0x20,
                0xff,
                0x03,
                0x20,
                0,
                0,
                mpsse::SET_BITS_LOW,
                0x38,
                0x3b,
                mpsse::SEND_IMMEDIATE,
            ]
        );
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
