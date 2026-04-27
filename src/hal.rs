//! `embedded-hal` 1.0 and `embedded-io` 0.7 trait implementations.
//!
//! This module provides trait implementations that let you use FTDI devices
//! with the embedded Rust ecosystem. Enable the `embedded-hal` feature in
//! your `Cargo.toml`:
//!
//! ```toml
//! [dependencies]
//! ftdi = { version = "0.1", features = ["embedded-hal"] }
//! ```
//!
//! # Provided implementations
//!
//! | Trait | Type | Notes |
//! |-------|------|-------|
//! | `embedded_hal::spi::SpiDevice` | [`FtdiSpiDevice`] | Wraps [`SpiDevice`] + context |
//! | `embedded_hal::i2c::I2c` | [`FtdiI2c`] | Wraps [`I2cBus`] + context |
//! | `embedded_io::Read` | [`FtdiDevice`] | Serial read |
//! | `embedded_io::Write` | [`FtdiDevice`] | Serial write |

use crate::context::FtdiDevice;
use crate::error::Error;
use crate::mpsse::i2c::I2cBus;
use crate::mpsse::spi::{SpiDevice, SpiMode};
use crate::mpsse::MpsseContext;

// ---- Error conversion ----

/// Embedded-hal error kind mapping for FTDI errors.
impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        // embedded-hal SPI ErrorKind has no finer categories that map to
        // FTDI/USB errors, so everything maps to Other.
        embedded_hal::spi::ErrorKind::Other
    }
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Error::I2cNack(msg) => {
                if msg.contains("address") {
                    embedded_hal::i2c::ErrorKind::NoAcknowledge(
                        embedded_hal::i2c::NoAcknowledgeSource::Address,
                    )
                } else {
                    embedded_hal::i2c::ErrorKind::NoAcknowledge(
                        embedded_hal::i2c::NoAcknowledgeSource::Data,
                    )
                }
            }
            _ => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Error::Transfer(nusb::transfer::TransferError::Cancelled) => {
                embedded_io::ErrorKind::TimedOut
            }
            Error::WriteZero => embedded_io::ErrorKind::WriteZero,
            _ => embedded_io::ErrorKind::Other,
        }
    }
}

// ---- embedded-io for FtdiDevice ----

impl embedded_io::ErrorType for FtdiDevice {
    type Error = Error;
}

impl embedded_io::Read for FtdiDevice {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_data(buf)
    }
}

impl embedded_io::Write for FtdiDevice {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_data(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx()
    }
}

// ---- embedded-hal SPI ----

/// Wrapper that implements `embedded_hal::spi::SpiDevice` for an FTDI MPSSE SPI bus.
///
/// This bundles the [`SpiDevice`],
/// [`MpsseContext`], and
/// [`FtdiDevice`] together so the combined type
/// satisfies the `SpiDevice` trait.
///
/// # Example
///
/// ```no_run
/// use ftdi_nusb::{FtdiDevice, hal::FtdiSpiDevice};
/// use ftdi_nusb::mpsse::spi::SpiMode;
///
/// let mut hal_spi = FtdiSpiDevice::open(0x0403, 0x6014, 1_000_000, SpiMode::Mode0)?;
///
/// // Now use with any embedded-hal SPI driver:
/// use embedded_hal::spi::SpiDevice;
/// let mut buf = [0u8; 4];
/// hal_spi.transfer(&mut buf, &[0x9F, 0, 0, 0])?;
/// # Ok::<(), ftdi_nusb::Error>(())
/// ```
pub struct FtdiSpiDevice {
    dev: FtdiDevice,
    ctx: MpsseContext,
    spi: SpiDevice,
}

impl FtdiSpiDevice {
    /// Open an FTDI device and configure it for SPI via MPSSE.
    ///
    /// This is a convenience constructor that opens the device, initializes
    /// MPSSE mode, and configures SPI in a single call.
    pub fn open(
        vendor: u16,
        product: u16,
        clock_hz: u32,
        mode: SpiMode,
    ) -> crate::error::Result<Self> {
        let mut dev = FtdiDevice::open(vendor, product)?;
        let mut ctx = MpsseContext::init(&mut dev, clock_hz)?;
        let spi = SpiDevice::new(&mut ctx, &mut dev, mode)?;
        Ok(Self { dev, ctx, spi })
    }

    /// Create from already-opened components.
    pub fn from_parts(dev: FtdiDevice, ctx: MpsseContext, spi: SpiDevice) -> Self {
        Self { dev, ctx, spi }
    }

    /// Borrow the underlying `FtdiDevice`.
    pub fn device(&self) -> &FtdiDevice {
        &self.dev
    }

    /// Mutably borrow the underlying `FtdiDevice`.
    pub fn device_mut(&mut self) -> &mut FtdiDevice {
        &mut self.dev
    }

    /// Borrow the underlying `MpsseContext`.
    pub fn context(&self) -> &MpsseContext {
        &self.ctx
    }

    /// Mutably borrow the underlying `MpsseContext`.
    pub fn context_mut(&mut self) -> &mut MpsseContext {
        &mut self.ctx
    }

    /// Decompose into the underlying parts.
    pub fn into_parts(self) -> (FtdiDevice, MpsseContext, SpiDevice) {
        (self.dev, self.ctx, self.spi)
    }
}

impl embedded_hal::spi::ErrorType for FtdiSpiDevice {
    type Error = Error;
}

impl embedded_hal::spi::SpiDevice for FtdiSpiDevice {
    fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;

        // Assert CS at the start of the transaction
        self.spi.cs_assert(&mut self.ctx, &mut self.dev)?;

        let result = (|| -> crate::error::Result<()> {
            for op in operations.iter_mut() {
                match op {
                    Operation::Read(buf) => {
                        let data = self.spi.read(&mut self.ctx, &mut self.dev, buf.len())?;
                        buf.copy_from_slice(&data);
                    }
                    Operation::Write(buf) => {
                        self.spi.write(&mut self.ctx, &mut self.dev, buf)?;
                    }
                    Operation::Transfer(read, write) => {
                        let data = self.spi.transfer(&mut self.ctx, &mut self.dev, write)?;
                        let copy_len = read.len().min(data.len());
                        read[..copy_len].copy_from_slice(&data[..copy_len]);
                    }
                    Operation::TransferInPlace(buf) => {
                        let data = self.spi.transfer(&mut self.ctx, &mut self.dev, buf)?;
                        buf.copy_from_slice(&data);
                    }
                    Operation::DelayNs(ns) => {
                        std::thread::sleep(std::time::Duration::from_nanos(*ns as u64));
                    }
                }
            }
            Ok(())
        })();

        // Always deassert CS, even on error
        let cs_result = self.spi.cs_deassert(&mut self.ctx, &mut self.dev);

        // Return the first error
        result?;
        cs_result
    }
}

// ---- embedded-hal I2C ----

/// Wrapper that implements `embedded_hal::i2c::I2c` for an FTDI MPSSE I2C bus.
///
/// # Example
///
/// ```no_run
/// use ftdi_nusb::hal::FtdiI2c;
///
/// let mut hal_i2c = FtdiI2c::open(0x0403, 0x6014, 100_000)?;
///
/// // Use with any embedded-hal I2C driver:
/// use embedded_hal::i2c::I2c;
/// let mut buf = [0u8; 2];
/// hal_i2c.write_read(0x48, &[0x00], &mut buf)?;
/// # Ok::<(), ftdi_nusb::Error>(())
/// ```
pub struct FtdiI2c {
    dev: FtdiDevice,
    ctx: MpsseContext,
    i2c: I2cBus,
}

impl FtdiI2c {
    /// Open an FTDI device and configure it for I2C via MPSSE.
    pub fn open(vendor: u16, product: u16, clock_hz: u32) -> crate::error::Result<Self> {
        let mut dev = FtdiDevice::open(vendor, product)?;
        let mut ctx = MpsseContext::init(&mut dev, clock_hz)?;
        let i2c = I2cBus::new(&mut ctx, &mut dev)?;
        Ok(Self { dev, ctx, i2c })
    }

    /// Create from already-opened components.
    pub fn from_parts(dev: FtdiDevice, ctx: MpsseContext, i2c: I2cBus) -> Self {
        Self { dev, ctx, i2c }
    }

    /// Borrow the underlying `FtdiDevice`.
    pub fn device(&self) -> &FtdiDevice {
        &self.dev
    }

    /// Mutably borrow the underlying `FtdiDevice`.
    pub fn device_mut(&mut self) -> &mut FtdiDevice {
        &mut self.dev
    }

    /// Decompose into the underlying parts.
    pub fn into_parts(self) -> (FtdiDevice, MpsseContext, I2cBus) {
        (self.dev, self.ctx, self.i2c)
    }
}

impl embedded_hal::i2c::ErrorType for FtdiI2c {
    type Error = Error;
}

impl embedded_hal::i2c::I2c for FtdiI2c {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        use embedded_hal::i2c::Operation;

        // Group contiguous reads/writes under a single START/STOP, using
        // repeated STARTs between direction changes — matching the I2C spec.
        //
        // The embedded-hal I2c::transaction contract says: issue a START,
        // process all operations with repeated STARTs between direction
        // changes, then STOP.

        if address > 0x7F {
            return Err(Error::InvalidArgument(
                "I2C address must be 7-bit (0x00..=0x7F)",
            ));
        }

        if operations.is_empty() {
            return Ok(());
        }

        // Determine direction of first operation
        let first_is_read = matches!(operations[0], Operation::Read(_));
        let first_addr = if first_is_read {
            (address << 1) | 0x01
        } else {
            (address << 1) & 0xFE
        };

        self.i2c.start(&mut self.ctx, &mut self.dev)?;

        if !self.i2c.write_byte(&mut self.dev, first_addr)? {
            self.i2c.stop(&mut self.ctx, &mut self.dev)?;
            return Err(Error::I2cNack("address not acknowledged"));
        }

        let mut prev_is_read = first_is_read;

        for op in operations.iter_mut() {
            let cur_is_read = matches!(op, Operation::Read(_));

            // If direction changes, issue a repeated START
            if cur_is_read != prev_is_read {
                self.i2c.start(&mut self.ctx, &mut self.dev)?;
                let addr_byte = if cur_is_read {
                    (address << 1) | 0x01
                } else {
                    (address << 1) & 0xFE
                };
                if !self.i2c.write_byte(&mut self.dev, addr_byte)? {
                    self.i2c.stop(&mut self.ctx, &mut self.dev)?;
                    return Err(Error::I2cNack("address not acknowledged"));
                }
            }

            match op {
                Operation::Read(buf) => {
                    for i in 0..buf.len() {
                        let ack = i < buf.len() - 1;
                        buf[i] = self.i2c.read_byte(&mut self.dev, ack)?;
                    }
                }
                Operation::Write(buf) => {
                    for &byte in buf.iter() {
                        if !self.i2c.write_byte(&mut self.dev, byte)? {
                            self.i2c.stop(&mut self.ctx, &mut self.dev)?;
                            return Err(Error::I2cNack("data byte not acknowledged"));
                        }
                    }
                }
            }

            prev_is_read = cur_is_read;
        }

        self.i2c.stop(&mut self.ctx, &mut self.dev)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_kind_mapping_spi() {
        use embedded_hal::spi::Error as _;
        let err = Error::MpsseBadCommand(0xAB);
        assert_eq!(err.kind(), embedded_hal::spi::ErrorKind::Other);
    }

    #[test]
    fn error_kind_mapping_i2c_address_nack() {
        use embedded_hal::i2c::Error as _;
        let err = Error::I2cNack("address not acknowledged");
        match err.kind() {
            embedded_hal::i2c::ErrorKind::NoAcknowledge(
                embedded_hal::i2c::NoAcknowledgeSource::Address,
            ) => {}
            other => panic!("expected address NACK, got {:?}", other),
        }
    }

    #[test]
    fn error_kind_mapping_i2c_data_nack() {
        use embedded_hal::i2c::Error as _;
        let err = Error::I2cNack("data byte not acknowledged");
        match err.kind() {
            embedded_hal::i2c::ErrorKind::NoAcknowledge(
                embedded_hal::i2c::NoAcknowledgeSource::Data,
            ) => {}
            other => panic!("expected data NACK, got {:?}", other),
        }
    }

    #[test]
    fn error_kind_mapping_io() {
        use embedded_io::Error as _;
        let err = Error::WriteZero;
        assert_eq!(err.kind(), embedded_io::ErrorKind::WriteZero);

        let err2 = Error::DeviceUnavailable;
        assert_eq!(err2.kind(), embedded_io::ErrorKind::Other);
    }
}
