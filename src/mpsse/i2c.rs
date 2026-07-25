//! I2C master protocol using MPSSE.
//!
//! Implements bit-banged I2C master communication using the FTDI MPSSE engine
//! with 3-phase data clocking. Requires an H-type chip (FT2232H, FT4232H,
//! FT232H).
//!
//! # Pin Mapping
//!
//! | FTDI Pin | I2C Signal | ADBUS Bit |
//! |----------|-----------|-----------|
//! | SK       | SCL       | 0         |
//! | DO/DI    | SDA       | 1 & 2     |
//!
//! Note: SDA requires DO (ADBUS1) and DI (ADBUS2) to be connected together
//! externally, with a pull-up resistor. DO is used for driving SDA low
//! (open-drain), and DI is used for reading SDA.
//!
//! # Example
//!
//! ```no_run
//! use ftdi_nusb::{FtdiDevice, mpsse::{MpsseContext, i2c::I2cBus}};
//!
//! let mut dev = FtdiDevice::open(0x0403, 0x6014)?; // FT232H
//! let mut mpsse = MpsseContext::init(&mut dev, 100_000)?; // 100 kHz I2C
//! let mut i2c = I2cBus::new(&mut mpsse, &mut dev)?;
//!
//! // Write to device at address 0x50
//! i2c.write(&mut mpsse, &mut dev, 0x50, &[0x00, 0x42])?;
//!
//! // Read 2 bytes from device at address 0x50
//! let data = i2c.read(&mut mpsse, &mut dev, 0x50, 2)?;
//! # Ok::<(), ftdi_nusb::Error>(())
//! ```

use maybe_async::maybe_async;

use crate::constants::mpsse;
use crate::context::FtdiDevice;
use crate::error::{Error, Result};

use super::MpsseContext;

/// I2C bus instance using MPSSE.
///
/// Manages the I2C bus state including the pin directions needed for
/// open-drain SDA signaling.
#[derive(Debug, Clone)]
pub struct I2cBus {
    /// Direction mask when driving SDA (DO=output for low, DI=input).
    /// SCL is always output via SK (bit 0).
    /// SDA driven: DO (bit 1) = output, DI (bit 2) = input.
    dir_sda_out: u8,
    /// Direction mask when releasing SDA (for reads / ACK sense).
    /// SDA released: DO (bit 1) = input (tri-state), DI (bit 2) = input.
    dir_sda_in: u8,
    /// Additional GPIO direction bits to preserve (bits 3-7).
    #[allow(dead_code)]
    extra_dir: u8,
    /// Additional GPIO value bits to preserve (bits 3-7).
    extra_val: u8,
}

/// I2C error type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    /// No ACK received from the slave device.
    Nack,
    /// Bus arbitration lost.
    ArbitrationLost,
}

impl core::fmt::Display for I2cError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Nack => write!(f, "I2C NACK received"),
            Self::ArbitrationLost => write!(f, "I2C arbitration lost"),
        }
    }
}

impl I2cBus {
    /// Initialize I2C mode on the MPSSE.
    ///
    /// Enables 3-phase data clocking and configures pins for I2C operation.
    /// SCL and SDA are driven high (idle) initially.
    ///
    /// The MPSSE clock should already be set to the desired I2C bus speed
    /// (typically 100 kHz or 400 kHz).
    #[maybe_async]
    pub async fn new(ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<Self> {
        if !ctx.is_h_type() {
            return Err(Error::InvalidArgument(
                "I2C requires an H-type chip (FT2232H/FT4232H/FT232H)",
            ));
        }

        // Enable 3-phase clocking for I2C
        ctx.enable_3phase_clocking(dev).await?;

        // Preserve any existing GPIO config on pins 3-7
        let extra_dir = ctx.gpio_low_dir() & 0xF8;
        let extra_val = ctx.gpio_low_value() & 0xF8;

        let bus = Self {
            dir_sda_out: 0x03 | extra_dir, // SK=out, DO=out (drive SDA)
            dir_sda_in: 0x01 | extra_dir,  // SK=out, DO=in (release SDA)
            extra_dir,
            extra_val,
        };

        // Set initial state: SCL=high, SDA=high (idle)
        // With open-drain: we release both lines (set direction to input)
        // Then pull both high by setting output values
        let idle_val = 0x03 | extra_val; // SK=1, DO=1
        let idle_dir = bus.dir_sda_out;
        ctx.set_gpio_low(dev, idle_val, idle_dir).await?;

        Ok(bus)
    }

    /// Generate an I2C START condition.
    ///
    /// SDA goes low while SCL is high.
    #[maybe_async]
    pub async fn start(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        let high = 0x03 | self.extra_val; // SCL=1, SDA=1
        let sda_low = 0x01 | self.extra_val; // SCL=1, SDA=0
        let both_low = self.extra_val; // SCL=0, SDA=0

        let mut cmd = Vec::with_capacity(20);

        // Ensure idle state: SDA=1, SCL=1
        // Repeat the set-bits commands a few times for setup/hold timing
        for _ in 0..4 {
            cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, high, self.dir_sda_out]);
        }

        // SDA goes low while SCL is still high
        for _ in 0..4 {
            cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, sda_low, self.dir_sda_out]);
        }

        // SCL goes low
        cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, both_low, self.dir_sda_out]);

        dev.write_all(&cmd).await?;

        // Keep tracked GPIO state in sync with the final SET_BITS_LOW we sent
        ctx.update_gpio_low_state(both_low, self.dir_sda_out);
        Ok(())
    }

    /// Generate an I2C STOP condition.
    ///
    /// SDA goes high while SCL is high.
    #[maybe_async]
    pub async fn stop(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        let both_low = self.extra_val; // SCL=0, SDA=0
        let scl_high = 0x01 | self.extra_val; // SCL=1, SDA=0
        let both_high = 0x03 | self.extra_val; // SCL=1, SDA=1

        let mut cmd = Vec::with_capacity(20);

        // Ensure SDA=0, SCL=0
        for _ in 0..4 {
            cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, both_low, self.dir_sda_out]);
        }

        // SCL goes high while SDA stays low
        for _ in 0..4 {
            cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, scl_high, self.dir_sda_out]);
        }

        // SDA goes high (STOP)
        for _ in 0..4 {
            cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, both_high, self.dir_sda_out]);
        }

        dev.write_all(&cmd).await?;

        // Keep tracked GPIO state in sync with the final SET_BITS_LOW (idle state)
        ctx.update_gpio_low_state(both_high, self.dir_sda_out);
        Ok(())
    }

    /// Write a single byte and return whether ACK was received.
    ///
    /// Returns `true` if ACK (SDA=0) was received, `false` for NACK.
    #[maybe_async]
    pub async fn write_byte(&self, dev: &mut FtdiDevice, byte: u8) -> Result<bool> {
        let mut cmd = Vec::with_capacity(20);

        // Set SDA as output for writing
        cmd.extend_from_slice(&[
            mpsse::SET_BITS_LOW,
            self.extra_val, // SCL=0, SDA=0
            self.dir_sda_out,
        ]);

        // Clock out 8 bits MSB first on negative edge (I2C convention)
        cmd.push(mpsse::DO_WRITE | mpsse::WRITE_NEG | mpsse::BITMODE);
        cmd.push(7); // 8 bits (0-indexed)
        cmd.push(byte);

        // Release SDA for ACK: set DO to input
        cmd.extend_from_slice(&[
            mpsse::SET_BITS_LOW,
            self.extra_val, // SCL=0, release SDA
            self.dir_sda_in,
        ]);

        // Read ACK bit
        cmd.push(mpsse::DO_READ | mpsse::BITMODE | mpsse::READ_NEG);
        cmd.push(0); // 1 bit

        cmd.push(mpsse::SEND_IMMEDIATE);

        dev.write_all(&cmd).await?;

        // Read the ACK bit
        let mut buf = [0u8; 1];
        let mut attempts = 0;
        let mut offset = 0;
        while offset < 1 && attempts < 10 {
            let n = dev.read_data(&mut buf[offset..]).await?;
            offset += n;
            attempts += 1;
        }

        if offset == 0 {
            return Err(Error::DeviceUnavailable);
        }

        // ACK bit is in bit 7 of the byte (MSB first read)
        let ack = (buf[0] & 0x01) == 0;
        Ok(ack)
    }

    /// Read a single byte, sending ACK or NACK.
    ///
    /// Set `ack` to `true` to acknowledge (continue reading) or `false`
    /// to NACK (signal end of read).
    #[maybe_async]
    pub async fn read_byte(&self, dev: &mut FtdiDevice, ack: bool) -> Result<u8> {
        let mut cmd = Vec::with_capacity(20);

        // Release SDA for reading
        cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, self.extra_val, self.dir_sda_in]);

        // Clock in 8 bits MSB first
        cmd.push(mpsse::DO_READ | mpsse::BITMODE | mpsse::READ_NEG);
        cmd.push(7); // 8 bits

        // Send ACK/NACK: drive SDA
        cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, self.extra_val, self.dir_sda_out]);

        // Clock out ACK/NACK bit
        cmd.push(mpsse::DO_WRITE | mpsse::WRITE_NEG | mpsse::BITMODE);
        cmd.push(0); // 1 bit
        cmd.push(if ack { 0x00 } else { 0x80 }); // ACK=0, NACK=1 (MSB)

        // Release SDA again
        cmd.extend_from_slice(&[mpsse::SET_BITS_LOW, self.extra_val, self.dir_sda_in]);

        cmd.push(mpsse::SEND_IMMEDIATE);

        dev.write_all(&cmd).await?;

        // Read the byte
        let mut buf = [0u8; 1];
        let mut attempts = 0;
        let mut offset = 0;
        while offset < 1 && attempts < 10 {
            let n = dev.read_data(&mut buf[offset..]).await?;
            offset += n;
            attempts += 1;
        }

        if offset == 0 {
            return Err(Error::DeviceUnavailable);
        }

        Ok(buf[0])
    }

    /// Write data to an I2C slave device.
    ///
    /// Sends START, address+W, data bytes, STOP. Returns an error if any
    /// byte is NACKed.
    #[maybe_async]
    pub async fn write(
        &self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        address: u8,
        data: &[u8],
    ) -> Result<()> {
        if address > 0x7F {
            return Err(Error::InvalidArgument(
                "I2C address must be 7-bit (0x00-0x7F)",
            ));
        }
        self.start(ctx, dev).await?;

        // Address byte with write bit (bit 0 = 0)
        let addr_byte = (address << 1) & 0xFE;
        if !self.write_byte(dev, addr_byte).await? {
            self.stop(ctx, dev).await?;
            return Err(Error::I2cNack("address not acknowledged"));
        }

        for &byte in data {
            if !self.write_byte(dev, byte).await? {
                self.stop(ctx, dev).await?;
                return Err(Error::I2cNack("data byte not acknowledged"));
            }
        }

        self.stop(ctx, dev).await
    }

    /// Read data from an I2C slave device.
    ///
    /// Sends START, address+R, reads `len` bytes (ACK all except last), STOP.
    #[maybe_async]
    pub async fn read(
        &self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        address: u8,
        len: usize,
    ) -> Result<Vec<u8>> {
        if address > 0x7F {
            return Err(Error::InvalidArgument(
                "I2C address must be 7-bit (0x00-0x7F)",
            ));
        }
        if len == 0 {
            return Ok(Vec::new());
        }

        self.start(ctx, dev).await?;

        // Address byte with read bit (bit 0 = 1)
        let addr_byte = (address << 1) | 0x01;
        if !self.write_byte(dev, addr_byte).await? {
            self.stop(ctx, dev).await?;
            return Err(Error::I2cNack("address not acknowledged"));
        }

        let mut result = Vec::with_capacity(len);
        for i in 0..len {
            let ack = i < len - 1; // ACK all bytes except the last one
            let byte = self.read_byte(dev, ack).await?;
            result.push(byte);
        }

        self.stop(ctx, dev).await?;
        Ok(result)
    }

    /// Write data then read data from an I2C slave (repeated START).
    ///
    /// Common pattern: write a register address, then read data from it.
    /// Uses a repeated START between write and read phases.
    #[maybe_async]
    pub async fn write_read(
        &self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        address: u8,
        write_data: &[u8],
        read_len: usize,
    ) -> Result<Vec<u8>> {
        if address > 0x7F {
            return Err(Error::InvalidArgument(
                "I2C address must be 7-bit (0x00-0x7F)",
            ));
        }
        // Write phase
        self.start(ctx, dev).await?;

        let addr_w = (address << 1) & 0xFE;
        if !self.write_byte(dev, addr_w).await? {
            self.stop(ctx, dev).await?;
            return Err(Error::I2cNack("address not acknowledged (write phase)"));
        }

        for &byte in write_data {
            if !self.write_byte(dev, byte).await? {
                self.stop(ctx, dev).await?;
                return Err(Error::I2cNack("data byte not acknowledged"));
            }
        }

        // Repeated START for read phase
        self.start(ctx, dev).await?;

        let addr_r = (address << 1) | 0x01;
        if !self.write_byte(dev, addr_r).await? {
            self.stop(ctx, dev).await?;
            return Err(Error::I2cNack("address not acknowledged (read phase)"));
        }

        let mut result = Vec::with_capacity(read_len);
        for i in 0..read_len {
            let ack = i < read_len - 1;
            let byte = self.read_byte(dev, ack).await?;
            result.push(byte);
        }

        self.stop(ctx, dev).await?;
        Ok(result)
    }
}
