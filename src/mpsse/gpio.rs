//! High-level GPIO pin abstraction for MPSSE.
//!
//! Provides an ergonomic API for controlling individual GPIO pins on FTDI
//! MPSSE-capable chips, without needing to manually manage bitmasks.
//!
//! # Pin Mapping
//!
//! FTDI chips expose two GPIO banks in MPSSE mode:
//!
//! - **Low byte** (ADBUS0-7): Pins 0-2 are used by the MPSSE protocol
//!   (SK/DO/DI). Pins 3-7 are available as general-purpose GPIO.
//! - **High byte** (ACBUS0-7): All 8 pins available as GPIO.
//!
//! # Example
//!
//! ```no_run
//! use ftdi_nusb::{FtdiDevice, mpsse::{MpsseContext, gpio::{GpioPin, GpioBank}}};
//!
//! let mut dev = FtdiDevice::open(0x0403, 0x6014)?; // FT232H
//! let mut mpsse = MpsseContext::init(&mut dev, 1_000_000)?;
//!
//! // Configure ADBUS4 as output, drive high
//! let mut pin = GpioPin::new(GpioBank::Low, 4);
//! pin.set_output(&mut mpsse, &mut dev, true)?;
//!
//! // Read pin state
//! let state = pin.read(&mpsse, &mut dev)?;
//!
//! // Set as input
//! pin.set_input(&mut mpsse, &mut dev)?;
//! # Ok::<(), ftdi_nusb::Error>(())
//! ```

use maybe_async::maybe_async;

use crate::context::FtdiDevice;
use crate::error::{Error, Result};

use super::MpsseContext;

/// GPIO bank selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GpioBank {
    /// Low byte GPIO (ADBUS0-7).
    Low,
    /// High byte GPIO (ACBUS0-7).
    High,
}

/// Direction of a GPIO pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    /// Input (high-impedance).
    Input,
    /// Output (driven).
    Output,
}

/// A single GPIO pin on the MPSSE.
///
/// Tracks the bank and bit position. Operations apply through the
/// `MpsseContext` so that other pins are not disturbed.
#[derive(Debug, Clone)]
pub struct GpioPin {
    bank: GpioBank,
    bit: u8,
    mask: u8,
}

impl GpioPin {
    /// Create a new GPIO pin reference.
    ///
    /// `bank` selects low (ADBUS) or high (ACBUS) byte.
    /// `bit` is the bit position (0-7).
    ///
    /// # Warning
    ///
    /// On the **Low** bank, pins 0-2 are reserved by the MPSSE protocol:
    /// - Pin 0 = SK (clock)
    /// - Pin 1 = DO (data out)
    /// - Pin 2 = DI (data in)
    ///
    /// Using this function with `GpioBank::Low` and `bit` 0, 1, or 2 will
    /// create a pin that conflicts with MPSSE SPI/I2C/JTAG signals. Only do
    /// this if you are not using any MPSSE protocol on those pins.
    ///
    /// # Panics
    ///
    /// Panics if `bit > 7`.
    pub fn new(bank: GpioBank, bit: u8) -> Self {
        assert!(bit <= 7, "GPIO bit must be 0-7, got {}", bit);
        Self {
            bank,
            bit,
            mask: 1 << bit,
        }
    }

    /// Get the bank this pin belongs to.
    pub fn bank(&self) -> GpioBank {
        self.bank
    }

    /// Get the bit position (0-7).
    pub fn bit(&self) -> u8 {
        self.bit
    }

    /// Get the bitmask for this pin.
    pub fn mask(&self) -> u8 {
        self.mask
    }

    /// Configure this pin as an output and set its initial value.
    ///
    /// `high` sets the pin high (true) or low (false).
    #[maybe_async]
    pub async fn set_output(
        &mut self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        high: bool,
    ) -> Result<()> {
        match self.bank {
            GpioBank::Low => {
                let dir = ctx.gpio_low_dir() | self.mask;
                let mut val = ctx.gpio_low_value();
                if high {
                    val |= self.mask;
                } else {
                    val &= !self.mask;
                }
                ctx.set_gpio_low(dev, val, dir).await
            }
            GpioBank::High => {
                let dir = ctx.gpio_high_dir() | self.mask;
                let mut val = ctx.gpio_high_value();
                if high {
                    val |= self.mask;
                } else {
                    val &= !self.mask;
                }
                ctx.set_gpio_high(dev, val, dir).await
            }
        }
    }

    /// Configure this pin as an input (high-impedance).
    #[maybe_async]
    pub async fn set_input(&mut self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        match self.bank {
            GpioBank::Low => {
                let dir = ctx.gpio_low_dir() & !self.mask;
                let val = ctx.gpio_low_value();
                ctx.set_gpio_low(dev, val, dir).await
            }
            GpioBank::High => {
                let dir = ctx.gpio_high_dir() & !self.mask;
                let val = ctx.gpio_high_value();
                ctx.set_gpio_high(dev, val, dir).await
            }
        }
    }

    /// Write a value to this pin (must already be configured as output).
    #[maybe_async]
    pub async fn write(
        &self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        high: bool,
    ) -> Result<()> {
        match self.bank {
            GpioBank::Low => {
                let dir = ctx.gpio_low_dir();
                if dir & self.mask == 0 {
                    return Err(Error::InvalidArgument("pin is not configured as output"));
                }
                let mut val = ctx.gpio_low_value();
                if high {
                    val |= self.mask;
                } else {
                    val &= !self.mask;
                }
                ctx.set_gpio_low(dev, val, dir).await
            }
            GpioBank::High => {
                let dir = ctx.gpio_high_dir();
                if dir & self.mask == 0 {
                    return Err(Error::InvalidArgument("pin is not configured as output"));
                }
                let mut val = ctx.gpio_high_value();
                if high {
                    val |= self.mask;
                } else {
                    val &= !self.mask;
                }
                ctx.set_gpio_high(dev, val, dir).await
            }
        }
    }

    /// Read the current state of this pin.
    #[maybe_async]
    pub async fn read(&self, ctx: &MpsseContext, dev: &mut FtdiDevice) -> Result<bool> {
        let byte = match self.bank {
            GpioBank::Low => ctx.get_gpio_low(dev).await?,
            GpioBank::High => ctx.get_gpio_high(dev).await?,
        };
        Ok(byte & self.mask != 0)
    }

    /// Check whether this pin is currently configured as an output.
    pub fn is_output(&self, ctx: &MpsseContext) -> bool {
        let dir = match self.bank {
            GpioBank::Low => ctx.gpio_low_dir(),
            GpioBank::High => ctx.gpio_high_dir(),
        };
        dir & self.mask != 0
    }
}

/// A group of GPIO pins for batch operations.
///
/// Useful for setting multiple pins at once in a single USB transfer.
#[derive(Debug, Clone)]
pub struct GpioGroup {
    bank: GpioBank,
    mask: u8,
}

impl GpioGroup {
    /// Create a new GPIO group for the given bank with the given pin mask.
    ///
    /// Each set bit in `mask` represents a pin in the group.
    pub fn new(bank: GpioBank, mask: u8) -> Self {
        Self { bank, mask }
    }

    /// Get the bank.
    pub fn bank(&self) -> GpioBank {
        self.bank
    }

    /// Get the group mask.
    pub fn mask(&self) -> u8 {
        self.mask
    }

    /// Configure all pins in this group as outputs with the given values.
    ///
    /// Only the bits corresponding to `self.mask` in `values` are used.
    #[maybe_async]
    pub async fn set_all_output(
        &self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        values: u8,
    ) -> Result<()> {
        match self.bank {
            GpioBank::Low => {
                let dir = ctx.gpio_low_dir() | self.mask;
                let mut val = ctx.gpio_low_value();
                val = (val & !self.mask) | (values & self.mask);
                ctx.set_gpio_low(dev, val, dir).await
            }
            GpioBank::High => {
                let dir = ctx.gpio_high_dir() | self.mask;
                let mut val = ctx.gpio_high_value();
                val = (val & !self.mask) | (values & self.mask);
                ctx.set_gpio_high(dev, val, dir).await
            }
        }
    }

    /// Configure all pins in this group as inputs.
    #[maybe_async]
    pub async fn set_all_input(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        match self.bank {
            GpioBank::Low => {
                let dir = ctx.gpio_low_dir() & !self.mask;
                let val = ctx.gpio_low_value();
                ctx.set_gpio_low(dev, val, dir).await
            }
            GpioBank::High => {
                let dir = ctx.gpio_high_dir() & !self.mask;
                let val = ctx.gpio_high_value();
                ctx.set_gpio_high(dev, val, dir).await
            }
        }
    }

    /// Read the current state of all pins in this group.
    ///
    /// Returns the raw byte with only the group's bits relevant.
    #[maybe_async]
    pub async fn read(&self, ctx: &MpsseContext, dev: &mut FtdiDevice) -> Result<u8> {
        let byte = match self.bank {
            GpioBank::Low => ctx.get_gpio_low(dev).await?,
            GpioBank::High => ctx.get_gpio_high(dev).await?,
        };
        Ok(byte & self.mask)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gpio_pin_new() {
        let pin = GpioPin::new(GpioBank::Low, 4);
        assert_eq!(pin.bank(), GpioBank::Low);
        assert_eq!(pin.bit(), 4);
        assert_eq!(pin.mask(), 0x10);
    }

    #[test]
    fn gpio_pin_bit_0() {
        let pin = GpioPin::new(GpioBank::High, 0);
        assert_eq!(pin.mask(), 0x01);
    }

    #[test]
    fn gpio_pin_bit_7() {
        let pin = GpioPin::new(GpioBank::Low, 7);
        assert_eq!(pin.mask(), 0x80);
    }

    #[test]
    #[should_panic(expected = "GPIO bit must be 0-7")]
    fn gpio_pin_bit_too_high() {
        GpioPin::new(GpioBank::Low, 8);
    }

    #[test]
    fn gpio_group_mask() {
        let group = GpioGroup::new(GpioBank::Low, 0xF0);
        assert_eq!(group.bank(), GpioBank::Low);
        assert_eq!(group.mask(), 0xF0);
    }

    #[test]
    fn is_output_when_not_initialized() {
        let pin = GpioPin::new(GpioBank::Low, 4);
        // MpsseContext with all zeros — pin should not be output
        let ctx = MpsseContext::test_new(false);
        assert!(!pin.is_output(&ctx));
    }

    #[test]
    fn is_output_when_dir_set() {
        let pin = GpioPin::new(GpioBank::Low, 4);
        let mut ctx = MpsseContext::test_new(false);
        ctx.update_gpio_low_state(0x10, 0x10); // pin 4 = output
        assert!(pin.is_output(&ctx));
    }
}
