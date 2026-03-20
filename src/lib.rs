//! Pure Rust library for communicating with FTDI USB devices.
//!
//! This crate provides a complete interface to FTDI USB-to-serial converter
//! chips, including the FT232R, FT2232H, FT4232H, FT232H, and FT230X families.
//! It uses [nusb](https://crates.io/crates/nusb) as the USB backend — no
//! C dependencies or `libusb` required.
//!
//! # Quick Start
//!
//! ```no_run
//! use ftdi_nusb::{FtdiDevice, constants::FTDI_VID, constants::pid};
//!
//! // Open the first FT232R connected
//! let mut dev = FtdiDevice::open(FTDI_VID, pid::FT232)?;
//! dev.set_baudrate(115200)?;
//! dev.write_all(b"Hello from Rust!\r\n")?;
//! # Ok::<(), ftdi_nusb::Error>(())
//! ```
//!
//! # Features
//!
//! - **Device discovery**: Enumerate and filter connected FTDI devices.
//! - **Serial I/O**: Baud rate, line properties, flow control, modem lines.
//! - **Bitbang / MPSSE**: Asynchronous and synchronous bitbang, MPSSE for
//!   SPI/I2C/JTAG.
//! - **High-level MPSSE**: SPI master and I2C master with typed APIs
//!   ([`mpsse::spi`], [`mpsse::i2c`]).
//! - **Async transfers**: Submit non-blocking USB reads/writes and wait
//!   for completion later ([`async_transfer`]).
//! - **EEPROM**: Read, write, erase, decode, and build EEPROM images with
//!   chip-aware defaults.
//! - **Streaming**: High-throughput continuous reads via concurrent USB
//!   transfers (FT2232H / FT232H).
//! - **`Read` / `Write` traits**: Use `FtdiDevice` anywhere `std::io::Read`
//!   or `std::io::Write` is expected.

#![cfg_attr(not(feature = "std"), no_std)]

// Always available (pure computation)
mod baudrate;
pub mod constants;

/// Internal sleep helper (synchronous).
#[cfg(feature = "std")]
pub(crate) mod sleep_util {
    use core::time::Duration;

    /// Sleep for the given duration, blocking the thread.
    #[maybe_async::maybe_async]
    pub(crate) async fn sleep(duration: Duration) {
        std::thread::sleep(duration);
    }
}
#[cfg(feature = "std")]
pub mod context;
#[cfg(feature = "std")]
pub mod eeprom;
#[cfg(feature = "std")]
pub mod error;
#[cfg(feature = "std")]
pub mod mpsse;
pub mod types;

// Native-only modules
#[cfg(feature = "std")]
pub mod async_transfer;
#[cfg(feature = "std")]
pub mod device_info;
#[cfg(feature = "embedded-hal")]
pub mod hal;
#[cfg(feature = "std")]
pub mod stream;

// ---- Convenience re-exports ----

pub use constants::FTDI_VID;
#[cfg(feature = "std")]
pub use context::FtdiDevice;
#[cfg(feature = "std")]
pub use eeprom::FtdiEeprom;
#[cfg(feature = "std")]
pub use error::{Error, Result};
pub use types::*;

#[cfg(feature = "std")]
pub use async_transfer::{ReadTransferControl, WriteTransferControl};
#[cfg(feature = "std")]
pub use device_info::{find_device, find_devices, DeviceFilter};
#[cfg(feature = "std")]
pub use stream::StreamProgress;
