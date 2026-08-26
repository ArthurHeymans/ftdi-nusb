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

// Always available (pure computation)
mod baudrate;
pub mod constants;

/// Internal platform-aware sleep helper.
///
/// In sync mode, uses `std::thread::sleep`. In WASM async mode, uses
/// `setTimeout` via a JS Promise. Works in both Window and Worker contexts.
pub(crate) mod sleep_util {
    use core::time::Duration;

    /// Sleep for the given duration.
    ///
    /// - Native targets block the thread with `std::thread::sleep`.
    /// - WASM targets yield via a `setTimeout` Promise.
    #[maybe_async::maybe_async]
    pub(crate) async fn sleep(duration: Duration) {
        let _ = duration;

        #[cfg(not(target_arch = "wasm32"))]
        {
            std::thread::sleep(duration);
        }

        #[cfg(target_arch = "wasm32")]
        {
            use wasm_bindgen::JsCast;

            let delay_ms = duration.as_millis() as i32;
            if delay_ms > 0 {
                let promise = js_sys::Promise::new(&mut |resolve, _| {
                    if let Some(window) = web_sys::window() {
                        window
                            .set_timeout_with_callback_and_timeout_and_arguments_0(
                                &resolve, delay_ms,
                            )
                            .unwrap();
                    } else {
                        let global: web_sys::WorkerGlobalScope = js_sys::global().unchecked_into();
                        global
                            .set_timeout_with_callback_and_timeout_and_arguments_0(
                                &resolve, delay_ms,
                            )
                            .unwrap();
                    }
                });
                let _ = wasm_bindgen_futures::JsFuture::from(promise).await;
            }
        }
    }
}
#[cfg(not(target_arch = "wasm32"))]
pub mod blocking;
pub mod context;
pub mod eeprom;
pub mod error;
pub mod mpsse;
pub mod types;

// Native-only modules
#[cfg(not(target_arch = "wasm32"))]
pub mod async_transfer;
#[cfg(not(target_arch = "wasm32"))]
pub mod device_info;
#[cfg(all(feature = "embedded-hal", not(target_arch = "wasm32")))]
pub mod hal;
#[cfg(not(target_arch = "wasm32"))]
pub mod stream;

// ---- Convenience re-exports ----

pub use constants::FTDI_VID;
pub use context::{AsyncFtdiDevice, FtdiDevice};
pub use eeprom::FtdiEeprom;
pub use error::{Error, Result};
pub use types::*;

#[cfg(not(target_arch = "wasm32"))]
pub use async_transfer::{ReadTransferControl, WriteTransferControl};
#[cfg(not(target_arch = "wasm32"))]
pub use device_info::{DeviceFilter, find_device, find_devices};
#[cfg(not(target_arch = "wasm32"))]
pub use stream::StreamProgress;
