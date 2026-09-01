# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.0] - 2026-08-28

This release makes the asynchronous API canonical on every target and
reworks MPSSE and streaming around it. It is a breaking release.

### Changed

- **`FtdiDevice` is now asynchronous on all targets.** Serial, EEPROM,
  MPSSE, and streaming methods are `async fn`s; the WASM and native APIs
  are the same API selected automatically by target
- **MPSSE I/O goes through an `MpsseSession`** obtained from
  `MpsseContext::session(&mut dev)`, pairing the context with a mutable
  borrow of the device it was initialized on. `SpiDevice`, `I2cBus`,
  `JtagBus`, and the GPIO types take the session instead of separate
  context/device arguments, making it impossible to drive one device
  with another device's MPSSE state
- **Streaming is a session type**: `FtdiDevice::start_stream()` returns
  an `FtdiStream` polled with `next().await`; end it explicitly with
  `finish().await`. Timeouts and USB transfer failures are reported as
  errors instead of a clean end-of-stream
- Feature flags reworked: `std`, `is_sync`, and `wasm` are gone. New
  `smol` and `tokio` features select how nusb offloads blocking
  discovery/open calls; USB transfers are runtime-independent
- `embedded-hal` adapters now drive the async implementation with an
  internal `block_on`
- Updated nusb to 0.2.7

### Added

- `blocking::FtdiDevice`: a blocking serial/EEPROM wrapper for native
  applications without an async runtime (`as_async_mut()`/`into_async()`
  reach the async MPSSE and streaming APIs)
- **Cancellation safety.** Dropping the future of a read preserves the
  in-flight USB transfer, so the next read resumes without losing serial
  input. Cancelling a write or stateful operation (MPSSE transaction,
  EEPROM read/write/erase, mode or baud-rate change, streaming session)
  poisons the device: further I/O returns `Error::RecoveryRequired`
  until `FtdiDevice::recover().await` succeeds
- `FtdiDevice::recover()` drains endpoint transfers, completes interrupted
  EEPROM writes or erases, resets the device, and re-applies baud rate and
  bitbang mode with the recorded direction mask. Recovery and successful
  USB/mode resets invalidate existing MPSSE contexts and bus objects, which
  then fail with `Error::InvalidMpsseContext`; switching SPI/I2C/JTAG
  configurations likewise invalidates objects from the previous protocol
- `Error::RecoveryRequired`, `Error::InvalidMpsseContext`, and
  `Error::ShortWrite` variants

### Removed

- The non-blocking submit/complete transfer API (`async_transfer`
  module); use the async `read_data`/`write_data` directly

### Migration from 0.2

- Blocking serial users: replace `ftdi_nusb::FtdiDevice` with
  `ftdi_nusb::blocking::FtdiDevice`
- Async users: enable the `smol` or `tokio` feature for the native
  constructors and `.await` the I/O methods
- MPSSE users: create a session per interaction —
  `let mut s = mpsse.session(&mut dev)?;` — and pass `&mut s` where
  `&mut ctx, &mut dev` was passed before

## [0.2.0] - 2026-08-02

### Added

- WebAssembly / WebUSB support via upstream nusb's WebUSB backend (`wasm`
  feature, builds for `wasm32-unknown-unknown` with
  `RUSTFLAGS='--cfg=web_sys_unstable_apis'`)
- `embedded-hal` 1.0 trait implementations behind the `embedded-hal` feature flag:
  - `embedded_hal::spi::SpiDevice` via `FtdiSpiDevice` wrapper
  - `embedded_hal::i2c::I2c` via `FtdiI2c` wrapper
  - `embedded_io::{Read, Write}` for `FtdiDevice`
- `FlowControl::XonXoff { xon, xoff }` variant for software flow control
- `set_flow_control()` now handles all four modes (disabled, RTS/CTS, DTR/DSR, XON/XOFF)
- MPSSE GPIO pin abstraction (`mpsse::gpio` module):
  - `GpioPin` for single-pin read/write/direction control
  - `GpioGroup` for batch pin operations
  - `GpioBank` enum for low/high byte selection
- Error recovery utilities:
  - `FtdiDevice::read_data_retry()` / `write_data_retry()` with configurable retries
  - `FtdiDevice::is_connected()` — check if device is still responding
  - `FtdiDevice::recover()` — reset device and re-apply configuration
  - `Error::Timeout` and `Error::Disconnected` variants
- Integration examples:
  - `spi_flash` — JEDEC ID and status register reading
  - `i2c_sensor` — TMP102 temperature sensor reading
  - `jtag_idcode` — JTAG chain scanning and IDCODE reading
- Property-based tests for EEPROM build/decode round-trips using `proptest`
- GitHub Actions CI workflow (build, test, clippy, feature combinations, wasm32 target)
- LICENSE-MIT and LICENSE-APACHE files
- CHANGELOG.md

### Changed

- Migrated to Rust edition 2024

## [0.1.0] - 2026-03-20

### Added

- Initial release with complete libftdi 1.5 API coverage
- Core device I/O: open, configure, read/write, baud rate, serial properties
- All FTDI chip types: AM, BM, FT2232C, FT232R, FT2232H, FT4232H, FT232H, FT230X
- Bitbang modes: async, sync, CBUS, MPSSE, sync FIFO
- MPSSE engine: clock configuration, GPIO, loopback, bad-command detection
- High-level SPI: full-duplex, half-duplex, configurable mode/CS/bit-order
- High-level I2C: bit-banged master with 3-phase clocking, ACK/NACK
- High-level JTAG: TAP state machine, TDI/TDO shifting, IR/DR scan
- EEPROM: read, write, erase, build, decode, init_defaults for all chip types
- Async transfers: non-blocking submit/complete pattern
- Streaming: high-throughput continuous reads (FT2232H/FT232H)
- Device discovery and filtering
- Flow control: disabled, RTS/CTS, DTR/DSR
- `impl Read + Write` for `FtdiDevice`
- 122 unit tests, 11 doc-tests, 0 clippy warnings
