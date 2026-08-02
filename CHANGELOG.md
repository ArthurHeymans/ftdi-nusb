# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
