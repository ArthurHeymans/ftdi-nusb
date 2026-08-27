# ftdi-nusb

Pure Rust library for communicating with FTDI USB-to-serial converter chips.
Uses [nusb](https://crates.io/crates/nusb) as the USB backend — no C
dependencies, no `libusb`, no `libftdi`.

This is a from-scratch reimplementation of [libftdi 1.5](https://www.intra2net.com/en/developer/libftdi/),
verified line-by-line against the original C source.

## Supported Chips

| Chip       | VID:PID     | Notes                     |
|------------|-------------|---------------------------|
| FT232AM    | 0403:6001   | Original FTDI chip        |
| FT232BM    | 0403:6001   | B-type                    |
| FT232R     | 0403:6001   | Internal EEPROM           |
| FT2232C/D  | 0403:6010   | Dual-port                 |
| FT2232H    | 0403:6010   | Dual hi-speed, MPSSE      |
| FT4232H    | 0403:6011   | Quad-port, MPSSE          |
| FT232H     | 0403:6014   | Single hi-speed, MPSSE    |
| FT230X     | 0403:6015   | MTP EEPROM                |

## Features

- **Serial I/O** — baud rate, line properties (bits/parity/stop), flow control, modem lines, `impl Read + Write`
- **Bitbang** — asynchronous, synchronous, and CBUS bitbang modes
- **MPSSE** — Multi-Protocol Synchronous Serial Engine for:
  - **SPI** — full-duplex, half-duplex, configurable CPOL/CPHA/bit-order, auto CS management
  - **I2C** — bit-banged master with 3-phase clocking, ACK/NACK detection
  - **JTAG** — TAP state machine navigation, TDI/TDO shifting, IR/DR scan
- **EEPROM** — read, write, erase, build, decode with chip-aware defaults for all chip types
- **Streaming** — high-throughput continuous reads via concurrent USB transfers (FT2232H/FT232H)
- **Device discovery** — enumerate and filter by VID/PID/serial/description

## Quick Start

`FtdiDevice` is asynchronous on every target. Native applications without an
async runtime can use the blocking serial wrapper:

```rust,no_run
use ftdi_nusb::{blocking::FtdiDevice, constants::FTDI_VID, constants::pid};

// Open the first FT232R connected
let mut dev = FtdiDevice::open(FTDI_VID, pid::FT232)?;
dev.set_baudrate(115200)?;
dev.write_all(b"Hello from Rust!\r\n")?;
# Ok::<(), ftdi_nusb::Error>(())
```

MPSSE and streaming are asynchronous APIs. A blocking application reaches them
through `dev.as_async_mut()` and a `block_on` of its choice (see the examples).

### SPI

```rust,no_run
use ftdi_nusb::{FtdiDevice, mpsse::{MpsseContext, spi::{SpiDevice, SpiMode}}};

# async fn example(dev: &mut FtdiDevice) -> ftdi_nusb::Result<()> {
let mut mpsse = MpsseContext::init(dev, 1_000_000).await?;
let spi = SpiDevice::new(&mut mpsse, dev, SpiMode::Mode0).await?;

// Read JEDEC ID from SPI flash
let id = spi.transfer(&mut mpsse, dev, &[0x9F, 0, 0, 0]).await?;
# Ok(())
# }
```

### I2C

```rust,no_run
use ftdi_nusb::{FtdiDevice, mpsse::{MpsseContext, i2c::I2cBus}};

# async fn example(dev: &mut FtdiDevice) -> ftdi_nusb::Result<()> {
let mut mpsse = MpsseContext::init(dev, 100_000).await?; // 100 kHz
let i2c = I2cBus::new(&mut mpsse, dev).await?;

// Write register address, read 2 bytes from I2C device at 0x48
let data = i2c.write_read(&mut mpsse, dev, 0x48, &[0x00], 2).await?;
# Ok(())
# }
```

### JTAG

```rust,no_run
use ftdi_nusb::{FtdiDevice, mpsse::{MpsseContext, jtag::JtagBus}};

# async fn example(dev: &mut FtdiDevice) -> ftdi_nusb::Result<()> {
let mut mpsse = MpsseContext::init(dev, 1_000_000).await?;
let mut jtag = JtagBus::new(&mut mpsse, dev).await?;

// Reset TAP and read IDCODE
jtag.reset(dev).await?;
let idcode = jtag.shift_dr(&mpsse, dev, &[0; 4], 32).await?;
# Ok(())
# }
```

### EEPROM

```rust,no_run
use ftdi_nusb::blocking::FtdiDevice;

let mut dev = FtdiDevice::open(0x0403, 0x6001)?;

// Read and decode
dev.read_eeprom()?;
dev.eeprom_decode()?;
let eeprom = dev.eeprom();
println!("Manufacturer: {:?}", eeprom.manufacturer);
println!("Product: {:?}", eeprom.product);
# Ok::<(), ftdi_nusb::Error>(())
```

## Feature Comparison with libftdi 1.5

| Feature                        | libftdi 1.5 | ftdi-nusb (this crate) |
|--------------------------------|:-----------:|:----------------------:|
| Serial I/O                     | Yes         | Yes                    |
| Baud rate (all chip types)     | Yes         | Yes                    |
| Line properties                | Yes         | Yes                    |
| Flow control (RTS/CTS, DTR/DSR, XON/XOFF) | Yes | Yes             |
| Modem control (DTR, RTS)       | Yes         | Yes                    |
| Bitbang modes                  | Yes         | Yes                    |
| MPSSE (raw)                    | Yes         | Yes                    |
| MPSSE SPI                      | No*         | Yes                    |
| MPSSE I2C                      | No*         | Yes                    |
| MPSSE JTAG                     | No*         | Yes                    |
| EEPROM read/write/erase        | Yes         | Yes                    |
| EEPROM build/decode            | Yes         | Yes                    |
| EEPROM init defaults           | Yes         | Yes                    |
| Streaming (sync FIFO)          | Yes         | Yes                    |
| Multi-interface (A/B/C/D)      | Yes         | Yes                    |
| Device discovery & filtering   | Yes         | Yes                    |
| Bad-command detection (0xFA)   | No          | Yes                    |
| `Read`/`Write` trait impls     | No          | Yes                    |

\* libftdi provides raw MPSSE access but no high-level SPI/I2C/JTAG protocols.

## Platform Support

Requires a platform supported by [nusb](https://docs.rs/nusb):

- **Linux** — via usbfs (no root required with proper udev rules)
- **macOS** — via IOKit
- **Windows** — via WinUSB
- **WebAssembly** — via WebUSB in Chromium-based browsers

On Linux, you may need to detach the `ftdi_sio` kernel driver. The library
handles this automatically via nusb's `detach_and_claim_interface()`.

## Native async support

Enable either runtime integration feature to use the asynchronous device
constructors:

```toml
ftdi-nusb = { version = "0.3", features = ["smol"] }
# or: features = ["tokio"]
```

```rust,no_run
use ftdi_nusb::FtdiDevice;

# async fn example() -> ftdi_nusb::Result<()> {
let mut dev = FtdiDevice::open(0x0403, 0x6001).await?;
dev.set_baudrate(115_200).await?;
dev.write_all(b"Hello from async Rust!\r\n").await?;
# Ok(())
# }
```

The features only select how nusb offloads blocking operating-system calls
needed for discovery and opening. USB transfers themselves are
runtime-independent, so `blocking::FtdiDevice` and the transfer methods work
without either feature. When both features are enabled, nusb uses its smol
path. The synchronous `embedded-hal` adapters drive the async implementation
with an internal `block_on`.

Async reads preserve an in-flight USB read when their future is cancelled, so
the next read resumes without silently discarding serial input. Writes may have
partially completed when cancelled. Stateful protocol or streaming sessions
should be finished explicitly. If one is cancelled or dropped, subsequent I/O
returns `Error::RecoveryRequired` until
`FtdiDevice::recover().await` succeeds. Recovery invalidates existing MPSSE
contexts and bus objects; initialize them again before issuing more MPSSE
commands.

### Async streaming (native only)

```rust,no_run
use ftdi_nusb::{FtdiDevice, StreamEvent};

# async fn example(dev: &mut FtdiDevice) -> ftdi_nusb::Result<()> {
let mut stream = dev.start_stream(8, 4).await?;
while let Some(event) = stream.next().await? {
    match event {
        StreamEvent::Data(data) => process(data).await,
        StreamEvent::Progress(progress) => println!("{} B/s", progress.current_rate),
    }
}
stream.finish().await?;
# async fn process(_: Vec<u8>) {}
# Ok(())
# }
```

`read_stream_async` is also available when an async callback is more convenient.
Streaming uses the configured read timeout and reports timeouts and USB transfer
failures instead of treating them as a clean end-of-stream.

## WASM / WebUSB Support

The `wasm32-unknown-unknown` target builds with WebUSB support using the WebUSB
backend included in upstream nusb. The target automatically selects the async
WASM API and browser dependencies.

Build for the WASM target:

```sh
rustup target add wasm32-unknown-unknown
RUSTFLAGS='--cfg=web_sys_unstable_apis' \
  cargo build --target wasm32-unknown-unknown
```

The `web_sys_unstable_apis` cfg is required because WebUSB bindings are still
marked unstable in `web-sys`.

On WASM, browser security requires a user gesture to open the WebUSB device
picker. Use the async WASM constructors instead of native device discovery:

```rust,no_run
use ftdi_nusb::{FtdiDevice, Interface};

# async fn example() -> ftdi_nusb::Result<()> {
let device = FtdiDevice::request_device().await?;
let mut dev = FtdiDevice::open_wasm(device, Interface::A).await?;

dev.set_baudrate(115_200).await?;
dev.write_all(b"Hello from WebUSB!\r\n").await?;
# Ok(())
# }
```

## Feature Flags

| Feature        | Description                                 |
|----------------|---------------------------------------------|
| `embedded-hal` | Native `embedded-hal` trait implementations |
| `smol`         | Native async via nusb's smol integration    |
| `tokio`        | Native async via nusb's Tokio integration   |

`FtdiDevice` is the asynchronous implementation on every target. Native
targets additionally expose `blocking::FtdiDevice`, a thin wrapper that drives
the serial, EEPROM, and configuration APIs to completion internally.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT License ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
