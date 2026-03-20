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
- **Async transfers** — non-blocking USB read/write with submit/complete pattern
- **Streaming** — high-throughput continuous reads via concurrent USB transfers (FT2232H/FT232H)
- **Device discovery** — enumerate and filter by VID/PID/serial/description

## Quick Start

```rust,no_run
use ftdi_nusb::{FtdiDevice, constants::FTDI_VID, constants::pid};

// Open the first FT232R connected
let mut dev = FtdiDevice::open(FTDI_VID, pid::FT232)?;
dev.set_baudrate(115200)?;
dev.write_all(b"Hello from Rust!\r\n")?;
# Ok::<(), ftdi_nusb::Error>(())
```

### SPI

```rust,no_run
use ftdi_nusb::{FtdiDevice, mpsse::{MpsseContext, spi::{SpiDevice, SpiMode}}};

let mut dev = FtdiDevice::open(0x0403, 0x6014)?; // FT232H
let mut mpsse = MpsseContext::init(&mut dev, 1_000_000)?;
let spi = SpiDevice::new(&mut mpsse, &mut dev, SpiMode::Mode0)?;

// Read JEDEC ID from SPI flash
let id = spi.transfer(&mut mpsse, &mut dev, &[0x9F, 0, 0, 0])?;
# Ok::<(), ftdi_nusb::Error>(())
```

### I2C

```rust,no_run
use ftdi_nusb::{FtdiDevice, mpsse::{MpsseContext, i2c::I2cBus}};

let mut dev = FtdiDevice::open(0x0403, 0x6014)?;
let mut mpsse = MpsseContext::init(&mut dev, 100_000)?; // 100 kHz
let i2c = I2cBus::new(&mut mpsse, &mut dev)?;

// Write register address, read 2 bytes from I2C device at 0x48
let data = i2c.write_read(&mut mpsse, &mut dev, 0x48, &[0x00], 2)?;
# Ok::<(), ftdi_nusb::Error>(())
```

### JTAG

```rust,no_run
use ftdi_nusb::{FtdiDevice, mpsse::{MpsseContext, jtag::JtagBus}};

let mut dev = FtdiDevice::open(0x0403, 0x6014)?;
let mut mpsse = MpsseContext::init(&mut dev, 1_000_000)?;
let mut jtag = JtagBus::new(&mut mpsse, &mut dev)?;

// Reset TAP and read IDCODE
jtag.reset(&mut dev)?;
let idcode = jtag.shift_dr(&mpsse, &mut dev, &[0; 4], 32)?;
# Ok::<(), ftdi_nusb::Error>(())
```

### EEPROM

```rust,no_run
use ftdi_nusb::FtdiDevice;

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
| Async transfers                | Yes         | Yes                    |
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

On Linux, you may need to detach the `ftdi_sio` kernel driver. The library
handles this automatically via nusb's `detach_and_claim_interface()`.

## Feature Flags

| Feature        | Description                                            |
|----------------|--------------------------------------------------------|
| `std` (default)| Native synchronous build (Linux/macOS/Windows)         |
| `is_sync`      | Enabled automatically by `std`; makes all I/O blocking |
| `embedded-hal` | `embedded-hal` trait implementations                   |

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT License ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
