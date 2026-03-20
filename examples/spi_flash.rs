//! SPI flash interaction example.
//!
//! Reads the JEDEC ID and status register from an SPI flash chip
//! (e.g., W25Q32, AT25SF, MX25L) connected to an FT232H.
//!
//! # Wiring
//!
//! | FT232H Pin | SPI Flash Pin |
//! |------------|---------------|
//! | ADBUS0 (SK) | CLK          |
//! | ADBUS1 (DO) | MOSI / DI    |
//! | ADBUS2 (DI) | MISO / DO    |
//! | ADBUS3      | CS#          |
//!
//! # Usage
//!
//! ```sh
//! cargo run --example spi_flash
//! ```

use ftdi_nusb::mpsse::spi::{SpiDevice, SpiMode};
use ftdi_nusb::mpsse::MpsseContext;
use ftdi_nusb::FtdiDevice;

fn main() -> Result<(), ftdi_nusb::Error> {
    env_logger::init();

    println!("Opening FT232H...");
    let mut dev = FtdiDevice::open(0x0403, 0x6014)?;
    println!("Chip: {:?}", dev.chip_type());

    // Initialize MPSSE at 1 MHz SPI clock
    let mut mpsse = MpsseContext::init(&mut dev, 1_000_000)?;
    println!("MPSSE clock: {} Hz", mpsse.clock_hz());

    // Configure SPI Mode 0 with default CS on ADBUS3
    let spi = SpiDevice::new(&mut mpsse, &mut dev, SpiMode::Mode0)?;
    println!("SPI configured in Mode 0");

    // Read JEDEC ID (command 0x9F, returns 3 bytes)
    let jedec = spi.transfer(&mut mpsse, &mut dev, &[0x9F, 0x00, 0x00, 0x00])?;
    println!(
        "JEDEC ID: manufacturer=0x{:02X}, memory_type=0x{:02X}, capacity=0x{:02X}",
        jedec[1], jedec[2], jedec[3]
    );

    // Read Status Register 1 (command 0x05, returns 1 byte)
    let status = spi.transfer(&mut mpsse, &mut dev, &[0x05, 0x00])?;
    println!("Status Register 1: 0x{:02X}", status[1]);
    println!("  WEL (Write Enable Latch): {}", (status[1] & 0x02) != 0);
    println!("  BUSY: {}", (status[1] & 0x01) != 0);

    // Read first 16 bytes of flash (command 0x03 + 3-byte address)
    let mut read_cmd = vec![0x03, 0x00, 0x00, 0x00]; // Read from address 0
    read_cmd.extend_from_slice(&[0x00; 16]); // 16 dummy bytes for read
    let data = spi.transfer(&mut mpsse, &mut dev, &read_cmd)?;
    println!("Flash data at 0x000000:");
    for (i, byte) in data[4..].iter().enumerate() {
        print!("{:02X} ", byte);
        if (i + 1) % 16 == 0 {
            println!();
        }
    }

    println!("\nDone.");
    Ok(())
}
