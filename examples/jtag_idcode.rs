//! JTAG IDCODE reader example.
//!
//! Resets the JTAG TAP and reads the IDCODE register from a target device
//! connected to an FT232H.
//!
//! # Wiring
//!
//! | FT232H Pin | JTAG Signal |
//! |------------|------------|
//! | ADBUS0 (SK) | TCK       |
//! | ADBUS1 (DO) | TDI       |
//! | ADBUS2 (DI) | TDO       |
//! | ADBUS3      | TMS       |
//!
//! # Usage
//!
//! ```sh
//! cargo run --example jtag_idcode
//! ```

use ftdi_nusb::mpsse::MpsseContext;
use ftdi_nusb::mpsse::jtag::JtagBus;

fn main() -> Result<(), ftdi_nusb::Error> {
    env_logger::init();
    futures_lite::future::block_on(run())
}

async fn run() -> Result<(), ftdi_nusb::Error> {
    println!("Opening FT232H...");
    let mut dev = ftdi_nusb::blocking::FtdiDevice::open(0x0403, 0x6014)?.into_async();
    println!("Chip: {:?}", dev.chip_type());

    // Initialize MPSSE at 1 MHz TCK
    let mut mpsse = MpsseContext::init(&mut dev, 1_000_000).await?;
    println!("MPSSE clock: {} Hz", mpsse.clock_hz());

    let mut s = mpsse.session(&mut dev)?;

    // Configure JTAG bus
    let mut jtag = JtagBus::new(&mut s).await?;
    println!("JTAG TAP state: {:?}", jtag.state());

    // Reset the TAP state machine
    jtag.reset(&mut s).await?;
    println!("TAP reset -> {:?}", jtag.state());

    // After reset, the default DR is usually IDCODE (32 bits).
    // Navigate to Shift-DR and read 32 bits.
    let idcode = jtag.shift_dr(&mut s, &[0; 4], 32).await?;
    println!("TAP state after shift: {:?}", jtag.state());

    // Parse IDCODE fields (IEEE 1149.1)
    let id = u32::from_le_bytes([idcode[0], idcode[1], idcode[2], idcode[3]]);
    println!("\nIDCODE: 0x{:08X}", id);
    println!("  Version:      0x{:X}", (id >> 28) & 0xF);
    println!("  Part number:  0x{:04X}", (id >> 12) & 0xFFFF);
    println!("  Manufacturer: 0x{:03X}", (id >> 1) & 0x7FF);
    println!("  LSB (must be 1): {}", id & 1);

    if id == 0 || id == 0xFFFFFFFF {
        println!("\nWarning: IDCODE is all-zeros or all-ones.");
        println!("Check wiring and ensure the target is powered.");
    }

    // Scan the chain for multiple devices
    println!("\nScanning chain (up to 8 devices)...");
    jtag.reset(&mut s).await?;
    let chain = jtag.shift_dr(&mut s, &[0; 32], 256).await?;

    for i in 0..8 {
        let offset = i * 4;
        let dev_id = u32::from_le_bytes([
            chain[offset],
            chain[offset + 1],
            chain[offset + 2],
            chain[offset + 3],
        ]);
        if dev_id == 0 || dev_id == 0xFFFFFFFF {
            if i == 0 {
                println!("  No devices detected in chain.");
            }
            break;
        }
        println!("  Device {}: IDCODE = 0x{:08X}", i, dev_id);
    }

    println!("\nDone.");
    Ok(())
}
