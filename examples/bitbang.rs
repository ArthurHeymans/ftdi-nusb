//! Bitbang mode example.
//!
//! Toggles all pins on a connected FT232R in asynchronous bitbang mode.
//! Connect LEDs (with appropriate resistors) to observe the output.
//!
//! Usage: cargo run --example bitbang

use std::thread;
use std::time::Duration;

use ftdi_nusb::constants::{pid, FTDI_VID};
use ftdi_nusb::{BitMode, FtdiDevice};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    println!("Opening FTDI device for bitbang...");
    let mut dev = FtdiDevice::open(FTDI_VID, pid::FT232)?;
    println!("Chip type: {:?}", dev.chip_type());

    // Enable async bitbang mode, all pins as outputs
    dev.set_bitmode(0xFF, BitMode::BitBang)?;
    dev.set_baudrate(9600)?; // Controls the bitbang clock rate (4x baud)
    println!("Bitbang mode enabled (all outputs, 9600 baud clock)");

    // Toggle pins
    for cycle in 0..10u32 {
        let val = if cycle % 2 == 0 { 0xFF } else { 0x00 };
        dev.write_data(&[val])?;

        let pins = dev.read_pins()?;
        println!("Cycle {cycle}: wrote 0x{val:02X}, pins=0x{pins:02X}");

        thread::sleep(Duration::from_millis(500));
    }

    // Return to normal mode
    dev.disable_bitbang()?;
    println!("Bitbang disabled.");

    Ok(())
}
