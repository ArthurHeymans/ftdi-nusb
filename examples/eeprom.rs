//! EEPROM read and decode example.
//!
//! Reads the EEPROM from the first connected FTDI device, decodes it,
//! and prints the stored configuration.
//!
//! Usage: cargo run --example eeprom

use ftdi_nusb::constants::{pid, FTDI_VID};
use ftdi_nusb::FtdiDevice;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    println!("Opening FTDI device...");
    let mut dev = FtdiDevice::open(FTDI_VID, pid::FT232)?;
    println!("Chip type: {:?}", dev.chip_type());

    // Read the EEPROM
    println!("Reading EEPROM...");
    dev.read_eeprom()?;

    let eeprom = dev.eeprom();
    println!("EEPROM size: {} bytes", eeprom.size);
    println!("EEPROM chip: 0x{:02X}", eeprom.chip);

    // Decode the EEPROM
    dev.eeprom_decode()?;

    let eeprom = dev.eeprom();
    println!("\nDecoded EEPROM contents:");
    println!("  Vendor ID:    0x{:04X}", eeprom.vendor_id);
    println!("  Product ID:   0x{:04X}", eeprom.product_id);
    println!("  Release:      0x{:04X}", eeprom.release_number);
    println!("  Self-powered: {}", eeprom.self_powered);
    println!("  Max power:    {} mA", eeprom.max_power);
    println!("  Use serial:   {}", eeprom.use_serial);

    let (mfr, prod, ser) = eeprom.strings();
    println!("  Manufacturer: {}", mfr.unwrap_or("(none)"));
    println!("  Product:      {}", prod.unwrap_or("(none)"));
    println!("  Serial:       {}", ser.unwrap_or("(none)"));

    // Read chip ID (FT232R only)
    if dev.chip_type() == ftdi_nusb::ChipType::Ft232R {
        match dev.read_chipid() {
            Ok(id) => println!("  Chip ID:      0x{id:08X}"),
            Err(e) => println!("  Chip ID:      (error: {e})"),
        }
    }

    // Hex dump of first 32 bytes
    println!("\nRaw EEPROM (first 32 bytes):");
    let buf = eeprom.raw_buf();
    for (i, chunk) in buf[..32].chunks(16).enumerate() {
        print!("  {:04X}: ", i * 16);
        for b in chunk {
            print!("{b:02X} ");
        }
        println!();
    }

    Ok(())
}
