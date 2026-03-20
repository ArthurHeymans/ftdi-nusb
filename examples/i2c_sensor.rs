//! I2C temperature sensor example.
//!
//! Reads temperature from a TMP102 (or compatible) I2C sensor
//! connected to an FT232H.
//!
//! # Wiring
//!
//! | FT232H Pin | I2C Signal | Notes |
//! |------------|-----------|-------|
//! | ADBUS0 (SK) | SCL      | Pull-up to 3.3V via 4.7k |
//! | ADBUS1 (DO) | SDA      | Connect to ADBUS2, pull-up via 4.7k |
//! | ADBUS2 (DI) | SDA      | Connected to ADBUS1 externally |
//!
//! # Usage
//!
//! ```sh
//! cargo run --example i2c_sensor
//! ```

use ftdi_nusb::mpsse::i2c::I2cBus;
use ftdi_nusb::mpsse::MpsseContext;
use ftdi_nusb::FtdiDevice;

/// Default TMP102 I2C address (ADD0 = GND).
const TMP102_ADDR: u8 = 0x48;

/// TMP102 register addresses.
const REG_TEMPERATURE: u8 = 0x00;
const REG_CONFIGURATION: u8 = 0x01;

fn main() -> Result<(), ftdi_nusb::Error> {
    env_logger::init();

    println!("Opening FT232H...");
    let mut dev = FtdiDevice::open(0x0403, 0x6014)?;
    println!("Chip: {:?}", dev.chip_type());

    // Initialize MPSSE at 100 kHz I2C clock
    let mut mpsse = MpsseContext::init(&mut dev, 100_000)?;
    println!("MPSSE clock: {} Hz (I2C standard mode)", mpsse.clock_hz());

    // Configure I2C bus
    let i2c = I2cBus::new(&mut mpsse, &mut dev)?;
    println!("I2C bus initialized");

    // Read configuration register (2 bytes)
    let config = i2c.write_read(&mut mpsse, &mut dev, TMP102_ADDR, &[REG_CONFIGURATION], 2)?;
    println!(
        "TMP102 config register: 0x{:02X}{:02X}",
        config[0], config[1]
    );

    // Read temperature register (2 bytes)
    let temp_raw = i2c.write_read(&mut mpsse, &mut dev, TMP102_ADDR, &[REG_TEMPERATURE], 2)?;
    let raw_value = ((temp_raw[0] as i16) << 4) | ((temp_raw[1] as i16) >> 4);

    // TMP102: 12-bit resolution, 0.0625 degrees C per LSB
    let temperature = raw_value as f32 * 0.0625;
    println!(
        "Temperature: {:.2} C (raw: 0x{:02X}{:02X})",
        temperature, temp_raw[0], temp_raw[1]
    );

    // Continuous reading loop (5 samples)
    println!("\nContinuous reading (5 samples, 1s interval):");
    for i in 0..5 {
        let data = i2c.write_read(&mut mpsse, &mut dev, TMP102_ADDR, &[REG_TEMPERATURE], 2)?;
        let raw = ((data[0] as i16) << 4) | ((data[1] as i16) >> 4);
        let temp = raw as f32 * 0.0625;
        println!("  Sample {}: {:.2} C", i + 1, temp);
        std::thread::sleep(std::time::Duration::from_secs(1));
    }

    println!("Done.");
    Ok(())
}
