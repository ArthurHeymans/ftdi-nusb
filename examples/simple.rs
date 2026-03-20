//! Simple serial I/O example.
//!
//! Opens the first FT232R/BM device, sends "Hello!\r\n", and reads back
//! any data received.
//!
//! Usage: cargo run --example simple

use std::io::Read;
use std::time::Duration;

use ftdi_nusb::constants::{pid, FTDI_VID};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    println!("Opening FTDI device...");
    let mut dev = ftdi_nusb::FtdiDevice::open(FTDI_VID, pid::FT232)?;
    println!("Opened: {:?}", dev);

    dev.set_baudrate(115200)?;
    dev.set_line_property(
        ftdi_nusb::DataBits::Eight,
        ftdi_nusb::StopBits::One,
        ftdi_nusb::Parity::None,
    )?;
    dev.set_flow_control(ftdi_nusb::FlowControl::Disabled)?;
    println!("Configured: 115200 8N1");

    // Write some data
    let msg = b"Hello from rs-ftdi!\r\n";
    dev.write_all(msg)?;
    println!("Sent {} bytes", msg.len());

    // Try to read back
    dev.set_read_timeout(Duration::from_secs(2));
    let mut buf = [0u8; 256];
    match dev.read(&mut buf) {
        Ok(n) if n > 0 => {
            println!("Received {} bytes: {:?}", n, &buf[..n]);
        }
        Ok(_) => {
            println!("No data received (timeout).");
        }
        Err(e) => {
            println!("Read error: {e}");
        }
    }

    Ok(())
}
