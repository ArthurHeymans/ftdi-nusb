//! List all connected FTDI devices.
//!
//! Usage: cargo run --example find_all

use ftdi_nusb::constants::{pid, FTDI_VID};

fn main() -> Result<(), ftdi_nusb::Error> {
    env_logger::init();

    let pids = [
        ("FT232/FT245", pid::FT232),
        ("FT2232", pid::FT2232),
        ("FT4232H", pid::FT4232),
        ("FT232H", pid::FT232H),
        ("FT230X", pid::FT230X),
    ];

    let mut found_any = false;

    for (name, product_id) in &pids {
        let devices = ftdi_nusb::find_devices(FTDI_VID, *product_id)?;
        for dev in &devices {
            found_any = true;
            #[cfg(target_os = "linux")]
            println!(
                "{name}: bus={} addr={} vid={:#06x} pid={:#06x}",
                dev.busnum(),
                dev.device_address(),
                dev.vendor_id(),
                dev.product_id(),
            );
            #[cfg(not(target_os = "linux"))]
            println!(
                "{name}: addr={} vid={:#06x} pid={:#06x}",
                dev.device_address(),
                dev.vendor_id(),
                dev.product_id(),
            );
        }
    }

    if !found_any {
        println!("No FTDI devices found.");
    }

    Ok(())
}
