//! Device discovery and enumeration.
//!
//! Use [`find_devices`] to list connected FTDI devices, or [`DeviceFilter`]
//! for more precise matching by description, serial number, or device index.

use nusb::{self, DeviceInfo, MaybeFuture};
use std::time::Duration;

use crate::error::{Error, Result};

/// Filtering criteria for finding FTDI devices.
///
/// All fields beyond `vendor_id` and `product_id` are optional. When set,
/// they further restrict which devices match.
///
/// # Example
///
/// ```no_run
/// use ftdi_nusb::DeviceFilter;
///
/// let filter = DeviceFilter::new(0x0403, 0x6001)
///     .serial("FT123456")
///     .index(0);
/// ```
#[derive(Debug, Clone)]
pub struct DeviceFilter {
    /// USB vendor ID to match.
    pub vendor_id: u16,
    /// USB product ID to match.
    pub product_id: u16,
    /// If set, match against the USB product description string.
    pub description: Option<String>,
    /// If set, match against the USB serial number string.
    pub serial: Option<String>,
    /// Select the Nth matching device (0-based). Defaults to 0.
    pub index: usize,
}

impl DeviceFilter {
    /// Create a new filter matching the given vendor and product IDs.
    pub fn new(vendor_id: u16, product_id: u16) -> Self {
        Self {
            vendor_id,
            product_id,
            description: None,
            serial: None,
            index: 0,
        }
    }

    /// Require the product description to match.
    pub fn description(mut self, desc: impl Into<String>) -> Self {
        self.description = Some(desc.into());
        self
    }

    /// Require the serial number to match.
    pub fn serial(mut self, serial: impl Into<String>) -> Self {
        self.serial = Some(serial.into());
        self
    }

    /// Select the Nth matching device (0-based).
    pub fn index(mut self, index: usize) -> Self {
        self.index = index;
        self
    }
}

/// USB string descriptor read timeout.
const STRING_TIMEOUT: Duration = Duration::from_secs(1);

/// List all connected FTDI devices matching the given vendor and product IDs.
///
/// Returns a `Vec` of [`nusb::DeviceInfo`] for each matching device.
///
/// # Example
///
/// ```no_run
/// use ftdi_nusb::{find_devices, constants::FTDI_VID};
///
/// let devices = find_devices(FTDI_VID, 0x6001).unwrap();
/// for dev in &devices {
///     println!("Found: vid={:#06x} pid={:#06x}", dev.vendor_id(), dev.product_id());
/// }
/// ```
pub fn find_devices(vendor: u16, product: u16) -> Result<Vec<DeviceInfo>> {
    let devices: Vec<DeviceInfo> = nusb::list_devices()
        .wait()?
        .filter(|d| d.vendor_id() == vendor && d.product_id() == product)
        .collect();
    Ok(devices)
}

/// Find a single device matching the given filter criteria.
///
/// This opens each candidate temporarily to read string descriptors for
/// matching when `description` or `serial` filters are set.
pub fn find_device(filter: &DeviceFilter) -> Result<DeviceInfo> {
    let candidates: Vec<DeviceInfo> = nusb::list_devices()
        .wait()?
        .filter(|d| d.vendor_id() == filter.vendor_id && d.product_id() == filter.product_id)
        .collect();

    let mut match_count = 0usize;

    for dev_info in candidates {
        // If we need to match strings, we have to open the device temporarily
        if filter.description.is_some() || filter.serial.is_some() {
            let device = dev_info.open().wait()?;
            let desc = device.device_descriptor();

            if let Some(ref expected_desc) = filter.description {
                if let Some(idx) = desc.product_string_index() {
                    let product = device
                        .get_string_descriptor(idx, 0x0409, STRING_TIMEOUT)
                        .wait()
                        .unwrap_or_default();
                    if product != *expected_desc {
                        continue;
                    }
                } else {
                    continue;
                }
            }

            if let Some(ref expected_serial) = filter.serial {
                if let Some(idx) = desc.serial_number_string_index() {
                    let serial = device
                        .get_string_descriptor(idx, 0x0409, STRING_TIMEOUT)
                        .wait()
                        .unwrap_or_default();
                    if serial != *expected_serial {
                        continue;
                    }
                } else {
                    continue;
                }
            }
        }

        if match_count == filter.index {
            return Ok(dev_info);
        }
        match_count += 1;
    }

    Err(Error::DeviceNotFound)
}
