//! Blocking wrapper around the asynchronous FTDI device implementation.

use core::time::Duration;

use nusb::MaybeFuture;

use crate::context::AsyncFtdiDevice;
use crate::error::{Error, Result};
use crate::types::*;

/// A blocking FTDI device handle for native applications.
///
/// Use [`AsyncFtdiDevice`] directly from asynchronous applications.
#[derive(Debug)]
pub struct FtdiDevice(AsyncFtdiDevice);

impl FtdiDevice {
    pub fn open(vendor: u16, product: u16) -> Result<Self> {
        Self::open_with_interface(vendor, product, Interface::Any)
    }

    pub fn open_with_interface(vendor: u16, product: u16, iface: Interface) -> Result<Self> {
        let info = nusb::list_devices()
            .wait()?
            .find(|d| d.vendor_id() == vendor && d.product_id() == product)
            .ok_or(Error::DeviceNotFound)?;
        Self::from_device_info(info, iface)
    }

    pub fn open_with_filter(
        filter: &crate::device_info::DeviceFilter,
        iface: Interface,
    ) -> Result<Self> {
        Self::from_device_info(crate::device_info::find_device(filter)?, iface)
    }

    #[cfg(target_os = "linux")]
    pub fn open_bus_addr(bus: u8, addr: u8, iface: Interface) -> Result<Self> {
        let info = nusb::list_devices()
            .wait()?
            .find(|d| d.busnum() == bus && d.device_address() == addr)
            .ok_or(Error::DeviceNotFound)?;
        Self::from_device_info(info, iface)
    }

    pub fn from_device_info(info: nusb::DeviceInfo, iface: Interface) -> Result<Self> {
        let config = iface.config();
        let device = info.open().wait()?;
        let interface = device
            .detach_and_claim_interface(config.interface_num)
            .wait()?;
        let mut inner = AsyncFtdiDevice::from_open_device(device, interface, iface)?;
        futures_lite::future::block_on(inner.initialize())?;
        Ok(Self(inner))
    }

    pub fn into_async(self) -> AsyncFtdiDevice {
        self.0
    }

    pub(crate) fn as_async_mut(&mut self) -> &mut AsyncFtdiDevice {
        &mut self.0
    }
}

impl AsyncFtdiDevice {
    /// Wrap this async device in the native blocking API.
    pub fn into_blocking(self) -> FtdiDevice {
        FtdiDevice(self)
    }
}

macro_rules! forward_ref {
    ($(fn $name:ident(&self $(, $arg:ident: $ty:ty)*) -> $ret:ty;)*) => {$(
        pub fn $name(&self, $($arg: $ty),*) -> $ret {
            futures_lite::future::block_on(self.0.$name($($arg),*))
        }
    )*};
}

macro_rules! forward_mut {
    ($(fn $name:ident(&mut self $(, $arg:ident: $ty:ty)*) -> $ret:ty;)*) => {$(
        pub fn $name(&mut self, $($arg: $ty),*) -> $ret {
            futures_lite::future::block_on(self.0.$name($($arg),*))
        }
    )*};
}

impl FtdiDevice {
    /// The detected FTDI chip type.
    pub fn chip_type(&self) -> ChipType {
        self.0.chip_type()
    }

    /// The currently configured baud rate.
    pub fn baudrate(&self) -> u32 {
        self.0.baudrate()
    }

    /// The USB maximum packet size for this device.
    pub fn max_packet_size(&self) -> usize {
        self.0.max_packet_size()
    }

    pub fn set_read_timeout(&mut self, timeout: Duration) {
        self.0.set_read_timeout(timeout);
    }

    pub fn set_write_timeout(&mut self, timeout: Duration) {
        self.0.set_write_timeout(timeout);
    }

    pub fn read_timeout(&self) -> Duration {
        self.0.read_timeout()
    }

    pub fn write_timeout(&self) -> Duration {
        self.0.write_timeout()
    }

    pub fn set_read_chunksize(&mut self, chunksize: usize) {
        self.0.set_read_chunksize(chunksize);
    }

    pub fn read_chunksize(&self) -> usize {
        self.0.read_chunksize()
    }

    pub fn set_write_chunksize(&mut self, chunksize: usize) {
        self.0.set_write_chunksize(chunksize);
    }

    pub fn write_chunksize(&self) -> usize {
        self.0.write_chunksize()
    }

    pub fn eeprom_decode(&mut self) -> Result<()> {
        self.0.eeprom_decode()
    }

    pub fn eeprom_init_defaults(
        &mut self,
        manufacturer: Option<&str>,
        product: Option<&str>,
        serial: Option<&str>,
    ) {
        self.0.eeprom_init_defaults(manufacturer, product, serial);
    }

    pub fn eeprom(&self) -> &crate::FtdiEeprom {
        self.0.eeprom()
    }

    pub fn eeprom_mut(&mut self) -> &mut crate::FtdiEeprom {
        self.0.eeprom_mut()
    }

    forward_mut! {
        fn usb_reset(&mut self) -> Result<()>;
        fn flush_rx(&mut self) -> Result<()>;
        fn flush_tx(&mut self) -> Result<()>;
        fn flush_all(&mut self) -> Result<()>;
        fn set_baudrate(&mut self, baudrate: u32) -> Result<()>;
        fn set_bitmode(&mut self, bitmask: u8, mode: BitMode) -> Result<()>;
        fn disable_bitbang(&mut self) -> Result<()>;
        fn write_data(&mut self, buf: &[u8]) -> Result<usize>;
        fn read_data(&mut self, buf: &mut [u8]) -> Result<usize>;
        fn write_all(&mut self, buf: &[u8]) -> Result<()>;
        fn read_data_retry(&mut self, buf: &mut [u8], max_retries: usize, retry_delay: Duration) -> Result<usize>;
        fn write_data_retry(&mut self, buf: &[u8], max_retries: usize, retry_delay: Duration) -> Result<usize>;
        fn recover(&mut self) -> Result<()>;
        fn read_eeprom(&mut self) -> Result<()>;
        fn write_eeprom(&mut self) -> Result<()>;
        fn erase_eeprom(&mut self) -> Result<()>;
        fn eeprom_build(&mut self) -> Result<usize>;
    }

    forward_ref! {
        fn set_line_property(&self, bits: DataBits, stop_bits: StopBits, parity: Parity) -> Result<()>;
        fn set_line_property_with_break(&self, bits: DataBits, stop_bits: StopBits, parity: Parity, break_type: BreakType) -> Result<()>;
        fn set_flow_control(&self, flow: FlowControl) -> Result<()>;
        fn set_flow_control_xonxoff(&self, xon: u8, xoff: u8) -> Result<()>;
        fn set_dtr(&self, state: bool) -> Result<()>;
        fn set_rts(&self, state: bool) -> Result<()>;
        fn set_dtr_rts(&self, dtr: bool, rts: bool) -> Result<()>;
        fn set_event_char(&self, ch: u8, enable: bool) -> Result<()>;
        fn set_error_char(&self, ch: u8, enable: bool) -> Result<()>;
        fn poll_modem_status(&self) -> Result<ModemStatus>;
        fn set_latency_timer(&self, latency_ms: u8) -> Result<()>;
        fn latency_timer(&self) -> Result<u8>;
        fn read_pins(&self) -> Result<u8>;
        fn is_connected(&self) -> bool;
        fn read_eeprom_location(&self, addr: u16) -> Result<u16>;
        fn write_eeprom_location(&self, addr: u16, value: u16) -> Result<()>;
        fn read_chipid(&self) -> Result<u32>;
    }
}

impl std::io::Read for FtdiDevice {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.read_data(buf).map_err(std::io::Error::other)
    }
}

impl std::io::Write for FtdiDevice {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.write_data(buf).map_err(std::io::Error::other)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.flush_tx().map_err(std::io::Error::other)
    }
}
