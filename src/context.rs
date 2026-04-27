//! Core FTDI device handle and operations.
//!
//! [`FtdiDevice`] is the main type in this crate. It represents an opened,
//! configured FTDI USB device and provides methods for serial communication,
//! bitbang/MPSSE mode, flow control, and EEPROM access.

use core::time::Duration;

use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};
#[cfg(feature = "is_sync")]
use nusb::MaybeFuture;

use maybe_async::maybe_async;

use crate::baudrate;
use crate::constants::*;
use crate::eeprom::FtdiEeprom;
use crate::error::{Error, Result};
use crate::types::*;

/// Default read/write timeout.
const DEFAULT_TIMEOUT: Duration = Duration::from_secs(5);

/// Default read/write buffer chunk size.
const DEFAULT_CHUNKSIZE: usize = 4096;

/// Macro for synchronous/asynchronous endpoint completion.
///
/// In sync mode (`is_sync`), uses `wait_next_complete` with a timeout.
/// In async mode (WASM), uses `.next_complete().await`.
macro_rules! ep_wait {
    ($ep:expr, $timeout:expr) => {{
        #[cfg(feature = "is_sync")]
        {
            $ep.wait_next_complete($timeout)
        }
        #[cfg(not(feature = "is_sync"))]
        {
            Some($ep.next_complete().await)
        }
    }};
}

/// Macro for synchronous/asynchronous completion of nusb `MaybeFuture` values.
///
/// nusb control transfer methods return `impl MaybeFuture + IntoFuture`.
/// In sync mode, we must call `.wait()` to resolve them synchronously.
/// In async mode, we use `.await`.
macro_rules! nusb_await {
    ($expr:expr) => {{
        #[cfg(feature = "is_sync")]
        {
            $expr.wait()
        }
        #[cfg(not(feature = "is_sync"))]
        {
            $expr.await
        }
    }};
}

/// An opened FTDI USB device.
///
/// This is the primary handle for communicating with an FTDI chip.
/// It owns the USB device and interface, manages internal read buffers,
/// and provides methods for all supported operations.
///
/// # Opening a device
///
/// ```no_run
/// use ftdi_nusb::FtdiDevice;
///
/// let mut dev = FtdiDevice::open(0x0403, 0x6001)?;
/// dev.set_baudrate(115200)?;
/// dev.write_all(b"Hello FTDI!\r\n")?;
/// # Ok::<(), ftdi_nusb::Error>(())
/// ```
///
/// # Implements `Read` and `Write`
///
/// On native builds, `FtdiDevice` implements [`std::io::Read`] and [`std::io::Write`],
/// so you can use it anywhere those traits are expected.
pub struct FtdiDevice {
    #[allow(dead_code)] // Kept to ensure the USB device stays open
    device: nusb::Device,
    interface: nusb::Interface,

    // Bulk endpoints — stored as struct fields for both native and WASM
    write_endpoint: nusb::Endpoint<nusb::transfer::Bulk, nusb::transfer::Out>,
    read_endpoint: nusb::Endpoint<nusb::transfer::Bulk, nusb::transfer::In>,

    // Chip identification
    chip_type: ChipType,

    // Transfer configuration
    baudrate: u32,
    bitbang_enabled: bool,
    bitbang_mode: BitMode,
    read_timeout: Duration,
    write_timeout: Duration,

    // Internal read buffer (modem status bytes are stripped here)
    readbuffer: Vec<u8>,
    readbuffer_offset: usize,
    readbuffer_remaining: usize,
    readbuffer_chunksize: usize,
    writebuffer_chunksize: usize,

    // USB endpoint configuration
    max_packet_size: usize,
    interface_num: u8,
    usb_index: u16,
    #[cfg_attr(all(feature = "wasm", not(feature = "std")), allow(dead_code))]
    write_ep: u8, // bulk OUT endpoint address
    #[cfg_attr(all(feature = "wasm", not(feature = "std")), allow(dead_code))]
    read_ep: u8, // bulk IN endpoint address

    // EEPROM
    pub(crate) eeprom: FtdiEeprom,
}

impl core::fmt::Debug for FtdiDevice {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("FtdiDevice")
            .field("chip_type", &self.chip_type)
            .field("baudrate", &self.baudrate)
            .field("interface", &self.interface_num)
            .field("bitbang_enabled", &self.bitbang_enabled)
            .field("max_packet_size", &self.max_packet_size)
            .finish_non_exhaustive()
    }
}

// ---- Native-only construction / Opening ----

#[cfg(feature = "std")]
impl FtdiDevice {
    /// Open the first FTDI device matching the given vendor and product IDs.
    ///
    /// Uses [`Interface::A`] by default. For multi-interface chips, use
    /// [`open_with_interface`](Self::open_with_interface).
    pub fn open(vendor: u16, product: u16) -> Result<Self> {
        Self::open_with_interface(vendor, product, Interface::Any)
    }

    /// Open the first matching device on a specific interface.
    pub fn open_with_interface(vendor: u16, product: u16, iface: Interface) -> Result<Self> {
        let dev_info = nusb::list_devices()
            .wait()?
            .find(|d| d.vendor_id() == vendor && d.product_id() == product)
            .ok_or(Error::DeviceNotFound)?;

        Self::from_device_info(dev_info, iface)
    }

    /// Open a device from a [`DeviceFilter`](crate::DeviceFilter).
    pub fn open_with_filter(
        filter: &crate::device_info::DeviceFilter,
        iface: Interface,
    ) -> Result<Self> {
        let dev_info = crate::device_info::find_device(filter)?;
        Self::from_device_info(dev_info, iface)
    }

    /// Open a device by USB bus number and device address.
    ///
    /// This function is only available on Linux, where USB bus numbers
    /// are exposed by the kernel.
    #[cfg(target_os = "linux")]
    pub fn open_bus_addr(bus: u8, addr: u8, iface: Interface) -> Result<Self> {
        let dev_info = nusb::list_devices()
            .wait()?
            .find(|d| d.busnum() == bus && d.device_address() == addr)
            .ok_or(Error::DeviceNotFound)?;

        Self::from_device_info(dev_info, iface)
    }

    /// Open a device from an already-discovered [`nusb::DeviceInfo`].
    pub fn from_device_info(dev_info: nusb::DeviceInfo, iface: Interface) -> Result<Self> {
        let config = iface.config();

        let device = dev_info.open().wait()?;

        // Detach kernel driver and claim interface
        let interface = device
            .detach_and_claim_interface(config.interface_num)
            .wait()?;

        let write_endpoint = interface
            .endpoint::<nusb::transfer::Bulk, nusb::transfer::Out>(config.write_ep)
            .map_err(Error::Usb)?;
        let read_endpoint = interface
            .endpoint::<nusb::transfer::Bulk, nusb::transfer::In>(config.read_ep)
            .map_err(Error::Usb)?;

        // Auto-detect chip type from bcdDevice
        let desc = device.device_descriptor();
        let bcd = desc.device_version();
        let has_serial = desc.serial_number_string_index().is_some();

        let chip_type = detect_chip_type(bcd, has_serial);

        // Determine max packet size from descriptors
        let max_packet_size = determine_max_packet_size(&device, chip_type, config.interface_num);

        // The proprietary FTDI driver uses index=0 for single-channel
        // chips and interface_num+1 for multi-channel chips.
        let usb_index = if chip_type.is_multi_channel() {
            config.usb_index
        } else {
            0
        };

        let mut ftdi = Self {
            device,
            interface,
            write_endpoint,
            read_endpoint,
            chip_type,
            baudrate: 0,
            bitbang_enabled: false,
            bitbang_mode: BitMode::Reset,
            read_timeout: DEFAULT_TIMEOUT,
            write_timeout: DEFAULT_TIMEOUT,
            readbuffer: vec![0u8; DEFAULT_CHUNKSIZE],
            readbuffer_offset: 0,
            readbuffer_remaining: 0,
            readbuffer_chunksize: DEFAULT_CHUNKSIZE,
            writebuffer_chunksize: DEFAULT_CHUNKSIZE,
            max_packet_size,
            interface_num: config.interface_num,
            usb_index,
            write_ep: config.write_ep,
            read_ep: config.read_ep,
            eeprom: FtdiEeprom::default(),
        };

        // Reset device
        ftdi.usb_reset()?;

        // Set default baud rate
        ftdi.set_baudrate(9600)?;

        Ok(ftdi)
    }
}

// ---- WASM-only construction ----

#[cfg(all(feature = "wasm", not(feature = "is_sync")))]
impl FtdiDevice {
    /// Show the browser's WebUSB device picker filtered by common FTDI VID/PIDs.
    ///
    /// Returns a [`nusb::DeviceInfo`] that can be passed to
    /// [`open_wasm`](Self::open_wasm).
    #[cfg(target_arch = "wasm32")]
    pub async fn request_device() -> Result<nusb::DeviceInfo> {
        use wasm_bindgen::JsCast;
        use wasm_bindgen_futures::JsFuture;
        use web_sys::{UsbDevice, UsbDeviceFilter, UsbDeviceRequestOptions};

        let usb = web_sys::window()
            .ok_or(Error::DeviceNotFound)?
            .navigator()
            .usb();

        let filters = js_sys::Array::new();

        // Common FTDI PIDs.
        let pids: &[u16] = &[
            0x6001, // FT232
            0x6010, // FT2232
            0x6011, // FT4232
            0x6014, // FT232H
            0x6015, // FT230X
        ];

        for &pid in pids {
            let filter = UsbDeviceFilter::new();
            filter.set_vendor_id(FTDI_VID);
            filter.set_product_id(pid);
            filters.push(&filter);
        }

        let options = UsbDeviceRequestOptions::new(&filters);

        let device_promise = usb.request_device(&options);
        let device_js = JsFuture::from(device_promise)
            .await
            .map_err(|e| Error::OpenFailed(format!("WebUSB request failed: {:?}", e)))?;

        let device: UsbDevice = device_js
            .dyn_into()
            .map_err(|_| Error::OpenFailed("failed to get USB device".to_string()))?;

        nusb::device_info_from_webusb(device)
            .await
            .map_err(|e| Error::OpenFailed(format!("failed to get device info: {e}")))
    }

    /// Open an FTDI device from a [`nusb::DeviceInfo`] in a WASM/WebUSB build.
    pub async fn open_wasm(dev_info: nusb::DeviceInfo, iface: Interface) -> Result<Self> {
        let config = iface.config();

        let device = dev_info.open().await.map_err(Error::Usb)?;

        let interface = device
            .claim_interface(config.interface_num)
            .await
            .map_err(Error::Usb)?;

        let write_endpoint = interface
            .endpoint::<nusb::transfer::Bulk, nusb::transfer::Out>(config.write_ep)
            .map_err(Error::Usb)?;
        let read_endpoint = interface
            .endpoint::<nusb::transfer::Bulk, nusb::transfer::In>(config.read_ep)
            .map_err(Error::Usb)?;

        // Auto-detect chip type from bcdDevice.
        let desc = device.device_descriptor();
        let bcd = desc.device_version();
        let has_serial = desc.serial_number_string_index().is_some();

        let chip_type = detect_chip_type(bcd, has_serial);

        // Determine max packet size from descriptors.
        let max_packet_size = determine_max_packet_size(&device, chip_type, config.interface_num);

        // The proprietary FTDI driver uses index=0 for single-channel
        // chips and interface_num+1 for multi-channel chips.
        let usb_index = if chip_type.is_multi_channel() {
            config.usb_index
        } else {
            0
        };

        let mut ftdi = Self {
            device,
            interface,
            write_endpoint,
            read_endpoint,
            chip_type,
            baudrate: 0,
            bitbang_enabled: false,
            bitbang_mode: BitMode::Reset,
            read_timeout: DEFAULT_TIMEOUT,
            write_timeout: DEFAULT_TIMEOUT,
            readbuffer: vec![0u8; DEFAULT_CHUNKSIZE],
            readbuffer_offset: 0,
            readbuffer_remaining: 0,
            readbuffer_chunksize: DEFAULT_CHUNKSIZE,
            writebuffer_chunksize: DEFAULT_CHUNKSIZE,
            max_packet_size,
            interface_num: config.interface_num,
            usb_index,
            write_ep: config.write_ep,
            read_ep: config.read_ep,
            eeprom: FtdiEeprom::default(),
        };

        // Reset device.
        ftdi.usb_reset().await?;

        // Set default baud rate.
        ftdi.set_baudrate(9600).await?;

        Ok(ftdi)
    }

    /// Async shutdown — WASM equivalent of Drop.
    ///
    /// Should be called before the device is dropped in WASM, since async Drop
    /// is not available in Rust.
    pub async fn shutdown(&mut self) {
        // Best-effort cleanup.
        let _ = self.flush_all().await;
    }
}

// ---- Accessors (always available) ----

impl FtdiDevice {
    /// The detected FTDI chip type.
    pub fn chip_type(&self) -> ChipType {
        self.chip_type
    }

    /// The currently configured baud rate.
    pub fn baudrate(&self) -> u32 {
        self.baudrate
    }

    /// The maximum USB packet size for this device.
    pub fn max_packet_size(&self) -> usize {
        self.max_packet_size
    }
}

// ---- Internal USB helpers ----

impl FtdiDevice {
    /// Open the bulk IN endpoint (device -> host) for reading.
    ///
    /// This is used by the streaming and async modules and should not be called directly.
    #[cfg_attr(all(feature = "wasm", not(feature = "std")), allow(dead_code))]
    pub(crate) fn bulk_in_endpoint(
        &self,
    ) -> Result<nusb::Endpoint<nusb::transfer::Bulk, nusb::transfer::In>> {
        self.interface
            .endpoint::<nusb::transfer::Bulk, nusb::transfer::In>(self.read_ep)
            .map_err(Error::Usb)
    }

    /// Open the bulk OUT endpoint (host -> device) for writing.
    ///
    /// This is used by the async module and should not be called directly.
    #[cfg_attr(all(feature = "wasm", not(feature = "std")), allow(dead_code))]
    pub(crate) fn bulk_out_endpoint(
        &self,
    ) -> Result<nusb::Endpoint<nusb::transfer::Bulk, nusb::transfer::Out>> {
        self.interface
            .endpoint::<nusb::transfer::Bulk, nusb::transfer::Out>(self.write_ep)
            .map_err(Error::Usb)
    }

    /// Get the write buffer chunk size (for internal use by async module).
    #[cfg_attr(all(feature = "wasm", not(feature = "std")), allow(dead_code))]
    pub(crate) fn writebuffer_chunksize(&self) -> usize {
        self.writebuffer_chunksize
    }

    /// Get the read buffer chunk size (for internal use by async module).
    #[cfg_attr(all(feature = "wasm", not(feature = "std")), allow(dead_code))]
    pub(crate) fn readbuffer_chunksize(&self) -> usize {
        self.readbuffer_chunksize
    }

    /// Drain up to `n` bytes from the internal read buffer (for async module).
    #[cfg_attr(all(feature = "wasm", not(feature = "std")), allow(dead_code))]
    pub(crate) fn drain_readbuffer(&mut self, max: usize) -> Vec<u8> {
        let n = self.readbuffer_remaining.min(max);
        if n == 0 {
            return Vec::new();
        }
        let data = self.readbuffer[self.readbuffer_offset..self.readbuffer_offset + n].to_vec();
        self.readbuffer_remaining -= n;
        self.readbuffer_offset += n;
        data
    }

    /// Send a vendor OUT control transfer to the device.
    #[maybe_async]
    pub(crate) async fn control_out(&self, request: u8, value: u16, index: u16) -> Result<()> {
        nusb_await!(self.interface.control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request,
                value,
                index,
                data: &[],
            },
            self.write_timeout,
        ))?;
        Ok(())
    }

    /// Send a vendor IN control transfer to the device.
    #[maybe_async]
    pub(crate) async fn control_in(
        &self,
        request: u8,
        value: u16,
        index: u16,
        length: u16,
    ) -> Result<Vec<u8>> {
        let data = nusb_await!(self.interface.control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request,
                value,
                index,
                length,
            },
            self.read_timeout,
        ))?;
        Ok(data)
    }
}

// ---- Reset / Flush ----

impl FtdiDevice {
    /// Perform a USB reset on the FTDI device.
    ///
    /// This resets the device to its default state. The internal read buffer
    /// is invalidated.
    ///
    /// The proprietary FTDI driver sends this with `index=0` (a full
    /// device reset, not interface-specific), which we replicate here.
    #[maybe_async]
    pub async fn usb_reset(&mut self) -> Result<()> {
        // The proprietary driver always uses index=0 for a full device reset,
        // not the interface-specific index.
        self.control_out(SIO_RESET_REQUEST, SIO_RESET_SIO, 0)
            .await?;
        self.readbuffer_offset = 0;
        self.readbuffer_remaining = 0;
        Ok(())
    }

    /// Flush the receive (RX) buffer.
    ///
    /// Clears data in the chip's RX FIFO (data flowing from the serial
    /// device toward the host) and the internal software read buffer.
    ///
    /// The purge command is sent 6 times to ensure the FIFO is fully
    /// drained, matching the behavior of the proprietary FTDI driver.
    /// This is important for reliable operation in FT245 FIFO mode.
    #[maybe_async]
    pub async fn flush_rx(&mut self) -> Result<()> {
        // The proprietary driver sends the RX purge command 6 times
        // to ensure reliable FIFO draining.
        for _ in 0..6 {
            self.control_out(SIO_RESET_REQUEST, SIO_TCIFLUSH, self.usb_index)
                .await?;
        }
        self.readbuffer_offset = 0;
        self.readbuffer_remaining = 0;
        Ok(())
    }

    /// Flush the transmit (TX) buffer.
    ///
    /// Clears data in the chip's TX FIFO (data flowing from the host
    /// toward the serial device).
    #[maybe_async]
    pub async fn flush_tx(&mut self) -> Result<()> {
        self.control_out(SIO_RESET_REQUEST, SIO_TCOFLUSH, self.usb_index)
            .await?;
        Ok(())
    }

    /// Flush both RX and TX buffers.
    ///
    /// Matches the order of `ftdi_tcioflush()`: TX first, then RX.
    #[maybe_async]
    pub async fn flush_all(&mut self) -> Result<()> {
        self.flush_tx().await?;
        self.flush_rx().await
    }
}

// ---- Serial Configuration ----

impl FtdiDevice {
    /// Set the baud rate.
    ///
    /// The actual baud rate achieved is determined by the chip's clock
    /// divider and may differ slightly from the requested value. An error
    /// is returned if the achievable rate deviates by more than ~5%.
    ///
    /// When bitbang mode is enabled, the baud rate is internally multiplied
    /// by 4 (the FTDI chip's bitbang clock runs at 4x the serial baud rate).
    #[maybe_async]
    pub async fn set_baudrate(&mut self, baudrate: u32) -> Result<()> {
        let effective = if self.bitbang_enabled {
            baudrate * 4
        } else {
            baudrate
        };

        let result = baudrate::convert_baudrate(effective, self.chip_type, self.usb_index)
            .ok_or(Error::InvalidArgument("baud rate must be > 0"))?;

        // Check within ~5% tolerance
        let actual = result.actual;
        if (actual as u64) * 2 < effective as u64
            || if actual < effective {
                (actual as u64) * 21 < (effective as u64) * 20
            } else {
                (effective as u64) * 21 < (actual as u64) * 20
            }
        {
            return Err(Error::UnsupportedBaudRate {
                requested: baudrate,
                actual,
            });
        }

        self.control_out(SIO_SET_BAUDRATE_REQUEST, result.value, result.index)
            .await?;
        self.baudrate = baudrate;
        Ok(())
    }

    /// Set the serial line properties (data bits, stop bits, parity).
    #[maybe_async]
    pub async fn set_line_property(
        &self,
        bits: DataBits,
        stop_bits: StopBits,
        parity: Parity,
    ) -> Result<()> {
        self.set_line_property_with_break(bits, stop_bits, parity, BreakType::Off)
            .await
    }

    /// Set the serial line properties including break control.
    #[maybe_async]
    pub async fn set_line_property_with_break(
        &self,
        bits: DataBits,
        stop_bits: StopBits,
        parity: Parity,
        break_type: BreakType,
    ) -> Result<()> {
        let value = bits.wire_value()
            | (parity.wire_value() << 8)
            | (stop_bits.wire_value() << 11)
            | (break_type.wire_value() << 14);

        self.control_out(SIO_SET_DATA_REQUEST, value, self.usb_index)
            .await
    }

    /// Set the read timeout for USB transfers.
    pub fn set_read_timeout(&mut self, timeout: Duration) {
        self.read_timeout = timeout;
    }

    /// Set the write timeout for USB transfers.
    pub fn set_write_timeout(&mut self, timeout: Duration) {
        self.write_timeout = timeout;
    }

    /// Get the current read timeout.
    pub fn read_timeout(&self) -> Duration {
        self.read_timeout
    }

    /// Get the current write timeout.
    pub fn write_timeout(&self) -> Duration {
        self.write_timeout
    }
}

// ---- Flow Control / Modem Lines ----

impl FtdiDevice {
    /// Set the flow control mode.
    #[maybe_async]
    pub async fn set_flow_control(&self, flow: FlowControl) -> Result<()> {
        match flow {
            FlowControl::Disabled => {
                self.control_out(
                    SIO_SET_FLOW_CTRL_REQUEST,
                    0,
                    SIO_DISABLE_FLOW_CTRL | self.usb_index,
                )
                .await
            }
            FlowControl::RtsCts => {
                self.control_out(
                    SIO_SET_FLOW_CTRL_REQUEST,
                    0,
                    SIO_RTS_CTS_HS | self.usb_index,
                )
                .await
            }
            FlowControl::DtrDsr => {
                self.control_out(
                    SIO_SET_FLOW_CTRL_REQUEST,
                    0,
                    SIO_DTR_DSR_HS | self.usb_index,
                )
                .await
            }
            FlowControl::XonXoff { xon, xoff } => {
                let xonxoff = (xon as u16) | ((xoff as u16) << 8);
                self.control_out(
                    SIO_SET_FLOW_CTRL_REQUEST,
                    xonxoff,
                    SIO_XON_XOFF_HS | self.usb_index,
                )
                .await
            }
        }
    }

    /// Set XON/XOFF software flow control with custom characters.
    #[maybe_async]
    pub async fn set_flow_control_xonxoff(&self, xon: u8, xoff: u8) -> Result<()> {
        self.set_flow_control(FlowControl::XonXoff { xon, xoff })
            .await
    }

    /// Set the DTR (Data Terminal Ready) line state.
    #[maybe_async]
    pub async fn set_dtr(&self, state: bool) -> Result<()> {
        let val = if state {
            SIO_SET_DTR_HIGH
        } else {
            SIO_SET_DTR_LOW
        };
        self.control_out(SIO_SET_MODEM_CTRL_REQUEST, val, self.usb_index)
            .await
    }

    /// Set the RTS (Request To Send) line state.
    #[maybe_async]
    pub async fn set_rts(&self, state: bool) -> Result<()> {
        let val = if state {
            SIO_SET_RTS_HIGH
        } else {
            SIO_SET_RTS_LOW
        };
        self.control_out(SIO_SET_MODEM_CTRL_REQUEST, val, self.usb_index)
            .await
    }

    /// Set both DTR and RTS lines in a single USB transfer.
    #[maybe_async]
    pub async fn set_dtr_rts(&self, dtr: bool, rts: bool) -> Result<()> {
        let mut val = if dtr {
            SIO_SET_DTR_HIGH
        } else {
            SIO_SET_DTR_LOW
        };
        val |= if rts {
            SIO_SET_RTS_HIGH
        } else {
            SIO_SET_RTS_LOW
        };
        self.control_out(SIO_SET_MODEM_CTRL_REQUEST, val, self.usb_index)
            .await
    }

    /// Set the special event character.
    #[maybe_async]
    pub async fn set_event_char(&self, ch: u8, enable: bool) -> Result<()> {
        let val = (ch as u16) | if enable { 1 << 8 } else { 0 };
        self.control_out(SIO_SET_EVENT_CHAR_REQUEST, val, self.usb_index)
            .await
    }

    /// Set the error character.
    #[maybe_async]
    pub async fn set_error_char(&self, ch: u8, enable: bool) -> Result<()> {
        let val = (ch as u16) | if enable { 1 << 8 } else { 0 };
        self.control_out(SIO_SET_ERROR_CHAR_REQUEST, val, self.usb_index)
            .await
    }

    /// Poll the modem status.
    #[maybe_async]
    pub async fn poll_modem_status(&self) -> Result<ModemStatus> {
        let data = self
            .control_in(SIO_POLL_MODEM_STATUS_REQUEST, 0, self.usb_index, 2)
            .await?;
        if data.len() < 2 {
            return Err(Error::DeviceUnavailable);
        }
        let raw = (data[0] as u16) | ((data[1] as u16) << 8);
        Ok(ModemStatus::from_raw(raw))
    }
}

// ---- Latency Timer ----

impl FtdiDevice {
    /// Set the latency timer value (1-255 ms).
    ///
    /// After setting the latency timer, this function sleeps for
    /// `min(latency_ms, 50)` ms to allow the device to apply the new
    /// value, matching the behavior of the proprietary FTDI driver.
    #[maybe_async]
    pub async fn set_latency_timer(&self, latency_ms: u8) -> Result<()> {
        if latency_ms < 1 {
            return Err(Error::InvalidArgument("latency must be between 1 and 255"));
        }
        self.control_out(
            SIO_SET_LATENCY_TIMER_REQUEST,
            latency_ms as u16,
            self.usb_index,
        )
        .await?;

        // The proprietary driver sleeps after setting the latency timer
        // to give the device time to apply the new value:
        //   usleep(min(latency_ms * 1000, 50000))
        let sleep_ms = (latency_ms as u64).min(50);
        crate::sleep_util::sleep(Duration::from_millis(sleep_ms)).await;

        Ok(())
    }

    /// Get the current latency timer value in milliseconds.
    #[maybe_async]
    pub async fn latency_timer(&self) -> Result<u8> {
        let data = self
            .control_in(SIO_GET_LATENCY_TIMER_REQUEST, 0, self.usb_index, 1)
            .await?;
        if data.is_empty() {
            return Err(Error::DeviceUnavailable);
        }
        Ok(data[0])
    }
}

// ---- Bitbang / MPSSE ----

impl FtdiDevice {
    /// Enable a bitbang or MPSSE mode.
    #[maybe_async]
    pub async fn set_bitmode(&mut self, bitmask: u8, mode: BitMode) -> Result<()> {
        let val = (bitmask as u16) | ((mode.wire_value() as u16) << 8);
        self.control_out(SIO_SET_BITMODE_REQUEST, val, self.usb_index)
            .await?;

        self.bitbang_mode = mode;
        self.bitbang_enabled = mode != BitMode::Reset;
        Ok(())
    }

    /// Disable bitbang mode and return to normal serial/FIFO operation.
    #[maybe_async]
    pub async fn disable_bitbang(&mut self) -> Result<()> {
        self.set_bitmode(0, BitMode::Reset).await
    }

    /// Read the current pin states directly, bypassing the read buffer.
    #[maybe_async]
    pub async fn read_pins(&self) -> Result<u8> {
        let data = self
            .control_in(SIO_READ_PINS_REQUEST, 0, self.usb_index, 1)
            .await?;
        if data.is_empty() {
            return Err(Error::DeviceUnavailable);
        }
        Ok(data[0])
    }
}

// ---- Chunk Size Configuration ----

impl FtdiDevice {
    /// Set the read buffer chunk size.
    pub fn set_read_chunksize(&mut self, chunksize: usize) {
        self.readbuffer_offset = 0;
        self.readbuffer_remaining = 0;
        self.readbuffer_chunksize = chunksize;
        self.readbuffer.resize(chunksize, 0);
    }

    /// Get the current read buffer chunk size.
    pub fn read_chunksize(&self) -> usize {
        self.readbuffer_chunksize
    }

    /// Set the write buffer chunk size.
    pub fn set_write_chunksize(&mut self, chunksize: usize) {
        self.writebuffer_chunksize = chunksize;
    }

    /// Get the current write buffer chunk size.
    pub fn write_chunksize(&self) -> usize {
        self.writebuffer_chunksize
    }
}

// ---- Data Transfer ----

impl FtdiDevice {
    /// Write data to the FTDI device.
    ///
    /// Data is sent in chunks of [`write_chunksize`](Self::write_chunksize).
    /// Returns the number of bytes written.
    #[maybe_async]
    pub async fn write_data(&mut self, buf: &[u8]) -> Result<usize> {
        let mut offset = 0;

        while offset < buf.len() {
            let end = (offset + self.writebuffer_chunksize).min(buf.len());
            let chunk = &buf[offset..end];

            let mut transfer_buf = nusb::transfer::Buffer::new(chunk.len());
            transfer_buf.extend_from_slice(chunk);

            self.write_endpoint.submit(transfer_buf);

            let completion = ep_wait!(self.write_endpoint, self.write_timeout)
                .ok_or_else(|| Error::Transfer(nusb::transfer::TransferError::Cancelled))?;
            completion.status.map_err(Error::Transfer)?;
            offset += completion.actual_len;
        }

        Ok(offset)
    }

    /// Read data from the FTDI device.
    ///
    /// Automatically strips the two modem status bytes that the FTDI chip
    /// prepends to every USB packet. Returns the number of payload bytes
    /// read into `buf`.
    ///
    /// Returns 0 if no data is available (the chip only sent status bytes).
    #[maybe_async]
    pub async fn read_data(&mut self, buf: &mut [u8]) -> Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }

        let packet_size = self.max_packet_size;
        if packet_size == 0 {
            return Err(Error::InvalidArgument("max_packet_size is zero"));
        }

        // Serve from internal buffer first
        if self.readbuffer_remaining > 0 {
            let n = self.readbuffer_remaining.min(buf.len());
            buf[..n].copy_from_slice(
                &self.readbuffer[self.readbuffer_offset..self.readbuffer_offset + n],
            );
            self.readbuffer_remaining -= n;
            self.readbuffer_offset += n;
            return Ok(n);
        }

        // Issue a USB bulk read via submit + ep_wait
        let transfer_buf = nusb::transfer::Buffer::new(self.readbuffer_chunksize);
        self.read_endpoint.submit(transfer_buf);

        let completion = ep_wait!(self.read_endpoint, self.read_timeout)
            .ok_or_else(|| Error::Transfer(nusb::transfer::TransferError::Cancelled))?;
        completion.status.map_err(Error::Transfer)?;

        let actual_length = completion.actual_len;

        if actual_length <= 2 {
            // Only modem status bytes, no payload
            return Ok(0);
        }

        // Copy raw data into our internal buffer for stripping
        let raw_data = completion.buffer.into_vec();
        self.readbuffer[..actual_length].copy_from_slice(&raw_data[..actual_length]);

        // Strip 2-byte modem status from each max_packet_size chunk
        let stripped = strip_modem_status(&mut self.readbuffer[..actual_length], packet_size);

        if stripped == 0 {
            return Ok(0);
        }

        let n = stripped.min(buf.len());
        buf[..n].copy_from_slice(&self.readbuffer[..n]);

        if stripped > buf.len() {
            // Save remainder - shift data to beginning of buffer
            self.readbuffer.copy_within(n..stripped, 0);
            self.readbuffer_offset = 0;
            self.readbuffer_remaining = stripped - n;
        } else {
            self.readbuffer_offset = 0;
            self.readbuffer_remaining = 0;
        }

        Ok(n)
    }

    /// Write all bytes to the device, retrying until complete.
    #[maybe_async]
    pub async fn write_all(&mut self, buf: &[u8]) -> Result<()> {
        let mut offset = 0;
        while offset < buf.len() {
            let n = self.write_data(&buf[offset..]).await?;
            if n == 0 {
                return Err(Error::WriteZero);
            }
            offset += n;
        }
        Ok(())
    }
}

/// Strip the 2-byte modem status header from each packet in a raw USB bulk
/// read result. Returns the total number of payload bytes after stripping.
fn strip_modem_status(data: &mut [u8], packet_size: usize) -> usize {
    let total = data.len();
    if total <= 2 {
        return 0;
    }

    let num_packets = total.div_ceil(packet_size);
    let mut write_pos = 0;

    for i in 0..num_packets {
        let pkt_start = i * packet_size;
        let pkt_end = (pkt_start + packet_size).min(total);
        let pkt_len = pkt_end - pkt_start;

        if pkt_len <= 2 {
            continue;
        }

        let payload_start = pkt_start + 2;
        let payload_len = pkt_len - 2;

        if write_pos != payload_start {
            data.copy_within(payload_start..payload_start + payload_len, write_pos);
        }
        write_pos += payload_len;
    }

    write_pos
}

/// Detect chip type from bcdDevice version.
fn detect_chip_type(bcd: u16, has_serial: bool) -> ChipType {
    match bcd {
        0x0400 => ChipType::Bm,
        0x0200 if !has_serial => ChipType::Bm,
        0x0200 => ChipType::Am,
        0x0500 => ChipType::Ft2232C,
        0x0600 => ChipType::Ft232R,
        0x0700 => ChipType::Ft2232H,
        0x0800 => ChipType::Ft4232H,
        0x0900 => ChipType::Ft232H,
        0x1000 => ChipType::Ft230X,
        _ => ChipType::Bm,
    }
}

/// Determine the maximum packet size for a device.
fn determine_max_packet_size(
    device: &nusb::Device,
    chip_type: ChipType,
    interface_num: u8,
) -> usize {
    let default_size = if chip_type.is_h_type() { 512 } else { 64 };

    let config = match device.active_configuration() {
        Ok(c) => c,
        Err(_) => return default_size,
    };

    for iface_group in config.interfaces() {
        if iface_group.interface_number() != interface_num {
            continue;
        }
        for alt in iface_group.alt_settings() {
            if let Some(ep) = alt.endpoints().next() {
                return ep.max_packet_size();
            }
        }
    }

    default_size
}

// ---- Error Recovery ----

impl FtdiDevice {
    /// Read data with retry on transient USB errors.
    #[maybe_async]
    pub async fn read_data_retry(
        &mut self,
        buf: &mut [u8],
        max_retries: usize,
        retry_delay: Duration,
    ) -> Result<usize> {
        let mut last_err = None;
        for _ in 0..=max_retries {
            match self.read_data(buf).await {
                Ok(n) => return Ok(n),
                Err(e @ Error::Transfer(_)) => {
                    last_err = Some(e);
                    crate::sleep_util::sleep(retry_delay).await;
                }
                Err(e) => return Err(e),
            }
        }
        Err(last_err.unwrap())
    }

    /// Write data with retry on transient USB errors.
    #[maybe_async]
    pub async fn write_data_retry(
        &mut self,
        buf: &[u8],
        max_retries: usize,
        retry_delay: Duration,
    ) -> Result<usize> {
        let mut last_err = None;
        for _ in 0..=max_retries {
            match self.write_data(buf).await {
                Ok(n) => return Ok(n),
                Err(e @ Error::Transfer(_)) => {
                    last_err = Some(e);
                    crate::sleep_util::sleep(retry_delay).await;
                }
                Err(e) => return Err(e),
            }
        }
        Err(last_err.unwrap())
    }

    /// Check if the USB device is still connected.
    #[maybe_async]
    pub async fn is_connected(&self) -> bool {
        self.control_in(SIO_GET_LATENCY_TIMER_REQUEST, 0, self.usb_index, 1)
            .await
            .is_ok()
    }

    /// Attempt to recover from a USB error by resetting the device.
    #[maybe_async]
    pub async fn recover(&mut self) -> Result<()> {
        self.usb_reset().await?;
        if self.baudrate > 0 {
            let baud = self.baudrate;
            self.set_baudrate(baud).await?;
        }
        if self.bitbang_enabled {
            let mode = self.bitbang_mode;
            self.set_bitmode(0xFF, mode).await?;
        }
        Ok(())
    }
}

// ---- std::io trait implementations (native only) ----

#[cfg(feature = "is_sync")]
impl std::io::Read for FtdiDevice {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.read_data(buf).map_err(std::io::Error::other)
    }
}

#[cfg(feature = "is_sync")]
impl std::io::Write for FtdiDevice {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.write_data(buf).map_err(std::io::Error::other)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.flush_tx().map_err(std::io::Error::other)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strip_modem_status_single_packet() {
        let mut data = vec![0u8; 64];
        data[0] = 0x01;
        data[1] = 0x60;
        for (i, byte) in data.iter_mut().enumerate().take(64).skip(2) {
            *byte = i as u8;
        }

        let stripped = strip_modem_status(&mut data, 64);
        assert_eq!(stripped, 62);
        for (i, byte) in data.iter().enumerate().take(62) {
            assert_eq!(*byte, (i + 2) as u8);
        }
    }

    #[test]
    fn strip_modem_status_multiple_packets() {
        let packet_size = 8;
        let mut data = vec![
            0xAA, 0xBB, 2, 3, 4, 5, 6, 7, 0xCC, 0xDD, 10, 11, 12, 13, 14, 15,
        ];

        let stripped = strip_modem_status(&mut data, packet_size);
        assert_eq!(stripped, 12);
        assert_eq!(&data[..12], &[2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15]);
    }

    #[test]
    fn strip_modem_status_short() {
        let mut data = vec![0x01, 0x60];
        assert_eq!(strip_modem_status(&mut data, 64), 0);
    }

    #[test]
    fn strip_modem_status_empty() {
        let mut data: Vec<u8> = vec![];
        assert_eq!(strip_modem_status(&mut data, 64), 0);
    }

    #[test]
    fn detect_chip_type_known_versions() {
        assert_eq!(detect_chip_type(0x0400, false), ChipType::Bm);
        assert_eq!(detect_chip_type(0x0200, true), ChipType::Am);
        assert_eq!(detect_chip_type(0x0200, false), ChipType::Bm);
        assert_eq!(detect_chip_type(0x0500, false), ChipType::Ft2232C);
        assert_eq!(detect_chip_type(0x0600, false), ChipType::Ft232R);
        assert_eq!(detect_chip_type(0x0700, false), ChipType::Ft2232H);
        assert_eq!(detect_chip_type(0x0800, false), ChipType::Ft4232H);
        assert_eq!(detect_chip_type(0x0900, false), ChipType::Ft232H);
        assert_eq!(detect_chip_type(0x1000, false), ChipType::Ft230X);
    }
}
