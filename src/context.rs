//! Core FTDI device handle and operations.
//!
//! [`FtdiDevice`] is the main type in this crate. It represents an opened,
//! configured FTDI USB device and provides methods for serial communication,
//! bitbang/MPSSE mode, flow control, and EEPROM access.

use core::sync::atomic::{AtomicBool, Ordering};
use core::time::Duration;
use std::sync::Arc;

use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};

use crate::baudrate;
use crate::constants::*;
use crate::eeprom::FtdiEeprom;
use crate::error::{Error, Result};
use crate::types::*;

/// Default read/write timeout.
const DEFAULT_TIMEOUT: Duration = Duration::from_secs(5);

/// Default read/write buffer chunk size.
const DEFAULT_CHUNKSIZE: usize = 4096;

#[cfg(not(target_arch = "wasm32"))]
#[derive(Clone, Copy)]
pub(crate) enum TransferDeadline {
    At(std::time::Instant),
    Never,
}

#[cfg(not(target_arch = "wasm32"))]
impl TransferDeadline {
    pub(crate) fn new(timeout: Duration) -> Self {
        std::time::Instant::now()
            .checked_add(timeout)
            .map_or(Self::Never, Self::At)
    }
}

#[cfg(target_arch = "wasm32")]
#[derive(Clone, Copy)]
pub(crate) struct TransferDeadline;

#[cfg(target_arch = "wasm32")]
impl TransferDeadline {
    pub(crate) fn new(_: Duration) -> Self {
        Self
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub(crate) async fn next_before_deadline<EpType, Dir>(
    endpoint: &mut nusb::Endpoint<EpType, Dir>,
    deadline: TransferDeadline,
) -> Option<nusb::transfer::Completion>
where
    EpType: nusb::transfer::BulkOrInterrupt,
    Dir: nusb::transfer::EndpointDirection,
{
    let TransferDeadline::At(expires_at) = deadline else {
        return Some(endpoint.next_complete().await);
    };
    let remaining = expires_at.checked_duration_since(std::time::Instant::now())?;

    futures_lite::future::race(async { Some(endpoint.next_complete().await) }, async {
        futures_timer::Delay::new(remaining).await;
        None
    })
    .await
}

#[cfg(not(target_arch = "wasm32"))]
async fn clear_pending_transfers<EpType, Dir>(
    endpoint: &mut nusb::Endpoint<EpType, Dir>,
    deadline: TransferDeadline,
) -> bool
where
    EpType: nusb::transfer::BulkOrInterrupt,
    Dir: nusb::transfer::EndpointDirection,
{
    while endpoint.pending() > 0 {
        if next_before_deadline(endpoint, deadline).await.is_none() {
            endpoint.cancel_all();
            return false;
        }
    }

    true
}

/// Cancel all queued transfers on an endpoint and drain their completions
/// within `timeout`.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) async fn cancel_and_drain<EpType, Dir>(
    endpoint: &mut nusb::Endpoint<EpType, Dir>,
    timeout: Duration,
) -> Result<()>
where
    EpType: nusb::transfer::BulkOrInterrupt,
    Dir: nusb::transfer::EndpointDirection,
{
    endpoint.cancel_all();
    if clear_pending_transfers(endpoint, TransferDeadline::new(timeout)).await {
        Ok(())
    } else {
        Err(Error::Timeout(timeout))
    }
}

async fn wait_for_completion<EpType, Dir>(
    endpoint: &mut nusb::Endpoint<EpType, Dir>,
    deadline: TransferDeadline,
) -> Option<nusb::transfer::Completion>
where
    EpType: nusb::transfer::BulkOrInterrupt,
    Dir: nusb::transfer::EndpointDirection,
{
    #[cfg(target_arch = "wasm32")]
    {
        let _ = deadline;
        Some(endpoint.next_complete().await)
    }

    #[cfg(not(target_arch = "wasm32"))]
    {
        // `Endpoint::next_complete` is cancellation-safe. Leaving a timed-out or
        // externally-cancelled transfer queued lets the next operation resume it
        // instead of discarding serial input that arrived at the timeout boundary.
        next_before_deadline(endpoint, deadline).await
    }
}

async fn async_sleep(duration: Duration) {
    #[cfg(not(target_arch = "wasm32"))]
    futures_timer::Delay::new(duration).await;

    #[cfg(target_arch = "wasm32")]
    crate::sleep_util::sleep(duration).await;
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) enum RecoveryAction {
    #[default]
    None,
    RewriteEeprom,
    EraseEeprom,
}

/// Allocate a process-unique device identity.
fn next_device_id() -> u64 {
    static NEXT_DEVICE_ID: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);
    NEXT_DEVICE_ID.fetch_add(1, core::sync::atomic::Ordering::Relaxed)
}

/// Marks a device as requiring recovery if a stateful future is dropped or
/// returns before explicitly completing its cleanup.
pub(crate) struct RecoveryGuard {
    recovery_required: Arc<AtomicBool>,
    armed: bool,
}

impl RecoveryGuard {
    pub(crate) fn new(recovery_required: Arc<AtomicBool>) -> Self {
        Self {
            recovery_required,
            armed: true,
        }
    }

    pub(crate) fn disarm(mut self) {
        self.armed = false;
    }
}

impl Drop for RecoveryGuard {
    fn drop(&mut self) {
        if self.armed {
            self.recovery_required.store(true, Ordering::Release);
        }
    }
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
/// # async fn example(dev: &mut FtdiDevice) -> ftdi_nusb::Result<()> {
/// dev.set_baudrate(115200).await?;
/// dev.write_all(b"Hello FTDI!\r\n").await?;
/// # Ok(())
/// # }
/// ```
///
/// Native async constructors require the `smol` or `tokio` feature. The
/// runtime-independent transfer methods are always available.
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

    // EEPROM
    pub(crate) eeprom: FtdiEeprom,

    // Set by cancellation/failure of stateful protocol operations. Shared with
    // guards so dropping a future can poison the device without async Drop.
    recovery_required: Arc<AtomicBool>,
    recovery_epoch: u64,
    recovery_action: RecoveryAction,
    // Unique per opened device; binds MPSSE contexts to the device they were
    // initialized on.
    device_id: u64,
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

#[cfg(not(target_arch = "wasm32"))]
impl FtdiDevice {
    pub(crate) fn from_open_device(
        device: nusb::Device,
        interface: nusb::Interface,
        iface: Interface,
    ) -> Result<Self> {
        let config = iface.config();
        let write_endpoint = interface
            .endpoint::<nusb::transfer::Bulk, nusb::transfer::Out>(config.write_ep)
            .map_err(Error::Usb)?;
        let read_endpoint = interface
            .endpoint::<nusb::transfer::Bulk, nusb::transfer::In>(config.read_ep)
            .map_err(Error::Usb)?;
        let desc = device.device_descriptor();
        let chip_type = detect_chip_type(
            desc.device_version(),
            desc.serial_number_string_index().is_some(),
        );
        let max_packet_size = determine_max_packet_size(&device, chip_type, config.interface_num);
        let usb_index = if chip_type.is_multi_channel() {
            config.usb_index
        } else {
            0
        };

        Ok(Self {
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
            eeprom: FtdiEeprom::default(),
            recovery_required: Arc::new(AtomicBool::new(false)),
            recovery_epoch: 0,
            recovery_action: RecoveryAction::None,
            device_id: next_device_id(),
        })
    }

    pub(crate) async fn initialize(&mut self) -> Result<()> {
        self.usb_reset().await?;
        self.set_baudrate(9600).await
    }
}

// ---- Native-only construction / Opening ----

#[cfg(all(not(target_arch = "wasm32"), any(feature = "smol", feature = "tokio")))]
impl FtdiDevice {
    /// Open the first FTDI device matching the given vendor and product IDs.
    ///
    /// Uses [`Interface::A`] by default. For multi-interface chips, use
    /// [`open_with_interface`](Self::open_with_interface).
    pub async fn open(vendor: u16, product: u16) -> Result<Self> {
        Self::open_with_interface(vendor, product, Interface::Any).await
    }

    /// Open the first matching device on a specific interface.
    pub async fn open_with_interface(vendor: u16, product: u16, iface: Interface) -> Result<Self> {
        let dev_info = nusb::list_devices()
            .await?
            .find(|d| d.vendor_id() == vendor && d.product_id() == product)
            .ok_or(Error::DeviceNotFound)?;

        Self::from_device_info(dev_info, iface).await
    }

    /// Open a device matching a [`DeviceFilter`](crate::DeviceFilter).
    pub async fn open_with_filter(filter: &crate::DeviceFilter, iface: Interface) -> Result<Self> {
        let candidates: Vec<_> = nusb::list_devices()
            .await?
            .filter(|device| {
                device.vendor_id() == filter.vendor_id && device.product_id() == filter.product_id
            })
            .collect();
        let mut match_count = 0;

        for info in candidates {
            if filter.description.is_some() || filter.serial.is_some() {
                let device = info.open().await?;
                let descriptor = device.device_descriptor();

                if let Some(expected) = &filter.description {
                    let Some(index) = descriptor.product_string_index() else {
                        continue;
                    };
                    let actual = device
                        .get_string_descriptor(index, 0x0409, Duration::from_secs(1))
                        .await
                        .unwrap_or_default();
                    if actual != *expected {
                        continue;
                    }
                }

                if let Some(expected) = &filter.serial {
                    let Some(index) = descriptor.serial_number_string_index() else {
                        continue;
                    };
                    let actual = device
                        .get_string_descriptor(index, 0x0409, Duration::from_secs(1))
                        .await
                        .unwrap_or_default();
                    if actual != *expected {
                        continue;
                    }
                }
            }

            if match_count == filter.index {
                return Self::from_device_info(info, iface).await;
            }
            match_count += 1;
        }

        Err(Error::DeviceNotFound)
    }

    /// Open a device by USB bus number and device address.
    ///
    /// This function is only available on Linux, where USB bus numbers
    /// are exposed by the kernel.
    #[cfg(target_os = "linux")]
    pub async fn open_bus_addr(bus: u8, addr: u8, iface: Interface) -> Result<Self> {
        let dev_info = nusb::list_devices()
            .await?
            .find(|d| d.busnum() == bus && d.device_address() == addr)
            .ok_or(Error::DeviceNotFound)?;

        Self::from_device_info(dev_info, iface).await
    }

    /// Open a device from an already-discovered [`nusb::DeviceInfo`].
    pub async fn from_device_info(dev_info: nusb::DeviceInfo, iface: Interface) -> Result<Self> {
        let config = iface.config();

        let device = dev_info.open().await?;

        // Detach kernel driver and claim interface
        let interface = device
            .detach_and_claim_interface(config.interface_num)
            .await?;

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
            eeprom: FtdiEeprom::default(),
            recovery_required: Arc::new(AtomicBool::new(false)),
            recovery_epoch: 0,
            recovery_action: RecoveryAction::None,
            device_id: next_device_id(),
        };

        // Reset device
        ftdi.usb_reset().await?;

        // Set default baud rate
        ftdi.set_baudrate(9600).await?;

        Ok(ftdi)
    }
}

// ---- WASM-only construction ----

#[cfg(target_arch = "wasm32")]
impl FtdiDevice {
    /// Show the browser's WebUSB device picker filtered by common FTDI VID/PIDs.
    ///
    /// Returns an open [`nusb::Device`] that can be passed to
    /// [`open_wasm`](Self::open_wasm).
    #[cfg(target_arch = "wasm32")]
    pub async fn request_device() -> Result<nusb::Device> {
        use wasm_bindgen::JsCast;
        use wasm_bindgen_futures::JsFuture;
        use web_sys::{UsbDevice, UsbDeviceFilter, UsbDeviceRequestOptions};

        let usb = web_sys::window()
            .ok_or(Error::DeviceNotFound)?
            .navigator()
            .usb();

        let mut filters = Vec::new();

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
            filters.push(filter);
        }

        let options = UsbDeviceRequestOptions::new(&filters);

        let device_promise = usb.request_device(&options);
        let device_js = JsFuture::from(device_promise)
            .await
            .map_err(|e| Error::OpenFailed(format!("WebUSB request failed: {:?}", e)))?;

        let device: UsbDevice = device_js
            .dyn_into()
            .map_err(|_| Error::OpenFailed("failed to get USB device".to_string()))?;

        nusb::Device::from_js(device)
            .await
            .map_err(|e| Error::OpenFailed(format!("failed to open WebUSB device: {e}")))
    }

    /// Initialize an FTDI device from an open [`nusb::Device`] in a WASM/WebUSB build.
    pub async fn open_wasm(device: nusb::Device, iface: Interface) -> Result<Self> {
        let config = iface.config();

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
            eeprom: FtdiEeprom::default(),
            recovery_required: Arc::new(AtomicBool::new(false)),
            recovery_epoch: 0,
            recovery_action: RecoveryAction::None,
            device_id: next_device_id(),
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
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn read_endpoint_mut(
        &mut self,
    ) -> &mut nusb::Endpoint<nusb::transfer::Bulk, nusb::transfer::In> {
        &mut self.read_endpoint
    }

    pub(crate) fn ensure_ready(&self) -> Result<()> {
        if self.recovery_required.load(Ordering::Acquire) {
            Err(Error::RecoveryRequired)
        } else {
            Ok(())
        }
    }

    pub(crate) fn begin_stateful_operation(&self) -> Result<RecoveryGuard> {
        self.ensure_ready()?;
        Ok(RecoveryGuard::new(Arc::clone(&self.recovery_required)))
    }

    pub(crate) fn set_recovery_action(&mut self, action: RecoveryAction) {
        self.recovery_action = action;
    }

    pub(crate) fn recovery_action(&self) -> RecoveryAction {
        self.recovery_action
    }

    pub(crate) fn clear_recovery_action(&mut self) {
        self.recovery_action = RecoveryAction::None;
    }

    pub(crate) fn recovery_epoch(&self) -> u64 {
        self.recovery_epoch
    }

    pub(crate) fn device_id(&self) -> u64 {
        self.device_id
    }

    /// Invalidate every MPSSE context and bus object created before now.
    pub(crate) fn bump_recovery_epoch(&mut self) {
        self.recovery_epoch = self.recovery_epoch.wrapping_add(1);
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn mark_recovery_required(&self) {
        self.recovery_required.store(true, Ordering::Release);
    }

    /// Temporarily permit best-effort cleanup while an outer armed
    /// [`RecoveryGuard`] still owns the final ready/poisoned decision.
    pub(crate) fn prepare_stateful_cleanup(&self) {
        self.recovery_required.store(false, Ordering::Release);
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn mark_stream_abandoned(&mut self) {
        // `recover` must return an abandoned stream to UART/reset mode rather
        // than restoring the synchronous-FIFO mode that was interrupted.
        self.bitbang_enabled = false;
        self.bitbang_mode = BitMode::Reset;
        self.mark_recovery_required();
    }

    /// Send a vendor OUT control transfer to the device.
    pub(crate) async fn control_out(&self, request: u8, value: u16, index: u16) -> Result<()> {
        self.ensure_ready()?;
        (self
            .interface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    data: &[],
                },
                self.write_timeout,
            )
            .await)?;
        Ok(())
    }

    /// Send a vendor IN control transfer to the device.
    pub(crate) async fn control_in(
        &self,
        request: u8,
        value: u16,
        index: u16,
        length: u16,
    ) -> Result<Vec<u8>> {
        self.ensure_ready()?;
        let data = (self
            .interface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    length,
                },
                self.read_timeout,
            )
            .await)?;
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
    pub async fn usb_reset(&mut self) -> Result<()> {
        // The proprietary driver always uses index=0 for a full device reset,
        // not the interface-specific index.
        self.control_out(SIO_RESET_REQUEST, SIO_RESET_SIO, 0)
            .await?;
        self.readbuffer_offset = 0;
        self.readbuffer_remaining = 0;
        self.bump_recovery_epoch();
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
    pub async fn flush_rx(&mut self) -> Result<()> {
        // A cancelled/timed-out read may still own an endpoint transfer. Drain
        // it before purging so pre-flush bytes cannot be resumed afterward.
        #[cfg(not(target_arch = "wasm32"))]
        cancel_and_drain(&mut self.read_endpoint, self.read_timeout).await?;
        #[cfg(target_arch = "wasm32")]
        while self.read_endpoint.pending() > 0 {
            // WebUSB has no cancellation API. FTDI IN transfers still finish
            // on the latency timer, so consume the old completion before purge.
            self.read_endpoint.next_complete().await;
        }

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
    pub async fn flush_tx(&mut self) -> Result<()> {
        self.control_out(SIO_RESET_REQUEST, SIO_TCOFLUSH, self.usb_index)
            .await?;
        Ok(())
    }

    /// Flush both RX and TX buffers.
    ///
    /// Matches the order of `ftdi_tcioflush()`: TX first, then RX.
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
    ///
    /// WebUSB does not support cancelling transfers, so this setting is ignored
    /// on WASM.
    pub fn set_read_timeout(&mut self, timeout: Duration) {
        self.read_timeout = timeout;
    }

    /// Set the write timeout for USB transfers.
    ///
    /// WebUSB does not support cancelling transfers, so this setting is ignored
    /// on WASM.
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
    pub async fn set_flow_control_xonxoff(&self, xon: u8, xoff: u8) -> Result<()> {
        self.set_flow_control(FlowControl::XonXoff { xon, xoff })
            .await
    }

    /// Set the DTR (Data Terminal Ready) line state.
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
    pub async fn set_event_char(&self, ch: u8, enable: bool) -> Result<()> {
        let val = (ch as u16) | if enable { 1 << 8 } else { 0 };
        self.control_out(SIO_SET_EVENT_CHAR_REQUEST, val, self.usb_index)
            .await
    }

    /// Set the error character.
    pub async fn set_error_char(&self, ch: u8, enable: bool) -> Result<()> {
        let val = (ch as u16) | if enable { 1 << 8 } else { 0 };
        self.control_out(SIO_SET_ERROR_CHAR_REQUEST, val, self.usb_index)
            .await
    }

    /// Poll the modem status.
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
        async_sleep(Duration::from_millis(sleep_ms)).await;

        Ok(())
    }

    /// Get the current latency timer value in milliseconds.
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
    pub async fn set_bitmode(&mut self, bitmask: u8, mode: BitMode) -> Result<()> {
        let val = (bitmask as u16) | ((mode.wire_value() as u16) << 8);
        self.control_out(SIO_SET_BITMODE_REQUEST, val, self.usb_index)
            .await?;

        self.bitbang_mode = mode;
        self.bitbang_enabled = mode != BitMode::Reset;
        self.bump_recovery_epoch();
        Ok(())
    }

    /// Disable bitbang mode and return to normal serial/FIFO operation.
    pub async fn disable_bitbang(&mut self) -> Result<()> {
        self.set_bitmode(0, BitMode::Reset).await
    }

    /// Read the current pin states directly, bypassing the read buffer.
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
    async fn complete_pending_writes(&mut self, deadline: TransferDeadline) -> Result<()> {
        while self.write_endpoint.pending() > 0 {
            let completion = wait_for_completion(&mut self.write_endpoint, deadline)
                .await
                .ok_or(Error::Timeout(self.write_timeout))?;
            completion.status.map_err(Error::Transfer)?;
            let expected = completion.buffer.requested_len();
            if completion.actual_len != expected {
                return Err(Error::ShortWrite {
                    expected,
                    actual: completion.actual_len,
                });
            }
        }
        Ok(())
    }

    /// Write data to the FTDI device.
    ///
    /// Data is sent in chunks of [`write_chunksize`](Self::write_chunksize).
    /// Returns the number of bytes written.
    ///
    /// If this future is cancelled, an already-submitted chunk may have been
    /// partially or fully transmitted. The device is poisoned so that a later
    /// flush or mode change cannot race that transfer; call [`recover`](Self::recover)
    /// before issuing more I/O.
    pub async fn write_data(&mut self, buf: &[u8]) -> Result<usize> {
        self.ensure_ready()?;
        if buf.is_empty() {
            return Ok(0);
        }

        // Explicit transfer errors are returned normally after their completion
        // has been consumed. Only dropping this future leaves the write outcome
        // unknown and therefore keeps this guard armed.
        let cancellation_guard = RecoveryGuard::new(Arc::clone(&self.recovery_required));
        let result = async {
            let mut offset = 0;

            while offset < buf.len() {
                let end = (offset + self.writebuffer_chunksize).min(buf.len());
                let chunk = &buf[offset..end];

                let mut transfer_buf = nusb::transfer::Buffer::new(chunk.len());
                transfer_buf.extend_from_slice(chunk);

                let deadline = TransferDeadline::new(self.write_timeout);
                self.complete_pending_writes(deadline).await?;
                self.write_endpoint.submit(transfer_buf);
                self.complete_pending_writes(deadline).await?;
                offset = end;
            }

            Ok(offset)
        }
        .await;
        cancellation_guard.disarm();
        result
    }

    /// Read data from the FTDI device.
    ///
    /// Automatically strips the two modem status bytes that the FTDI chip
    /// prepends to every USB packet. Returns the number of payload bytes
    /// read into `buf`.
    ///
    /// Returns 0 if no data is available (the chip only sent status bytes).
    ///
    /// This operation is cancellation-safe with respect to input consumption:
    /// an in-flight USB read remains queued and the next call resumes it.
    pub async fn read_data(&mut self, buf: &mut [u8]) -> Result<usize> {
        self.ensure_ready()?;
        if buf.is_empty() {
            return Ok(0);
        }

        let packet_size = self.read_endpoint.max_packet_size();
        if packet_size <= 2 {
            return Err(Error::InvalidArgument("invalid read endpoint packet size"));
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

        // Resume an in-flight read left by a timed-out or cancelled future.
        // `next_complete` is cancellation-safe, so this preserves serial input
        // instead of draining and discarding it on the next call.
        let deadline = TransferDeadline::new(self.read_timeout);
        if self.read_endpoint.pending() == 0 {
            let transfer_size =
                read_transfer_size(buf.len(), self.readbuffer_chunksize, packet_size)
                    .ok_or(Error::InvalidArgument("read transfer size overflow"))?;
            self.readbuffer.resize(transfer_size, 0);
            self.read_endpoint
                .submit(nusb::transfer::Buffer::new(transfer_size));
        }

        let completion = wait_for_completion(&mut self.read_endpoint, deadline)
            .await
            .ok_or(Error::Timeout(self.read_timeout))?;
        completion.status.map_err(Error::Transfer)?;

        let actual_length = completion.actual_len;
        log::trace!(
            "bulk IN completed: requested={} actual={actual_length}",
            completion.buffer.requested_len()
        );

        if actual_length <= 2 {
            // Only modem status bytes, no payload
            return Ok(0);
        }

        // Copy raw data into our internal buffer for stripping. A transfer
        // left pending by a cancelled read may be larger than the current
        // buffer if `set_read_chunksize` shrank it in between, so grow the
        // buffer to fit the completion instead of slicing out of bounds.
        let raw_data = completion.buffer.into_vec();
        if self.readbuffer.len() < actual_length {
            self.readbuffer.resize(actual_length, 0);
        }
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

/// Choose a raw USB transfer size large enough to hold `payload_len` bytes
/// after accounting for the two FTDI status bytes in every packet.
fn read_transfer_size(
    payload_len: usize,
    configured_size: usize,
    packet_size: usize,
) -> Option<usize> {
    let payload_per_packet = packet_size.checked_sub(2)?;
    let payload_packets = payload_len.div_ceil(payload_per_packet);
    let payload_raw_size = payload_packets.checked_mul(packet_size)?;
    let configured_packets = configured_size.div_ceil(packet_size).max(1);
    let configured_raw_size = configured_packets.checked_mul(packet_size)?;
    // Treat the configured chunk size as a cap. Requesting more than the
    // caller needs can hang when an MPSSE response ends exactly on a full USB
    // packet: the device has no short packet left to terminate a larger URB.
    let transfer_size = payload_raw_size.min(configured_raw_size);
    (transfer_size <= u32::MAX as usize).then_some(transfer_size)
}

/// Like [`strip_modem_status`], but collect the payload into a new `Vec`.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn strip_modem_status_to_vec(data: &[u8], packet_size: usize) -> Vec<u8> {
    let mut payload = Vec::with_capacity(data.len().saturating_sub(2));
    for packet_start in (0..data.len()).step_by(packet_size) {
        let packet_end = (packet_start + packet_size).min(data.len());
        if packet_end - packet_start > 2 {
            payload.extend_from_slice(&data[packet_start + 2..packet_end]);
        }
    }
    payload
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
                    async_sleep(retry_delay).await;
                }
                Err(e) => return Err(e),
            }
        }
        Err(last_err.unwrap())
    }

    /// Write data with retry on transient USB errors.
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
                    async_sleep(retry_delay).await;
                }
                Err(e) => return Err(e),
            }
        }
        Err(last_err.unwrap())
    }

    /// Check if the USB device is still connected.
    pub async fn is_connected(&self) -> bool {
        self.control_in(SIO_GET_LATENCY_TIMER_REQUEST, 0, self.usb_index, 1)
            .await
            .is_ok()
    }

    /// Attempt to recover from an interrupted or failed operation.
    ///
    /// On native targets this first cancels and drains queued endpoint transfers.
    /// Call this after dropping a stateful operation such as a streaming session
    /// before issuing unrelated protocol commands.
    pub async fn recover(&mut self) -> Result<()> {
        let recovery_guard = RecoveryGuard::new(Arc::clone(&self.recovery_required));
        // Recovery is the only operation allowed to clear the poison
        // temporarily. The armed guard restores it if recovery fails or is
        // cancelled before all device state has been rebuilt.
        self.recovery_required.store(false, Ordering::Release);

        #[cfg(not(target_arch = "wasm32"))]
        {
            cancel_and_drain(&mut self.read_endpoint, self.read_timeout).await?;
            cancel_and_drain(&mut self.write_endpoint, self.write_timeout).await?;
        }
        #[cfg(target_arch = "wasm32")]
        {
            // WebUSB cannot cancel submitted transfers. Consume completions
            // from cancelled futures before resetting the device so stale I/O
            // cannot leak into the recovered session.
            while self.read_endpoint.pending() > 0 {
                self.read_endpoint.next_complete().await;
            }
            while self.write_endpoint.pending() > 0 {
                self.write_endpoint.next_complete().await;
            }
        }

        // Finish any interrupted persistent EEPROM operation before restoring
        // the volatile USB/serial configuration. The action remains recorded if
        // recovery itself is cancelled or fails, so a later retry resumes it.
        self.recover_eeprom_action().await?;

        let baudrate = self.baudrate;
        let restore_bitbang = self.bitbang_enabled;
        let bitbang_mode = self.bitbang_mode;

        self.usb_reset().await?;
        // The FTDI USB reset request does not reliably leave synchronous FIFO
        // or bitbang mode, so explicitly return the pins to UART/reset mode.
        self.set_bitmode(0xFF, BitMode::Reset).await?;
        if baudrate > 0 {
            self.set_baudrate(baudrate).await?;
        }
        if restore_bitbang {
            self.set_bitmode(0xFF, bitbang_mode).await?;
        }
        self.bump_recovery_epoch();
        self.recovery_required.store(false, Ordering::Release);
        recovery_guard.disarm();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovery_guard_marks_cancelled_operation_dirty() {
        let recovery_required = Arc::new(AtomicBool::new(false));
        drop(RecoveryGuard::new(Arc::clone(&recovery_required)));
        assert!(recovery_required.load(Ordering::Acquire));
    }

    #[test]
    fn disarmed_recovery_guard_leaves_device_ready() {
        let recovery_required = Arc::new(AtomicBool::new(false));
        RecoveryGuard::new(Arc::clone(&recovery_required)).disarm();
        assert!(!recovery_required.load(Ordering::Acquire));
    }

    #[test]
    fn read_transfer_size_accounts_for_status_bytes() {
        assert_eq!(read_transfer_size(2040, 4096, 512), Some(2048));
        assert_eq!(read_transfer_size(4080, 4096, 512), Some(4096));
        assert_eq!(read_transfer_size(4081, 4096, 512), Some(4096));
        assert_eq!(read_transfer_size(4096, 4096, 512), Some(4096));
    }

    #[test]
    fn read_transfer_size_rounds_configured_size_to_packets() {
        assert_eq!(read_transfer_size(1, 4097, 512), Some(512));
    }

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
