//! Type definitions for FTDI chip communication.
//!
//! These types model the various configuration options for FTDI chips:
//! chip variants, serial line properties, bitbang modes, and modem status.

/// Supported FTDI chip types.
///
/// The chip type is auto-detected when a device is opened, based on the
/// USB `bcdDevice` descriptor field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ChipType {
    /// Original FTDI chip (FT8U232AM).
    Am,
    /// B-type chip (FT232BM, FT245BM).
    Bm,
    /// Dual-port chip (FT2232C/D/L).
    Ft2232C,
    /// FT232R / FT245R.
    Ft232R,
    /// Dual hi-speed chip (FT2232H).
    Ft2232H,
    /// Quad-port chip (FT4232H).
    Ft4232H,
    /// Single hi-speed chip (FT232H).
    Ft232H,
    /// FT230X / FT231X / FT234XD.
    Ft230X,
}

impl ChipType {
    /// Whether this is an H-type (hi-speed) chip.
    #[inline]
    pub fn is_h_type(self) -> bool {
        matches!(self, Self::Ft2232H | Self::Ft4232H | Self::Ft232H)
    }

    /// Whether this chip has multiple independent interfaces.
    ///
    /// Multi-channel chips (FT2232C/H, FT4232H) use a 1-based interface
    /// index in USB control transfers. Single-channel chips (FT232BM,
    /// FT245BM, FT232R, FT245R, FT232H, FT230X) use index 0.
    ///
    /// This matches the behavior of the proprietary FTDI driver's
    /// `SetDeviceContext` function.
    #[inline]
    pub fn is_multi_channel(self) -> bool {
        matches!(self, Self::Ft2232C | Self::Ft2232H | Self::Ft4232H)
    }

    /// Default product string for this chip type.
    pub fn default_product_name(self) -> &'static str {
        match self {
            Self::Am => "AM",
            Self::Bm => "BM",
            Self::Ft2232C => "Dual RS232",
            Self::Ft232R => "FT232R USB UART",
            Self::Ft2232H => "Dual RS232-HS",
            Self::Ft4232H => "FT4232H",
            Self::Ft232H => "Single-RS232-HS",
            Self::Ft230X => "FT230X Basic UART",
        }
    }

    /// Default product ID for this chip type.
    pub fn default_product_id(self) -> u16 {
        match self {
            Self::Am | Self::Bm | Self::Ft232R => 0x6001,
            Self::Ft2232C | Self::Ft2232H => 0x6010,
            Self::Ft4232H => 0x6011,
            Self::Ft232H => 0x6014,
            Self::Ft230X => 0x6015,
        }
    }

    /// Default USB release number (bcdDevice) for this chip type.
    pub fn release_number(self) -> u16 {
        match self {
            Self::Am => 0x0200,
            Self::Bm => 0x0400,
            Self::Ft2232C => 0x0500,
            Self::Ft232R => 0x0600,
            Self::Ft2232H => 0x0700,
            Self::Ft4232H => 0x0800,
            Self::Ft232H => 0x0900,
            Self::Ft230X => 0x1000,
        }
    }
}

/// Parity mode for serial communication.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Parity {
    /// No parity bit.
    #[default]
    None,
    /// Odd parity.
    Odd,
    /// Even parity.
    Even,
    /// Mark parity (always 1).
    Mark,
    /// Space parity (always 0).
    Space,
}

impl Parity {
    /// Wire encoding for the SIO_SET_DATA request.
    pub(crate) fn wire_value(self) -> u16 {
        match self {
            Self::None => 0x00,
            Self::Odd => 0x01,
            Self::Even => 0x02,
            Self::Mark => 0x03,
            Self::Space => 0x04,
        }
    }
}

/// Number of stop bits for serial communication.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum StopBits {
    /// 1 stop bit.
    #[default]
    One,
    /// 1.5 stop bits.
    OnePointFive,
    /// 2 stop bits.
    Two,
}

impl StopBits {
    /// Wire encoding for the SIO_SET_DATA request.
    pub(crate) fn wire_value(self) -> u16 {
        match self {
            Self::One => 0x00,
            Self::OnePointFive => 0x01,
            Self::Two => 0x02,
        }
    }
}

/// Number of data bits for serial communication.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum DataBits {
    /// 7 data bits.
    Seven,
    /// 8 data bits.
    #[default]
    Eight,
}

impl DataBits {
    /// Wire encoding for the SIO_SET_DATA request.
    pub(crate) fn wire_value(self) -> u16 {
        match self {
            Self::Seven => 7,
            Self::Eight => 8,
        }
    }
}

/// Break signal state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BreakType {
    /// Break off (normal operation).
    #[default]
    Off,
    /// Break on (hold TX low).
    On,
}

impl BreakType {
    /// Wire encoding for the SIO_SET_DATA request.
    pub(crate) fn wire_value(self) -> u16 {
        match self {
            Self::Off => 0x00,
            Self::On => 0x01,
        }
    }
}

/// Bitbang / MPSSE mode selection.
///
/// Used with [`FtdiDevice::set_bitmode`](crate::FtdiDevice::set_bitmode).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BitMode {
    /// Normal serial/FIFO mode (bitbang disabled).
    #[default]
    Reset,
    /// Asynchronous bitbang mode (B-type and later).
    BitBang,
    /// MPSSE mode (FT2232x and later).
    Mpsse,
    /// Synchronous bitbang mode (FT2232x, FT232R and later).
    SyncBB,
    /// MCU host bus emulation mode (FT2232x).
    Mcu,
    /// Fast opto-isolated serial mode (FT2232x).
    Opto,
    /// CBUS bitbang mode (FT232R, configure in EEPROM first).
    Cbus,
    /// Synchronous FIFO mode (FT2232H).
    SyncFf,
    /// FT1284 mode (FT232H).
    Ft1284,
}

impl BitMode {
    /// Wire value for the SIO_SET_BITMODE request.
    pub(crate) fn wire_value(self) -> u8 {
        match self {
            Self::Reset => 0x00,
            Self::BitBang => 0x01,
            Self::Mpsse => 0x02,
            Self::SyncBB => 0x04,
            Self::Mcu => 0x08,
            Self::Opto => 0x10,
            Self::Cbus => 0x20,
            Self::SyncFf => 0x40,
            Self::Ft1284 => 0x80,
        }
    }
}

/// Port interface selection for multi-interface chips.
///
/// Chips like the FT2232H (dual) and FT4232H (quad) expose multiple
/// independent interfaces. Select which one to use before opening.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Interface {
    /// Use the first available interface (same as `A`).
    #[default]
    Any,
    /// Interface A (port 0).
    A,
    /// Interface B (port 1).
    B,
    /// Interface C (port 2, FT4232H only).
    C,
    /// Interface D (port 3, FT4232H only).
    D,
}

/// Kernel module detach behavior when opening a device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ModuleDetachMode {
    /// Automatically detach the kernel module (e.g. `ftdi_sio`).
    #[default]
    AutoDetach,
    /// Do not detach the kernel module.
    DontDetach,
    /// Detach on open, re-attach on close.
    AutoDetachReattach,
}

/// Flow control mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum FlowControl {
    /// No flow control.
    #[default]
    Disabled,
    /// Hardware RTS/CTS flow control.
    RtsCts,
    /// Hardware DTR/DSR flow control.
    DtrDsr,
    /// Software XON/XOFF flow control.
    ///
    /// The `xon` and `xoff` fields specify the characters that signal
    /// the remote end to resume (XON) and pause (XOFF) transmission.
    /// Common values are `xon = 0x11` (DC1) and `xoff = 0x13` (DC3).
    XonXoff {
        /// The XON character (resume transmission). Default: `0x11` (DC1).
        xon: u8,
        /// The XOFF character (pause transmission). Default: `0x13` (DC3).
        xoff: u8,
    },
}

/// Decoded modem status from the FTDI chip.
///
/// The FTDI chip sends two status bytes as a header with every USB read.
/// This struct represents the decoded content.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModemStatus {
    raw: u16,
}

impl ModemStatus {
    /// Create from the raw two-byte status value.
    pub(crate) fn from_raw(raw: u16) -> Self {
        Self { raw }
    }

    /// Raw 16-bit status value.
    pub fn raw(self) -> u16 {
        self.raw
    }

    // -- Byte 0 (modem status lines) --

    /// Clear To Send (CTS) is active.
    pub fn cts(self) -> bool {
        self.raw & 0x10 != 0
    }

    /// Data Set Ready (DSR) is active.
    pub fn dsr(self) -> bool {
        self.raw & 0x20 != 0
    }

    /// Ring Indicator (RI) is active.
    pub fn ri(self) -> bool {
        self.raw & 0x40 != 0
    }

    /// Receive Line Signal Detect (RLSD / DCD) is active.
    pub fn rlsd(self) -> bool {
        self.raw & 0x80 != 0
    }

    // -- Byte 1 (line status) --

    /// Data Ready (DR).
    pub fn data_ready(self) -> bool {
        self.raw & 0x0100 != 0
    }

    /// Overrun Error (OE).
    pub fn overrun_error(self) -> bool {
        self.raw & 0x0200 != 0
    }

    /// Parity Error (PE).
    pub fn parity_error(self) -> bool {
        self.raw & 0x0400 != 0
    }

    /// Framing Error (FE).
    pub fn framing_error(self) -> bool {
        self.raw & 0x0800 != 0
    }

    /// Break Interrupt (BI).
    pub fn break_interrupt(self) -> bool {
        self.raw & 0x1000 != 0
    }

    /// Transmitter Holding Register Empty (THRE).
    pub fn transmitter_holding_empty(self) -> bool {
        self.raw & 0x2000 != 0
    }

    /// Transmitter Empty (TEMT).
    pub fn transmitter_empty(self) -> bool {
        self.raw & 0x4000 != 0
    }

    /// Error in RCVR FIFO.
    pub fn fifo_error(self) -> bool {
        self.raw & 0x8000 != 0
    }
}

/// Interface configuration resolved to concrete USB endpoint values.
#[derive(Debug, Clone, Copy)]
pub(crate) struct InterfaceConfig {
    /// The USB interface number (0-based).
    pub interface_num: u8,
    /// The USB index value used in control transfers (1-based).
    pub usb_index: u16,
    /// The bulk OUT endpoint address (host-to-device, for writing data).
    pub write_ep: u8,
    /// The bulk IN endpoint address (device-to-host, for reading data).
    pub read_ep: u8,
}

impl Interface {
    /// Resolve to concrete USB endpoint configuration.
    pub(crate) fn config(self) -> InterfaceConfig {
        match self {
            Self::Any | Self::A => InterfaceConfig {
                interface_num: 0,
                usb_index: 1,
                write_ep: 0x02,
                read_ep: 0x81,
            },
            Self::B => InterfaceConfig {
                interface_num: 1,
                usb_index: 2,
                write_ep: 0x04,
                read_ep: 0x83,
            },
            Self::C => InterfaceConfig {
                interface_num: 2,
                usb_index: 3,
                write_ep: 0x06,
                read_ep: 0x85,
            },
            Self::D => InterfaceConfig {
                interface_num: 3,
                usb_index: 4,
                write_ep: 0x08,
                read_ep: 0x87,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- ChipType tests ----

    #[test]
    fn h_type_detection() {
        assert!(ChipType::Ft2232H.is_h_type());
        assert!(ChipType::Ft4232H.is_h_type());
        assert!(ChipType::Ft232H.is_h_type());
        assert!(!ChipType::Ft232R.is_h_type());
        assert!(!ChipType::Ft230X.is_h_type());
        assert!(!ChipType::Bm.is_h_type());
        assert!(!ChipType::Am.is_h_type());
        assert!(!ChipType::Ft2232C.is_h_type());
    }

    #[test]
    fn multi_channel_detection() {
        // Multi-channel chips
        assert!(ChipType::Ft2232C.is_multi_channel());
        assert!(ChipType::Ft2232H.is_multi_channel());
        assert!(ChipType::Ft4232H.is_multi_channel());
        // Single-channel chips
        assert!(!ChipType::Am.is_multi_channel());
        assert!(!ChipType::Bm.is_multi_channel());
        assert!(!ChipType::Ft232R.is_multi_channel());
        assert!(!ChipType::Ft232H.is_multi_channel());
        assert!(!ChipType::Ft230X.is_multi_channel());
    }

    #[test]
    fn chip_type_product_ids() {
        assert_eq!(ChipType::Am.default_product_id(), 0x6001);
        assert_eq!(ChipType::Bm.default_product_id(), 0x6001);
        assert_eq!(ChipType::Ft232R.default_product_id(), 0x6001);
        assert_eq!(ChipType::Ft2232C.default_product_id(), 0x6010);
        assert_eq!(ChipType::Ft2232H.default_product_id(), 0x6010);
        assert_eq!(ChipType::Ft4232H.default_product_id(), 0x6011);
        assert_eq!(ChipType::Ft232H.default_product_id(), 0x6014);
        assert_eq!(ChipType::Ft230X.default_product_id(), 0x6015);
    }

    #[test]
    fn chip_type_release_numbers_ascending() {
        // Release numbers should be distinct and ascending by generation
        let types = [
            ChipType::Am,
            ChipType::Bm,
            ChipType::Ft2232C,
            ChipType::Ft232R,
            ChipType::Ft2232H,
            ChipType::Ft4232H,
            ChipType::Ft232H,
            ChipType::Ft230X,
        ];
        for i in 1..types.len() {
            assert!(
                types[i].release_number() > types[i - 1].release_number(),
                "{:?} release ({:#06X}) should be > {:?} release ({:#06X})",
                types[i],
                types[i].release_number(),
                types[i - 1],
                types[i - 1].release_number()
            );
        }
    }

    // ---- Interface endpoint mapping tests ----

    #[test]
    fn interface_a_endpoints() {
        let cfg = Interface::A.config();
        assert_eq!(cfg.interface_num, 0);
        assert_eq!(cfg.usb_index, 1);
        assert_eq!(cfg.write_ep, 0x02);
        assert_eq!(cfg.read_ep, 0x81);
    }

    #[test]
    fn interface_any_equals_a() {
        let a = Interface::A.config();
        let any = Interface::Any.config();
        assert_eq!(a.interface_num, any.interface_num);
        assert_eq!(a.usb_index, any.usb_index);
        assert_eq!(a.write_ep, any.write_ep);
        assert_eq!(a.read_ep, any.read_ep);
    }

    #[test]
    fn interface_b_endpoints() {
        let cfg = Interface::B.config();
        assert_eq!(cfg.interface_num, 1);
        assert_eq!(cfg.usb_index, 2);
        assert_eq!(cfg.write_ep, 0x04);
        assert_eq!(cfg.read_ep, 0x83);
    }

    #[test]
    fn interface_c_endpoints() {
        let cfg = Interface::C.config();
        assert_eq!(cfg.interface_num, 2);
        assert_eq!(cfg.usb_index, 3);
        assert_eq!(cfg.write_ep, 0x06);
        assert_eq!(cfg.read_ep, 0x85);
    }

    #[test]
    fn interface_d_endpoints() {
        let cfg = Interface::D.config();
        assert_eq!(cfg.interface_num, 3);
        assert_eq!(cfg.usb_index, 4);
        assert_eq!(cfg.write_ep, 0x08);
        assert_eq!(cfg.read_ep, 0x87);
    }

    #[test]
    fn endpoint_addresses_have_correct_direction_bits() {
        // Write endpoints (host->device): bit 7 should be 0
        // Read endpoints (device->host): bit 7 should be 1
        for iface in [Interface::A, Interface::B, Interface::C, Interface::D] {
            let cfg = iface.config();
            assert_eq!(
                cfg.write_ep & 0x80,
                0x00,
                "{:?} write_ep should have bit 7 clear",
                iface
            );
            assert_eq!(
                cfg.read_ep & 0x80,
                0x80,
                "{:?} read_ep should have bit 7 set",
                iface
            );
        }
    }

    // ---- Wire encoding tests ----

    #[test]
    fn parity_wire_values() {
        assert_eq!(Parity::None.wire_value(), 0x00);
        assert_eq!(Parity::Odd.wire_value(), 0x01);
        assert_eq!(Parity::Even.wire_value(), 0x02);
        assert_eq!(Parity::Mark.wire_value(), 0x03);
        assert_eq!(Parity::Space.wire_value(), 0x04);
    }

    #[test]
    fn stop_bits_wire_values() {
        assert_eq!(StopBits::One.wire_value(), 0x00);
        assert_eq!(StopBits::OnePointFive.wire_value(), 0x01);
        assert_eq!(StopBits::Two.wire_value(), 0x02);
    }

    #[test]
    fn data_bits_wire_values() {
        assert_eq!(DataBits::Seven.wire_value(), 7);
        assert_eq!(DataBits::Eight.wire_value(), 8);
    }

    #[test]
    fn break_type_wire_values() {
        assert_eq!(BreakType::Off.wire_value(), 0x00);
        assert_eq!(BreakType::On.wire_value(), 0x01);
    }

    #[test]
    fn bitmode_wire_values() {
        assert_eq!(BitMode::Reset.wire_value(), 0x00);
        assert_eq!(BitMode::BitBang.wire_value(), 0x01);
        assert_eq!(BitMode::Mpsse.wire_value(), 0x02);
        assert_eq!(BitMode::SyncBB.wire_value(), 0x04);
        assert_eq!(BitMode::Mcu.wire_value(), 0x08);
        assert_eq!(BitMode::Opto.wire_value(), 0x10);
        assert_eq!(BitMode::Cbus.wire_value(), 0x20);
        assert_eq!(BitMode::SyncFf.wire_value(), 0x40);
        assert_eq!(BitMode::Ft1284.wire_value(), 0x80);
    }

    // ---- ModemStatus tests ----

    #[test]
    fn modem_status_cts() {
        let ms = ModemStatus::from_raw(0x0010);
        assert!(ms.cts());
        assert!(!ms.dsr());
        assert!(!ms.ri());
        assert!(!ms.rlsd());
    }

    #[test]
    fn modem_status_all_lines() {
        let ms = ModemStatus::from_raw(0x00F0);
        assert!(ms.cts());
        assert!(ms.dsr());
        assert!(ms.ri());
        assert!(ms.rlsd());
    }

    #[test]
    fn modem_status_line_errors() {
        let ms = ModemStatus::from_raw(0xFF00);
        assert!(ms.data_ready());
        assert!(ms.overrun_error());
        assert!(ms.parity_error());
        assert!(ms.framing_error());
        assert!(ms.break_interrupt());
        assert!(ms.transmitter_holding_empty());
        assert!(ms.transmitter_empty());
        assert!(ms.fifo_error());
    }

    #[test]
    fn modem_status_raw_round_trip() {
        let ms = ModemStatus::from_raw(0x1234);
        assert_eq!(ms.raw(), 0x1234);
    }
}
