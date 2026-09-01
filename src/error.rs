//! Error types for the ftdi crate.

use crate::types::ChipType;

/// The error type for FTDI operations.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// An error from the nusb USB layer.
    #[error("USB error: {0}")]
    Usb(#[from] nusb::Error),

    /// A USB transfer error.
    #[error("USB transfer error: {0}")]
    Transfer(#[from] nusb::transfer::TransferError),

    /// No matching device was found.
    #[error("device not found")]
    DeviceNotFound,

    /// The USB device is unavailable (not opened or disconnected).
    #[error("USB device unavailable")]
    DeviceUnavailable,

    /// Invalid argument(s) were provided.
    #[error("invalid argument: {0}")]
    InvalidArgument(&'static str),

    /// The requested baud rate cannot be achieved within tolerance.
    #[error("unsupported baud rate: requested {requested}, nearest achievable {actual}")]
    UnsupportedBaudRate {
        /// The requested baud rate.
        requested: u32,
        /// The nearest achievable baud rate.
        actual: u32,
    },

    /// An EEPROM-related error.
    #[error("EEPROM error: {0}")]
    Eeprom(String),

    /// EEPROM checksum verification failed.
    #[error("EEPROM checksum mismatch")]
    EepromChecksum,

    /// EEPROM size exceeded by custom strings.
    #[error("EEPROM size exceeded by custom strings")]
    EepromSizeExceeded,

    /// The operation is not supported for this chip type.
    #[error("unsupported operation for chip type {0:?}")]
    UnsupportedChip(ChipType),

    /// Could not set USB configuration.
    #[error("unable to set USB configuration; make sure the default FTDI driver is not in use")]
    Configuration,

    /// Could not claim the USB interface.
    #[error("unable to claim USB device; make sure the default FTDI driver is not in use")]
    ClaimFailed,

    /// A write operation completed with zero bytes transferred.
    #[error("write returned zero bytes")]
    WriteZero,

    /// A USB write completed without transferring the full submitted buffer.
    #[error("short USB write: expected {expected} bytes, wrote {actual}")]
    ShortWrite {
        /// Number of bytes submitted.
        expected: usize,
        /// Number of bytes reported as transferred.
        actual: usize,
    },

    /// A cancelled or failed stateful operation left device state uncertain.
    #[error("device recovery is required before further I/O")]
    RecoveryRequired,

    /// An MPSSE context predates the most recent device recovery.
    #[error("MPSSE context is invalid after recovery; initialize a new context")]
    InvalidMpsseContext,

    /// USB reset failed.
    #[error("USB reset failed")]
    ResetFailed,

    /// An I2C operation received a NACK from the slave device.
    #[error("I2C NACK: {0}")]
    I2cNack(&'static str),

    /// The MPSSE engine returned a bad-command response (0xFA).
    ///
    /// This means the MPSSE received an opcode it doesn't recognize.
    /// The `u8` value is the rejected opcode.
    #[error("MPSSE bad command: rejected opcode 0x{0:02X}")]
    MpsseBadCommand(u8),

    /// A descriptor read failed.
    #[error("descriptor error: {0}")]
    Descriptor(#[from] nusb::GetDescriptorError),

    /// A USB operation timed out.
    ///
    /// This is distinct from `Transfer` errors — it indicates that the
    /// device did not respond within the configured timeout period.
    #[error("operation timed out after {0:?}")]
    Timeout(core::time::Duration),

    /// A device open operation failed (e.g., WebUSB request denied).
    #[error("device open failed: {0}")]
    OpenFailed(String),
}

/// A specialized `Result` type for FTDI operations.
pub type Result<T> = core::result::Result<T, Error>;
