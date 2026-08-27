//! EEPROM USB I/O operations: reading, writing, and erasing the physical EEPROM.

use crate::constants::*;
use crate::context::FtdiDevice;
use crate::error::{Error, Result};
use crate::types::ChipType;

const MAGIC: u16 = 0x55AA;

impl FtdiDevice {
    /// Read the entire EEPROM from the device.
    ///
    /// Performs 128 USB control transfers (2 bytes each) to read the full
    /// 256-byte EEPROM. The result is stored in `self.eeprom.buf`.
    ///
    /// After reading, the EEPROM size is auto-detected by comparing halves
    /// of the buffer.
    /// If this future is cancelled mid-read, the partially filled buffer is
    /// inconsistent; the device is poisoned until [`recover`](Self::recover).
    pub async fn read_eeprom(&mut self) -> Result<()> {
        let guard = self.begin_stateful_operation()?;
        for i in 0..(FTDI_MAX_EEPROM_SIZE / 2) {
            let data = self
                .control_in(SIO_READ_EEPROM_REQUEST, 0, i as u16, 2)
                .await?;
            if data.len() < 2 {
                return Err(Error::Eeprom("EEPROM read failed: short transfer".into()));
            }
            self.eeprom.buf[i * 2] = data[0];
            self.eeprom.buf[i * 2 + 1] = data[1];
        }

        // Auto-detect EEPROM size (matching libftdi's ftdi_read_eeprom logic)
        if self.chip_type() == ChipType::Ft232R {
            self.eeprom.size = 0x80;
        } else {
            let buf = &self.eeprom.buf;
            // Check for blank EEPROM: last byte is 0xFF and it's the last occurrence
            // (C uses strrchr to find last 0xFF position == end of buffer)
            if buf[FTDI_MAX_EEPROM_SIZE - 1] == 0xFF && buf.iter().all(|&b| b == 0xFF) {
                self.eeprom.size = -1; // Blank EEPROM
            } else if buf[..0x80] == buf[0x80..0x100] {
                // First half equals second half -> 128 bytes (93x56 wrap)
                self.eeprom.size = 0x80;
            } else if buf[..0x40] == buf[0x40..0x80] {
                // First quarter equals second quarter -> 64 bytes (93x46 wrap)
                self.eeprom.size = 0x40;
            } else {
                self.eeprom.size = 0x100;
            }
        }

        guard.disarm();
        Ok(())
    }

    /// Write the entire EEPROM to the device.
    ///
    /// The EEPROM must have been initialized (via `eeprom_build` or manual
    /// setup). Performs the same initialization sequence observed from FTDI's
    /// MProg tool.
    /// If this future is cancelled mid-write, the EEPROM holds a partial
    /// image; the device is poisoned until [`recover`](Self::recover).
    pub async fn write_eeprom(&mut self) -> Result<()> {
        if !self.eeprom.initialized_for_connected_device {
            return Err(Error::Eeprom(
                "EEPROM not initialized for the connected device".into(),
            ));
        }

        if self.eeprom.size <= 0 {
            return Err(Error::Eeprom("invalid EEPROM size".into()));
        }
        let eeprom_size = self.eeprom.size as usize;
        let chip_type = self.chip_type();
        let guard = self.begin_stateful_operation()?;

        // Initialization sequence (from MProg traces)
        self.usb_reset().await?;
        let _ = self.poll_modem_status().await;
        let _ = self.set_latency_timer(0x77).await;

        for i in 0..eeprom_size / 2 {
            // Skip reserved area on FT230X
            if chip_type == ChipType::Ft230X && i == 0x40 {
                continue;
            }
            if chip_type == ChipType::Ft230X && (0x40..0x50).contains(&i) {
                continue;
            }

            let val = (self.eeprom.buf[i * 2] as u16) | ((self.eeprom.buf[i * 2 + 1] as u16) << 8);

            self.control_out(SIO_WRITE_EEPROM_REQUEST, val, i as u16)
                .await?;
        }

        guard.disarm();
        Ok(())
    }

    /// Erase the EEPROM.
    ///
    /// After erasing, the EEPROM chip type is auto-detected using a magic
    /// word wraparound test (93x46 vs 93x56 vs 93x66).
    ///
    /// Not supported on FT232R/FT245R (internal EEPROM) or FT230X (MTP).
    /// If this future is cancelled between the wraparound test and the final
    /// erase, the magic word remains programmed; the device is poisoned until
    /// [`recover`](Self::recover).
    pub async fn erase_eeprom(&mut self) -> Result<()> {
        let chip_type = self.chip_type();

        if chip_type == ChipType::Ft232R || chip_type == ChipType::Ft230X {
            self.eeprom.chip = 0;
            return Ok(());
        }

        let guard = self.begin_stateful_operation()?;
        self.control_out(SIO_ERASE_EEPROM_REQUEST, 0, 0).await?;

        // Detect EEPROM chip type via wraparound test
        self.control_out(SIO_WRITE_EEPROM_REQUEST, MAGIC, 0xC0)
            .await?;

        let val = self.read_eeprom_location(0x00).await?;
        if val == MAGIC {
            self.eeprom.chip = 0x46; // 93x46
        } else {
            let val = self.read_eeprom_location(0x40).await?;
            if val == MAGIC {
                self.eeprom.chip = 0x56; // 93x56
            } else {
                let val = self.read_eeprom_location(0xC0).await?;
                if val == MAGIC {
                    self.eeprom.chip = 0x66; // 93x66
                } else {
                    self.eeprom.chip = -1;
                }
            }
        }

        // Erase again to clean up the magic word
        self.control_out(SIO_ERASE_EEPROM_REQUEST, 0, 0).await?;

        guard.disarm();
        Ok(())
    }

    /// Read a single 16-bit EEPROM location.
    pub async fn read_eeprom_location(&self, addr: u16) -> Result<u16> {
        let data = self.control_in(SIO_READ_EEPROM_REQUEST, 0, addr, 2).await?;
        if data.len() < 2 {
            return Err(Error::Eeprom("EEPROM read location failed".into()));
        }
        Ok((data[0] as u16) | ((data[1] as u16) << 8))
    }

    /// Write a single 16-bit EEPROM location.
    ///
    /// Only valid for addresses >= 0x80 on 93x66 EEPROMs.
    pub async fn write_eeprom_location(&self, addr: u16, value: u16) -> Result<()> {
        if addr < 0x80 {
            return Err(Error::InvalidArgument(
                "cannot write to checksum-protected area below 0x80",
            ));
        }
        self.control_out(SIO_WRITE_EEPROM_REQUEST, value, addr)
            .await
    }

    /// Read the FTDIChip-ID from R-type devices.
    ///
    /// The chip ID is a unique identifier burned into the silicon.
    pub async fn read_chipid(&self) -> Result<u32> {
        let a_data = self.control_in(SIO_READ_EEPROM_REQUEST, 0, 0x43, 2).await?;
        let b_data = self.control_in(SIO_READ_EEPROM_REQUEST, 0, 0x44, 2).await?;

        if a_data.len() < 2 || b_data.len() < 2 {
            return Err(Error::Eeprom("read of FTDIChip-ID failed".into()));
        }

        let mut a = ((a_data[0] as u32) | ((a_data[1] as u32) << 8)).swap_bytes() >> 16;
        let b = ((b_data[0] as u32) | ((b_data[1] as u32) << 8)).swap_bytes() >> 16;
        a = (a << 16) | (b & 0xFFFF);

        let shifted = chipid_shift(a)
            | (chipid_shift(a >> 8) << 8)
            | (chipid_shift(a >> 16) << 16)
            | (chipid_shift(a >> 24) << 24);

        Ok(shifted ^ 0xA5F0_F7D1)
    }

    /// Decode the EEPROM buffer into the decoded structure fields.
    pub fn eeprom_decode(&mut self) -> Result<()> {
        let chip_type = self.chip_type();
        super::decode::decode(&mut self.eeprom, chip_type)
    }

    /// Build the EEPROM binary image from the decoded structure.
    ///
    /// Returns the number of bytes available for user data.
    ///
    /// For FT230X devices, this reads the factory configuration data
    /// (addresses 0x40-0x4F) from the device to include in the checksum,
    /// matching the behavior of `ftdi_eeprom_build()` in libftdi.
    pub async fn eeprom_build(&mut self) -> Result<usize> {
        let chip_type = self.chip_type();

        // For FT230X, read factory data area (0x40-0x4F) from the device
        // so it's included in the checksum calculation.
        if chip_type == ChipType::Ft230X {
            // The factory-area reads mutate the EEPROM buffer across awaits;
            // poison the device if this future is dropped mid-loop so a
            // half-updated buffer is never written back.
            let guard = self.begin_stateful_operation()?;
            for i in 0x40..0x50u16 {
                if let Ok(data) = self.control_in(SIO_READ_EEPROM_REQUEST, 0, i, 2).await {
                    if data.len() >= 2 {
                        self.eeprom.buf[(i as usize) * 2] = data[0];
                        self.eeprom.buf[(i as usize) * 2 + 1] = data[1];
                    }
                }
            }
            guard.disarm();
        }

        super::build::build(&mut self.eeprom, chip_type)
    }

    /// Initialize the EEPROM with chip-appropriate defaults.
    ///
    /// Sets vendor/product IDs, USB version, CBUS functions, EEPROM size,
    /// and other fields to sensible defaults for the connected chip. If
    /// string parameters are `None`, appropriate defaults are used.
    ///
    /// This is the equivalent of `ftdi_eeprom_initdefaults()` from libftdi.
    pub fn eeprom_init_defaults(
        &mut self,
        manufacturer: Option<&str>,
        product: Option<&str>,
        serial: Option<&str>,
    ) {
        let chip_type = self.chip_type();
        self.eeprom
            .init_defaults(chip_type, manufacturer, product, serial);
    }

    /// Get a reference to the EEPROM structure.
    pub fn eeprom(&self) -> &super::FtdiEeprom {
        &self.eeprom
    }

    /// Get a mutable reference to the EEPROM structure.
    pub fn eeprom_mut(&mut self) -> &mut super::FtdiEeprom {
        &mut self.eeprom
    }
}

/// Bitshift operation for FTDIChip-ID decoding.
fn chipid_shift(value: u32) -> u32 {
    let v = value as u8;
    (((v & 1) << 1)
        | ((v & 2) << 5)
        | ((v & 4) >> 2)
        | ((v & 8) << 4)
        | ((v & 16) >> 1)
        | ((v & 32) >> 1)
        | ((v & 64) >> 4)
        | ((v & 128) >> 2)) as u32
}
