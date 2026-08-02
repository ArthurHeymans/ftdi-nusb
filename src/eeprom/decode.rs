//! EEPROM decoding: parse a binary EEPROM image into an [`FtdiEeprom`] struct.

use crate::error::{Error, Result};
use crate::types::ChipType;

use super::FtdiEeprom;
use super::build::checksum;

/// Decode a string descriptor from the EEPROM buffer.
///
/// The EEPROM stores strings as USB string descriptors: length byte,
/// type byte (0x03), then UTF-16LE characters. We only take the low
/// byte of each character (ASCII subset).
fn decode_string(
    buf: &[u8],
    eeprom_size: usize,
    offset_addr: usize,
    length_addr: usize,
) -> Option<String> {
    let raw_len = buf[length_addr] as usize;
    let char_count = raw_len / 2;
    if char_count <= 1 {
        return None;
    }

    let mask = eeprom_size - 1;
    let start = (buf[offset_addr] as usize) & mask;

    let mut s = String::with_capacity(char_count - 1);
    for j in 0..char_count - 1 {
        let idx = (start + 2 + j * 2) & mask;
        if idx < buf.len() {
            s.push(buf[idx] as char);
        }
    }

    Some(s)
}

/// Decode a binary EEPROM image into an [`FtdiEeprom`] structure.
///
/// The `chip_type` is needed to interpret chip-specific fields.
/// The EEPROM checksum is verified; an error is returned on mismatch.
pub fn decode(eeprom: &mut FtdiEeprom, chip_type: ChipType) -> Result<()> {
    let eeprom_size = eeprom.size as usize;

    if eeprom_size == 0 || eeprom_size > eeprom.buf.len() {
        return Err(Error::Eeprom("invalid EEPROM size".into()));
    }

    // Verify checksum
    let computed = checksum(&eeprom.buf, eeprom_size, chip_type == ChipType::Ft230X);
    let stored = (eeprom.buf[eeprom_size - 2] as u16) | ((eeprom.buf[eeprom_size - 1] as u16) << 8);
    if computed != stored {
        return Err(Error::EepromChecksum);
    }

    // Common fields
    eeprom.vendor_id = (eeprom.buf[0x02] as u16) | ((eeprom.buf[0x03] as u16) << 8);
    eeprom.product_id = (eeprom.buf[0x04] as u16) | ((eeprom.buf[0x05] as u16) << 8);
    eeprom.release_number = (eeprom.buf[0x06] as u16) | ((eeprom.buf[0x07] as u16) << 8);
    eeprom.self_powered = eeprom.buf[0x08] & 0x40 != 0;
    eeprom.remote_wakeup = eeprom.buf[0x08] & 0x20 != 0;
    eeprom.max_power = (eeprom.buf[0x09] as u16) * 2;

    // Config byte 0x0A
    eeprom.in_is_isochronous = eeprom.buf[0x0A] & 0x01 != 0;
    eeprom.out_is_isochronous = eeprom.buf[0x0A] & 0x02 != 0;
    eeprom.suspend_pull_downs = eeprom.buf[0x0A] & 0x04 != 0;
    eeprom.use_serial = eeprom.buf[0x0A] & 0x08 != 0;
    eeprom.use_usb_version = eeprom.buf[0x0A] & 0x10 != 0;

    // USB version
    eeprom.usb_version = (eeprom.buf[0x0C] as u16) | ((eeprom.buf[0x0D] as u16) << 8);

    // Strings
    eeprom.manufacturer = decode_string(&eeprom.buf, eeprom_size, 0x0E, 0x0F);
    eeprom.product = decode_string(&eeprom.buf, eeprom_size, 0x10, 0x11);
    eeprom.serial = decode_string(&eeprom.buf, eeprom_size, 0x12, 0x13);

    // Reset channel fields
    eeprom.channel_a_type = 0;

    // Chip-specific decoding
    match chip_type {
        ChipType::Am | ChipType::Bm => {
            eeprom.chip = -1;
        }
        ChipType::Ft2232C => {
            eeprom.channel_a_type = eeprom.buf[0x00] & 0x07;
            eeprom.channel_a_driver = eeprom.buf[0x00] & 0x08 != 0;
            eeprom.high_current_a = eeprom.buf[0x00] & 0x10 != 0;
            eeprom.channel_b_type = eeprom.buf[0x01] & 0x07;
            eeprom.channel_b_driver = eeprom.buf[0x01] & 0x08 != 0;
            eeprom.high_current_b = eeprom.buf[0x01] & 0x10 != 0;
            eeprom.chip = eeprom.buf[0x14] as i32;
        }
        ChipType::Ft232R => {
            // R-type inverts the VCP flag
            eeprom.channel_a_driver = eeprom.buf[0x00] & 0x08 == 0;
            eeprom.high_current = eeprom.buf[0x00] & 0x04 != 0;
            eeprom.external_oscillator = eeprom.buf[0x00] & 0x02 != 0;
            eeprom.invert = eeprom.buf[0x0B];
            eeprom.chip = eeprom.buf[0x16] as i32;
            // CBUS functions
            eeprom.cbus_function[0] = eeprom.buf[0x14] & 0x0F;
            eeprom.cbus_function[1] = (eeprom.buf[0x14] >> 4) & 0x0F;
            eeprom.cbus_function[2] = eeprom.buf[0x15] & 0x0F;
            eeprom.cbus_function[3] = (eeprom.buf[0x15] >> 4) & 0x0F;
            eeprom.cbus_function[4] = eeprom.buf[0x16] & 0x0F;
        }
        ChipType::Ft2232H => {
            eeprom.channel_a_type = eeprom.buf[0x00] & 0x07;
            eeprom.channel_a_driver = eeprom.buf[0x00] & 0x08 != 0;
            eeprom.channel_b_type = eeprom.buf[0x01] & 0x07;
            eeprom.channel_b_driver = eeprom.buf[0x01] & 0x08 != 0;
            eeprom.suspend_dbus7 = eeprom.buf[0x01] & 0x80 != 0;
            eeprom.chip = eeprom.buf[0x18] as i32;
            decode_drive_groups(eeprom);
        }
        ChipType::Ft4232H => {
            eeprom.channel_a_driver = eeprom.buf[0x00] & 0x08 != 0;
            eeprom.channel_b_driver = eeprom.buf[0x01] & 0x08 != 0;
            eeprom.channel_c_driver = (eeprom.buf[0x00] >> 4) & 0x08 != 0;
            eeprom.channel_d_driver = (eeprom.buf[0x01] >> 4) & 0x08 != 0;
            eeprom.channel_a_rs485enable = eeprom.buf[0x0B] & 0x10 != 0;
            eeprom.channel_b_rs485enable = eeprom.buf[0x0B] & 0x20 != 0;
            eeprom.channel_c_rs485enable = eeprom.buf[0x0B] & 0x40 != 0;
            eeprom.channel_d_rs485enable = eeprom.buf[0x0B] & 0x80 != 0;
            eeprom.chip = eeprom.buf[0x18] as i32;
            decode_drive_groups(eeprom);
        }
        ChipType::Ft232H => {
            eeprom.channel_a_type = eeprom.buf[0x00] & 0x0F;
            eeprom.channel_a_driver = eeprom.buf[0x00] & 0x10 != 0;
            eeprom.clock_polarity = eeprom.buf[0x01] & 0x01 != 0;
            eeprom.data_order = eeprom.buf[0x01] & 0x02 != 0;
            eeprom.flow_control = eeprom.buf[0x01] & 0x04 != 0;
            eeprom.powersave = eeprom.buf[0x01] & 0x80 != 0;
            eeprom.group0_drive = eeprom.buf[0x0C] & 0x03;
            eeprom.group0_schmitt = eeprom.buf[0x0C] & 0x08 != 0;
            eeprom.group0_slew = eeprom.buf[0x0C] & 0x04 != 0;
            eeprom.group1_drive = eeprom.buf[0x0D] & 0x03;
            eeprom.group1_schmitt = eeprom.buf[0x0D] & 0x08 != 0;
            eeprom.group1_slew = eeprom.buf[0x0D] & 0x04 != 0;
            // CBUS functions (10 pins in 5 bytes)
            for i in 0..5 {
                eeprom.cbus_function[2 * i] = eeprom.buf[0x18 + i] & 0x0F;
                eeprom.cbus_function[2 * i + 1] = (eeprom.buf[0x18 + i] >> 4) & 0x0F;
            }
            eeprom.chip = eeprom.buf[0x1E] as i32;
        }
        ChipType::Ft230X => {
            for i in 0..4 {
                eeprom.cbus_function[i] = eeprom.buf[0x1A + i];
            }
            eeprom.group0_drive = eeprom.buf[0x0C] & 0x03;
            eeprom.group0_schmitt = eeprom.buf[0x0C] & 0x08 != 0;
            eeprom.group0_slew = eeprom.buf[0x0C] & 0x04 != 0;
            eeprom.group1_drive = (eeprom.buf[0x0C] >> 4) & 0x03;
            eeprom.group1_schmitt = (eeprom.buf[0x0C] >> 4) & 0x08 != 0;
            eeprom.group1_slew = (eeprom.buf[0x0C] >> 4) & 0x04 != 0;
            eeprom.invert = eeprom.buf[0x0B];
        }
    }

    Ok(())
}

/// Decode the 4 IO drive groups from bytes 0x0C and 0x0D.
fn decode_drive_groups(eeprom: &mut FtdiEeprom) {
    eeprom.group0_drive = eeprom.buf[0x0C] & 0x03;
    eeprom.group0_schmitt = eeprom.buf[0x0C] & 0x08 != 0;
    eeprom.group0_slew = eeprom.buf[0x0C] & 0x04 != 0;
    eeprom.group1_drive = (eeprom.buf[0x0C] >> 4) & 0x03;
    eeprom.group1_schmitt = (eeprom.buf[0x0C] >> 4) & 0x08 != 0;
    eeprom.group1_slew = (eeprom.buf[0x0C] >> 4) & 0x04 != 0;
    eeprom.group2_drive = eeprom.buf[0x0D] & 0x03;
    eeprom.group2_schmitt = eeprom.buf[0x0D] & 0x08 != 0;
    eeprom.group2_slew = eeprom.buf[0x0D] & 0x04 != 0;
    eeprom.group3_drive = (eeprom.buf[0x0D] >> 4) & 0x03;
    eeprom.group3_schmitt = (eeprom.buf[0x0D] >> 4) & 0x08 != 0;
    eeprom.group3_slew = (eeprom.buf[0x0D] >> 4) & 0x04 != 0;
}
