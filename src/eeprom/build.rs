//! EEPROM encoding: convert a decoded [`FtdiEeprom`] struct into the binary format.

use crate::constants::*;
use crate::error::{Error, Result};
use crate::types::ChipType;

use super::FtdiEeprom;

/// Compute the EEPROM checksum over a buffer.
///
/// The algorithm is: XOR each 16-bit word, then rotate-left-1 the accumulator.
/// Starting seed is 0xAAAA.
pub(crate) fn checksum(buf: &[u8], size: usize, is_230x: bool) -> u16 {
    let mut csum: u16 = 0xAAAA;
    let mut i = 0;
    while i < size / 2 - 1 {
        if is_230x && i == 0x12 {
            // FT230X has a user section that is not part of the checksum
            i = 0x40;
        }
        let value = (buf[i * 2] as u16) | ((buf[i * 2 + 1] as u16) << 8);
        csum ^= value;
        csum = csum.rotate_left(1);
        i += 1;
    }
    csum
}

/// Encode a UTF-16LE string descriptor into the EEPROM buffer at position `pos`.
///
/// Returns the number of bytes written (including the length/type header).
fn write_string_descriptor(buf: &mut [u8], mask: u8, pos: &mut usize, s: &str) -> usize {
    let char_count = s.len();
    let total_len = char_count * 2 + 2; // UTF-16LE + length/type

    let idx = *pos & (mask as usize);
    buf[idx] = total_len as u8;
    *pos += 1;
    let idx = *pos & (mask as usize);
    buf[idx] = 0x03; // USB string descriptor type
    *pos += 1;

    for ch in s.bytes() {
        let idx = *pos & (mask as usize);
        buf[idx] = ch;
        *pos += 1;
        let idx = *pos & (mask as usize);
        buf[idx] = 0x00;
        *pos += 1;
    }

    total_len
}

/// Build the binary EEPROM image from a decoded structure.
///
/// Returns the number of bytes available for user data, or an error.
pub fn build(eeprom: &mut FtdiEeprom, chip_type: ChipType) -> Result<usize> {
    if eeprom.chip == -1 {
        return Err(Error::Eeprom(
            "no connected EEPROM or EEPROM type unknown".into(),
        ));
    }

    if eeprom.size == -1 {
        eeprom.size = match eeprom.chip {
            0x56 | 0x66 => 0x100,
            _ => 0x80,
        };
    }
    let eeprom_size = eeprom.size as usize;

    let manufacturer_size = eeprom.manufacturer.as_ref().map_or(0, |s| s.len());
    let product_size = eeprom.product.as_ref().map_or(0, |s| s.len());
    let serial_size = eeprom.serial.as_ref().map_or(0, |s| s.len());

    // Check size
    let user_area_base = match chip_type {
        ChipType::Am | ChipType::Bm | ChipType::Ft232R => 96,
        ChipType::Ft2232C => 90,
        ChipType::Ft230X => 88,
        ChipType::Ft2232H | ChipType::Ft4232H => 86,
        ChipType::Ft232H => 80,
    };
    let string_space = (manufacturer_size + product_size + serial_size) * 2;
    if string_space > user_area_base {
        return Err(Error::EepromSizeExceeded);
    }
    let user_area_size = user_area_base - string_space;

    // Clear buffer
    if chip_type == ChipType::Ft230X {
        eeprom.buf[..0x80].fill(0);
        eeprom.buf[0xa0..FTDI_MAX_EEPROM_SIZE].fill(0);
    } else {
        eeprom.buf[..FTDI_MAX_EEPROM_SIZE].fill(0);
    }

    // Common fields
    eeprom.buf[0x02] = eeprom.vendor_id as u8;
    eeprom.buf[0x03] = (eeprom.vendor_id >> 8) as u8;
    eeprom.buf[0x04] = eeprom.product_id as u8;
    eeprom.buf[0x05] = (eeprom.product_id >> 8) as u8;
    eeprom.buf[0x06] = eeprom.release_number as u8;
    eeprom.buf[0x07] = (eeprom.release_number >> 8) as u8;

    // Config descriptor byte 0x08
    let mut cfg = 0x80u8;
    if eeprom.self_powered {
        cfg |= 0x40;
    }
    if eeprom.remote_wakeup {
        cfg |= 0x20;
    }
    eeprom.buf[0x08] = cfg;

    // Max power
    eeprom.buf[0x09] = (eeprom.max_power / MAX_POWER_MILLIAMP_PER_UNIT) as u8;

    // Chip configuration byte 0x0A
    if chip_type != ChipType::Am && chip_type != ChipType::Ft230X {
        let mut j = 0u8;
        if eeprom.in_is_isochronous {
            j |= 1;
        }
        if eeprom.out_is_isochronous {
            j |= 2;
        }
        eeprom.buf[0x0A] = j;
    }

    // String descriptors
    let string_start = match chip_type {
        ChipType::Ft2232H | ChipType::Ft4232H => 0x9A,
        ChipType::Ft232R => 0x98,
        ChipType::Ft2232C => 0x96,
        ChipType::Am | ChipType::Bm => 0x94,
        ChipType::Ft232H | ChipType::Ft230X => 0xA0,
    };

    let size_mask = (eeprom_size - 1) as u8;
    let mut pos = string_start;

    // Manufacturer string
    eeprom.buf[0x0E] = pos as u8;
    let mfr_len = if let Some(ref mfr) = eeprom.manufacturer {
        write_string_descriptor(&mut eeprom.buf, size_mask, &mut pos, mfr)
    } else {
        0
    };
    eeprom.buf[0x0F] = mfr_len as u8;

    // Product string
    eeprom.buf[0x10] = (pos as u8) | 0x80;
    let prod_len = if let Some(ref prod) = eeprom.product {
        write_string_descriptor(&mut eeprom.buf, size_mask, &mut pos, prod)
    } else {
        0
    };
    eeprom.buf[0x11] = prod_len as u8;

    // Serial string
    eeprom.buf[0x12] = (pos as u8) | 0x80;
    let ser_len = if let Some(ref ser) = eeprom.serial {
        write_string_descriptor(&mut eeprom.buf, size_mask, &mut pos, ser)
    } else {
        0
    };
    eeprom.buf[0x13] = ser_len as u8;

    // PnP and legacy fields for FT2232C+
    if chip_type != ChipType::Am && chip_type != ChipType::Bm {
        let idx = pos & (size_mask as usize);
        eeprom.buf[idx] = 0x02;
        pos += 1;
        let idx = pos & (size_mask as usize);
        eeprom.buf[idx] = 0x03;
        pos += 1;
        let idx = pos & (size_mask as usize);
        eeprom.buf[idx] = if eeprom.is_not_pnp { 1 } else { 0 };
        pos += 1;
    }
    let _ = pos; // suppress unused warning

    // use_serial flag
    if chip_type != ChipType::Am {
        if eeprom.use_serial {
            eeprom.buf[0x0A] |= 0x08; // USE_SERIAL_NUM
        } else {
            eeprom.buf[0x0A] &= !0x08;
        }
    }

    // Chip-specific fields
    match chip_type {
        ChipType::Am => {}
        ChipType::Bm => {
            eeprom.buf[0x0C] = eeprom.usb_version as u8;
            eeprom.buf[0x0D] = (eeprom.usb_version >> 8) as u8;
            if eeprom.use_usb_version {
                eeprom.buf[0x0A] |= 0x10;
            }
        }
        ChipType::Ft2232C => {
            eeprom.buf[0x00] = type2bit(eeprom.channel_a_type, ChipType::Ft2232C);
            if eeprom.channel_a_driver {
                eeprom.buf[0x00] |= 0x08; // DRIVER_VCP
            }
            if eeprom.high_current_a {
                eeprom.buf[0x00] |= 0x10; // HIGH_CURRENT_DRIVE
            }
            eeprom.buf[0x01] = type2bit(eeprom.channel_b_type, ChipType::Ft2232C);
            if eeprom.channel_b_driver {
                eeprom.buf[0x01] |= 0x08;
            }
            if eeprom.high_current_b {
                eeprom.buf[0x01] |= 0x10;
            }
            if eeprom.in_is_isochronous {
                eeprom.buf[0x0A] |= 0x01;
            }
            if eeprom.out_is_isochronous {
                eeprom.buf[0x0A] |= 0x02;
            }
            if eeprom.suspend_pull_downs {
                eeprom.buf[0x0A] |= 0x04;
            }
            if eeprom.use_usb_version {
                eeprom.buf[0x0A] |= 0x10;
            }
            eeprom.buf[0x0C] = eeprom.usb_version as u8;
            eeprom.buf[0x0D] = (eeprom.usb_version >> 8) as u8;
            eeprom.buf[0x14] = eeprom.chip as u8;
        }
        ChipType::Ft232R => {
            eeprom.buf[0x00] = type2bit(eeprom.channel_a_type, ChipType::Ft232R);
            if eeprom.high_current {
                eeprom.buf[0x00] |= 0x04; // HIGH_CURRENT_DRIVE_R
            }
            // Field is inverted for TYPE_R: Bit 3 set = D2XX, clear = VCP
            if !eeprom.channel_a_driver {
                eeprom.buf[0x00] |= 0x08;
            }
            if eeprom.external_oscillator {
                eeprom.buf[0x00] |= 0x02;
            }
            eeprom.buf[0x01] = 0x40;
            if eeprom.suspend_pull_downs {
                eeprom.buf[0x0A] |= 0x04;
            }
            eeprom.buf[0x0B] = eeprom.invert;
            eeprom.buf[0x0C] = eeprom.usb_version as u8;
            eeprom.buf[0x0D] = (eeprom.usb_version >> 8) as u8;
            // CBUS functions (packed into nibbles)
            eeprom.buf[0x14] =
                (eeprom.cbus_function[0] & 0x0F) | ((eeprom.cbus_function[1] & 0x0F) << 4);
            eeprom.buf[0x15] =
                (eeprom.cbus_function[2] & 0x0F) | ((eeprom.cbus_function[3] & 0x0F) << 4);
            eeprom.buf[0x16] = eeprom.cbus_function[4] & 0x0F;
        }
        ChipType::Ft2232H => {
            eeprom.buf[0x00] = type2bit(eeprom.channel_a_type, ChipType::Ft2232H);
            if eeprom.channel_a_driver {
                eeprom.buf[0x00] |= 0x08; // DRIVER_VCP
            }
            eeprom.buf[0x01] = type2bit(eeprom.channel_b_type, ChipType::Ft2232H);
            if eeprom.channel_b_driver {
                eeprom.buf[0x01] |= 0x08;
            }
            if eeprom.suspend_dbus7 {
                eeprom.buf[0x01] |= 0x80;
            }
            if eeprom.suspend_pull_downs {
                eeprom.buf[0x0A] |= 0x04;
            }
            encode_drive_groups_into(
                &mut eeprom.buf,
                eeprom.group0_drive,
                eeprom.group0_schmitt,
                eeprom.group0_slew,
                eeprom.group1_drive,
                eeprom.group1_schmitt,
                eeprom.group1_slew,
                eeprom.group2_drive,
                eeprom.group2_schmitt,
                eeprom.group2_slew,
                eeprom.group3_drive,
                eeprom.group3_schmitt,
                eeprom.group3_slew,
            );
            eeprom.buf[0x18] = eeprom.chip as u8;
        }
        ChipType::Ft4232H => {
            if eeprom.channel_a_driver {
                eeprom.buf[0x00] |= 0x08;
            }
            if eeprom.channel_b_driver {
                eeprom.buf[0x01] |= 0x08;
            }
            if eeprom.channel_c_driver {
                eeprom.buf[0x00] |= 0x80;
            }
            if eeprom.channel_d_driver {
                eeprom.buf[0x01] |= 0x80;
            }
            if eeprom.suspend_pull_downs {
                eeprom.buf[0x0A] |= 0x04;
            }
            let mut rs485 = 0u8;
            if eeprom.channel_a_rs485enable {
                rs485 |= 0x10;
            }
            if eeprom.channel_b_rs485enable {
                rs485 |= 0x20;
            }
            if eeprom.channel_c_rs485enable {
                rs485 |= 0x40;
            }
            if eeprom.channel_d_rs485enable {
                rs485 |= 0x80;
            }
            eeprom.buf[0x0B] = rs485;
            encode_drive_groups_into(
                &mut eeprom.buf,
                eeprom.group0_drive,
                eeprom.group0_schmitt,
                eeprom.group0_slew,
                eeprom.group1_drive,
                eeprom.group1_schmitt,
                eeprom.group1_slew,
                eeprom.group2_drive,
                eeprom.group2_schmitt,
                eeprom.group2_slew,
                eeprom.group3_drive,
                eeprom.group3_schmitt,
                eeprom.group3_slew,
            );
            eeprom.buf[0x18] = eeprom.chip as u8;
        }
        ChipType::Ft232H => {
            eeprom.buf[0x00] = type2bit(eeprom.channel_a_type, ChipType::Ft232H);
            if eeprom.channel_a_driver {
                eeprom.buf[0x00] |= 0x10; // DRIVER_VCPH
            }
            if eeprom.powersave {
                eeprom.buf[0x01] |= 0x80;
            }
            if eeprom.suspend_pull_downs {
                eeprom.buf[0x0A] |= 0x04;
            }
            if eeprom.clock_polarity {
                eeprom.buf[0x01] |= 0x01;
            }
            if eeprom.data_order {
                eeprom.buf[0x01] |= 0x02;
            }
            if eeprom.flow_control {
                eeprom.buf[0x01] |= 0x04;
            }
            // Drive groups (only 0 and 1 for 232H)
            eeprom.buf[0x0C] = encode_single_group(
                eeprom.group0_drive,
                eeprom.group0_schmitt,
                eeprom.group0_slew,
            );
            eeprom.buf[0x0D] = encode_single_group(
                eeprom.group1_drive,
                eeprom.group1_schmitt,
                eeprom.group1_slew,
            );
            // CBUS functions (FT232H has 10 CBUS pins packed into 5 bytes)
            for i in 0..5 {
                let lo = eeprom.cbus_function[2 * i] & 0x0F;
                let hi = eeprom.cbus_function[2 * i + 1] & 0x0F;
                eeprom.buf[0x18 + i] = (hi << 4) | lo;
            }
            eeprom.buf[0x1E] = eeprom.chip as u8;
        }
        ChipType::Ft230X => {
            eeprom.buf[0x00] = 0x80;
            eeprom.buf[0x0C] = 0;
            for j in 0..7usize {
                if 0x1A + j < FTDI_MAX_EEPROM_SIZE {
                    eeprom.buf[0x1A + j] = eeprom.cbus_function[j];
                }
            }
            eeprom.buf[0x0B] = eeprom.invert;
        }
    }

    // Checksum
    let csum = checksum(&eeprom.buf, eeprom_size, chip_type == ChipType::Ft230X);
    eeprom.buf[eeprom_size - 2] = csum as u8;
    eeprom.buf[eeprom_size - 1] = (csum >> 8) as u8;

    eeprom.initialized_for_connected_device = true;
    Ok(user_area_size)
}

/// Encode the 4 IO drive groups into bytes 0x0C and 0x0D of the buffer.
#[allow(clippy::too_many_arguments)]
fn encode_drive_groups_into(
    buf: &mut [u8],
    g0_drive: u8,
    g0_schmitt: bool,
    g0_slew: bool,
    g1_drive: u8,
    g1_schmitt: bool,
    g1_slew: bool,
    g2_drive: u8,
    g2_schmitt: bool,
    g2_slew: bool,
    g3_drive: u8,
    g3_schmitt: bool,
    g3_slew: bool,
) {
    buf[0x0C] = encode_single_group(g0_drive, g0_schmitt, g0_slew)
        | (encode_single_group(g1_drive, g1_schmitt, g1_slew) << 4);
    buf[0x0D] = encode_single_group(g2_drive, g2_schmitt, g2_slew)
        | (encode_single_group(g3_drive, g3_schmitt, g3_slew) << 4);
}

/// Encode a single drive/schmitt/slew group into a nibble.
fn encode_single_group(drive: u8, schmitt: bool, slew: bool) -> u8 {
    let mut v = drive.min(3);
    if schmitt {
        v |= 0x08;
    }
    if slew {
        v |= 0x04;
    }
    v
}

/// Convert a channel type value to the encoded EEPROM bit pattern.
///
/// This is the equivalent of `type2bit()` in libftdi.
pub(crate) fn type2bit(channel_type: u8, chip: ChipType) -> u8 {
    match chip {
        ChipType::Ft2232H | ChipType::Ft2232C => match channel_type {
            CHANNEL_IS_UART => 0,
            CHANNEL_IS_FIFO => 0x01,
            CHANNEL_IS_OPTO => 0x02,
            CHANNEL_IS_CPU => 0x04,
            _ => 0,
        },
        ChipType::Ft232H => match channel_type {
            CHANNEL_IS_UART => 0,
            CHANNEL_IS_FIFO => 0x01,
            CHANNEL_IS_OPTO => 0x02,
            CHANNEL_IS_CPU => 0x04,
            CHANNEL_IS_FT1284 => 0x08,
            _ => 0,
        },
        ChipType::Ft232R => match channel_type {
            CHANNEL_IS_UART => 0,
            CHANNEL_IS_FIFO => 0x01,
            _ => 0,
        },
        _ => 0,
    }
}

#[cfg(test)]
#[allow(clippy::field_reassign_with_default)]
mod tests {
    use super::*;
    use crate::eeprom::FtdiEeprom;
    use crate::eeprom::decode;
    use crate::types::ChipType;

    // ---- type2bit tests ----

    #[test]
    fn type2bit_ft2232c_uart() {
        assert_eq!(type2bit(CHANNEL_IS_UART, ChipType::Ft2232C), 0);
    }

    #[test]
    fn type2bit_ft2232c_fifo() {
        assert_eq!(type2bit(CHANNEL_IS_FIFO, ChipType::Ft2232C), 0x01);
    }

    #[test]
    fn type2bit_ft2232c_opto() {
        assert_eq!(type2bit(CHANNEL_IS_OPTO, ChipType::Ft2232C), 0x02);
    }

    #[test]
    fn type2bit_ft2232c_cpu() {
        assert_eq!(type2bit(CHANNEL_IS_CPU, ChipType::Ft2232C), 0x04);
    }

    #[test]
    fn type2bit_ft2232c_unknown_defaults_zero() {
        assert_eq!(type2bit(0xFF, ChipType::Ft2232C), 0);
    }

    #[test]
    fn type2bit_ft2232h_matches_ft2232c() {
        // FT2232H and FT2232C share the same encoding
        for ch_type in [
            CHANNEL_IS_UART,
            CHANNEL_IS_FIFO,
            CHANNEL_IS_OPTO,
            CHANNEL_IS_CPU,
        ] {
            assert_eq!(
                type2bit(ch_type, ChipType::Ft2232H),
                type2bit(ch_type, ChipType::Ft2232C),
                "mismatch for channel type 0x{:02X}",
                ch_type
            );
        }
    }

    #[test]
    fn type2bit_ft232h_ft1284() {
        assert_eq!(type2bit(CHANNEL_IS_FT1284, ChipType::Ft232H), 0x08);
    }

    #[test]
    fn type2bit_ft232h_cpu() {
        assert_eq!(type2bit(CHANNEL_IS_CPU, ChipType::Ft232H), 0x04);
    }

    #[test]
    fn type2bit_ft232r_fifo() {
        assert_eq!(type2bit(CHANNEL_IS_FIFO, ChipType::Ft232R), 0x01);
    }

    #[test]
    fn type2bit_ft232r_opto_unsupported() {
        // FT232R doesn't support opto, should default to 0
        assert_eq!(type2bit(CHANNEL_IS_OPTO, ChipType::Ft232R), 0);
    }

    #[test]
    fn type2bit_am_always_zero() {
        // AM chip has no type2bit support
        for ch_type in [
            CHANNEL_IS_UART,
            CHANNEL_IS_FIFO,
            CHANNEL_IS_OPTO,
            CHANNEL_IS_CPU,
        ] {
            assert_eq!(type2bit(ch_type, ChipType::Am), 0);
        }
    }

    #[test]
    fn type2bit_bm_always_zero() {
        for ch_type in [
            CHANNEL_IS_UART,
            CHANNEL_IS_FIFO,
            CHANNEL_IS_OPTO,
            CHANNEL_IS_CPU,
        ] {
            assert_eq!(type2bit(ch_type, ChipType::Bm), 0);
        }
    }

    // ---- Checksum tests ----

    #[test]
    fn checksum_seed_is_aaaa() {
        // An all-zero buffer of size 4 (2 words): only word 0 is processed (size/2 - 1 = 1)
        // csum = 0xAAAA ^ 0x0000 = 0xAAAA, rotated left by 1 = 0x5555
        let buf = [0u8; 4];
        assert_eq!(checksum(&buf, 4, false), 0x5555);
    }

    #[test]
    fn checksum_round_trip() {
        // Build an EEPROM and verify the stored checksum matches computed
        let mut eeprom = FtdiEeprom::default();
        eeprom.init_defaults(ChipType::Bm, None, None, None);
        eeprom.chip = 0x46;
        eeprom.size = 0x80;
        let _ = build(&mut eeprom, ChipType::Bm).unwrap();

        let size = eeprom.size as usize;
        let stored = (eeprom.buf[size - 2] as u16) | ((eeprom.buf[size - 1] as u16) << 8);
        let computed = checksum(&eeprom.buf, size, false);
        assert_eq!(stored, computed);
    }

    #[test]
    fn checksum_ft230x_skips_user_section() {
        // FT230X checksum should skip addresses 0x12*2..0x40*2
        let mut buf = [0u8; 0x100];
        // Put some data in the skipped region
        buf[0x24] = 0xFF;
        buf[0x25] = 0xFF;
        buf[0x50] = 0xFF;
        buf[0x51] = 0xFF;

        let csum_a = checksum(&buf, 0x100, true);

        // Change data in skipped region — should NOT affect checksum
        buf[0x24] = 0x00;
        buf[0x25] = 0x00;
        let csum_b = checksum(&buf, 0x100, true);
        assert_eq!(csum_a, csum_b);

        // Change data outside skipped region — SHOULD affect checksum
        buf[0x02] = 0xFF;
        let csum_c = checksum(&buf, 0x100, true);
        assert_ne!(csum_a, csum_c);
    }

    // ---- EEPROM build/decode round-trip tests ----

    /// Helper: build an EEPROM image and then decode it, verifying all common fields survive.
    fn round_trip_test(chip_type: ChipType) {
        let mut eeprom = FtdiEeprom::default();
        eeprom.init_defaults(chip_type, Some("TestMfr"), Some("TestProd"), Some("SER123"));

        // Set chip and size so build() will work
        eeprom.chip = match chip_type {
            ChipType::Am | ChipType::Bm => 0x46,
            ChipType::Ft2232C => 0x46,
            ChipType::Ft232R => 0,
            ChipType::Ft2232H => 0x56,
            ChipType::Ft4232H => 0x56,
            ChipType::Ft232H => 0x56,
            ChipType::Ft230X => 0,
        };
        if eeprom.size == -1 {
            eeprom.size = 0x100;
        }

        let user_area = build(&mut eeprom, chip_type).unwrap();
        assert!(user_area > 0, "should have some user area space");

        // Save the fields we set
        let vid = eeprom.vendor_id;
        let pid = eeprom.product_id;
        let release = eeprom.release_number;
        let max_power = eeprom.max_power;
        let use_serial = eeprom.use_serial;
        let mfr = eeprom.manufacturer.clone();
        let prod = eeprom.product.clone();
        let ser = eeprom.serial.clone();

        // Now decode
        let mut decoded = FtdiEeprom::default();
        decoded.buf = eeprom.buf;
        decoded.size = eeprom.size;

        decode::decode(&mut decoded, chip_type).unwrap();

        // Verify common fields
        assert_eq!(
            decoded.vendor_id, vid,
            "vendor_id mismatch for {:?}",
            chip_type
        );
        assert_eq!(
            decoded.product_id, pid,
            "product_id mismatch for {:?}",
            chip_type
        );
        assert_eq!(
            decoded.release_number, release,
            "release_number mismatch for {:?}",
            chip_type
        );
        assert_eq!(
            decoded.max_power, max_power,
            "max_power mismatch for {:?}",
            chip_type
        );
        assert_eq!(
            decoded.use_serial, use_serial,
            "use_serial mismatch for {:?}",
            chip_type
        );
        assert_eq!(
            decoded.manufacturer, mfr,
            "manufacturer mismatch for {:?}",
            chip_type
        );
        assert_eq!(
            decoded.product, prod,
            "product mismatch for {:?}",
            chip_type
        );
        assert_eq!(decoded.serial, ser, "serial mismatch for {:?}", chip_type);
    }

    #[test]
    fn round_trip_bm() {
        round_trip_test(ChipType::Bm);
    }

    #[test]
    fn round_trip_ft2232c() {
        round_trip_test(ChipType::Ft2232C);
    }

    #[test]
    fn round_trip_ft232r() {
        round_trip_test(ChipType::Ft232R);
    }

    #[test]
    fn round_trip_ft2232h() {
        round_trip_test(ChipType::Ft2232H);
    }

    #[test]
    fn round_trip_ft4232h() {
        round_trip_test(ChipType::Ft4232H);
    }

    #[test]
    fn round_trip_ft232h() {
        round_trip_test(ChipType::Ft232H);
    }

    #[test]
    fn round_trip_ft230x() {
        round_trip_test(ChipType::Ft230X);
    }

    #[test]
    fn round_trip_ft232r_cbus_functions() {
        let mut eeprom = FtdiEeprom::default();
        eeprom.init_defaults(ChipType::Ft232R, None, None, None);
        eeprom.chip = 0;
        // init_defaults sets CBUS for FT232R: TXLED, RXLED, TXDEN, PWREN, SLEEP
        let cbus_orig = eeprom.cbus_function;

        build(&mut eeprom, ChipType::Ft232R).unwrap();

        let mut decoded = FtdiEeprom::default();
        decoded.buf = eeprom.buf;
        decoded.size = eeprom.size;
        decode::decode(&mut decoded, ChipType::Ft232R).unwrap();

        // First 5 CBUS functions should survive the round trip
        for (i, (dec, orig)) in decoded
            .cbus_function
            .iter()
            .zip(cbus_orig.iter())
            .enumerate()
            .take(5)
        {
            assert_eq!(
                *dec, *orig,
                "CBUS[{}] mismatch: decoded={}, original={}",
                i, dec, orig
            );
        }
    }

    #[test]
    fn round_trip_ft232h_cbus_functions() {
        let mut eeprom = FtdiEeprom::default();
        eeprom.init_defaults(ChipType::Ft232H, None, None, None);
        eeprom.chip = 0x56;
        eeprom.size = 0x100;

        // Set some CBUS functions
        eeprom.cbus_function[0] = 1; // TXLED
        eeprom.cbus_function[1] = 2; // RXLED
        eeprom.cbus_function[2] = 3; // TXRXLED
        eeprom.cbus_function[3] = 4; // PWREN
        eeprom.cbus_function[4] = 5; // SLEEP
        eeprom.cbus_function[5] = 6; // DRIVE_0
        eeprom.cbus_function[6] = 7; // DRIVE_1
        eeprom.cbus_function[7] = 8; // IOMODE
        eeprom.cbus_function[8] = 9; // TXDEN
        eeprom.cbus_function[9] = 10; // CLK30

        build(&mut eeprom, ChipType::Ft232H).unwrap();

        let mut decoded = FtdiEeprom::default();
        decoded.buf = eeprom.buf;
        decoded.size = eeprom.size;
        decode::decode(&mut decoded, ChipType::Ft232H).unwrap();

        for i in 0..10 {
            assert_eq!(
                decoded.cbus_function[i], eeprom.cbus_function[i],
                "CBUS[{}] mismatch for FT232H",
                i
            );
        }
    }

    #[test]
    fn round_trip_ft2232h_channel_types() {
        let mut eeprom = FtdiEeprom::default();
        eeprom.init_defaults(ChipType::Ft2232H, None, None, None);
        eeprom.chip = 0x56;
        eeprom.size = 0x100;
        eeprom.channel_a_type = CHANNEL_IS_FIFO;
        eeprom.channel_b_type = CHANNEL_IS_OPTO;
        eeprom.channel_a_driver = true;
        eeprom.channel_b_driver = false;
        eeprom.suspend_dbus7 = true;

        build(&mut eeprom, ChipType::Ft2232H).unwrap();

        let mut decoded = FtdiEeprom::default();
        decoded.buf = eeprom.buf;
        decoded.size = eeprom.size;
        decode::decode(&mut decoded, ChipType::Ft2232H).unwrap();

        assert_eq!(decoded.channel_a_type, CHANNEL_IS_FIFO);
        assert_eq!(decoded.channel_b_type, CHANNEL_IS_OPTO);
        assert!(decoded.channel_a_driver);
        assert!(!decoded.channel_b_driver);
        assert!(decoded.suspend_dbus7);
    }

    #[test]
    fn round_trip_ft4232h_rs485_and_drivers() {
        let mut eeprom = FtdiEeprom::default();
        eeprom.init_defaults(ChipType::Ft4232H, None, None, None);
        eeprom.chip = 0x56;
        eeprom.size = 0x100;
        eeprom.channel_a_driver = true;
        eeprom.channel_b_driver = false;
        eeprom.channel_c_driver = true;
        eeprom.channel_d_driver = false;
        eeprom.channel_a_rs485enable = true;
        eeprom.channel_b_rs485enable = false;
        eeprom.channel_c_rs485enable = true;
        eeprom.channel_d_rs485enable = true;

        build(&mut eeprom, ChipType::Ft4232H).unwrap();

        let mut decoded = FtdiEeprom::default();
        decoded.buf = eeprom.buf;
        decoded.size = eeprom.size;
        decode::decode(&mut decoded, ChipType::Ft4232H).unwrap();

        assert!(decoded.channel_a_driver);
        assert!(!decoded.channel_b_driver);
        assert!(decoded.channel_c_driver);
        assert!(!decoded.channel_d_driver);
        assert!(decoded.channel_a_rs485enable);
        assert!(!decoded.channel_b_rs485enable);
        assert!(decoded.channel_c_rs485enable);
        assert!(decoded.channel_d_rs485enable);
    }

    #[test]
    fn round_trip_ft232h_drive_groups() {
        let mut eeprom = FtdiEeprom::default();
        eeprom.init_defaults(ChipType::Ft232H, None, None, None);
        eeprom.chip = 0x56;
        eeprom.size = 0x100;
        eeprom.group0_drive = 2;
        eeprom.group0_schmitt = true;
        eeprom.group0_slew = true;
        eeprom.group1_drive = 3;
        eeprom.group1_schmitt = false;
        eeprom.group1_slew = true;

        build(&mut eeprom, ChipType::Ft232H).unwrap();

        let mut decoded = FtdiEeprom::default();
        decoded.buf = eeprom.buf;
        decoded.size = eeprom.size;
        decode::decode(&mut decoded, ChipType::Ft232H).unwrap();

        assert_eq!(decoded.group0_drive, 2);
        assert!(decoded.group0_schmitt);
        assert!(decoded.group0_slew);
        assert_eq!(decoded.group1_drive, 3);
        assert!(!decoded.group1_schmitt);
        assert!(decoded.group1_slew);
    }

    #[test]
    fn round_trip_ft2232h_drive_groups_all_four() {
        let mut eeprom = FtdiEeprom::default();
        eeprom.init_defaults(ChipType::Ft2232H, None, None, None);
        eeprom.chip = 0x56;
        eeprom.size = 0x100;
        eeprom.group0_drive = 0;
        eeprom.group0_schmitt = false;
        eeprom.group0_slew = false;
        eeprom.group1_drive = 1;
        eeprom.group1_schmitt = true;
        eeprom.group1_slew = false;
        eeprom.group2_drive = 2;
        eeprom.group2_schmitt = false;
        eeprom.group2_slew = true;
        eeprom.group3_drive = 3;
        eeprom.group3_schmitt = true;
        eeprom.group3_slew = true;

        build(&mut eeprom, ChipType::Ft2232H).unwrap();

        let mut decoded = FtdiEeprom::default();
        decoded.buf = eeprom.buf;
        decoded.size = eeprom.size;
        decode::decode(&mut decoded, ChipType::Ft2232H).unwrap();

        assert_eq!(decoded.group0_drive, 0);
        assert!(!decoded.group0_schmitt);
        assert!(!decoded.group0_slew);
        assert_eq!(decoded.group1_drive, 1);
        assert!(decoded.group1_schmitt);
        assert!(!decoded.group1_slew);
        assert_eq!(decoded.group2_drive, 2);
        assert!(!decoded.group2_schmitt);
        assert!(decoded.group2_slew);
        assert_eq!(decoded.group3_drive, 3);
        assert!(decoded.group3_schmitt);
        assert!(decoded.group3_slew);
    }

    #[test]
    fn round_trip_ft232r_inverted_vcp_flag() {
        // FT232R has inverted VCP flag: bit 3 set = D2XX, bit 3 clear = VCP
        let mut eeprom = FtdiEeprom::default();
        eeprom.init_defaults(ChipType::Ft232R, None, None, None);
        eeprom.chip = 0;
        eeprom.channel_a_driver = true; // VCP driver

        build(&mut eeprom, ChipType::Ft232R).unwrap();

        let mut decoded = FtdiEeprom::default();
        decoded.buf = eeprom.buf;
        decoded.size = eeprom.size;
        decode::decode(&mut decoded, ChipType::Ft232R).unwrap();

        // The inverted flag should decode correctly back to true (VCP)
        assert!(
            decoded.channel_a_driver,
            "VCP flag should survive round trip for FT232R"
        );
    }

    #[test]
    fn round_trip_ft232r_inverted_vcp_flag_d2xx() {
        let mut eeprom = FtdiEeprom::default();
        eeprom.init_defaults(ChipType::Ft232R, None, None, None);
        eeprom.chip = 0;
        eeprom.channel_a_driver = false; // D2XX driver

        build(&mut eeprom, ChipType::Ft232R).unwrap();

        let mut decoded = FtdiEeprom::default();
        decoded.buf = eeprom.buf;
        decoded.size = eeprom.size;
        decode::decode(&mut decoded, ChipType::Ft232R).unwrap();

        assert!(
            !decoded.channel_a_driver,
            "D2XX flag should survive round trip for FT232R"
        );
    }

    #[test]
    fn build_error_on_uninit_chip() {
        let mut eeprom = FtdiEeprom::default();
        // chip = -1 by default -> should error
        let result = build(&mut eeprom, ChipType::Bm);
        assert!(result.is_err());
    }

    #[test]
    fn build_error_on_strings_too_long() {
        let mut eeprom = FtdiEeprom::default();
        eeprom.chip = 0x46;
        eeprom.size = 0x80;
        // Create strings that exceed the available space
        eeprom.manufacturer = Some("A".repeat(50));
        eeprom.product = Some("B".repeat(50));
        eeprom.serial = Some("C".repeat(50));
        let result = build(&mut eeprom, ChipType::Bm);
        assert!(result.is_err());
    }

    #[test]
    fn decode_checksum_mismatch_errors() {
        let mut eeprom = FtdiEeprom::default();
        eeprom.init_defaults(ChipType::Bm, None, None, None);
        eeprom.chip = 0x46;
        eeprom.size = 0x80;
        build(&mut eeprom, ChipType::Bm).unwrap();

        // Corrupt a byte
        eeprom.buf[0x10] ^= 0xFF;

        let mut decoded = FtdiEeprom::default();
        decoded.buf = eeprom.buf;
        decoded.size = eeprom.size;
        let result = decode::decode(&mut decoded, ChipType::Bm);
        assert!(result.is_err(), "should fail on checksum mismatch");
    }

    #[test]
    fn round_trip_power_and_flags() {
        let mut eeprom = FtdiEeprom::default();
        eeprom.init_defaults(ChipType::Ft2232H, None, None, None);
        eeprom.chip = 0x56;
        eeprom.size = 0x100;
        eeprom.self_powered = true;
        eeprom.remote_wakeup = true;
        eeprom.max_power = 200;
        eeprom.suspend_pull_downs = true;

        build(&mut eeprom, ChipType::Ft2232H).unwrap();

        let mut decoded = FtdiEeprom::default();
        decoded.buf = eeprom.buf;
        decoded.size = eeprom.size;
        decode::decode(&mut decoded, ChipType::Ft2232H).unwrap();

        assert!(decoded.self_powered);
        assert!(decoded.remote_wakeup);
        assert_eq!(decoded.max_power, 200);
        assert!(decoded.suspend_pull_downs);
    }

    // ---- encode_single_group tests ----

    #[test]
    fn encode_single_group_all_off() {
        assert_eq!(encode_single_group(0, false, false), 0);
    }

    #[test]
    fn encode_single_group_drive_clamps_at_3() {
        assert_eq!(encode_single_group(5, false, false), 3);
    }

    #[test]
    fn encode_single_group_all_on() {
        // drive=3, schmitt=true (0x08), slew=true (0x04) = 0x0F
        assert_eq!(encode_single_group(3, true, true), 0x0F);
    }

    #[test]
    fn encode_single_group_just_schmitt() {
        assert_eq!(encode_single_group(0, true, false), 0x08);
    }

    #[test]
    fn encode_single_group_just_slew() {
        assert_eq!(encode_single_group(0, false, true), 0x04);
    }
}
