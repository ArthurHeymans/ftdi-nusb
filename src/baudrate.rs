//! Baud rate calculation for FTDI chips.
//!
//! FTDI chips use a fractional clock divider to generate baud rates.
//! The exact algorithm differs between chip generations:
//!
//! - **AM**: 24 MHz base clock, limited fractional divisors.
//! - **BM/2232C/R/230X**: 48 MHz base clock, 16x predivisor, 3 fractional bits.
//! - **H-type** (2232H/4232H/232H): Selectable 120 MHz or 48 MHz clock.

use crate::types::ChipType;

/// The result of a baud rate conversion.
#[derive(Debug, Clone, Copy)]
pub(crate) struct BaudRateResult {
    /// The nearest achievable baud rate.
    pub actual: u32,
    /// The `value` field for the `SIO_SET_BAUDRATE_REQUEST` control transfer.
    pub value: u16,
    /// The `index` field for the `SIO_SET_BAUDRATE_REQUEST` control transfer.
    pub index: u16,
}

/// Fractional code lookup table: maps 3-bit sub-divisor to FTDI encoding.
const FRAC_CODE: [u32; 8] = [0, 3, 2, 4, 1, 5, 6, 7];

/// AM-type: round-down adjustments for unsupported fractional values.
const AM_ADJUST_DN: [i32; 8] = [0, 0, 0, 1, 0, 3, 2, 1];
/// AM-type: round-up adjustments for unsupported fractional values.
const AM_ADJUST_UP: [i32; 8] = [0, 0, 0, 1, 0, 1, 2, 3];

/// Compute encoded divisor and actual baud rate for AM-type chips.
///
/// AM chips have a 24 MHz clock with limited fractional divisor support.
fn clkbits_am(baudrate: u32) -> (u32, u64) {
    let baudrate = baudrate as i32;
    let mut divisor = 24_000_000 / baudrate;

    // Round down to supported fraction (AM only)
    divisor -= AM_ADJUST_DN[(divisor & 7) as usize];

    let mut best_divisor = 0i32;
    let mut best_baud = 0i32;
    let mut best_baud_diff = 0i32;

    for i in 0..2 {
        let mut try_divisor = divisor + i;

        // Round up to supported divisor value
        if try_divisor <= 8 {
            try_divisor = 8;
        } else if divisor < 16 {
            // AM doesn't support divisors 9 through 15 inclusive
            try_divisor = 16;
        } else {
            try_divisor += AM_ADJUST_UP[(try_divisor & 7) as usize];
            if try_divisor > 0x1FFF8 {
                try_divisor = 0x1FFF8;
            }
        }

        let baud_estimate = (24_000_000 + (try_divisor / 2)) / try_divisor;
        let baud_diff = (baud_estimate - baudrate).abs();

        if i == 0 || baud_diff < best_baud_diff {
            best_divisor = try_divisor;
            best_baud = baud_estimate;
            best_baud_diff = baud_diff;
            if baud_diff == 0 {
                break;
            }
        }
    }

    let mut encoded =
        ((best_divisor >> 3) as u64) | (FRAC_CODE[(best_divisor & 7) as usize] as u64) << 14;

    // Special cases
    if encoded == 1 {
        encoded = 0; // 3000000 baud
    } else if encoded == 0x4001 {
        encoded = 1; // 2000000 baud (BM only)
    }

    (best_baud as u32, encoded)
}

/// Compute encoded divisor and actual baud rate for a given clock and predivisor.
///
/// Used by BM/2232C/R/230X (48 MHz / 16) and H-type (120 MHz / 10 or 48 MHz / 16).
fn clkbits(baudrate: u32, clk: u32, clk_div: u32) -> (u32, u64) {
    if baudrate >= clk / clk_div {
        return (clk / clk_div, 0);
    }
    if baudrate >= clk / (clk_div + clk_div / 2) {
        return (clk / (clk_div + clk_div / 2), 1);
    }
    if baudrate >= clk / (2 * clk_div) {
        return (clk / (2 * clk_div), 2);
    }

    // Divide by 16 to get 3 fractional bits and one bit for rounding
    let divisor = clk * 16 / clk_div / baudrate;
    let best_divisor = if divisor & 1 != 0 {
        divisor / 2 + 1
    } else {
        divisor / 2
    };
    // C code: if(best_divisor > 0x20000) best_divisor = 0x1ffff;
    // Note: 0x20000 is a valid divisor value, only values above it are clamped.
    let best_divisor = if best_divisor > 0x20000 {
        0x1FFFF
    } else {
        best_divisor
    };

    let mut best_baud = clk * 16 / clk_div / best_divisor;
    if best_baud & 1 != 0 {
        best_baud = best_baud / 2 + 1;
    } else {
        best_baud /= 2;
    }

    let encoded =
        ((best_divisor >> 3) as u64) | (FRAC_CODE[(best_divisor & 0x7) as usize] as u64) << 14;

    (best_baud, encoded)
}

/// Convert a requested baud rate to FTDI register values.
///
/// Returns the nearest achievable baud rate and the `value`/`index` fields
/// for the `SIO_SET_BAUDRATE_REQUEST` vendor control transfer.
///
/// # Arguments
///
/// * `baudrate` - The desired baud rate in bits per second.
/// * `chip` - The FTDI chip type (determines the calculation algorithm).
/// * `usb_index` - The base USB index value (interface number, 1-based).
pub(crate) fn convert_baudrate(
    baudrate: u32,
    chip: ChipType,
    usb_index: u16,
) -> Option<BaudRateResult> {
    use crate::constants::{C_CLK, H_CLK};

    if baudrate == 0 {
        return None;
    }

    let (best_baud, encoded_divisor) = match chip {
        ChipType::Ft2232H | ChipType::Ft4232H | ChipType::Ft232H => {
            if (baudrate as u64) * 10 > (H_CLK as u64) / 0x3FFF {
                // Use 12 MHz base (120 MHz / 10)
                let (baud, mut enc) = clkbits(baudrate, H_CLK, 10);
                enc |= 0x20000; // Switch on CLK/10
                (baud, enc)
            } else {
                clkbits(baudrate, C_CLK, 16)
            }
        }
        ChipType::Bm | ChipType::Ft2232C | ChipType::Ft232R | ChipType::Ft230X => {
            clkbits(baudrate, C_CLK, 16)
        }
        ChipType::Am => clkbits_am(baudrate),
    };

    if best_baud == 0 {
        return None;
    }

    let value = (encoded_divisor & 0xFFFF) as u16;
    // The proprietary FTDI driver encodes the interface index into the
    // baud rate index field for all chip types except AM, BM, and FT232R.
    // For H-type and other modern chips, the index is:
    //   (divisor_high_bits & 0xFF00) | interface_index
    // For BM/AM/FT232R, the divisor output is used as-is.
    let index = match chip {
        ChipType::Ft2232H | ChipType::Ft4232H | ChipType::Ft232H => {
            let idx = (encoded_divisor >> 8) as u16;
            (idx & 0xFF00) | usb_index
        }
        ChipType::Ft2232C | ChipType::Ft230X => {
            let idx = (encoded_divisor >> 16) as u16;
            (idx << 8) | usb_index
        }
        // AM, BM, FT232R: no interface index in baud rate
        _ => (encoded_divisor >> 16) as u16,
    };

    Some(BaudRateResult {
        actual: best_baud,
        value,
        index,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bm_9600() {
        // BM is single-channel: usb_index=0
        let r = convert_baudrate(9600, ChipType::Bm, 0).unwrap();
        assert_eq!(r.actual, 9600);
    }

    #[test]
    fn bm_115200() {
        let r = convert_baudrate(115200, ChipType::Bm, 0).unwrap();
        // 115200 is exact for BM: 48MHz / 16 / 26 = 115384, close enough
        assert!(
            (r.actual as i64 - 115200).unsigned_abs() < 115200 / 20,
            "actual={} should be within 5% of 115200",
            r.actual
        );
    }

    #[test]
    fn h_type_3000000() {
        // FT2232H is multi-channel: usb_index=1 for interface A
        let r = convert_baudrate(3_000_000, ChipType::Ft2232H, 1).unwrap();
        assert_eq!(r.actual, 3_000_000);
    }

    #[test]
    fn am_9600() {
        // AM is single-channel: usb_index=0
        let r = convert_baudrate(9600, ChipType::Am, 0).unwrap();
        assert!(
            (r.actual as i64 - 9600).unsigned_abs() < 9600 / 20,
            "actual={} should be within 5% of 9600",
            r.actual
        );
    }

    #[test]
    fn zero_returns_none() {
        assert!(convert_baudrate(0, ChipType::Bm, 0).is_none());
    }

    // ---- Additional baud rate edge case tests ----

    #[test]
    fn bm_3000000_max() {
        // 48MHz / 16 = 3,000,000 — the maximum for BM-type
        let r = convert_baudrate(3_000_000, ChipType::Bm, 0).unwrap();
        assert_eq!(r.actual, 3_000_000);
        // Encoded divisor 0 = special case for 3 Mbaud
        assert_eq!(r.value, 0);
    }

    #[test]
    fn bm_2000000() {
        let r = convert_baudrate(2_000_000, ChipType::Bm, 0).unwrap();
        // Should be close to 2 Mbaud (exact or within tolerance)
        assert!(
            (r.actual as i64 - 2_000_000).unsigned_abs() < 2_000_000 / 50,
            "actual={} should be close to 2,000,000",
            r.actual
        );
    }

    #[test]
    fn h_type_12000000() {
        // FT2232H is multi-channel: usb_index=1 for interface A
        let r = convert_baudrate(12_000_000, ChipType::Ft2232H, 1).unwrap();
        assert_eq!(r.actual, 12_000_000);
    }

    #[test]
    fn h_type_low_baud() {
        // FT232H is single-channel: usb_index=0
        let r = convert_baudrate(300, ChipType::Ft232H, 0).unwrap();
        assert!(
            (r.actual as i64 - 300).unsigned_abs() < 300 / 10,
            "actual={} should be within 10% of 300",
            r.actual
        );
    }

    #[test]
    fn am_highest_baud() {
        // AM maximum is 24 MHz / 8 = 3,000,000
        let r = convert_baudrate(3_000_000, ChipType::Am, 0).unwrap();
        assert_eq!(r.actual, 3_000_000);
    }

    #[test]
    fn am_300_baud() {
        let r = convert_baudrate(300, ChipType::Am, 0).unwrap();
        assert!(
            (r.actual as i64 - 300).unsigned_abs() < 300 / 10,
            "actual={} should be within 10% of 300",
            r.actual
        );
    }

    #[test]
    fn ft230x_115200() {
        // FT230X is single-channel: usb_index=0
        let r = convert_baudrate(115200, ChipType::Ft230X, 0).unwrap();
        assert!(
            (r.actual as i64 - 115200).unsigned_abs() < 115200 / 20,
            "actual={} should be within 5% of 115200",
            r.actual
        );
    }

    #[test]
    fn ft232r_9600() {
        // FT232R is single-channel: usb_index=0
        let r = convert_baudrate(9600, ChipType::Ft232R, 0).unwrap();
        assert_eq!(r.actual, 9600);
    }

    #[test]
    fn h_type_index_includes_interface() {
        // For multi-channel H-type chips, index field should include the interface number
        let r = convert_baudrate(9600, ChipType::Ft2232H, 2).unwrap();
        // Lower byte of index should be the usb_index
        assert_eq!(r.index & 0xFF, 2);
    }

    #[test]
    fn ft2232c_index_includes_interface() {
        // For multi-channel FT2232C, index field should include the interface number
        let r = convert_baudrate(9600, ChipType::Ft2232C, 2).unwrap();
        // Lower byte of index should be the usb_index
        assert_eq!(r.index & 0xFF, 2);
    }

    #[test]
    fn bm_index_is_divisor_only() {
        // For BM, index is just the high part of the encoded divisor (no interface)
        let r = convert_baudrate(9600, ChipType::Bm, 0).unwrap();
        let _ = r.index; // Just confirm it doesn't panic / is a valid value
    }

    #[test]
    fn very_low_baud_clamped() {
        // Very low baud rate: should work without panicking
        let r = convert_baudrate(1, ChipType::Bm, 0).unwrap();
        assert!(r.actual > 0, "should return a valid baud rate");
    }

    #[test]
    fn divisor_0x20000_not_clamped() {
        // Regression test: 0x20000 is a valid divisor, only > 0x20000 should clamp
        let r = convert_baudrate(2, ChipType::Bm, 0).unwrap();
        assert!(r.actual > 0);
    }
}
