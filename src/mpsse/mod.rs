//! High-level MPSSE (Multi-Protocol Synchronous Serial Engine) API.

pub mod gpio;
pub mod i2c;
pub mod jtag;
pub mod spi;

use crate::constants::mpsse;
use crate::context::FtdiDevice;
use crate::error::{Error, Result};
use crate::types::{BitMode, ChipType};

/// MPSSE context holding pin state and clock configuration.
#[derive(Debug, Clone)]
pub struct MpsseContext {
    clock_hz: u32,
    is_h_type: bool,
    gpio_low_value: u8,
    gpio_low_dir: u8,
    gpio_high_value: u8,
    gpio_high_dir: u8,
}

impl MpsseContext {
    /// Initialize MPSSE mode on the device and configure the clock frequency.
    pub async fn init(dev: &mut FtdiDevice, clock_hz: u32) -> Result<Self> {
        let chip = dev.chip_type();
        let is_h_type = chip.is_h_type();

        match chip {
            ChipType::Ft2232C | ChipType::Ft2232H | ChipType::Ft4232H | ChipType::Ft232H => {}
            _ => return Err(Error::UnsupportedChip(chip)),
        }

        dev.set_bitmode(0, BitMode::Reset).await?;
        dev.flush_all().await?;
        dev.set_bitmode(0, BitMode::Mpsse).await?;

        // Short delay for the MPSSE engine to start
        crate::sleep_util::sleep(core::time::Duration::from_millis(50)).await;

        let _ = dev.flush_rx().await;

        let mut ctx = Self {
            clock_hz: 0,
            is_h_type,
            gpio_low_value: 0x00,
            gpio_low_dir: 0x00,
            gpio_high_value: 0x00,
            gpio_high_dir: 0x00,
        };

        let mut cmd = Vec::with_capacity(16);
        cmd.push(mpsse::LOOPBACK_END);
        if is_h_type {
            cmd.push(mpsse::DIS_ADAPTIVE);
        }
        dev.write_all(&cmd).await?;

        ctx.set_clock(dev, clock_hz).await?;

        Ok(ctx)
    }

    pub fn clock_hz(&self) -> u32 {
        self.clock_hz
    }

    pub async fn set_clock(&mut self, dev: &mut FtdiDevice, clock_hz: u32) -> Result<()> {
        if clock_hz == 0 {
            return Err(Error::InvalidArgument("clock frequency must be > 0"));
        }

        let max_freq = if self.is_h_type {
            30_000_000
        } else {
            6_000_000
        };
        if clock_hz > max_freq {
            return Err(Error::InvalidArgument(
                "clock frequency exceeds maximum for this chip",
            ));
        }

        let mut cmd = Vec::with_capacity(8);

        if self.is_h_type {
            if clock_hz > 6_000_000 {
                cmd.push(mpsse::DIS_DIV_5);
                let divisor = (60_000_000u32 / clock_hz.saturating_mul(2)).saturating_sub(1);
                let divisor = divisor.min(0xFFFF) as u16;
                cmd.push(mpsse::TCK_DIVISOR);
                cmd.push(divisor as u8);
                cmd.push((divisor >> 8) as u8);
                self.clock_hz = 60_000_000 / ((1 + divisor as u32) * 2);
            } else {
                cmd.push(mpsse::EN_DIV_5);
                let divisor = (12_000_000u32 / clock_hz.saturating_mul(2)).saturating_sub(1);
                let divisor = divisor.min(0xFFFF) as u16;
                cmd.push(mpsse::TCK_DIVISOR);
                cmd.push(divisor as u8);
                cmd.push((divisor >> 8) as u8);
                self.clock_hz = 12_000_000 / ((1 + divisor as u32) * 2);
            }
        } else {
            let divisor = (12_000_000u32 / clock_hz.saturating_mul(2)).saturating_sub(1);
            let divisor = divisor.min(0xFFFF) as u16;
            cmd.push(mpsse::TCK_DIVISOR);
            cmd.push(divisor as u8);
            cmd.push((divisor >> 8) as u8);
            self.clock_hz = 12_000_000 / ((1 + divisor as u32) * 2);
        }

        dev.write_all(&cmd).await
    }

    pub async fn enable_3phase_clocking(&self, dev: &mut FtdiDevice) -> Result<()> {
        if !self.is_h_type {
            return Err(Error::InvalidArgument(
                "3-phase clocking only supported on H-type chips",
            ));
        }
        dev.write_all(&[mpsse::EN_3_PHASE]).await
    }

    pub async fn disable_3phase_clocking(&self, dev: &mut FtdiDevice) -> Result<()> {
        if !self.is_h_type {
            return Err(Error::InvalidArgument(
                "3-phase clocking only supported on H-type chips",
            ));
        }
        dev.write_all(&[mpsse::DIS_3_PHASE]).await
    }

    pub async fn enable_loopback(&self, dev: &mut FtdiDevice) -> Result<()> {
        dev.write_all(&[mpsse::LOOPBACK_START]).await
    }

    pub async fn disable_loopback(&self, dev: &mut FtdiDevice) -> Result<()> {
        dev.write_all(&[mpsse::LOOPBACK_END]).await
    }

    pub async fn set_gpio_low(
        &mut self,
        dev: &mut FtdiDevice,
        value: u8,
        direction: u8,
    ) -> Result<()> {
        self.gpio_low_value = value;
        self.gpio_low_dir = direction;
        dev.write_all(&[mpsse::SET_BITS_LOW, value, direction])
            .await
    }

    pub async fn get_gpio_low(&self, dev: &mut FtdiDevice) -> Result<u8> {
        dev.write_all(&[mpsse::GET_BITS_LOW, mpsse::SEND_IMMEDIATE])
            .await?;
        let mut buf = [0u8; 1];
        let n = dev.read_data(&mut buf).await?;
        if n == 0 {
            return Err(Error::DeviceUnavailable);
        }
        Ok(buf[0])
    }

    pub async fn set_gpio_high(
        &mut self,
        dev: &mut FtdiDevice,
        value: u8,
        direction: u8,
    ) -> Result<()> {
        self.gpio_high_value = value;
        self.gpio_high_dir = direction;
        dev.write_all(&[mpsse::SET_BITS_HIGH, value, direction])
            .await
    }

    pub async fn get_gpio_high(&self, dev: &mut FtdiDevice) -> Result<u8> {
        dev.write_all(&[mpsse::GET_BITS_HIGH, mpsse::SEND_IMMEDIATE])
            .await?;
        let mut buf = [0u8; 1];
        let n = dev.read_data(&mut buf).await?;
        if n == 0 {
            return Err(Error::DeviceUnavailable);
        }
        Ok(buf[0])
    }

    pub fn gpio_low_dir(&self) -> u8 {
        self.gpio_low_dir
    }

    pub fn gpio_low_value(&self) -> u8 {
        self.gpio_low_value
    }

    pub fn gpio_high_dir(&self) -> u8 {
        self.gpio_high_dir
    }

    pub fn gpio_high_value(&self) -> u8 {
        self.gpio_high_value
    }

    pub fn is_h_type(&self) -> bool {
        self.is_h_type
    }

    pub(crate) fn update_gpio_low_state(&mut self, value: u8, direction: u8) {
        self.gpio_low_value = value;
        self.gpio_low_dir = direction;
    }

    pub const BAD_COMMAND: u8 = 0xFA;

    pub fn check_bad_command(response: &[u8]) -> Result<()> {
        for i in 0..response.len() {
            if response[i] == Self::BAD_COMMAND && i + 1 < response.len() {
                return Err(Error::MpsseBadCommand(response[i + 1]));
            }
        }
        Ok(())
    }

    pub async fn sync_mpsse(&self, dev: &mut FtdiDevice) -> Result<()> {
        const BOGUS_CMD: u8 = 0xAB;

        dev.write_all(&[BOGUS_CMD, mpsse::SEND_IMMEDIATE]).await?;

        let mut buf = [0u8; 64];
        let mut attempts = 0;
        while attempts < 10 {
            let n = dev.read_data(&mut buf).await?;
            if n >= 2 {
                for i in 0..n - 1 {
                    if buf[i] == Self::BAD_COMMAND && buf[i + 1] == BOGUS_CMD {
                        return Ok(());
                    }
                }
            }
            attempts += 1;

            crate::sleep_util::sleep(core::time::Duration::from_millis(10)).await;
        }

        Err(Error::InvalidArgument(
            "MPSSE synchronization failed: no bad-command echo received",
        ))
    }

    pub async fn command_response(
        &self,
        dev: &mut FtdiDevice,
        cmd: &[u8],
        read_len: usize,
    ) -> Result<Vec<u8>> {
        let mut full_cmd = Vec::with_capacity(cmd.len() + 1);
        full_cmd.extend_from_slice(cmd);
        full_cmd.push(mpsse::SEND_IMMEDIATE);
        dev.write_all(&full_cmd).await?;

        let mut buf = vec![0u8; read_len];
        let mut offset = 0;
        while offset < read_len {
            let n = dev.read_data(&mut buf[offset..]).await?;
            if n == 0 {
                break;
            }
            offset += n;
        }
        buf.truncate(offset);
        Ok(buf)
    }

    pub async fn write_commands(&self, dev: &mut FtdiDevice, cmd: &[u8]) -> Result<()> {
        dev.write_all(cmd).await
    }

    #[cfg(test)]
    pub(crate) fn test_new(is_h_type: bool) -> Self {
        Self {
            clock_hz: 0,
            is_h_type,
            gpio_low_value: 0x00,
            gpio_low_dir: 0x00,
            gpio_high_value: 0x00,
            gpio_high_dir: 0x00,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_divisor_calculations() {
        let div = (12_000_000u32 / (1_000_000 * 2)).saturating_sub(1);
        assert_eq!(div, 5);
        let actual = 12_000_000 / ((1 + div) * 2);
        assert_eq!(actual, 1_000_000);

        let div = (12_000_000u32 / (100_000 * 2)).saturating_sub(1);
        assert_eq!(div, 59);
        let actual = 12_000_000 / ((1 + div) * 2);
        assert_eq!(actual, 100_000);

        let div = (60_000_000u32 / (10_000_000 * 2)).saturating_sub(1);
        assert_eq!(div, 2);
        let actual = 60_000_000 / ((1 + div) * 2);
        assert_eq!(actual, 10_000_000);
    }

    #[test]
    fn clock_divisor_30mhz() {
        let div = (60_000_000u32 / (30_000_000 * 2)).saturating_sub(1);
        assert_eq!(div, 0);
        let actual = 60_000_000 / ((1 + div) * 2);
        assert_eq!(actual, 30_000_000);
    }

    #[test]
    fn clock_divisor_6mhz_boundary() {
        let div = (12_000_000u32 / (6_000_000 * 2)).saturating_sub(1);
        assert_eq!(div, 0);
        let actual = 12_000_000 / ((1 + div) * 2);
        assert_eq!(actual, 6_000_000);
    }

    #[test]
    fn clock_divisor_400khz_i2c() {
        let div = (12_000_000u32 / (400_000 * 2)).saturating_sub(1);
        assert_eq!(div, 14);
        let actual = 12_000_000 / ((1 + div) * 2);
        assert_eq!(actual, 400_000);
    }

    #[test]
    fn mpsse_context_default_state() {
        let ctx = MpsseContext::test_new(true);
        assert_eq!(ctx.gpio_low_value(), 0);
        assert_eq!(ctx.gpio_low_dir(), 0);
        assert_eq!(ctx.gpio_high_value(), 0);
        assert_eq!(ctx.gpio_high_dir(), 0);
        assert!(ctx.is_h_type());
        assert_eq!(ctx.clock_hz(), 0);
    }

    #[test]
    fn update_gpio_low_state_tracks_values() {
        let mut ctx = MpsseContext::test_new(true);
        ctx.update_gpio_low_state(0xAB, 0xCD);
        assert_eq!(ctx.gpio_low_value(), 0xAB);
        assert_eq!(ctx.gpio_low_dir(), 0xCD);
    }

    #[test]
    fn mpsse_command_constants() {
        assert_eq!(mpsse::SET_BITS_LOW, 0x80);
        assert_eq!(mpsse::GET_BITS_LOW, 0x81);
        assert_eq!(mpsse::SET_BITS_HIGH, 0x82);
        assert_eq!(mpsse::GET_BITS_HIGH, 0x83);
        assert_eq!(mpsse::LOOPBACK_START, 0x84);
        assert_eq!(mpsse::LOOPBACK_END, 0x85);
        assert_eq!(mpsse::TCK_DIVISOR, 0x86);
        assert_eq!(mpsse::SEND_IMMEDIATE, 0x87);
        assert_eq!(mpsse::DIS_DIV_5, 0x8A);
        assert_eq!(mpsse::EN_DIV_5, 0x8B);
        assert_eq!(mpsse::EN_3_PHASE, 0x8C);
        assert_eq!(mpsse::DIS_3_PHASE, 0x8D);
        assert_eq!(mpsse::DIS_ADAPTIVE, 0x97);
    }

    #[test]
    fn mpsse_shifting_flags() {
        assert_eq!(mpsse::WRITE_NEG, 0x01);
        assert_eq!(mpsse::BITMODE, 0x02);
        assert_eq!(mpsse::READ_NEG, 0x04);
        assert_eq!(mpsse::LSB, 0x08);
        assert_eq!(mpsse::DO_WRITE, 0x10);
        assert_eq!(mpsse::DO_READ, 0x20);
        assert_eq!(mpsse::WRITE_TMS, 0x40);
    }

    #[test]
    fn div_value_helper() {
        assert_eq!(mpsse::div_value(1_000_000), 5);
        assert_eq!(mpsse::div_value(6_000_000), 0);
        assert_eq!(mpsse::div_value(10_000_000), 0);
        assert_eq!(mpsse::div_value(0), 0xFFFF);
    }

    #[test]
    fn check_bad_command_empty() {
        assert!(MpsseContext::check_bad_command(&[]).is_ok());
    }

    #[test]
    fn check_bad_command_normal_data() {
        assert!(MpsseContext::check_bad_command(&[0x00, 0x01, 0xFF]).is_ok());
    }

    #[test]
    fn check_bad_command_detected() {
        let response = [0xFA, 0xAB];
        let err = MpsseContext::check_bad_command(&response).unwrap_err();
        match err {
            crate::error::Error::MpsseBadCommand(opcode) => assert_eq!(opcode, 0xAB),
            _ => panic!("expected MpsseBadCommand error, got {:?}", err),
        }
    }

    #[test]
    fn check_bad_command_in_middle_of_data() {
        let response = [0x01, 0x02, 0xFA, 0x99, 0x03];
        let err = MpsseContext::check_bad_command(&response).unwrap_err();
        match err {
            crate::error::Error::MpsseBadCommand(opcode) => assert_eq!(opcode, 0x99),
            _ => panic!("expected MpsseBadCommand error"),
        }
    }

    #[test]
    fn check_bad_command_fa_at_end_no_match() {
        let response = [0x01, 0x02, 0xFA];
        assert!(MpsseContext::check_bad_command(&response).is_ok());
    }

    #[test]
    fn bad_command_constant() {
        assert_eq!(MpsseContext::BAD_COMMAND, 0xFA);
    }
}
