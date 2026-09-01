//! High-level MPSSE (Multi-Protocol Synchronous Serial Engine) API.

pub mod gpio;
pub mod i2c;
pub mod jtag;
pub mod spi;

use crate::constants::mpsse;
use crate::context::FtdiDevice;
use crate::error::{Error, Result};
use crate::types::{BitMode, ChipType};

#[cfg(not(target_arch = "wasm32"))]
struct ReadDeadline(Option<std::time::Instant>);

#[cfg(target_arch = "wasm32")]
struct ReadDeadline(f64);

impl ReadDeadline {
    fn new(timeout: core::time::Duration) -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        {
            Self(std::time::Instant::now().checked_add(timeout))
        }
        #[cfg(target_arch = "wasm32")]
        {
            Self(js_sys::Date::now() + timeout.as_secs_f64() * 1_000.0)
        }
    }

    fn expired(&self) -> bool {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.0
                .is_some_and(|deadline| std::time::Instant::now() >= deadline)
        }
        #[cfg(target_arch = "wasm32")]
        {
            js_sys::Date::now() >= self.0
        }
    }
}

fn clock_divisor_command(is_h_type: bool, divisor: u16) -> Result<(Vec<u8>, u32)> {
    if divisor < 2 || divisor % 2 != 0 {
        return Err(Error::InvalidArgument(
            "clock divisor must be even and at least 2",
        ));
    }

    let register_divisor = divisor / 2 - 1;
    let mut cmd = Vec::with_capacity(4);
    let source_clock = if is_h_type {
        cmd.push(mpsse::DIS_DIV_5);
        60_000_000
    } else {
        12_000_000
    };
    cmd.extend_from_slice(&[
        mpsse::TCK_DIVISOR,
        register_divisor as u8,
        (register_divisor >> 8) as u8,
    ]);
    Ok((cmd, source_clock / divisor as u32))
}

pub(super) async fn read_exact(dev: &mut FtdiDevice, len: usize) -> Result<Vec<u8>> {
    let deadline = ReadDeadline::new(dev.read_timeout());
    let mut buf = vec![0; len];
    let mut offset = 0;
    while offset < len {
        if deadline.expired() {
            return Err(Error::Timeout(dev.read_timeout()));
        }
        let read = dev.read_data(&mut buf[offset..]).await?;
        if read == 0 {
            continue;
        }
        offset += read;
    }
    Ok(buf)
}

/// MPSSE context holding pin state and clock configuration.
///
/// All MPSSE I/O happens through an [`MpsseSession`] obtained from
/// [`session`](Self::session), which binds this context to the device it was
/// initialized on for the duration of the borrow.
#[derive(Debug)]
pub struct MpsseContext {
    clock_hz: u32,
    is_h_type: bool,
    gpio_low_value: u8,
    gpio_low_dir: u8,
    gpio_high_value: u8,
    gpio_high_dir: u8,
    recovery_epoch: u64,
    protocol_epoch: u64,
    device_id: u64,
}

/// An MPSSE context paired with a mutable borrow of its device.
///
/// Created by [`MpsseContext::session`]. Validity (device identity and
/// recovery epoch) is checked once at creation; the exclusive device borrow
/// guarantees it cannot change while the session exists.
pub struct MpsseSession<'a> {
    pub(crate) ctx: &'a mut MpsseContext,
    pub(crate) dev: &'a mut FtdiDevice,
}

impl MpsseContext {
    /// Initialize MPSSE mode on the device and configure the clock frequency.
    pub async fn init(dev: &mut FtdiDevice, clock_hz: u32) -> Result<Self> {
        dev.ensure_ready()?;
        let chip = dev.chip_type();
        let is_h_type = chip.is_h_type();

        match chip {
            ChipType::Ft2232C | ChipType::Ft2232H | ChipType::Ft4232H | ChipType::Ft232H => {}
            _ => return Err(Error::UnsupportedChip(chip)),
        }

        let guard = dev.begin_stateful_operation()?;
        // Re-initialization resets clock, GPIO, and engine state, so any
        // previously created context or bus object is now stale.
        dev.bump_recovery_epoch();
        dev.set_bitmode(0, BitMode::Reset).await?;
        dev.flush_all().await?;
        dev.set_bitmode(0, BitMode::Mpsse).await?;

        // Short delay for the MPSSE engine to start
        crate::sleep_util::sleep(core::time::Duration::from_millis(50)).await;

        dev.flush_rx().await?;

        let mut ctx = Self {
            clock_hz: 0,
            is_h_type,
            gpio_low_value: 0x00,
            gpio_low_dir: 0x00,
            gpio_high_value: 0x00,
            gpio_high_dir: 0x00,
            recovery_epoch: dev.recovery_epoch(),
            protocol_epoch: 0,
            device_id: dev.device_id(),
        };

        let mut cmd = Vec::with_capacity(16);
        cmd.push(mpsse::LOOPBACK_END);
        if is_h_type {
            cmd.push(mpsse::DIS_ADAPTIVE);
        }
        dev.write_all(&cmd).await?;

        MpsseSession { ctx: &mut ctx, dev }
            .set_clock(clock_hz)
            .await?;

        guard.disarm();
        Ok(ctx)
    }

    /// Borrow this context together with its device for MPSSE I/O.
    ///
    /// Fails with [`Error::InvalidMpsseContext`] when `dev` is not the device
    /// this context was initialized on, or when the device has been recovered
    /// since initialization.
    pub fn session<'a>(&'a mut self, dev: &'a mut FtdiDevice) -> Result<MpsseSession<'a>> {
        if self.device_id != dev.device_id() {
            return Err(Error::InvalidMpsseContext);
        }
        self.ensure_current(dev)?;
        Ok(MpsseSession { ctx: self, dev })
    }

    pub fn clock_hz(&self) -> u32 {
        self.clock_hz
    }

    pub(crate) fn ensure_current(&self, dev: &FtdiDevice) -> Result<()> {
        if self.recovery_epoch == dev.recovery_epoch() {
            Ok(())
        } else {
            Err(Error::InvalidMpsseContext)
        }
    }
}

impl MpsseSession<'_> {
    /// Read-only access to the underlying context state.
    pub fn ctx(&self) -> &MpsseContext {
        self.ctx
    }

    pub(crate) fn protocol_epoch(&self) -> u64 {
        self.ctx.protocol_epoch
    }

    pub(crate) fn bump_protocol_epoch(&mut self) -> u64 {
        self.ctx.protocol_epoch = self.ctx.protocol_epoch.wrapping_add(1);
        self.ctx.protocol_epoch
    }

    pub async fn set_clock(&mut self, clock_hz: u32) -> Result<()> {
        if clock_hz == 0 {
            return Err(Error::InvalidArgument("clock frequency must be > 0"));
        }

        let max_freq = if self.ctx.is_h_type {
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
        let actual_clock;

        if self.ctx.is_h_type {
            if clock_hz > 6_000_000 {
                cmd.push(mpsse::DIS_DIV_5);
                let divisor = (60_000_000u32 / clock_hz.saturating_mul(2)).saturating_sub(1);
                let divisor = divisor.min(0xFFFF) as u16;
                cmd.extend_from_slice(&[mpsse::TCK_DIVISOR, divisor as u8, (divisor >> 8) as u8]);
                actual_clock = 60_000_000 / ((1 + divisor as u32) * 2);
            } else {
                cmd.push(mpsse::EN_DIV_5);
                let divisor = (12_000_000u32 / clock_hz.saturating_mul(2)).saturating_sub(1);
                let divisor = divisor.min(0xFFFF) as u16;
                cmd.extend_from_slice(&[mpsse::TCK_DIVISOR, divisor as u8, (divisor >> 8) as u8]);
                actual_clock = 12_000_000 / ((1 + divisor as u32) * 2);
            }
        } else {
            let divisor = (12_000_000u32 / clock_hz.saturating_mul(2)).saturating_sub(1);
            let divisor = divisor.min(0xFFFF) as u16;
            cmd.extend_from_slice(&[mpsse::TCK_DIVISOR, divisor as u8, (divisor >> 8) as u8]);
            actual_clock = 12_000_000 / ((1 + divisor as u32) * 2);
        }

        let guard = self.dev.begin_stateful_operation()?;
        self.dev.write_all(&cmd).await?;
        self.ctx.clock_hz = actual_clock;
        guard.disarm();
        Ok(())
    }

    /// Set the MPSSE clock using the FTDI frequency divisor directly.
    ///
    /// `divisor` is the even divisor applied to the chip's fastest MPSSE
    /// source clock: 60 MHz on H-type parts and 12 MHz on older parts. This
    /// matches the divisor convention used by libftdi and flashrom adapters.
    pub async fn set_clock_divisor(&mut self, divisor: u16) -> Result<()> {
        let (cmd, actual_clock) = clock_divisor_command(self.ctx.is_h_type, divisor)?;

        let guard = self.dev.begin_stateful_operation()?;
        self.dev.write_all(&cmd).await?;
        self.ctx.clock_hz = actual_clock;
        guard.disarm();
        Ok(())
    }

    pub async fn enable_3phase_clocking(&mut self) -> Result<()> {
        if !self.ctx.is_h_type {
            return Err(Error::InvalidArgument(
                "3-phase clocking only supported on H-type chips",
            ));
        }
        self.write_commands(&[mpsse::EN_3_PHASE]).await?;
        self.bump_protocol_epoch();
        Ok(())
    }

    pub async fn disable_3phase_clocking(&mut self) -> Result<()> {
        if !self.ctx.is_h_type {
            return Err(Error::InvalidArgument(
                "3-phase clocking only supported on H-type chips",
            ));
        }
        self.write_commands(&[mpsse::DIS_3_PHASE]).await?;
        self.bump_protocol_epoch();
        Ok(())
    }

    pub async fn enable_loopback(&mut self) -> Result<()> {
        self.write_commands(&[mpsse::LOOPBACK_START]).await
    }

    pub async fn disable_loopback(&mut self) -> Result<()> {
        self.write_commands(&[mpsse::LOOPBACK_END]).await
    }

    pub async fn set_gpio_low(&mut self, value: u8, direction: u8) -> Result<()> {
        let guard = self.dev.begin_stateful_operation()?;
        self.dev
            .write_all(&[mpsse::SET_BITS_LOW, value, direction])
            .await?;
        self.ctx.gpio_low_value = value;
        self.ctx.gpio_low_dir = direction;
        guard.disarm();
        Ok(())
    }

    pub async fn get_gpio_low(&mut self) -> Result<u8> {
        let guard = self.dev.begin_stateful_operation()?;
        self.dev
            .write_all(&[mpsse::GET_BITS_LOW, mpsse::SEND_IMMEDIATE])
            .await?;
        let value = read_exact(self.dev, 1).await?[0];
        guard.disarm();
        Ok(value)
    }

    pub async fn set_gpio_high(&mut self, value: u8, direction: u8) -> Result<()> {
        let guard = self.dev.begin_stateful_operation()?;
        self.dev
            .write_all(&[mpsse::SET_BITS_HIGH, value, direction])
            .await?;
        self.ctx.gpio_high_value = value;
        self.ctx.gpio_high_dir = direction;
        guard.disarm();
        Ok(())
    }

    /// Tristate all low- and high-byte MPSSE GPIO pins.
    pub async fn release_pins(&mut self) -> Result<()> {
        let guard = self.dev.begin_stateful_operation()?;
        self.dev
            .write_all(&[mpsse::SET_BITS_LOW, 0, 0, mpsse::SET_BITS_HIGH, 0, 0])
            .await?;
        self.ctx.gpio_low_value = 0;
        self.ctx.gpio_low_dir = 0;
        self.ctx.gpio_high_value = 0;
        self.ctx.gpio_high_dir = 0;
        guard.disarm();
        Ok(())
    }

    pub async fn get_gpio_high(&mut self) -> Result<u8> {
        let guard = self.dev.begin_stateful_operation()?;
        self.dev
            .write_all(&[mpsse::GET_BITS_HIGH, mpsse::SEND_IMMEDIATE])
            .await?;
        let value = read_exact(self.dev, 1).await?[0];
        guard.disarm();
        Ok(value)
    }

    pub async fn sync_mpsse(&mut self) -> Result<()> {
        const BOGUS_CMD: u8 = 0xAB;
        let guard = self.dev.begin_stateful_operation()?;
        let deadline = ReadDeadline::new(self.dev.read_timeout());

        self.dev
            .write_all(&[BOGUS_CMD, mpsse::SEND_IMMEDIATE])
            .await?;

        let mut buf = [0u8; 64];
        let mut previous_was_bad_command = false;
        loop {
            if deadline.expired() {
                return Err(Error::Timeout(self.dev.read_timeout()));
            }
            let n = self.dev.read_data(&mut buf).await?;
            for &byte in &buf[..n] {
                if previous_was_bad_command && byte == BOGUS_CMD {
                    guard.disarm();
                    return Ok(());
                }
                previous_was_bad_command = byte == MpsseContext::BAD_COMMAND;
            }
        }
    }

    pub async fn command_response(&mut self, cmd: &[u8], read_len: usize) -> Result<Vec<u8>> {
        let guard = self.dev.begin_stateful_operation()?;
        let mut full_cmd = Vec::with_capacity(cmd.len() + 1);
        full_cmd.extend_from_slice(cmd);
        full_cmd.push(mpsse::SEND_IMMEDIATE);
        self.dev.write_all(&full_cmd).await?;

        let response = read_exact(self.dev, read_len).await?;
        guard.disarm();
        Ok(response)
    }

    pub async fn write_commands(&mut self, cmd: &[u8]) -> Result<()> {
        let guard = self.dev.begin_stateful_operation()?;
        self.dev.write_all(cmd).await?;
        guard.disarm();
        Ok(())
    }
}

impl MpsseContext {
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

    #[cfg(test)]
    pub(crate) fn test_new(is_h_type: bool) -> Self {
        Self {
            clock_hz: 0,
            is_h_type,
            gpio_low_value: 0x00,
            gpio_low_dir: 0x00,
            gpio_high_value: 0x00,
            gpio_high_dir: 0x00,
            recovery_epoch: 0,
            protocol_epoch: 0,
            device_id: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_clock_divisor_matches_flashrom_convention() {
        assert_eq!(
            clock_divisor_command(true, 2).unwrap(),
            (vec![mpsse::DIS_DIV_5, mpsse::TCK_DIVISOR, 0, 0], 30_000_000)
        );
        assert_eq!(
            clock_divisor_command(false, 6).unwrap(),
            (vec![mpsse::TCK_DIVISOR, 2, 0], 2_000_000)
        );
        assert!(clock_divisor_command(true, 1).is_err());
        assert!(clock_divisor_command(true, 3).is_err());
    }

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
