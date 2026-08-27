//! Native blocking wrappers for the asynchronous MPSSE implementation.

use crate::blocking::FtdiDevice;
use crate::error::Result;

use super::AsyncMpsseContext;
use super::gpio::{AsyncGpioGroup, AsyncGpioPin, GpioBank};
use super::i2c::AsyncI2cBus;
use super::jtag::{AsyncJtagBus, TapState};
use super::spi::{AsyncSpiDevice, SpiMode};

#[derive(Debug, Clone)]
pub struct MpsseContext(AsyncMpsseContext);

impl MpsseContext {
    pub fn init(dev: &mut FtdiDevice, clock_hz: u32) -> Result<Self> {
        futures_lite::future::block_on(AsyncMpsseContext::init(dev.as_async_mut(), clock_hz))
            .map(Self)
    }

    pub fn into_async(self) -> AsyncMpsseContext {
        self.0
    }

    pub fn clock_hz(&self) -> u32 {
        self.0.clock_hz()
    }
    pub fn gpio_low_dir(&self) -> u8 {
        self.0.gpio_low_dir()
    }
    pub fn gpio_low_value(&self) -> u8 {
        self.0.gpio_low_value()
    }
    pub fn gpio_high_dir(&self) -> u8 {
        self.0.gpio_high_dir()
    }
    pub fn gpio_high_value(&self) -> u8 {
        self.0.gpio_high_value()
    }
    pub fn is_h_type(&self) -> bool {
        self.0.is_h_type()
    }

    pub const BAD_COMMAND: u8 = AsyncMpsseContext::BAD_COMMAND;
    pub fn check_bad_command(response: &[u8]) -> Result<()> {
        AsyncMpsseContext::check_bad_command(response)
    }

    pub fn set_clock(&mut self, dev: &mut FtdiDevice, clock_hz: u32) -> Result<()> {
        futures_lite::future::block_on(self.0.set_clock(dev.as_async_mut(), clock_hz))
    }
    pub fn enable_3phase_clocking(&self, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.enable_3phase_clocking(dev.as_async_mut()))
    }
    pub fn disable_3phase_clocking(&self, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.disable_3phase_clocking(dev.as_async_mut()))
    }
    pub fn enable_loopback(&self, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.enable_loopback(dev.as_async_mut()))
    }
    pub fn disable_loopback(&self, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.disable_loopback(dev.as_async_mut()))
    }
    pub fn set_gpio_low(&mut self, dev: &mut FtdiDevice, value: u8, direction: u8) -> Result<()> {
        futures_lite::future::block_on(self.0.set_gpio_low(dev.as_async_mut(), value, direction))
    }
    pub fn get_gpio_low(&self, dev: &mut FtdiDevice) -> Result<u8> {
        futures_lite::future::block_on(self.0.get_gpio_low(dev.as_async_mut()))
    }
    pub fn set_gpio_high(&mut self, dev: &mut FtdiDevice, value: u8, direction: u8) -> Result<()> {
        futures_lite::future::block_on(self.0.set_gpio_high(dev.as_async_mut(), value, direction))
    }
    pub fn get_gpio_high(&self, dev: &mut FtdiDevice) -> Result<u8> {
        futures_lite::future::block_on(self.0.get_gpio_high(dev.as_async_mut()))
    }
    pub fn sync_mpsse(&self, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.sync_mpsse(dev.as_async_mut()))
    }
    pub fn command_response(
        &self,
        dev: &mut FtdiDevice,
        cmd: &[u8],
        read_len: usize,
    ) -> Result<Vec<u8>> {
        futures_lite::future::block_on(self.0.command_response(dev.as_async_mut(), cmd, read_len))
    }
    pub fn write_commands(&self, dev: &mut FtdiDevice, cmd: &[u8]) -> Result<()> {
        futures_lite::future::block_on(self.0.write_commands(dev.as_async_mut(), cmd))
    }

    pub(crate) fn as_async(&self) -> &AsyncMpsseContext {
        &self.0
    }
    pub(crate) fn as_async_mut(&mut self) -> &mut AsyncMpsseContext {
        &mut self.0
    }
}

impl AsyncMpsseContext {
    pub fn into_blocking(self) -> MpsseContext {
        MpsseContext(self)
    }
}

#[derive(Debug, Clone)]
pub struct SpiDevice(AsyncSpiDevice);

impl SpiDevice {
    pub fn new(ctx: &mut MpsseContext, dev: &mut FtdiDevice, mode: SpiMode) -> Result<Self> {
        futures_lite::future::block_on(AsyncSpiDevice::new(
            ctx.as_async_mut(),
            dev.as_async_mut(),
            mode,
        ))
        .map(Self)
    }
    pub fn with_cs_pin(
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        mode: SpiMode,
        cs_pin: u8,
        cs_active_low: bool,
        lsb_first: bool,
    ) -> Result<Self> {
        futures_lite::future::block_on(AsyncSpiDevice::with_cs_pin(
            ctx.as_async_mut(),
            dev.as_async_mut(),
            mode,
            cs_pin,
            cs_active_low,
            lsb_first,
        ))
        .map(Self)
    }
    pub fn cs_assert(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.cs_assert(ctx.as_async_mut(), dev.as_async_mut()))
    }
    pub fn cs_deassert(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.cs_deassert(ctx.as_async_mut(), dev.as_async_mut()))
    }
    #[cfg(feature = "embedded-hal")]
    pub(crate) fn transfer_into_raw(
        &self,
        dev: &mut FtdiDevice,
        read: &mut [u8],
        write: &[u8],
    ) -> Result<()> {
        futures_lite::future::block_on(self.0.transfer_into_raw(dev.as_async_mut(), read, write))
    }
    #[cfg(feature = "embedded-hal")]
    pub(crate) fn write_raw(&self, dev: &mut FtdiDevice, tx: &[u8]) -> Result<()> {
        futures_lite::future::block_on(self.0.write_raw(dev.as_async_mut(), tx))
    }
    #[cfg(feature = "embedded-hal")]
    pub(crate) fn read_raw(&self, dev: &mut FtdiDevice, len: usize) -> Result<Vec<u8>> {
        futures_lite::future::block_on(self.0.read_raw(dev.as_async_mut(), len))
    }
    pub fn transfer(
        &self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        tx: &[u8],
    ) -> Result<Vec<u8>> {
        futures_lite::future::block_on(self.0.transfer(ctx.as_async_mut(), dev.as_async_mut(), tx))
    }
    pub fn write(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice, tx: &[u8]) -> Result<()> {
        futures_lite::future::block_on(self.0.write(ctx.as_async_mut(), dev.as_async_mut(), tx))
    }
    pub fn read(
        &self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        len: usize,
    ) -> Result<Vec<u8>> {
        futures_lite::future::block_on(self.0.read(ctx.as_async_mut(), dev.as_async_mut(), len))
    }
    pub fn write_read(
        &self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        tx: &[u8],
        read_len: usize,
    ) -> Result<Vec<u8>> {
        futures_lite::future::block_on(self.0.write_read(
            ctx.as_async_mut(),
            dev.as_async_mut(),
            tx,
            read_len,
        ))
    }
    pub fn mode(&self) -> SpiMode {
        self.0.mode()
    }
    pub fn is_lsb_first(&self) -> bool {
        self.0.is_lsb_first()
    }
    pub fn cs_pin(&self) -> u8 {
        self.0.cs_pin()
    }
    pub fn into_async(self) -> AsyncSpiDevice {
        self.0
    }
}
impl AsyncSpiDevice {
    pub fn into_blocking(self) -> SpiDevice {
        SpiDevice(self)
    }
}

#[derive(Debug, Clone)]
pub struct I2cBus(AsyncI2cBus);
impl I2cBus {
    pub fn new(ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<Self> {
        futures_lite::future::block_on(AsyncI2cBus::new(ctx.as_async_mut(), dev.as_async_mut()))
            .map(Self)
    }
    pub fn start(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.start(ctx.as_async_mut(), dev.as_async_mut()))
    }
    pub fn stop(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.stop(ctx.as_async_mut(), dev.as_async_mut()))
    }
    pub fn write_byte(&self, dev: &mut FtdiDevice, byte: u8) -> Result<bool> {
        futures_lite::future::block_on(self.0.write_byte(dev.as_async_mut(), byte))
    }
    pub fn read_byte(&self, dev: &mut FtdiDevice, ack: bool) -> Result<u8> {
        futures_lite::future::block_on(self.0.read_byte(dev.as_async_mut(), ack))
    }
    pub fn write(
        &self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        address: u8,
        data: &[u8],
    ) -> Result<()> {
        futures_lite::future::block_on(self.0.write(
            ctx.as_async_mut(),
            dev.as_async_mut(),
            address,
            data,
        ))
    }
    pub fn read(
        &self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        address: u8,
        len: usize,
    ) -> Result<Vec<u8>> {
        futures_lite::future::block_on(self.0.read(
            ctx.as_async_mut(),
            dev.as_async_mut(),
            address,
            len,
        ))
    }
    pub fn write_read(
        &self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        address: u8,
        write_data: &[u8],
        read_len: usize,
    ) -> Result<Vec<u8>> {
        futures_lite::future::block_on(self.0.write_read(
            ctx.as_async_mut(),
            dev.as_async_mut(),
            address,
            write_data,
            read_len,
        ))
    }
    pub fn into_async(self) -> AsyncI2cBus {
        self.0
    }
}
impl AsyncI2cBus {
    pub fn into_blocking(self) -> I2cBus {
        I2cBus(self)
    }
}

#[derive(Debug, Clone)]
pub struct GpioPin(AsyncGpioPin);
impl GpioPin {
    pub fn new(bank: GpioBank, bit: u8) -> Self {
        Self(AsyncGpioPin::new(bank, bit))
    }
    pub fn bank(&self) -> GpioBank {
        self.0.bank()
    }
    pub fn bit(&self) -> u8 {
        self.0.bit()
    }
    pub fn mask(&self) -> u8 {
        self.0.mask()
    }
    pub fn set_output(
        &mut self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        high: bool,
    ) -> Result<()> {
        futures_lite::future::block_on(self.0.set_output(
            ctx.as_async_mut(),
            dev.as_async_mut(),
            high,
        ))
    }
    pub fn set_input(&mut self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.set_input(ctx.as_async_mut(), dev.as_async_mut()))
    }
    pub fn write(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice, high: bool) -> Result<()> {
        futures_lite::future::block_on(self.0.write(ctx.as_async_mut(), dev.as_async_mut(), high))
    }
    pub fn read(&self, ctx: &MpsseContext, dev: &mut FtdiDevice) -> Result<bool> {
        futures_lite::future::block_on(self.0.read(ctx.as_async(), dev.as_async_mut()))
    }
    pub fn is_output(&self, ctx: &MpsseContext) -> bool {
        self.0.is_output(ctx.as_async())
    }
    pub fn into_async(self) -> AsyncGpioPin {
        self.0
    }
}
impl AsyncGpioPin {
    pub fn into_blocking(self) -> GpioPin {
        GpioPin(self)
    }
}

#[derive(Debug, Clone)]
pub struct GpioGroup(AsyncGpioGroup);
impl GpioGroup {
    pub fn new(bank: GpioBank, mask: u8) -> Self {
        Self(AsyncGpioGroup::new(bank, mask))
    }
    pub fn bank(&self) -> GpioBank {
        self.0.bank()
    }
    pub fn mask(&self) -> u8 {
        self.0.mask()
    }
    pub fn set_all_output(
        &self,
        ctx: &mut MpsseContext,
        dev: &mut FtdiDevice,
        values: u8,
    ) -> Result<()> {
        futures_lite::future::block_on(self.0.set_all_output(
            ctx.as_async_mut(),
            dev.as_async_mut(),
            values,
        ))
    }
    pub fn set_all_input(&self, ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.set_all_input(ctx.as_async_mut(), dev.as_async_mut()))
    }
    pub fn read(&self, ctx: &MpsseContext, dev: &mut FtdiDevice) -> Result<u8> {
        futures_lite::future::block_on(self.0.read(ctx.as_async(), dev.as_async_mut()))
    }
    pub fn into_async(self) -> AsyncGpioGroup {
        self.0
    }
}
impl AsyncGpioGroup {
    pub fn into_blocking(self) -> GpioGroup {
        GpioGroup(self)
    }
}

#[derive(Debug, Clone)]
pub struct JtagBus(AsyncJtagBus);
impl JtagBus {
    pub fn new(ctx: &mut MpsseContext, dev: &mut FtdiDevice) -> Result<Self> {
        futures_lite::future::block_on(AsyncJtagBus::new(ctx.as_async_mut(), dev.as_async_mut()))
            .map(Self)
    }
    pub fn state(&self) -> TapState {
        self.0.state()
    }
    pub fn reset(&mut self, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.reset(dev.as_async_mut()))
    }
    pub fn goto_shift_dr(&mut self, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.goto_shift_dr(dev.as_async_mut()))
    }
    pub fn goto_shift_ir(&mut self, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.goto_shift_ir(dev.as_async_mut()))
    }
    pub fn goto_idle(&mut self, dev: &mut FtdiDevice) -> Result<()> {
        futures_lite::future::block_on(self.0.goto_idle(dev.as_async_mut()))
    }
    pub fn idle_clocks(&self, dev: &mut FtdiDevice, count: u32) -> Result<()> {
        futures_lite::future::block_on(self.0.idle_clocks(dev.as_async_mut(), count))
    }
    pub fn shift_bits(
        &mut self,
        ctx: &MpsseContext,
        dev: &mut FtdiDevice,
        tdi_data: &[u8],
        bit_count: usize,
        exit_shift: bool,
    ) -> Result<Vec<u8>> {
        futures_lite::future::block_on(self.0.shift_bits(
            ctx.as_async(),
            dev.as_async_mut(),
            tdi_data,
            bit_count,
            exit_shift,
        ))
    }
    pub fn write_ir(
        &mut self,
        ctx: &MpsseContext,
        dev: &mut FtdiDevice,
        ir_data: &[u8],
        ir_len: usize,
    ) -> Result<Vec<u8>> {
        futures_lite::future::block_on(self.0.write_ir(
            ctx.as_async(),
            dev.as_async_mut(),
            ir_data,
            ir_len,
        ))
    }
    pub fn shift_dr(
        &mut self,
        ctx: &MpsseContext,
        dev: &mut FtdiDevice,
        dr_data: &[u8],
        dr_len: usize,
    ) -> Result<Vec<u8>> {
        futures_lite::future::block_on(self.0.shift_dr(
            ctx.as_async(),
            dev.as_async_mut(),
            dr_data,
            dr_len,
        ))
    }
    pub fn tms_pin(&self) -> u8 {
        self.0.tms_pin()
    }
    pub fn dir_mask(&self) -> u8 {
        self.0.dir_mask()
    }
    pub fn into_async(self) -> AsyncJtagBus {
        self.0
    }
}
impl AsyncJtagBus {
    pub fn into_blocking(self) -> JtagBus {
        JtagBus(self)
    }
}
