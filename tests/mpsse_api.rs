//! Compile-only checks for the MPSSE API surface.

#![allow(dead_code)]

use ftdi_nusb::mpsse::{
    MpsseContext,
    gpio::{GpioBank, GpioPin},
    i2c::I2cBus,
    jtag::JtagBus,
    spi::{SpiConfig, SpiDevice, SpiMode},
};
use ftdi_nusb::{FtdiDevice, Result};

async fn mpsse_api(dev: &mut FtdiDevice) -> Result<()> {
    let mut ctx = MpsseContext::init(dev, 1_000_000).await?;
    let mut s = ctx.session(dev)?;

    s.set_clock_divisor(6).await?;
    let spi = SpiDevice::with_config(
        &mut s,
        SpiConfig::new(SpiMode::Mode0)
            .with_cs_mask(0x18, true)
            .with_low_pins(0x20, 0x30)
            .with_high_pins(0, 0x70),
    )
    .await?;
    spi.write_read(&mut s, &[0x9f], 3).await?;

    let i2c = I2cBus::new(&mut s).await?;
    i2c.write(&mut s, 0x50, &[0]).await?;

    let mut gpio = GpioPin::new(GpioBank::High, 0);
    gpio.set_output(&mut s, true).await?;

    let mut jtag = JtagBus::new(&mut s).await?;
    jtag.reset(&mut s).await?;

    s.release_pins().await
}

/// A blocking device reaches the asynchronous MPSSE API through
/// `as_async_mut` and a `block_on` of the caller's choice.
#[cfg(not(target_arch = "wasm32"))]
fn blocking_device_reaches_mpsse(dev: &mut ftdi_nusb::blocking::FtdiDevice) -> Result<()> {
    futures_lite::future::block_on(mpsse_api(dev.as_async_mut()))
}
