#![allow(dead_code)]

use ftdi_nusb::mpsse::{
    AsyncMpsseContext,
    gpio::{AsyncGpioPin, GpioBank},
    i2c::AsyncI2cBus,
    jtag::AsyncJtagBus,
    spi::{AsyncSpiDevice, SpiMode},
};
use ftdi_nusb::{AsyncFtdiDevice, Result};

#[cfg(not(target_arch = "wasm32"))]
use ftdi_nusb::FtdiDevice;
#[cfg(not(target_arch = "wasm32"))]
use ftdi_nusb::mpsse::{MpsseContext, gpio::GpioPin, i2c::I2cBus, jtag::JtagBus, spi::SpiDevice};

async fn async_mpsse_api(dev: &mut AsyncFtdiDevice) -> Result<()> {
    let mut ctx = AsyncMpsseContext::init(dev, 1_000_000).await?;
    let spi = AsyncSpiDevice::new(&mut ctx, dev, SpiMode::Mode0).await?;
    spi.write(&mut ctx, dev, &[0x9f]).await?;

    let i2c = AsyncI2cBus::new(&mut ctx, dev).await?;
    i2c.write(&mut ctx, dev, 0x50, &[0]).await?;

    let mut gpio = AsyncGpioPin::new(GpioBank::High, 0);
    gpio.set_output(&mut ctx, dev, true).await?;

    let mut jtag = AsyncJtagBus::new(&mut ctx, dev).await?;
    jtag.reset(dev).await
}

#[cfg(not(target_arch = "wasm32"))]
fn blocking_mpsse_api(dev: &mut FtdiDevice) -> Result<()> {
    let mut ctx = MpsseContext::init(dev, 1_000_000)?;
    let spi = SpiDevice::new(&mut ctx, dev, SpiMode::Mode0)?;
    spi.write(&mut ctx, dev, &[0x9f])?;

    let i2c = I2cBus::new(&mut ctx, dev)?;
    i2c.write(&mut ctx, dev, 0x50, &[0])?;

    let mut gpio = GpioPin::new(GpioBank::High, 0);
    gpio.set_output(&mut ctx, dev, true)?;

    let mut jtag = JtagBus::new(&mut ctx, dev)?;
    jtag.reset(dev)
}
