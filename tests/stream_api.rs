//! Compile-only checks for the streaming API surface.

#![cfg(not(target_arch = "wasm32"))]
#![allow(dead_code)]

use ftdi_nusb::{FtdiDevice, Result, StreamEvent};

async fn async_stream_api(dev: &mut FtdiDevice) -> Result<()> {
    dev.read_stream(|_, _| false, 8, 4).await?;
    dev.read_stream_async(
        |event| async move { matches!(event, StreamEvent::Progress(_)) },
        8,
        4,
    )
    .await?;

    let mut stream = dev.start_stream(8, 4).await?;
    let _ = stream.next().await?;
    stream.finish().await
}

fn blocking_device_reaches_streaming(dev: &mut ftdi_nusb::blocking::FtdiDevice) -> Result<()> {
    futures_lite::future::block_on(dev.as_async_mut().read_stream(|_, _| false, 8, 4))
}
