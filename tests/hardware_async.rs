#![cfg(all(not(target_arch = "wasm32"), feature = "smol"))]

use std::time::{Duration, Instant};

use ftdi_nusb::mpsse::{
    MpsseContext,
    spi::{SpiDevice, SpiMode},
};
use ftdi_nusb::{FtdiDevice, BitMode, Error, Result};

const FTDI_VID: u16 = 0x0403;
const FT232H_PID: u16 = 0x6014;

async fn read_exact(dev: &mut FtdiDevice, len: usize) -> Result<Vec<u8>> {
    let timeout = dev.read_timeout();
    let deadline = Instant::now() + timeout;
    let mut received = Vec::with_capacity(len);
    while received.len() < len {
        let mut buffer = vec![0; len - received.len()];
        let count = dev.read_data(&mut buffer).await?;
        if count == 0 {
            if Instant::now() >= deadline {
                return Err(Error::Timeout(timeout));
            }
            continue;
        }
        received.extend_from_slice(&buffer[..count]);
    }
    Ok(received)
}

async fn assert_loopback(dev: &mut FtdiDevice, payload: &[u8]) -> Result<()> {
    dev.flush_all().await?;
    dev.write_all(payload).await?;
    assert_eq!(read_exact(dev, payload.len()).await?, payload);
    Ok(())
}

/// Requires an FT232H with TXD wired to RXD.
///
/// Run serially because the test claims the USB interface exclusively:
///
/// ```text
/// cargo test --features smol --test hardware_async -- --ignored --test-threads=1 --nocapture
/// ```
#[test]
#[ignore = "requires an FT232H UART loopback fixture"]
fn ft232h_uart_async_cancellation_and_recovery() {
    futures_lite::future::block_on(async {
        eprintln!("opening FT232H");
        let mut dev = FtdiDevice::open(FTDI_VID, FT232H_PID).await?;
        eprintln!("opened FT232H");
        dev.set_bitmode(0xff, BitMode::Reset).await?;
        dev.set_baudrate(115_200).await?;
        dev.set_latency_timer(16).await?;
        dev.set_read_timeout(Duration::from_secs(2));
        dev.set_write_timeout(Duration::from_secs(2));

        assert_loopback(&mut dev, b"ftdi-nusb basic async loopback").await?;
        eprintln!("basic async UART loopback passed");

        // Cancel a pending read before the FTDI latency timer completes. The
        // subsequent read must resume that USB transfer and receive the bytes
        // written after cancellation.
        dev.set_latency_timer(255).await?;
        dev.flush_all().await?;
        let mut cancelled_buffer = [0; 64];
        let read_was_cancelled = futures_lite::future::race(
            async {
                let _ = dev.read_data(&mut cancelled_buffer).await;
                false
            },
            async {
                futures_timer::Delay::new(Duration::from_millis(20)).await;
                true
            },
        )
        .await;
        assert!(read_was_cancelled, "read completed before cancellation");

        let after_cancel = b"input survives cancelled read";
        dev.write_all(after_cancel).await?;
        assert_eq!(
            read_exact(&mut dev, after_cancel.len()).await?,
            after_cancel
        );
        eprintln!("cancelled read resumed without losing loopback input");

        // Dropping a streaming session cannot await cleanup. `recover` must
        // drain its queued transfers and return the FT232H to UART/reset mode.
        eprintln!("starting synchronous-FIFO stream");
        let stream = dev.start_stream(1, 2).await?;
        eprintln!("dropping synchronous-FIFO stream");
        drop(stream);
        eprintln!("recovering abandoned stream");
        dev.recover().await?;
        eprintln!("recovery completed");
        dev.set_baudrate(115_200).await?;
        dev.set_latency_timer(16).await?;

        assert_loopback(&mut dev, b"UART works after abandoned stream recovery").await?;
        eprintln!("abandoned stream recovery restored UART loopback");

        Ok::<(), ftdi_nusb::Error>(())
    })
    .unwrap();
}

/// Requires an FT232H with ADBUS1/MOSI wired to ADBUS2/MISO.
#[test]
#[ignore = "requires an FT232H SPI loopback fixture"]
fn ft232h_async_spi_loopback() {
    let _ = env_logger::try_init();
    futures_lite::future::block_on(async {
        let mut dev = FtdiDevice::open(FTDI_VID, FT232H_PID).await?;
        dev.set_read_timeout(Duration::from_secs(2));
        dev.set_write_timeout(Duration::from_secs(2));

        let mut mpsse = MpsseContext::init(&mut dev, 1_000_000).await?;
        let spi = SpiDevice::new(&mut mpsse, &mut dev, SpiMode::Mode0).await?;

        for length in [
            1, 4, 257, 510, 511, 1020, 1021, 2040, 2041, 4080, 4081, 4096,
        ] {
            let transmitted: Vec<u8> = (0..length)
                .map(|index| (index as u8).wrapping_mul(73).wrapping_add(19))
                .collect();
            let received = spi.transfer(&mut mpsse, &mut dev, &transmitted).await?;
            assert_eq!(
                received, transmitted,
                "SPI loopback mismatch at {length} bytes"
            );
            eprintln!("async SPI loopback passed for {length} bytes");
        }

        dev.recover().await?;
        assert!(matches!(
            mpsse.set_clock(&mut dev, 2_000_000).await,
            Err(Error::InvalidMpsseContext)
        ));
        let mut mpsse = MpsseContext::init(&mut dev, 1_000_000).await?;
        let spi = SpiDevice::new(&mut mpsse, &mut dev, SpiMode::Mode0).await?;
        assert_eq!(
            spi.transfer(&mut mpsse, &mut dev, b"reinitialized after recovery")
                .await?,
            b"reinitialized after recovery"
        );
        Ok::<(), ftdi_nusb::Error>(())
    })
    .unwrap();
}

/// Exercises synchronous-FIFO timeout and cleanup without an FT245 producer,
/// then verifies the device can immediately return to MPSSE SPI operation.
#[test]
#[ignore = "requires an FT232H SPI loopback fixture"]
fn ft232h_sync_fifo_timeout_restores_device() {
    futures_lite::future::block_on(async {
        let mut dev = FtdiDevice::open(FTDI_VID, FT232H_PID).await?;
        let timeout = Duration::from_millis(100);
        dev.set_read_timeout(timeout);
        dev.set_write_timeout(Duration::from_secs(2));

        let mut stream = dev.start_stream(1, 4).await?;
        let error = stream.next().await.unwrap_err();
        assert!(matches!(error, Error::Timeout(actual) if actual == timeout));
        drop(stream);
        eprintln!("synchronous-FIFO inactivity returned the configured timeout");

        {
            let mut mpsse = MpsseContext::init(&mut dev, 1_000_000).await?;
            let spi = SpiDevice::new(&mut mpsse, &mut dev, SpiMode::Mode0).await?;
            let transmitted = b"SPI works after synchronous-FIFO timeout";
            let received = spi.transfer(&mut mpsse, &mut dev, transmitted).await?;
            assert_eq!(received, transmitted);
            eprintln!("synchronous-FIFO timeout cleanup restored MPSSE SPI");
        }

        // Status-only USB packets must not count as payload activity, but they
        // should still drive periodic progress. Stop cleanly on that event.
        dev.set_read_timeout(Duration::from_secs(2));
        dev.read_stream(|_, progress| progress.is_none(), 1, 4)
            .await?;
        eprintln!("synchronous-FIFO progress callback stopped cleanly");

        let mut mpsse = MpsseContext::init(&mut dev, 1_000_000).await?;
        let spi = SpiDevice::new(&mut mpsse, &mut dev, SpiMode::Mode0).await?;
        let transmitted = b"SPI works after clean synchronous-FIFO finish";
        let received = spi.transfer(&mut mpsse, &mut dev, transmitted).await?;
        assert_eq!(received, transmitted);
        eprintln!("clean synchronous-FIFO finish restored MPSSE SPI");

        dev.recover().await?;
        Ok::<(), ftdi_nusb::Error>(())
    })
    .unwrap();
}
