//! High-performance synchronous-FIFO streaming.
//!
//! [`FtdiStream`] is the asynchronous core API. It owns the queued USB reads
//! until [`finish`](FtdiStream::finish) is awaited. The callback APIs are thin
//! compatibility wrappers around that session.

use core::future::Future;
use std::time::{Duration, Instant};

use crate::context::FtdiDevice;
use crate::error::{Error, Result};
use crate::types::{BitMode, ChipType};

/// Progress information for a streaming read operation.
#[derive(Debug, Clone)]
pub struct StreamProgress {
    /// Total payload bytes transferred since the stream started.
    pub total_bytes: u64,
    /// Total elapsed time since the stream started.
    pub total_time: Duration,
    /// Overall average transfer rate in bytes/second.
    pub total_rate: f64,
    /// Transfer rate for the most recent reporting interval in bytes/second.
    pub current_rate: f64,
}

/// An item produced by [`FtdiStream::next`].
#[derive(Debug)]
pub enum StreamEvent {
    /// FTDI payload bytes with per-packet modem status headers removed.
    Data(Vec<u8>),
    /// Periodic transfer statistics.
    Progress(StreamProgress),
}

type ReadEndpoint = nusb::Endpoint<nusb::transfer::Bulk, nusb::transfer::In>;

async fn next_with_timeout(
    endpoint: &mut ReadEndpoint,
    timeout: Duration,
) -> Option<nusb::transfer::Completion> {
    let Some(deadline) = Instant::now().checked_add(timeout) else {
        return Some(endpoint.next_complete().await);
    };
    let remaining = deadline.checked_duration_since(Instant::now())?;

    futures_lite::future::race(async { Some(endpoint.next_complete().await) }, async {
        futures_timer::Delay::new(remaining).await;
        None
    })
    .await
}

async fn cancel_and_drain(endpoint: &mut ReadEndpoint, timeout: Duration) -> Result<()> {
    endpoint.cancel_all();
    let deadline = Instant::now().checked_add(timeout);

    while endpoint.pending() > 0 {
        let completion = match deadline {
            Some(deadline) => {
                let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
                    return Err(Error::Timeout(timeout));
                };
                futures_lite::future::race(
                    async {
                        endpoint.next_complete().await;
                        true
                    },
                    async {
                        futures_timer::Delay::new(remaining).await;
                        false
                    },
                )
                .await
            }
            None => {
                endpoint.next_complete().await;
                true
            }
        };

        if !completion {
            return Err(Error::Timeout(timeout));
        }
    }

    Ok(())
}

fn strip_modem_status(data: &[u8], actual_len: usize, packet_size: usize) -> Vec<u8> {
    let mut payload = Vec::with_capacity(actual_len.saturating_sub(2));
    for packet_start in (0..actual_len).step_by(packet_size) {
        let packet_end = (packet_start + packet_size).min(actual_len);
        if packet_end - packet_start > 2 {
            payload.extend_from_slice(&data[packet_start + 2..packet_end]);
        }
    }
    payload
}

/// An active asynchronous synchronous-FIFO read session.
///
/// Call [`finish`](Self::finish) for deterministic cleanup. Dropping the
/// session requests cancellation but cannot asynchronously drain transfers;
/// call [`FtdiDevice::recover`] before using the device for another
/// protocol if a session future was cancelled or the session was dropped.
pub struct FtdiStream<'a> {
    device: &'a mut FtdiDevice,
    timeout: Duration,
    start: Instant,
    last_payload_time: Instant,
    previous_time: Instant,
    previous_bytes: u64,
    total_bytes: u64,
    progress_interval: Duration,
    pending_progress: Option<StreamProgress>,
    finished: bool,
}

impl FtdiStream<'_> {
    /// Wait for the next payload or progress event.
    ///
    /// USB transfer failures and inactivity timeouts are returned as errors.
    /// Transfer buffers are recycled after each successful completion.
    pub async fn next(&mut self) -> Result<Option<StreamEvent>> {
        if self.finished {
            return Ok(None);
        }
        if let Some(progress) = self.pending_progress.take() {
            return Ok(Some(StreamEvent::Progress(progress)));
        }

        loop {
            let remaining = self
                .timeout
                .checked_sub(Instant::now().duration_since(self.last_payload_time))
                .ok_or(Error::Timeout(self.timeout));
            let completion = match remaining {
                Ok(remaining) => next_with_timeout(self.device.read_endpoint_mut(), remaining)
                    .await
                    .ok_or(Error::Timeout(self.timeout)),
                Err(error) => Err(error),
            };

            let mut completion = match completion {
                Ok(completion) => completion,
                Err(error) => {
                    if let Err(cleanup_error) = self.finish_inner().await {
                        log::warn!("stream cleanup after {error} failed: {cleanup_error}");
                    }
                    return Err(error);
                }
            };

            if let Err(error) = completion.status {
                if let Err(cleanup_error) = self.finish_inner().await {
                    log::warn!(
                        "stream cleanup after transfer error {error} failed: {cleanup_error}"
                    );
                }
                return Err(Error::Transfer(error));
            }

            let actual_len = completion.actual_len;
            let packet_size = self.device.read_endpoint_mut().max_packet_size();
            let payload = strip_modem_status(&completion.buffer, actual_len, packet_size);

            // Reuse the transfer allocation rather than allocating a new USB
            // buffer for every completion.
            completion.buffer.clear();
            self.device.read_endpoint_mut().submit(completion.buffer);

            self.total_bytes += payload.len() as u64;
            let now = Instant::now();
            if !payload.is_empty() {
                self.last_payload_time = now;
            }
            if now.duration_since(self.previous_time) >= self.progress_interval {
                let total_time = now.duration_since(self.start);
                let interval_time = now.duration_since(self.previous_time);
                self.pending_progress = Some(StreamProgress {
                    total_bytes: self.total_bytes,
                    total_time,
                    total_rate: self.total_bytes as f64 / total_time.as_secs_f64(),
                    current_rate: (self.total_bytes - self.previous_bytes) as f64
                        / interval_time.as_secs_f64(),
                });
                self.previous_time = now;
                self.previous_bytes = self.total_bytes;
            }

            if !payload.is_empty() {
                return Ok(Some(StreamEvent::Data(payload)));
            }
            if let Some(progress) = self.pending_progress.take() {
                return Ok(Some(StreamEvent::Progress(progress)));
            }
        }
    }

    /// Cancel and drain all queued reads and leave synchronous-FIFO mode.
    pub async fn finish(mut self) -> Result<()> {
        self.finish_inner().await
    }

    async fn finish_inner(&mut self) -> Result<()> {
        if self.finished {
            return Ok(());
        }

        cancel_and_drain(self.device.read_endpoint_mut(), self.timeout).await?;
        self.device.set_bitmode(0xFF, BitMode::Reset).await?;
        self.finished = true;
        Ok(())
    }
}

impl Drop for FtdiStream<'_> {
    fn drop(&mut self) {
        if !self.finished {
            self.device.mark_stream_abandoned();
            if self.device.read_endpoint_mut().pending() > 0 {
                self.device.read_endpoint_mut().cancel_all();
            }
        }
    }
}

impl FtdiDevice {
    /// Start a synchronous-FIFO streaming session.
    ///
    /// `packets_per_transfer` controls each USB buffer size and
    /// `num_transfers` controls queue depth. The configured read timeout is
    /// used for both reads and deterministic cleanup.
    pub async fn start_stream(
        &mut self,
        packets_per_transfer: usize,
        num_transfers: usize,
    ) -> Result<FtdiStream<'_>> {
        let chip = self.chip_type();
        if chip != ChipType::Ft2232H && chip != ChipType::Ft232H {
            return Err(Error::UnsupportedChip(chip));
        }
        if packets_per_transfer == 0 {
            return Err(Error::InvalidArgument(
                "packets_per_transfer must be greater than zero",
            ));
        }
        if num_transfers == 0 {
            return Err(Error::InvalidArgument(
                "num_transfers must be greater than zero",
            ));
        }

        let packet_size = self.read_endpoint_mut().max_packet_size();
        let buffer_size = packets_per_transfer
            .checked_mul(packet_size)
            .filter(|size| *size <= u32::MAX as usize)
            .ok_or(Error::InvalidArgument("stream buffer size overflow"))?;
        let timeout = self.read_timeout();
        let setup_guard = self.begin_stateful_operation()?;

        self.set_bitmode(0xFF, BitMode::Reset).await?;
        self.flush_all().await?;
        if self.read_endpoint_mut().pending() > 0 {
            cancel_and_drain(self.read_endpoint_mut(), timeout).await?;
        }
        // Queue reads before enabling synchronous FIFO so the host is ready for
        // data immediately when the mode switch takes effect.
        for _ in 0..num_transfers {
            let buffer = self.read_endpoint_mut().allocate(buffer_size);
            self.read_endpoint_mut().submit(buffer);
        }

        if let Err(error) = self.set_bitmode(0xFF, BitMode::SyncFf).await {
            let _ = cancel_and_drain(self.read_endpoint_mut(), timeout).await;
            return Err(error);
        }

        let start = Instant::now();
        setup_guard.disarm();
        Ok(FtdiStream {
            device: self,
            timeout,
            start,
            last_payload_time: start,
            previous_time: start,
            previous_bytes: 0,
            total_bytes: 0,
            progress_interval: Duration::from_secs(1),
            pending_progress: None,
            finished: false,
        })
    }

    /// Compatibility callback wrapper around [`Self::start_stream`].
    pub async fn read_stream<F>(
        &mut self,
        mut callback: F,
        packets_per_transfer: usize,
        num_transfers: usize,
    ) -> Result<()>
    where
        F: FnMut(&[u8], Option<&StreamProgress>) -> bool,
    {
        let mut stream = self
            .start_stream(packets_per_transfer, num_transfers)
            .await?;

        while let Some(event) = stream.next().await? {
            let keep_going = match event {
                StreamEvent::Data(data) => callback(&data, None),
                StreamEvent::Progress(progress) => callback(&[], Some(&progress)),
            };
            if !keep_going {
                return stream.finish().await;
            }
        }

        stream.finish().await
    }

    /// Async-callback wrapper for applications that need to await processing
    /// and apply backpressure between stream events.
    pub async fn read_stream_async<F, Fut>(
        &mut self,
        mut callback: F,
        packets_per_transfer: usize,
        num_transfers: usize,
    ) -> Result<()>
    where
        F: FnMut(StreamEvent) -> Fut,
        Fut: Future<Output = bool>,
    {
        let mut stream = self
            .start_stream(packets_per_transfer, num_transfers)
            .await?;

        while let Some(event) = stream.next().await? {
            if !callback(event).await {
                return stream.finish().await;
            }
        }

        stream.finish().await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strips_status_from_every_packet() {
        let raw = [
            0xaa, 0xbb, 1, 2, 3, 4, 5, 6, // packet one
            0xcc, 0xdd, 7, 8, 9, 10, 11, 12, // packet two
        ];
        assert_eq!(
            strip_modem_status(&raw, raw.len(), 8),
            (1..=12).collect::<Vec<_>>()
        );
    }

    #[test]
    fn ignores_status_only_packets() {
        assert!(strip_modem_status(&[0xaa, 0xbb], 2, 64).is_empty());
    }
}
