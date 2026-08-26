//! High-performance streaming read API.
//!
//! The streaming API is designed for continuous, high-throughput data
//! acquisition from FTDI devices in synchronous FIFO mode. It is
//! only supported on FT2232H and FT232H chips.
//!
//! This module is only available on native targets.
//!
//! Unlike the regular [`read_data`](crate::FtdiDevice::read_data) method,
//! streaming uses multiple concurrent USB transfers to maximize throughput.

use std::time::{Duration, Instant};

use crate::context::FtdiDevice;
use crate::error::{Error, Result};
use crate::types::{BitMode, ChipType};

/// Progress information for a streaming read operation.
#[derive(Debug, Clone)]
pub struct StreamProgress {
    /// Total bytes transferred since the stream started.
    pub total_bytes: u64,
    /// Total elapsed time since the stream started.
    pub total_time: Duration,
    /// Overall average transfer rate in bytes/second.
    pub total_rate: f64,
    /// Transfer rate for the most recent reporting interval in bytes/second.
    pub current_rate: f64,
}

impl FtdiDevice {
    /// Continuously read data from the device using multiple concurrent transfers.
    ///
    /// This function sets up synchronous FIFO mode and submits multiple bulk
    /// transfers for maximum throughput. Data is delivered to the callback
    /// with the 2-byte modem status headers stripped from each packet.
    ///
    /// The callback receives:
    /// - `data`: The payload bytes (modem status stripped), or empty for progress updates.
    /// - `progress`: Progress information (provided periodically, about once per second).
    ///
    /// Return `false` from the callback to stop streaming.
    ///
    /// # Supported chips
    ///
    /// Only FT2232H and FT232H support synchronous FIFO mode.
    ///
    /// # Arguments
    ///
    /// * `callback` - Called for each completed transfer with payload data.
    /// * `packets_per_transfer` - Number of packets per USB transfer buffer.
    /// * `num_transfers` - Number of concurrent transfers to keep in flight.
    pub fn read_stream<F>(
        &mut self,
        mut callback: F,
        packets_per_transfer: usize,
        num_transfers: usize,
    ) -> Result<()>
    where
        F: FnMut(&[u8], Option<&StreamProgress>) -> bool,
    {
        let chip = self.chip_type();
        if chip != ChipType::Ft2232H && chip != ChipType::Ft232H {
            return Err(Error::UnsupportedChip(chip));
        }

        // Reset to known state
        self.set_bitmode(0xFF, BitMode::Reset)?;
        self.flush_all()?;

        let packet_size = self.max_packet_size();
        let buffer_size = packets_per_transfer * packet_size;

        // Open the bulk IN endpoint
        let mut ep = self.bulk_in_endpoint()?;

        // Submit initial transfers
        for _ in 0..num_transfers {
            let buf = nusb::transfer::Buffer::new(buffer_size);
            ep.submit(buf);
        }

        // Now switch to synchronous FIFO mode
        self.set_bitmode(0xFF, BitMode::SyncFf)?;

        let start = Instant::now();
        let mut total_bytes: u64 = 0;
        let mut prev_time = start;
        let mut prev_bytes: u64 = 0;
        let progress_interval = Duration::from_secs(1);

        loop {
            // Wait for a transfer to complete
            let Some(completion) = ep.wait_next_complete(Duration::from_secs(5)) else {
                break;
            };

            if completion.status.is_err() {
                break;
            }

            let actual_length = completion.actual_len;
            let raw = completion.buffer.into_vec();

            // Strip modem status and deliver payload
            if actual_length > 2 {
                let num_packets = actual_length.div_ceil(packet_size);

                for i in 0..num_packets {
                    let pkt_start = i * packet_size;
                    let pkt_end = (pkt_start + packet_size).min(actual_length);
                    let pkt_len = pkt_end - pkt_start;

                    if pkt_len > 2 {
                        let payload = &raw[pkt_start + 2..pkt_end];
                        total_bytes += payload.len() as u64;

                        if !callback(payload, None) {
                            // User requested stop
                            return Ok(());
                        }
                    }
                }
            }

            // Re-submit a transfer
            let buf = nusb::transfer::Buffer::new(buffer_size);
            ep.submit(buf);

            // Periodic progress update
            let now = Instant::now();
            if now.duration_since(prev_time) >= progress_interval {
                let total_elapsed = now.duration_since(start);
                let interval_elapsed = now.duration_since(prev_time);

                let progress = StreamProgress {
                    total_bytes,
                    total_time: total_elapsed,
                    total_rate: if total_elapsed.as_secs_f64() > 0.0 {
                        total_bytes as f64 / total_elapsed.as_secs_f64()
                    } else {
                        0.0
                    },
                    current_rate: if interval_elapsed.as_secs_f64() > 0.0 {
                        (total_bytes - prev_bytes) as f64 / interval_elapsed.as_secs_f64()
                    } else {
                        0.0
                    },
                };

                if !callback(&[], Some(&progress)) {
                    return Ok(());
                }

                prev_time = now;
                prev_bytes = total_bytes;
            }
        }

        Ok(())
    }
}
