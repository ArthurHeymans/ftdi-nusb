//! Asynchronous (non-blocking) data transfer API.
//!
//! This module provides the ability to submit USB transfers without blocking,
//! then later wait for or cancel them. This is the Rust equivalent of
//! `ftdi_write_data_submit` / `ftdi_read_data_submit` / `ftdi_transfer_data_done`
//! / `ftdi_transfer_data_cancel` from libftdi.
//!
//! This module is only available with the `std` feature (native builds).
//!
//! # Usage
//!
//! ```no_run
//! use ftdi_nusb::FtdiDevice;
//!
//! let mut dev = FtdiDevice::open(0x0403, 0x6001)?;
//! dev.set_baudrate(115200)?;
//!
//! // Submit a non-blocking write
//! let tc = dev.write_data_submit(b"Hello!")?;
//!
//! // ... do other work ...
//!
//! // Wait for the transfer to complete
//! let bytes_written = tc.done()?;
//! # Ok::<(), ftdi_nusb::Error>(())
//! ```

use std::time::Duration;

use nusb::transfer::{Bulk, In, Out};

use crate::context::FtdiDevice;
use crate::error::{Error, Result};

/// Handle for an in-progress asynchronous write transfer.
///
/// Created by [`FtdiDevice::write_data_submit`]. Call [`done`](Self::done) to
/// block until the transfer completes, or [`cancel`](Self::cancel) to abort it.
///
/// If dropped without calling `done` or `cancel`, the transfer is cancelled.
pub struct WriteTransferControl {
    endpoint: nusb::Endpoint<Bulk, Out>,
    /// Total bytes submitted to USB so far (across all chunks).
    total_submitted: usize,
    /// Total bytes confirmed written by completed transfers.
    total_written: usize,
    /// The complete data to send.
    data: Vec<u8>,
    /// Write chunk size.
    chunksize: usize,
    /// Write timeout.
    timeout: Duration,
    /// Whether all chunks have been submitted.
    all_submitted: bool,
    /// Number of transfers currently in flight.
    in_flight: usize,
    /// Whether the transfer has completed or been cancelled.
    completed: bool,
}

/// Handle for an in-progress asynchronous read transfer.
///
/// Created by [`FtdiDevice::read_data_submit`]. Call [`done`](Self::done) to
/// block until data is available, or [`cancel`](Self::cancel) to abort it.
///
/// If dropped without calling `done` or `cancel`, the transfer is cancelled.
pub struct ReadTransferControl {
    endpoint: Option<nusb::Endpoint<Bulk, In>>,
    /// The buffer accumulating read data (modem status stripped).
    data: Vec<u8>,
    /// Requested read size.
    requested_size: usize,
    /// Max USB packet size (for modem status stripping).
    max_packet_size: usize,
    /// Read timeout.
    timeout: Duration,
    /// Read chunk size.
    chunksize: usize,
    /// Whether the transfer has completed.
    completed: bool,
    /// Whether a USB transfer is currently in flight.
    in_flight: bool,
}

impl WriteTransferControl {
    /// Block until the write transfer completes and return the number of bytes written.
    pub fn done(mut self) -> Result<usize> {
        self.drain()?;
        Ok(self.total_written)
    }

    /// Cancel the transfer. Any data already sent is not rolled back.
    pub fn cancel(mut self) {
        self.completed = true;
        // Drain any in-flight transfers
        while self.in_flight > 0 {
            if let Some(completion) = self.endpoint.wait_next_complete(Duration::from_millis(100)) {
                self.in_flight -= 1;
                if completion.status.is_ok() {
                    self.total_written += completion.actual_len;
                }
            } else {
                break;
            }
        }
    }

    /// Submit the next chunk if there is more data.
    fn submit_next_chunk(&mut self) {
        if self.all_submitted {
            return;
        }

        let offset = self.total_submitted;
        if offset >= self.data.len() {
            self.all_submitted = true;
            return;
        }

        let end = (offset + self.chunksize).min(self.data.len());
        let chunk = &self.data[offset..end];

        let mut buf = nusb::transfer::Buffer::new(chunk.len());
        buf.extend_from_slice(chunk);

        self.endpoint.submit(buf);
        self.total_submitted = end;
        self.in_flight += 1;

        if end >= self.data.len() {
            self.all_submitted = true;
        }
    }

    /// Wait for all in-flight transfers to complete, submitting more chunks as needed.
    fn drain(&mut self) -> Result<()> {
        // Submit the first chunk if we haven't already
        if self.in_flight == 0 && !self.all_submitted {
            self.submit_next_chunk();
        }

        while self.in_flight > 0 {
            let completion = self
                .endpoint
                .wait_next_complete(self.timeout)
                .ok_or(Error::Transfer(nusb::transfer::TransferError::Cancelled))?;

            self.in_flight -= 1;
            completion.status.map_err(Error::Transfer)?;
            self.total_written += completion.actual_len;

            // Submit next chunk if available
            if !self.all_submitted {
                self.submit_next_chunk();
            }
        }

        self.completed = true;
        Ok(())
    }
}

impl Drop for WriteTransferControl {
    fn drop(&mut self) {
        if !self.completed {
            self.completed = true;
            // Best-effort drain of in-flight transfers
            while self.in_flight > 0 {
                if self
                    .endpoint
                    .wait_next_complete(Duration::from_millis(100))
                    .is_some()
                {
                    self.in_flight -= 1;
                } else {
                    break;
                }
            }
        }
    }
}

impl ReadTransferControl {
    /// Block until the requested amount of data is read (or a short read occurs).
    ///
    /// Returns the data read with modem status bytes stripped. The returned
    /// buffer may be shorter than the requested size if the device had less
    /// data available.
    pub fn done(mut self) -> Result<Vec<u8>> {
        self.drain()?;
        Ok(std::mem::take(&mut self.data))
    }

    /// Cancel the read transfer and return whatever data has been collected so far.
    pub fn cancel(mut self) -> Vec<u8> {
        self.completed = true;
        // Drain any in-flight transfer
        if self.in_flight {
            if let Some(ref mut ep) = self.endpoint {
                if let Some(completion) = ep.wait_next_complete(Duration::from_millis(100)) {
                    if completion.status.is_ok() && completion.actual_len > 2 {
                        let raw = completion.buffer.into_vec();
                        let stripped = strip_modem_status_to_vec(
                            &raw[..completion.actual_len],
                            self.max_packet_size,
                        );
                        self.data.extend_from_slice(&stripped);
                    }
                }
            }
            self.in_flight = false;
        }
        std::mem::take(&mut self.data)
    }

    /// Submit a USB read transfer.
    fn submit_transfer(&mut self) {
        if let Some(ref mut ep) = self.endpoint {
            let buf = nusb::transfer::Buffer::new(self.chunksize);
            ep.submit(buf);
            self.in_flight = true;
        }
    }

    /// Wait for transfers, stripping modem status and accumulating data.
    fn drain(&mut self) -> Result<()> {
        // If already completed from readbuffer data, return immediately
        if self.completed {
            return Ok(());
        }

        // Submit initial transfer
        if !self.in_flight {
            self.submit_transfer();
        }

        loop {
            let ep = self
                .endpoint
                .as_mut()
                .ok_or(Error::InvalidArgument("no endpoint"))?;

            let completion = ep
                .wait_next_complete(self.timeout)
                .ok_or(Error::Transfer(nusb::transfer::TransferError::Cancelled))?;

            self.in_flight = false;
            completion.status.map_err(Error::Transfer)?;

            let actual_length = completion.actual_len;

            if actual_length > 2 {
                let raw = completion.buffer.into_vec();
                let stripped =
                    strip_modem_status_to_vec(&raw[..actual_length], self.max_packet_size);
                self.data.extend_from_slice(&stripped);
            }

            // Check if we've got enough data or got a short/empty read
            if self.data.len() >= self.requested_size || actual_length <= 2 {
                // Truncate to requested size
                self.data.truncate(self.requested_size);
                self.completed = true;
                return Ok(());
            }

            // Submit another transfer
            self.submit_transfer();
        }
    }
}

impl Drop for ReadTransferControl {
    fn drop(&mut self) {
        if !self.completed {
            self.completed = true;
            if self.in_flight {
                if let Some(ref mut ep) = self.endpoint {
                    let _ = ep.wait_next_complete(Duration::from_millis(100));
                }
                self.in_flight = false;
            }
        }
    }
}

/// Strip 2-byte modem status headers from each packet, returning a new Vec.
fn strip_modem_status_to_vec(data: &[u8], packet_size: usize) -> Vec<u8> {
    let total = data.len();
    if total <= 2 {
        return Vec::new();
    }

    let num_packets = total.div_ceil(packet_size);
    let mut result = Vec::with_capacity(total);

    for i in 0..num_packets {
        let pkt_start = i * packet_size;
        let pkt_end = (pkt_start + packet_size).min(total);
        let pkt_len = pkt_end - pkt_start;

        if pkt_len > 2 {
            result.extend_from_slice(&data[pkt_start + 2..pkt_end]);
        }
    }

    result
}

// ---- Methods on FtdiDevice ----

impl FtdiDevice {
    /// Submit a non-blocking write of `data` to the device.
    ///
    /// Returns a [`WriteTransferControl`] handle that can be used to wait for
    /// completion or cancel the transfer. The data is copied internally, so
    /// the caller does not need to keep the buffer alive.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use ftdi_nusb::FtdiDevice;
    /// # let mut dev = FtdiDevice::open(0x0403, 0x6001)?;
    /// let tc = dev.write_data_submit(b"MPSSE commands here")?;
    /// // ... do other work ...
    /// let n = tc.done()?;
    /// println!("Wrote {n} bytes");
    /// # Ok::<(), ftdi_nusb::Error>(())
    /// ```
    pub fn write_data_submit(&mut self, data: &[u8]) -> Result<WriteTransferControl> {
        let ep = self.bulk_out_endpoint()?;

        let mut tc = WriteTransferControl {
            endpoint: ep,
            total_submitted: 0,
            total_written: 0,
            data: data.to_vec(),
            chunksize: self.writebuffer_chunksize(),
            timeout: self.write_timeout(),
            all_submitted: data.is_empty(),
            in_flight: 0,
            completed: data.is_empty(),
        };

        // Submit the first chunk immediately
        if !data.is_empty() {
            tc.submit_next_chunk();
        }

        Ok(tc)
    }

    /// Submit a non-blocking read of up to `size` bytes from the device.
    ///
    /// If there is data already buffered from a previous read, it is returned
    /// immediately (the transfer completes synchronously in that case).
    ///
    /// Returns a [`ReadTransferControl`] handle. Call [`done`](ReadTransferControl::done)
    /// to block until data is available.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use ftdi_nusb::FtdiDevice;
    /// # let mut dev = FtdiDevice::open(0x0403, 0x6001)?;
    /// let tc = dev.read_data_submit(64)?;
    /// // ... do other work ...
    /// let data = tc.done()?;
    /// println!("Read {} bytes", data.len());
    /// # Ok::<(), ftdi_nusb::Error>(())
    /// ```
    pub fn read_data_submit(&mut self, size: usize) -> Result<ReadTransferControl> {
        let pkt_size = self.max_packet_size();
        let rd_timeout = self.read_timeout();
        let rd_chunksize = self.readbuffer_chunksize();

        if size == 0 {
            return Ok(ReadTransferControl {
                endpoint: None,
                data: Vec::new(),
                requested_size: 0,
                max_packet_size: pkt_size,
                timeout: rd_timeout,
                chunksize: rd_chunksize,
                completed: true,
                in_flight: false,
            });
        }

        // Check if we can satisfy from the internal read buffer
        let initial_data = self.drain_readbuffer(size);
        if initial_data.len() >= size {
            return Ok(ReadTransferControl {
                endpoint: None,
                data: initial_data,
                requested_size: size,
                max_packet_size: pkt_size,
                timeout: rd_timeout,
                chunksize: rd_chunksize,
                completed: true,
                in_flight: false,
            });
        }

        // Need a USB transfer for the rest
        let ep = self.bulk_in_endpoint()?;

        let mut tc = ReadTransferControl {
            endpoint: Some(ep),
            data: initial_data,
            requested_size: size,
            max_packet_size: pkt_size,
            timeout: rd_timeout,
            chunksize: rd_chunksize,
            completed: false,
            in_flight: false,
        };

        // Submit the first read immediately
        tc.submit_transfer();

        Ok(tc)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strip_modem_status_single_packet() {
        let mut data = vec![0u8; 64];
        data[0] = 0x01;
        data[1] = 0x60;
        for (i, byte) in data.iter_mut().enumerate().take(64).skip(2) {
            *byte = i as u8;
        }

        let result = strip_modem_status_to_vec(&data, 64);
        assert_eq!(result.len(), 62);
        for (i, byte) in result.iter().enumerate().take(62) {
            assert_eq!(*byte, (i + 2) as u8);
        }
    }

    #[test]
    fn strip_modem_status_multiple_packets() {
        let data = vec![
            0xAA, 0xBB, 2, 3, 4, 5, 6, 7, // packet 1
            0xCC, 0xDD, 10, 11, 12, 13, 14, 15, // packet 2
        ];

        let result = strip_modem_status_to_vec(&data, 8);
        assert_eq!(result.len(), 12);
        assert_eq!(result, vec![2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15]);
    }

    #[test]
    fn strip_modem_status_short() {
        let data = vec![0x01, 0x60];
        assert!(strip_modem_status_to_vec(&data, 64).is_empty());
    }

    #[test]
    fn strip_modem_status_empty() {
        let data: Vec<u8> = vec![];
        assert!(strip_modem_status_to_vec(&data, 64).is_empty());
    }
}
