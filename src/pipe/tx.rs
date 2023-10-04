//! Interface to a NRF24L01(+) device's RX pipe.



use core::{
    future::Future,

    pin::Pin,

    task::{
        Context, Poll,
    },
};

use super::*;



pub struct TXDataPipe {
    /// Wrapped data pipe.
    pub(self) pipe: &'static mut DataPipe,
}

impl TXDataPipe {
    /// Static initializer.
    pub(crate) const fn new(pipe: &'static mut DataPipe) -> Self {
        Self { pipe, }
    }

    /// Waits for the pipe to be empty.
    /// WARNING : This method may block indefinitely.
    pub async fn wait(&mut self) -> Result<(), Error> {
        Watcher::new( self.pipe, BufferState::Empty ).await
    }

    /// Sends a buffer to be transmitted.
    /// If the `ack` flag is set, the driver will expect an acknowledgement
    /// response from the receiver.
    pub async fn send(&mut self, data: &[u8], ack: bool) -> Result<(), Error> {
        // Check if the pipe is registered to a driver.
        if self.pipe.orphan() { return Err( Error::Orphaned ) }

        // Chunk the data into the payload size.
        let chunks = data.chunks( self.pipe.pldsize.into() );

        // Wait until the pipe is empty.
        self.wait().await?;

        // Set the amount of packets that will be sent.
        self.pipe.ntx = chunks.len();

        // Configure the ACK bit.
        self.pipe.ack = ack;

        // Start transmitting the chunks.
        for chunk in chunks {
            // Wait until the pipe is empty.
            self.wait().await?;

            // Write the chunk to the pipe buffer.
            self.write( chunk );

            // Set the ready flag.
            self.pipe.bufstate = BufferState::Ready;
        }

        // Wait for the pipe to complete all transactions.
        TXResult::new(self.pipe).await?;

        Ok( () )
    }

    /// Internal function to write a chunk to the pipe's buffer.
    fn write(&mut self, chunk: &[u8]) {
        // Write the data.
        self.pipe.buf[1..=chunk.len()].copy_from_slice( chunk );

        // Set the valid size to the size of the chunk (which comes prevalidated).
        self.pipe.valid = chunk.len() as u8;

        // If the payload is fixed size fill with 0s.
        if let PayloadSize::Fixed(s) = self.pipe.pldsize {
            if s < 32 {
                for byte in &mut self.pipe.buf[chunk.len()+1..] {
                    *byte = 0;
                }
            }
        }
    }
}

impl Drop for TXDataPipe {
    fn drop(&mut self) {
        // Close the pipe.
        self.pipe.pipestate = PipeState::Closed;
    }
}



/// Internal future to check for a TX pipe result.
struct TXResult<'a> {
    /// The pipe that is transmitting data.
    pipe: &'a DataPipe,
}

impl<'a> TXResult<'a> {
    /// Creates a new TX result for the given pipe.
    pub(self) fn new(pipe: &'a DataPipe) -> Self {
        Self { pipe, }
    }
}

impl<'a> Future for TXResult<'a> {
    type Output = Result<(), Error>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Check if the driver was dropped.
        if self.pipe.orphan() { return Poll::Ready( Err( Error::Orphaned ) ) }

        // Check if there was an error.
        if let Some( e ) = self.pipe.error { return Poll::Ready( Err( e ) ) }

        // Check if all transactions are done and the buffer is empty.
        if (self.pipe.ntx == 0) && self.pipe.empty() {
            return Poll::Ready( Ok( () ) );
        }

        // If neither happenned, pend the future.
        cx.waker().wake_by_ref();

        Poll::Pending
    }
}



/// Configuration of an RX data pipe.
#[derive(Clone, Debug)]
pub struct TXConfig {
    /// Maximum number of retries per packet sent.
    pub(crate) retries: u8,

    /// Delay between retries [(n+1) * 250 us].
    pub(crate) delay: u8,
}

impl Default for TXConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl TXConfig {
    /// Creates the default configuration.
    pub const fn new() -> Self {
        TXConfig {
            retries: 0b0011,
            delay: 0,
        }
    }

    /// Sets the maximum number of retries per packet sent.
    /// Set to 0 to disable retries.
    pub const fn retries(mut self, retries: u8) -> Self {
        self.retries = if retries > 14 { 15 } else { retries };
        self
    }

    /// Sets the delay between packet retransmission for failed packets.
    /// The delay is set in increments of 250 microseconds (delay = n * 250 us).
    /// If set to 0, the delay will be set to the minimum of 250 us.
    pub const fn delay(mut self, delay: u8) -> Self {
        self.delay = if delay == 0 { 0 } else if delay > 15 { 15 } else { delay - 1 };
        self
    }
}
