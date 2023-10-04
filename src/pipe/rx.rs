//! Interface to a NRF24L01(+) device's RX pipe.



use super::*;



pub struct RXDataPipe {
    /// Wrapped data pipe.
    pub(self) pipe: &'static mut DataPipe,
}

impl RXDataPipe {
    /// Static initializer.
    pub(crate) const fn new(pipe: &'static mut DataPipe) -> Self {
        Self { pipe, }
    }

    /// Waits for new data in the pipe.
    /// WARNING : This method may block indefinitely.
    pub async fn wait(&mut self) -> Result<(), Error> {
        Watcher::new( self.pipe, BufferState::Ready ).await
    }

    /// Returns an iterator over the data in the pipe if there is any.
    pub fn read(&mut self) -> Option<Reader> {
        // Return `None` if there is no data ready.
        if self.pipe.empty() {
            return None;
        }

        // Create the reader.
        Some( Reader { pipe: self, cursor: 1, } )
    }

    /// Flushes the data pipe without reading it.
    pub fn flush(&mut self) {
        if self.pipe.ready() {
            self.pipe.bufstate = BufferState::Empty;
        }
    }
}


/// Reader of an RX pipe buffer.
/// Must be consumed (or dropped) to flush the RX data pipe and allow new data in.
pub struct Reader<'a> {
    /// Reference to the pipe being read 
    pipe: &'a mut RXDataPipe,

    /// Current byte in the iterator.
    cursor: usize,
}

impl<'a> Iterator for Reader<'a> {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        // Already sent all bytes.
        if self.cursor > self.pipe.pipe.valid as usize {
            return None;
        }

        // Get the byte.
        let byte = self.pipe.pipe.buf[self.cursor];

        // Increase the counter.
        self.cursor += 1;

        // Return the byte.
        Some( byte )
    }
}

impl<'a> Drop for Reader<'a> {
    fn drop(&mut self) {
        self.pipe.pipe.bufstate = BufferState::Empty;
    }
}



/// Configuration of an RX data pipe.
#[derive(Clone, Debug)]
pub struct RXConfig {
    /// ACK required.
    pub(crate) ack: bool,

    /// Payload size.
    pub(crate) size: PayloadSize,
}

impl Default for RXConfig {
    fn default() -> Self {
        Self::modern()
    }
}

impl RXConfig {
    /// Creates a legacy configuration. Requires the payload size in bytes.
    /// This configuration is compatible with NRF2401, NRF24E1 and NRF24E2
    /// devices and with NRF24L01(+) devices set in legacy mode.
    pub const fn legacy(size: u8) -> Self {
        Self { ack: false, size: PayloadSize::Fixed(size), }
    }

    /// Creates the default modern configuration.
    /// The packets are dynamically sized and require acknowledge.
    pub const fn modern() -> Self {
        Self { ack: true, size: PayloadSize::Dynamic, }
    }

    /// Creates a free configuration. This configuration must be equal to other
    /// devices to be able to communicate with them.
    pub const fn free(ack: bool, size: PayloadSize) -> Self {
        Self { ack, size, }
    }
}
