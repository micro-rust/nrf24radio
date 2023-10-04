//! Buffers for NRF24L01(+) data pipes.



mod rx;
mod tx;
mod watcher;



pub use rx::*;
pub use tx::*;
pub use watcher::*;



pub(self) use super::common::*;
pub(self) use super::Error;



/// Common pipe buffer.
pub struct DataPipe {
    /// Driver presence synchronization flag.
    /// Indicates if this pipe is registered with a driver.
    /// SAFETY : This flag is only modified by the driver.
    pub(super) driver: DriverState,

    /// Data Pipe State flag.
    /// Indicates the state of the pipe.
    /// SAFETY :
    ///   - Driver : Closed to Open
    ///   - Pipe   : Open to Closed
    pub(super) pipestate: PipeState,

    /// Buffer synchronization flag.
    /// Indicates the state of the internal buffer.
    /// SAFETY :
    ///   - RX :
    ///     + Driver : Empty to Ready
    ///     + Pipe   : Ready to Empty
    ///   - TX :
    ///     + Driver : Ready to Empty
    ///     + Pipe   : Empty to Ready
    pub(super) bufstate: BufferState,

    /// Set when an error ocurred during the communication.
    pub(super) error: Option<super::Error>,

    /// Acknowledge flag.
    /// [TX] : Indicates if the transaction requires ACK response.
    /// [RX] : Unused.
    pub(super) ack: bool,

    /// Maximum packet size of the pipe. Only used in TX.
    pub(super) pldsize: PayloadSize,

    /// Number of packets in a TX stream. Only used in TX.
    pub(super) ntx: usize,

    /// The amount of valid bytes in the buffer.
    pub(super) valid: u8,

    /// Internal buffer that contains the data of the pipe.
    /// The buffer is able to contain the maximum packet size + command or status.
    pub(super) buf: [u8; 33],
}

impl DataPipe {
    /// Static initializer.
    /// Use this method to create statis instances of a data pipe.
    pub const fn new() -> Self {
        Self {
            driver: DriverState::Orphaned,
            pipestate: PipeState::Closed,
            bufstate: BufferState::Empty,
            error: None,
            ack: false,
            pldsize: PayloadSize::Dynamic,
            ntx: 0,
            valid: 0,
            buf: [0u8; 33],
        }
    }

    /// Returns `true` if the pipe is not registered with a driver.
    /// WARNING : This method does not guarantee synchronization. This is an internal method.
    pub fn orphan(&self) -> bool {
        self.driver == DriverState::Orphaned
    }

    /// Returns `true` if the pipe is registered with a driver.
    /// WARNING : This method does not guarantee synchronization. This is an internal method.
    pub fn registered(&self) -> bool {
        self.driver == DriverState::Registered
    }

    /// Returns `true` if the pipe buffer is empty.
    /// WARNING : This method does not guarantee synchronization. This is an internal method.
    pub fn empty(&self) -> bool {
        self.bufstate == BufferState::Empty
    }

    /// Returns `true` if the pipe buffer has valid data.
    /// WARNING : This method does not guarantee synchronization. This is an internal method.
    pub fn ready(&self) -> bool {
        self.bufstate == BufferState::Ready
    }

    /// Returns `true` if the pipe is closed.
    /// WARNING : This method does not guarantee synchronization. This is an internal method.
    pub fn closed(&self) -> bool {
        self.pipestate == PipeState::Closed
    }

    /// Returns `true` if the pipe is open.
    /// WARNING : This method does not guarantee synchronization. This is an internal method.
    pub fn open(&self) -> bool {
        self.pipestate == PipeState::Open
    }
}
