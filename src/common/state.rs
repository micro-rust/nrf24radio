//! Internal state flags of the `RXDataPipe` and `TXDataPipe`.



/// State of the driver registration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DriverState {
    /// There is a driver registered.
    Registered,

    /// There is no driver registered.
    Orphaned,
}



/// State of the pipe.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PipeState {
    /// The pipe is open.
    Open,

    /// The pipe is closed.
    Closed,
}



/// State of the buffer and the data in it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum BufferState {
    /// The pipe is clear and can be written to.
    Empty,

    /// The pipe contains data.
    Ready,
}
