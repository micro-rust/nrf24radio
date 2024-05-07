//! Control policy of a radio.
//! Sets the behaviour of the independent control loop.



/// Behaviour of the control loop.
#[derive(Clone, Copy, Eq, PartialEq)]
pub enum RadioPolicy {
    /// Configures the radio to only receive data.
    Receive,

    /// Configures the radio to only send data.
    Transmit,

    /// Configures the radio to receive data until there is data to transmit.
    ReceiveUntilTransmit,

    /// Sets the radio in standby mode.
    Standby,

    /// Sets the radio in power down mode.
    PowerDown,
}



/// Behaviour of the control loop when it encounters RX errors.
#[derive(Clone, Copy, Eq, PartialEq)]
pub enum RXErrorPolicy {
    /// The radio will continue its operation on errors.
    Skip,

    /// The radio will go into standby on errors.
    Standby,

    /// The radio will power down on errors.
    PowerDown,
}



/// Behaviour of the control loop when it encounters TX errors.
#[derive(Clone, Copy, Eq, PartialEq)]
pub enum TXErrorPolicy {
    /// The radio will continue its operation on errors.
    Skip,

    /// The radio will continue its operation unless it reached max retries.
    SkipUnlessMaxRetries,

    /// The radio will go into standby on errors.
    Standby,

    /// The radio will power down on errors.
    PowerDown,
}
