//! Internal state of NRF24L01(+) device drivers.



#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum State {
    /// The device is powered down.
    /// This is the mode with the lowest consumption and highest startup time.
    PowerDown,

    /// Then device is powered up.
    /// In this mode the current consumption is increased relative to the
    /// powered down mode but startup time is reduced to 130 microseconds.
    Standby,

    /// The device is transmitting data or ready to transmit data.
    Transmitting,

    /// The device is receiving data or ready to receive data.
    Receiving,
}
