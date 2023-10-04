//! `enum`s used in NRF24L01(+) configuration structs.






/// CRC checksum configurations of the NRF24L01(+) device.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CRCBytes {
    /// No CRC bytes with each packet.
    None,

    /// One CRC bytes with each packet.
    One,

    /// Two CRC bytes with each packet.
    Two,
}



/// Payload size configurations of NRF24L01(+) devices' pipes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PayloadSize {
    /// Dynamic payload size.
    /// Must use modern compatibility mode.
    Dynamic,

    /// Fixed size payload
    Fixed( u8 ),
}

impl Into<usize> for PayloadSize {
    fn into(self) -> usize {
        match self {
            PayloadSize::Dynamic => 32,
            PayloadSize::Fixed(s) => s as usize,
        }
    }
}

impl Into<u8> for PayloadSize {
    fn into(self) -> u8 {
        match self {
            PayloadSize::Dynamic => 32,
            PayloadSize::Fixed(s) => s,
        }
    }
}



/// RF gain configurations of NRF24L01(+) devices.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Gain {
    /// Maximum gain [0 dBm].
    Max,

    /// High gain [-6 dBm].
    High,

    /// Mid gain [-12 dBm].
    Mid,

    /// Low gain [-18 dBm].
    Low,
}



/// RF data rate configurations of NRF24L01(+) devices.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DataRate {
    /// High data rate [2 Mbps].
    High,

    /// Mid data rate [1 Mbps].
    Mid,

    /// Low data rate [250 kbps].
    /// WARNING : This data rate is incompatible with NRF24L01 (non plus variant) devices.
    Low,
}



/// Compatibility configuration of the 
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Compatibility {
    /// Compatible with older NRF24 devices.
    Legacy,

    /// Compatible only with NRF24L01(+) devices.
    Modern,
}
