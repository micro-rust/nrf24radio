//! Registers of NRF24L01(+) devices.



#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    /// Configuration register.
    Config = 0x00,

    /// Enable auto-acknowledge register.
    EnableAutoACK = 0x01,

    /// Enabled RX addresses register.
    EnabledRXAddress = 0x02,

    /// Setup address width register.
    AddressWidth = 0x03,

    /// Setup retries register.
    SetupRetries = 0x04,

    /// RF Channel register.
    RFChannel = 0x05,

    /// RF Setup register.
    RFSetup = 0x06,

    /// Status register.
    Status = 0x07,

    /// Observe TX register.
    ObserveTX = 0x08,

    /// Receive Power Detector register.
    PowerDetector = 0x09,

    /// Pipe 0 RX Address register.
    RX0Address = 0x0A,

    /// Pipe 1 RX Address register.
    RX1Address = 0x0B,

    /// Pipe 2 RX Address register.
    RX2Address = 0x0C,

    /// Pipe 3 RX Address register.
    RX3Address = 0x0D,

    /// Pipe 4 RX Address register.
    RX4Address = 0x0E,

    /// Pipe 5 RX Address register.
    RX5Address = 0x0F,

    /// TX Address register.
    TXAddress = 0x10,

    /// Pipe 0 RX Payload Width register.
    RX0PayloadWidth = 0x11,

    /// Pipe 1 RX Payload Width register.
    RX1PayloadWidth = 0x12,

    /// Pipe 2 RX Payload Width register.
    RX2PayloadWidth = 0x13,

    /// Pipe 3 RX Payload Width register.
    RX3PayloadWidth = 0x14,

    /// Pipe 4 RX Payload Width register.
    RX4PayloadWidth = 0x15,

    /// Pipe 5 RX Payload Width register.
    RX5PayloadWidth = 0x16,

    /// FIFO Status register.
    FIFOStatus = 0x17,

    /// Dynamic Payload register.
    DynamicPayload = 0x1C,

    /// Features register.
    Features = 0x1D,
}

impl Into<u8> for Register {
    fn into(self) -> u8 {
        self as u8
    }
}

impl Into<u8> for &Register {
    fn into(self) -> u8 {
        *self as u8
    }
}
