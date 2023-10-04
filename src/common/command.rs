//! Commands accepted by NRF24L01(+) devices.



#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Command {
    /// Command to read a payload packet.
    ReadPayload = 0b01100001,

    /// Command to write a TX payload with ACK.
    WritePayload = 0b10100000,

    /// Read the RX payload width for the top payload in the FIFO.
    PayloadWidth = 0b01100000,

    /// Command to write a TX payload with no ACK.
    WritePayloadNoAck = 0b10110000,

    /// Flushes the TX FIFO.
    FlushTX = 0b11100001,

    /// Flushes the RX FIFO.
    FlushRX = 0b11100010,

    /// No operation.
    NOP = 0b11111111,
}

impl Into<u8> for Command {
    fn into(self) -> u8 {
        self as u8
    }
}
