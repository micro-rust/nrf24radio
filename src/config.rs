//! Configuration of NRF24L01(+) devices.



use super::common::*;



#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// CRC checksum configuration.
    pub(crate) crc: CRCBytes,

    /// Address Width (in bytes).
    pub(crate) aw: u8,

    /// RF channel of the device.
    pub(crate) ch: u8,

    /// RF data rate.
    pub(crate) dr: DataRate,

    /// RF gain.
    pub(crate) gain: Gain,

    /// Main channel address (Pipe 0 and TX Pipe).
    pub(crate) main: [u8; 5],

    /// Base channel address (Pipe 1-5).
    pub(crate) base: [u8; 4],

    /// Channel sub-addresses (Pipe 1-5).
    pub(crate) sub: [u8; 5],
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

impl Config {
    /// Creates a new configuration instance.
    pub const fn new() -> Self {
        Self {
            crc: CRCBytes::One,
            aw: 5,
            ch: 2,
            dr: DataRate::High,
            gain: Gain::Max,
            main: [0xE7; 5],
            base: [0xC2; 4],
            sub: [0xC2, 0xC3, 0xC4, 0xC5, 0xC6],
        }
    }

    /// Set the CRC configuration.
    pub const fn crc(mut self, crc: CRCBytes) -> Self {
        self.crc = crc;
        self
    }

    /// Set the addres width.
    pub const fn addresswidth(mut self, aw: u8) -> Self {
        self.aw = match aw {
            0..=3 => 3,
            4 => 4,
            _ => 5,
        };

        self
    }

    /// Set the RF channel.
    pub const fn channel(mut self, ch: u8) -> Self {
        self.ch = ch & 0x7F;
        self
    }

    /// Set the RF data rate.
    pub const fn datarate(mut self, dr: DataRate) -> Self {
        self.dr = dr;
        self
    }

    /// Set the RF gain.
    pub const fn gain(mut self, gain: Gain) -> Self {
        self.gain = gain;
        self
    }

    /// Sets the main address (Pipe 0).
    pub const fn main(mut self, main: [u8; 5]) -> Self {
        self.main = main;
        self
    }

    /// Sets the base address for the multi channel pipes (Pipe 1-5).
    pub const fn base(mut self, base: [u8; 4]) -> Self {
        self.base = base;
        self
    }

    /// Sets the subaddress for a multi channel pipe (Pipe 1-5).
    pub const fn subaddress(mut self, pipe: usize, addr: u8) -> Self {
        self.sub[if pipe > 3 { 4 } else { pipe }] = addr;
        self
    }
}
