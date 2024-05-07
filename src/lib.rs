//! NRF24L01(+) no-std multi-level device driver for executor agnostic embedded `Rust`.
//! 
//! What do each of these mean:
//!   - NRF24L01(+) : Works with both the NRF24L01 and NRF24L01+ devices.
//!   - No std : The driver works in all embedded environments.
//!   - Multi-level : The driver exposes both a low level and a high level API.
//!   - Executor agnostic : The driver does not depend on a specific executor.



#![no_std]

#![feature(let_chains)]
#![feature(const_mut_refs)]
#![feature(inherent_associated_types)]



pub mod common;
pub mod pipe;



mod config;
mod error;
mod state;

#[cfg(feature = "control-loop")]
mod control;




pub use config::*;

pub use error::*;

pub use state::*;

#[cfg(feature = "control-loop")]
pub use control::*;



use common::*;

use embedded_hal::digital::OutputPin;

use embedded_hal_async::{
    digital::Wait,
    spi::SpiDevice,
};

#[cfg(feature = "control-loop")]
use embassy_sync::{
    blocking_mutex::raw::RawMutex,
    signal::Signal,
};

#[cfg(feature = "control-loop")]
use embassy_futures::select::{
    select, select3,
    
    Either, Either3,
};

#[cfg(feature = "control-loop")]
use pipe::Watcher;



/// Global garbage buffer.
/// This buffer is used by all drivers to dump garbage data or flush caches.
/// Data races don't matter because this is garbage data.
#[link_section = ".uninit.NRF24RADIO.scratch"]
#[used]
static mut SCRATCH: [u8; 33] = [0u8; 33];



/// TODO : Make pipes optional to allow for lower memory footprint.
/// TODO : Improve (optional) logging in all methods.
/// TODO : Enforce correct pipe configurations (TX == RX0) (TX == DYN & AA).
pub struct Driver<SPI, CE, IRQ> {
    /// The SPI interface to the NRF24 device.
    spi: SPI,

    /// The CE pin output to the NRF24 device.
    ce: CE,

    /// Optional IRQ pin to wait for device events.
    /// This is set optional to facilitate using the driver in low pin count
    /// devices, altoug the performance and power consumption of the driver
    /// will worsen if this pin is not provided.
    irq: Option<IRQ>,

    /// Current state of the device.
    state: State,

    /// Current configuration of the device.
    #[allow(dead_code)]
    config: Config,

    /// References to the RX pipes' buffers.
    /// The buffers must be static to be able to use them from different tasks.
    /// Synchronization is handled internally by the driver.
    rxpipes: [&'static mut pipe::DataPipe; 6],

    /// Reference to the TX pipe buffer.
    /// The buffers must be static to be able to use them from different tasks.
    /// Synchronization is handled internally by the driver.
    txpipe: &'static mut pipe::DataPipe,
}

/// Common high level method for general use of NRF24L01(+) devices.
/// Creating / Configuring the driver and control loop.
impl<SPI: SpiDevice, CE: OutputPin, IRQ: Wait> Driver<SPI, CE, IRQ> {
    /// Hardware error type emitted by this driver.
    pub type HWError = HardwareError<SPI::Error, CE::Error, IRQ::Error>;

    /// Common error type emitted by this driver.
    pub type Error = (Error, Option<HardwareError<SPI::Error, CE::Error, IRQ::Error>>);

    /// Creates a new NRF24L01(+) driver.
    pub async fn create(mut spi: SPI, mut ce: CE, irq: Option<IRQ>, rxpipes: [&'static mut pipe::DataPipe; 6], txpipe: &'static mut pipe::DataPipe) -> Result<Self, Self::Error> {
        // Disable CE.
        if let Err( hwe ) = ce.set_low() {
            return Err( (Error::FailedDriverCreation, Some( HardwareError::ChipEnable(hwe) )) );
        }

        // Clear PWR UP bit.
        if let Err( hwe ) = spi.write( &[(1 << 5) | Register::Config as u8, 0b00001000] ).await {
            return Err( (Error::FailedDriverCreation, Some( HardwareError::Serial(hwe) )) );
        }

        // Flush RX FIFO.
        if let Err( hwe ) = spi.write( &[Command::FlushRX as u8] ).await {
            return Err( (Error::FailedDriverCreation, Some( HardwareError::Serial(hwe) )) );
        }

        // Flush TX FIFO.
        if let Err( hwe ) = spi.write( &[Command::FlushTX as u8] ).await {
            return Err( (Error::FailedDriverCreation, Some( HardwareError::Serial(hwe) )) );
        }

        Ok( Self {
            spi,
            ce,
            irq,
            state: State::PowerDown,
            config: Config::new(),
            rxpipes,
            txpipe,
        })
    }

    /// Configures the nRF24L01(+) device driver.
    /// If `validate` is set, the register writes will be validated.
    pub async fn configure(&mut self, config: Config) -> Result<(), Self::Error> {
        // Set to at least standby.
        if (self.state == State::Transmitting) || (self.state == State::Receiving) {
            self.standby().await?;
        }

        // Enable all features (can go unused).
        if let Err( hwe ) = self.writereg(Register::Features, 0b111).await {
            return Err( (Error::FailedDriverConfiguration, Some(hwe)) )
        }

        // Create a buffer for address transmission.
        let mut buf;

        // Write the main address (Pipe 0).
        buf = [(1 << 5) | Register::RX0Address as u8, config.main[4], config.main[3], config.main[2], config.main[1], config.main[0]];

        if let Err( hwe ) = self.writebuf( &buf ).await {
            return Err( (Error::FailedDriverConfiguration, Some(hwe)) )
        }

        // Write the main address (TX Pipe).
        buf[0] = (1 << 5) | Register::TXAddress as u8;

        if let Err( hwe ) = self.writebuf( &buf ).await {
            return Err( (Error::FailedDriverConfiguration, Some(hwe)) )
        }

        // Write the base address with Pipe 1.
        buf = [(1 << 5) | Register::RX1Address as u8, config.sub[0], config.base[3], config.base[2], config.base[1], config.base[0]];

        if let Err( hwe ) = self.writebuf( &buf ).await {
            return Err( (Error::FailedDriverConfiguration, Some(hwe)) )
        }

        // Write all the sub-addresses.
        let regs = [Register::RX2Address, Register::RX3Address, Register::RX4Address, Register::RX5Address];
        for (r, v) in regs.iter().zip( &config.sub[1..5] ) {
            if let Err( hwe ) = self.writereg(r, *v).await {
                return Err( (Error::FailedDriverConfiguration, Some(hwe)) )
            }
        }

        // Configure the RF channel.
        if let Err( hwe ) = self.writereg( Register::RFChannel, config.ch).await {
            return Err( (Error::FailedDriverConfiguration, Some(hwe)) )
        }

        // Configure the RF power and data rate.
        let mut rfsetup = 0;

        match config.dr {
            DataRate::High => rfsetup |= 1 << 3,
            DataRate::Low  => rfsetup |= 1 << 5,
            _ => (),
        }

        match config.gain {
            Gain::Max  => rfsetup |= 0b110,
            Gain::High => rfsetup |= 0b100,
            Gain::Mid  => rfsetup |= 0b010,
            _ => (),
        }
        
        if let Err( hwe ) = self.writereg( Register::RFSetup, rfsetup).await {
            return Err( (Error::FailedDriverConfiguration, Some(hwe)) )
        }

        // Configure the address width.
        let aw = match config.aw {
            0..=3 => 0b01,
            4     => 0b10,
            _     => 0b11,
        };

        if let Err( hwe ) = self.writereg( Register::AddressWidth, aw).await {
            return Err( (Error::FailedDriverConfiguration, Some(hwe)) )
        }

        // Configure the CRC bytes.
        let cfg = match config.crc {
            CRCBytes::Two => 0b1100,
            CRCBytes::One => 0b1000,
            _             => 0b0000,
        };

        if let Err( hwe ) = self.writereg( Register::Config, cfg).await {
            return Err( (Error::FailedDriverConfiguration, Some(hwe)) )
        }

        #[cfg(feature = "log")]
        defmt::debug!("Finished driver configuration");

        Ok( () )
    }
}

#[cfg(feature = "control-loop")]
impl<SPI: SpiDevice, CE: OutputPin, IRQ: Wait> Driver<SPI, CE, IRQ> {
    pub async fn run<M: RawMutex>(&mut self, policy: &'static Signal<M, RadioPolicy>, rxfail: RXErrorPolicy, txfail: TXErrorPolicy) {
        // Internal enum to validate actions.
        // This is created to reduce the state machine size.
        #[derive(Clone, Copy, Eq, PartialEq)]
        enum Action { Recv, Send, Change( RadioPolicy ), None, }

        // Current radio behaviour.
        let mut current = RadioPolicy::PowerDown;

        loop {
            // Update the policy if necessary.
            if policy.signaled() {
                current = policy.wait().await;
            }

            // Check for the passive modes.
            if (current == RadioPolicy::PowerDown) || (current == RadioPolicy::Standby) {
                // Set power down mode if necessary.
                if current == RadioPolicy::PowerDown {
                    let _ = self.powerdown().await;
                }

                // Set standby mode if necessary.
                if current == RadioPolicy::Standby {
                    let _ = self.standby().await;
                }

                // Wait until the policy changes.
                current = policy.wait().await;
            }

            // Set the RX mode if necessary.
            if (current == RadioPolicy::Receive) || (current == RadioPolicy::ReceiveUntilTransmit) {
                let _ = self.rxmode().await;
                //defmt::warn!("Ready to check for data");
            }

            // Check the active modes.
            let action = match current {
                // Handle the receive policy.
                RadioPolicy::Receive => match select( policy.wait(), status::<_, CE, _>(&mut self.irq, &mut self.spi) ).await {
                    // Radio policy changed.
                    Either::First(new) => Action::Change( new ),

                    // There is data available to receive.
                    Either::Second(result) => match result {
                        Err(error) => {
                            // Handle the error.
                            current = self.rxerror(error, current, rxfail);

                            // Do nothing.
                            Action::None
                        },

                        Ok(status) => {
                            //defmt::warn!("Status of RX driver: {:b}", status);

                            match (status >> 6) & 1 {
                                0 => Action::None,
                                _ => Action::Recv,
                            }
                        },
                    },
                },

                // Handle the receive until transmit policy.
                RadioPolicy::ReceiveUntilTransmit => match select3( policy.wait(), status::<_, CE, _>(&mut self.irq, &mut self.spi), Watcher::new( self.txpipe, BufferState::Ready ) ).await {
                    // Radio policy changed.
                    Either3::First(new) => Action::Change( new ),

                    // There is data available to receive.
                    Either3::Second(_) => Action::Recv,

                    // There is data available to send.
                    Either3::Third(_) => Action::Send,
                },

                // Handle the transmit policy.
                RadioPolicy::Transmit => match select( policy.wait(), Watcher::new( self.txpipe, BufferState::Ready ) ).await {
                    // Radio policy changed.
                    Either::First(new) => Action::Change( new ),

                    // There is data available to send.
                    Either::Second(_) => Action::Send,
                },

                _ => Action::None,
            };

            // Check which action to perform.
            match action {
                Action::Change( new ) => current = new,

                Action::Recv => {
                    // Ensure we are in TX mode.
                    let _ = self.rxmode().await;

                    // Send the data.
                    match self.rxdata(true).await {
                        Err(error) => {
                            // Handle the error.
                            current = self.rxerror(error, current, rxfail);
                        },

                        Ok(Some((pipe, bytes, more))) => defmt::debug!("RX Driver: Received {} bytes on pipe {} [more = {}]", bytes, pipe, more),

                        Ok(None) => defmt::debug!("RX Driver: Expected data but no data was available"),
                    }
                },

                // Send a packet.
                Action::Send => {
                    // Ensure we are in TX mode.
                    let _ = self.txmode().await;

                    // Send the data.
                    if let Err(error) = self.txdata(true).await {
                        // Handle the error.
                        current = self.txerror(error, current, txfail);
                    }
                },

                Action::None => (),
            }
        }

        // Yield to allow other tasks to run.
        embassy_futures::yield_now().await;
    }

    /// Handles the RX errors.
    fn rxerror(&mut self, (error, hw): Self::Error, policy: RadioPolicy, rxfail: RXErrorPolicy) -> RadioPolicy {
        // Log the error.
        #[cfg(feature = "log")]
        match hw {
            Some( hw ) => defmt::error!("RX Driver Error: {} [{}]", error, hw),
            None => defmt::error!("RX Driver Error: {}", error),
        }

        // Check behaviour after error.
        match rxfail {
            // Power down the radio.
            RXErrorPolicy::PowerDown => RadioPolicy::PowerDown,

            // Standby the radio.
            RXErrorPolicy::Standby => RadioPolicy::Standby,

            _ => policy,
        }
    }

    /// Handles the RX errors.
    fn txerror(&mut self, (error, hw): Self::Error, policy: RadioPolicy, txfail: TXErrorPolicy) -> RadioPolicy {
        // Log the error.
        /*
        #[cfg(feature = "log")]
        match hw {
            Some( hw ) => defmt::error!("TX Driver Error: {} [{}]", error, hw),
            None => defmt::error!("TX Driver Error: {}", error),
        }
        */

        // Check behaviour after error.
        match txfail {
            // Power down the radio.
            TXErrorPolicy::PowerDown => RadioPolicy::PowerDown,

            // Standby the radio.
            TXErrorPolicy::Standby => RadioPolicy::Standby,

            // Check if it's a max retries error.
            TXErrorPolicy::SkipUnlessMaxRetries if error == Error::MaxRetries => {
                RadioPolicy::Standby
            },

            _ => policy,
        }
    }

    #[cfg(feature = "log")]
    pub async fn report(&mut self) {
        // Read the registers.
        let regs = [
            self.readreg(0x00).await.unwrap_or(0xFF),
            self.readreg(0x01).await.unwrap_or(0xFF),
            self.readreg(0x02).await.unwrap_or(0xFF),
            self.readreg(0x03).await.unwrap_or(0xFF),
            self.readreg(0x04).await.unwrap_or(0xFF),
            self.readreg(0x05).await.unwrap_or(0xFF),
            self.readreg(0x06).await.unwrap_or(0xFF),
            self.readreg(0x07).await.unwrap_or(0xFF),
            self.readreg(0x08).await.unwrap_or(0xFF),
            self.readreg(0x09).await.unwrap_or(0xFF),

            self.readreg(0x11).await.unwrap_or(0xFF),
            self.readreg(0x12).await.unwrap_or(0xFF),
            self.readreg(0x13).await.unwrap_or(0xFF),
            self.readreg(0x14).await.unwrap_or(0xFF),
            self.readreg(0x15).await.unwrap_or(0xFF),
            self.readreg(0x16).await.unwrap_or(0xFF),

            self.readreg(0x17).await.unwrap_or(0xFF),
            self.readreg(0x1C).await.unwrap_or(0xFF),
            self.readreg(0x1D).await.unwrap_or(0xFF),
        ];

        let names = [
            "Config",
            "Enable Auto Acknowledge",
            "Enable RX Address",
            "Setup Address Width",
            "Setup Retries",
            "RF Channel",
            "RF Setup",
            "Status",
            "Observe TX",
            "Power Detect",

            "Bytes in Pipe 0",
            "Bytes in Pipe 1",
            "Bytes in Pipe 2",
            "Bytes in Pipe 3",
            "Bytes in Pipe 4",
            "Bytes in Pipe 5",

            "FIFO Status",
            "Dynamic Payload",
            "Features",
        ];

        for (r, n) in regs.iter().zip(names.iter()) {
            defmt::info!("Register {}: {:b}", n, r)
        }
    }
}



/// High level methods to use the RX interface of NRF24L01(+) devices.
/// Opening / Handling the RX pipes.
impl<SPI: SpiDevice, CE: OutputPin, IRQ: Wait> Driver<SPI, CE, IRQ> {
    /// Open the RX pipe with the given configuration.
    /// WARNING : RX pipe 0 cannot be set as fixed size pipe.
    pub async fn openrx(&mut self, n: u8, mut config: pipe::RXConfig) -> Result<pipe::RXDataPipe, Self::Error> {
        use pipe::*;

        // Validate the pipe number.
        if n > 5 { return Err( ( Error::IllegalPipeNumber(n), None ) ) }

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Pipe number validated");

        // Check if the pipe is already open.
        if self.rxpipes[n as usize].open() { return Err( ( Error::PipeNotAvailable, None ) ) }

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Pipe {} is open", n);

        // If configuring pipe 0, reset to default.
        if n == 0 { config = Default::default() }

        // Read the DYNPD register.
        let mut dynpd = match self.readreg( Register::DynamicPayload ).await {
            Err(hwe) => return Err( ( Error::RXPipeCreationFailed(n), Some(hwe) ) ),
            Ok(d) => d,
        };

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Read DYNPD register");

        match config.size {
            // Set fixed size payload.
            PayloadSize::Fixed( size ) => {
                // Set the payload size.
                if let Err(hwe) = self.writereg(0x11 + (n as u8), size).await {
                    return Err( ( Error::RXPipeCreationFailed(n), Some(hwe) ) )
                }

                // Clear dynamic payload.
                dynpd &= !(1 << n);
            },

            // Set dynamic payload.
            PayloadSize::Dynamic => dynpd |= 1 << n,
        }

        // Write the dynamic payload register.
        if let Err(hwe) = self.writereg(Register::DynamicPayload, dynpd).await {
            return Err( ( Error::RXPipeCreationFailed(n), Some(hwe) ) )
        }

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Configured DYNPD register");

        // Read the AUTO ACK register.
        let mut autoack = match self.readreg(Register::EnableAutoACK).await {
            Err(hwe) => return Err( ( Error::RXPipeCreationFailed(n), Some(hwe) ) ),
            Ok(a) => a,
        };

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Read auto-ACK register");

        // Check if auto-ACK is enabled.
        match config.ack {
            false => autoack &= !(1 << n),
            true  => autoack |=   1 << n ,
        }

        // Write the auto-ack register.
        if let Err(hwe) = self.writereg(Register::EnableAutoACK, autoack).await {
            return Err( ( Error::RXPipeCreationFailed(n), Some(hwe) ) )
        }

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Configured auto-ACK register");

        // Read the EN RX register.
        let enable = match self.readreg(Register::EnabledRXAddress).await {
            Err(hwe) => return Err( ( Error::RXPipeCreationFailed(n), Some(hwe) ) ),
            Ok(a) => a,
        };

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Read EN RX register");

        // Write the EN RX register.
        if let Err(hwe) = self.writereg(Register::EnableAutoACK, enable | (1 << n)).await {
            return Err( ( Error::RXPipeCreationFailed(n), Some(hwe) ) )
        }

        // Set the driver flag on the pipe and open the pipe.
        self.rxpipes[n as usize].driver = DriverState::Registered;
        self.rxpipes[n as usize].pipestate = PipeState::Open;
        self.rxpipes[n as usize].bufstate = BufferState::Empty;
        self.rxpipes[n as usize].error = None;

        #[cfg(feature = "log")]
        defmt::debug!("RX Driver : RX Data pipe configured");

        Ok( RXDataPipe::new( unsafe { &mut *(self.rxpipes[n as usize] as *mut _) } ) )
    }

    /// Attempts to send a payload. Returns the number of bytes sent in total.
    /// This method may wait indefinitely if the TX pipe is dropped / crashes
    /// or the SPI hardware fails.
    pub async fn rxdata(&mut self, wait: bool) -> Result<Option<(u8, usize, bool)>, Self::Error> {
        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Setting RX Mode");

        // Assert the RX mode. Would be NOOP if already in TX mode.
        self.rxmode().await?;

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Reached RX Mode");

        // Read the status register to check if the RX DR IRQ is set.
        let mut status = self.status(wait).await?;

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Status {:b}", status);

        // If the IRQ is not set return early.
        if (status & (1 << 6)) == 0 { return Ok( None ) }

        // Read the payload width.
        let width = match self.pldwidth().await {
            Err(hwe) => return Err( ( Error::UnknownPayloadWidth, Some(hwe) ) ),
            Ok(w) => w,
        };

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Payload width {}", width);

        // Get the pipe number.
        let mut npipe = (status >> 1) & 0b111;

        // If pipe number is bigger than 5 recheck the number.
        if npipe > 5 {
            #[cfg(feature = "log")]
            defmt::warn!("RX Driver : Illegal pipe number ({}). Reading again...", npipe);

            // Reread status and validate.
            status = self.status(false).await?;

            // Get pipe number again.
            npipe = (status >> 1) & 0b111;

            // If the pipe number is wrong again abort.
            if npipe > 5 {
                #[cfg(feature = "log")]
                defmt::error!("RX Driver : Illegal pipe number ({})", npipe);

                // Clear the IRQ.
                let _ = self.clearirqs().await;

                return Ok( None )
            }
        }

        // Validate payload width.
        if width > 32 { return Err( ( Error::RXOverflow, None ) ) }

        #[cfg(feature = "log")]
        defmt::debug!("RX Driver : Pipe {} has {} bytes", npipe, width);

        // Create the write buffer.
        let mut write = [0; 33];
        write[0] = Command::ReadPayload as u8;

        // Check the state of the pipe.
        let read = match self.rxpipes[npipe as usize].pipestate {
            PipeState::Open => match self.rxpipes[npipe as usize].bufstate {
                BufferState::Empty => &mut self.rxpipes[npipe as usize].buf[0..=width as usize],
                BufferState::Ready => unsafe { &mut SCRATCH[0..=width as usize] },
            },

            PipeState::Closed => unsafe { &mut SCRATCH[0..=width as usize] },
        };

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Submitting Read Payload command...");

        // Read the data into the correct buffer.
        if let Err( hwe ) = self.spi.transfer(read, &write[0..=width as usize]).await {
            return Err( ( Error::FailedPacketDownload, Some( HardwareError::Serial(hwe) ) ) )
        }

        #[cfg(feature = "log")]
        defmt::debug!("RX Driver : Read Payload command completed");

        // Clear the IRQs.
        let _ = self.clearirqs().await;

        // Set the buffer as ready.
        self.rxpipes[npipe as usize].valid = width as u8;
        self.rxpipes[npipe as usize].bufstate = BufferState::Ready;

        // If the pipe was closed, disable the pipe.
        if self.rxpipes[npipe as usize].closed() {
            #[cfg(feature = "log")]
            defmt::warn!("RX Driver : Target pipe ({}) is closed. Disabling pipe on device", npipe);

            // Read the enable register.
            let enable = match self.readreg(Register::EnabledRXAddress).await {
                Err(_) => return Ok( None ),
                Ok(e) => e,
            };

            #[cfg(feature = "log")]
            defmt::trace!("RX Driver : Read RX enable register");

            // Write the modified register.
            let _ = self.writereg(Register::EnabledRXAddress, enable & !(1 << npipe)).await;

            #[cfg(feature = "log")]
            defmt::trace!("RX Driver : Updated RX enable register");

            // Return `None` because no open pipe was modified.
            return Ok( None )
        }

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : Checking for more data in FIFO");

        // Check if there is more data in the FIFO.
        let more = match self.statuses(false).await {
            Ok([_, fifo]) => (fifo & 1) == 0,
            Err(_) => false,
        };

        #[cfg(feature = "log")]
        defmt::trace!("RX Driver : TX FIFO -> {}", more);

        Ok( Some( (npipe, width as usize, more) ) )
    }
}


/// High level methods to use the TX interface of NRF24L01(+) devices.
/// Opening / Handling the TX pipe.
impl<SPI: SpiDevice, CE: OutputPin, IRQ: Wait> Driver<SPI, CE, IRQ> {
    /// Opens the TX pipe with the given configuration.
    pub async fn opentx(&mut self, config: pipe::TXConfig) -> Result<pipe::TXDataPipe, Self::Error> {
        use pipe::*;

        // Check if the pipe is already open or registered.
        if self.txpipe.open() { return Err( ( Error::PipeCurrentlyUsed, None ) ) }

        // Create the SETUP RETR register.
        let setup = (config.delay << 4) | (config.retries & 0xF);

        #[cfg(feature = "log")]
        defmt::trace!("TX Driver : Writing setup register ({:b})", setup);

        // Write the SETUP RETR register.
        if let Err( hwe ) = self.writereg(Register::SetupRetries, setup).await {
            return Err( ( Error::TXPipeCreationFailed, Some(hwe) ) );
        }

        #[cfg(feature = "log")]
        defmt::debug!("TX Driver : Wrote setup register ({:b})", setup);

        #[cfg(feature = "log")]
        defmt::trace!("TX Driver : Reading DYNPD register");

        // Read the DYNPD register.
        let dynpd = match self.readreg(Register::DynamicPayload).await {
            Err(hwe) => return Err( ( Error::TXPipeCreationFailed, Some(hwe) ) ),
            Ok(d) => d,
        };

        #[cfg(feature = "log")]
        defmt::trace!("TX Driver : Read DYNPD register ({:b})", dynpd);

        // Write the DYNPD register.
        if let Err( hwe ) = self.writereg(Register::DynamicPayload, dynpd | 1).await {
            return Err( ( Error::TXPipeCreationFailed, Some(hwe) ) );
        }

        #[cfg(feature = "log")]
        defmt::debug!("TX Driver : Wrote DYNPD register ({:b})", dynpd | 1);

        // Set the flags to open the pipe.
        self.txpipe.driver = DriverState::Registered;
        self.txpipe.pipestate = PipeState::Open;
        self.txpipe.bufstate = BufferState::Empty;
        self.txpipe.error = None;

        #[cfg(feature = "log")]
        defmt::debug!("TX Driver : TX Pipe is open");

        // Create the TX pipe.
        Ok( TXDataPipe::new( unsafe { &mut *(self.txpipe as *mut _) } ) )
    }

    /// Attempts to send a payload. Returns the number of bytes sent in total.
    /// This method may wait indefinitely if the TX pipe is dropped / crashes
    /// or the SPI hardware fails.
    pub async fn txdata(&mut self, wait: bool) -> Result<Option<usize>, Self::Error> {
        use pipe::*;

        // Check that the TX pipe is open.
        if self.txpipe.closed() { return Err( ( Error::PipeClosed, None ) ) }

        #[cfg(feature = "log")]
        defmt::trace!("TX Driver : Checking for new data");

        // Check for data in the TX pipe.
        match self.txpipe.bufstate {
            // Wait for data to send.
            BufferState::Empty if wait => if let Err( e ) = Watcher::new( &self.txpipe, BufferState::Ready ).await {
                return Err( ( e, None ) )
            },

            // There is no data to send.
            BufferState::Empty => return Ok( None ),

            // There is data to send.
            BufferState::Ready => (),
        }

        #[cfg(feature = "log")]
        defmt::trace!("TX Driver : Found data in the pipe");

        // Set the TX command.
        self.txpipe.buf[0] = if self.txpipe.ack { Command::WritePayload as u8 } else { Command::WritePayloadNoAck as u8 };

        // Assert the TX mode. Would be NOOP if already in TX mode.
        if let Err( ( error, hwe ) ) = self.txmode().await {
            // Write the error to the pipe.
            self.txpipe.error = Some( error );

            // Set the pipe as empty to release it.
            self.txpipe.bufstate = BufferState::Empty;

            // Report the error.
            return Err( ( error, hwe ) );
        }

        // Flag to mark the first transaction.
        let mut first = true;

        // Byte counter.
        let mut bytes = 0;

        // Send all packets in the transaction, capture any errors.
        let (error, hwe) = loop {
            // Check if the pipe was dropped.
            if self.txpipe.closed() { break (Error::PipeClosed, None) }

            // Read the status registers.
            if let Ok( [status, fifo] ) = self.statuses(!first).await {
                #[cfg(feature = "log")]
                defmt::debug!("TX Driver : Status while uploading {:b}", status);

                // Check for a retry error.
                if (status & (1 << 4)) != 0 { break (Error::MaxRetries, None) }

                // Check if there are no more packets and the FIFO is empty.
                if (self.txpipe.ntx == 0) && ((fifo & (1 << 4)) != 0) {
                    // Set the empty flag to allow for the next packet.
                    self.txpipe.bufstate = BufferState::Empty;

                    // End the transaction.
                    return Ok( Some(bytes) );
                }

                // Check if there is space in the FIFO and there are packets left.
                if ((status & 1) == 0) && (self.txpipe.ntx > 0) && (self.txpipe.ready()) {
                    // Only clear the TX DS IRQ when sending new data to avoid deadlocks.
                    while let Err(_) = self.clearirq(false, true, false).await {}

                    // Push the packet to the device.
                    if let Err(e) = self.spi.write( &self.txpipe.buf[0..=self.txpipe.valid as usize] ).await {
                        break ( Error::FailedPacketUpload, Some( HardwareError::Serial( e ) ) )
                    }

                    // Increment the byte counter.
                    bytes += self.txpipe.valid as usize;

                    // Decrement the counter of transactions.
                    self.txpipe.ntx -= 1;

                    // Sync to avoid the compiler reordering this read.
                    core::sync::atomic::compiler_fence( core::sync::atomic::Ordering::SeqCst );

                    // Set the empty flag to allow for the next packet.
                    if self.txpipe.ntx > 0 { self.txpipe.bufstate = BufferState::Empty }

                    // Clear the first packet flag.
                    first = false;
                }
            }
        };

        // Cancel the transaction.
        self.txcancel( Some(error) ).await;

        Err( ( error, hwe ) )
    }

    /// Cancels an ongoing transaction and clears the TX pipe.
    /// This method may wait indefinitely if the SPI hardware fails.
    pub async fn txcancel(&mut self, error: Option<Error>) {
        // Flush the TX FIFO.
        while let Err(_) = self.command( Command::FlushTX ).await {}

        // Clear all IRQs.
        while let Err(_) = self.clearirqs().await {}

        // Set the error in the pipe.
        self.txpipe.error = error;

        // Set the empty flag to allow for the next packet.
        self.txpipe.bufstate = BufferState::Empty;
    }
}


/// Intermediate methods for interacting with NRF24L01(+) devices.
/// Reading specific registers, complex commands and routine interactions.
impl<SPI: SpiDevice, CE: OutputPin, IRQ: Wait> Driver<SPI, CE, IRQ> {
    /// Sets the retries configuration of the device.
    /// Please see the `TXConfig` struct for reference of the fields of this method.
    /// WARNING : After changing the configuration the device will be in standby mode.
    pub async fn retries(&mut self, retries: u8, delay: u8) -> Result<pipe::TXConfig, Self::Error> {
        use pipe::TXConfig;

        // Validate the values.
        let r = if retries > 14 { 15 } else { retries };
        let d = if delay == 0 { 0 } else if delay > 15 { 15 } else { delay - 1 };

        // Create the register value to write.
        let v = (d << 4) | (r & 0xF);

        // Write the register.
        if let Err( hwe ) = self.writereg(Register::SetupRetries, v).await {
            return Err( (Error::FailedRegisterWrite, Some(hwe)) )
        }

        Ok( TXConfig { retries, delay, } )
    }

    /// Sets the RF channel used by the device.
    /// The channel will be validated against the valid values. The method
    /// returns the channel that was actually set.
    /// WARNING : After changing channel the device will be in standby mode.
    pub async fn setchannel(&mut self, ch: u8) -> Result<u8, Self::Error> {
        // Write to the register.
        if let Err( hwe ) = self.writereg(Register::RFChannel, ch & 0x7F).await {
            return Err( (Error::FailedRegisterWrite, Some(hwe)) )
        }

        // Modify the configuration.
        self.config.ch = ch & 0x7F;

        Ok( ch & 0x7F )
    }

    /// Sets the Least Significant Byte of the address for the given pipe.
    /// WARNING : After changing address the device will be in standby mode.
    pub async fn subaddr(&mut self, pipe: u8, lsb: u8) -> Result<(), Self::Error> {
        // Set in standby.
        self.standby().await?;

        // Check that the pipe number is valid.
        if (pipe < 2) || (pipe > 5) { return Err( (Error::IllegalPipeNumber(pipe), None) ) }

        // Write the register.
        if let Err( hwe ) = self.writereg(Register::RX0Address as u8 + pipe, lsb).await {
            return Err( (Error::FailedRegisterWrite, Some(hwe)) )
        }

        // Modify the configuration.
        self.config.sub[pipe as usize - 1] = lsb;

        Ok( () )
    }

    /// Sets the base address for Pipes 1-5 and the Least Significant Byte for Pipe 1.
    /// WARNING : After changing address the device will be in standby mode.
    pub async fn baseaddr(&mut self, addr: [u8; 5]) -> Result<(), Self::Error> {
        // Set in standby.
        self.standby().await?;

        // Create the buffer to send.
        let buf = [(1 << 5) | Register::RX1Address as u8, addr[0], addr[1], addr[2], addr[3], addr[4]];

        // Send the buffer.
        if let Err( hwe ) = self.writebuf(&buf).await {
            return Err( (Error::FailedRegisterWrite, Some(hwe)) )
        }

        // Modify the configuration.
        self.config.base = [addr[0], addr[1], addr[2], addr[3]];
        self.config.sub[0] = addr[4];

        Ok( () )
    }

    /// Sets the main address for Pipe 0 and the TX Pipe.
    /// WARNING : After changing address the device will be in standby mode.
    pub async fn mainaddr(&mut self, addr: [u8; 5]) -> Result<(), Self::Error> {
        // Set in standby.
        self.standby().await?;

        // Create the buffer to send.
        let mut buf = [(1 << 5) | Register::RX0Address as u8, addr[0], addr[1], addr[2], addr[3], addr[4]];

        // Send the buffer.
        if let Err( hwe ) = self.writebuf(&buf).await {
            return Err( (Error::FailedRegisterWrite, Some(hwe)) )
        }

        // Change to write to the TX address register.
        buf[0] = (1 << 5) | Register::TXAddress as u8;

        // Send the buffer.
        if let Err( hwe ) = self.writebuf(&buf).await {
            return Err( (Error::FailedRegisterWrite, Some(hwe)) )
        }

        // Modify the configuration.
        self.config.main = addr;

        Ok( () )
    }

    /// Clears the given IRQs from the status register.
    pub async fn clearirq(&mut self, rxdr: bool, txds: bool, retries: bool) -> Result<(), Self::HWError> {
        // Build the IRQ clear byte.
        let mut irq = 0;

        if retries { irq |= 1 << 4 }
        if txds    { irq |= 1 << 5 }
        if rxdr    { irq |= 1 << 6 }

        // Write the register.
        self.writereg(Register::Status, irq).await
    }

    /// Clears all IRQs from the status register.
    pub async fn clearirqs(&mut self) -> Result<(), Self::HWError> {
        self.writereg(Register::Status, 0b111 << 4).await
    }

    /// Reads the payload width of the top level FIFO packet.
    pub async fn pldwidth(&mut self) -> Result<u8, Self::HWError> {
        // Create the read buffer.
        let mut read = [0; 2];

        // Perform the SPI transfer.
        self.spi.transfer(&mut read, &[Command::PayloadWidth as u8, 0]).await.map_err( HardwareError::Serial )?;

        Ok( read[1] )
    }

    /// Reads the device's STATUS register.
    /// If the wait flag is set the driver will wait for the IRQ pin to go low.
    /// If the pin is not available, it will perform an immediate read.
    pub async fn status(&mut self, wait: bool) -> Result<u8, Self::Error> {
        // Wait if the flag is set and IRQ pin is available.
        if wait && let Some(ref mut irq) = self.irq {
            if let Err( hwe ) = irq.wait_for_low().await.map_err( HardwareError::Interrupt ) {
                return Err( ( Error::CouldNotReadStatus, Some(hwe) ) );
            }
        }

        // RX buffer.
        let mut read = [0u8];

        // Perform the SPI transfer.
        if let Err( hwe ) = self.spi.transfer(&mut read, &[0xFF]).await {
            return Err( ( Error::CouldNotReadStatus, Some( HardwareError::Serial(hwe) ) ) );
        }

        Ok( read[0] )
    }

    /// Reads the device's STATUS and FIFO STATUS registers.
    /// If the wait flag is set the driver will wait for the IRQ pin to go low.
    /// If the pin is not available, it will perform an immediate read.
    pub async fn statuses(&mut self, wait: bool) -> Result<[u8; 2], Self::Error> {
        // Wait if flag is set and IRQ pin is available.
        if wait && let Some(ref mut irq) = self.irq {
            if let Err( hwe ) = irq.wait_for_low().await.map_err( HardwareError::Interrupt ) {
                return Err( ( Error::CouldNotReadStatus, Some(hwe) ) );
            }
        }

        // Create the read buffer.
        let mut read = [0; 2];

        // Perform the SPI transfer.
        if let Err( hwe ) = self.spi.transfer(&mut read, &[Register::FIFOStatus as u8, 0]).await {
            return Err( ( Error::CouldNotReadStatus, Some( HardwareError::Serial(hwe) ) ) );
        }

        Ok( read )
    }

    /// Sets the device in a power down state.
    /// In this state the device consumes the least energy but will take longer
    /// to start transmitting or receiving.
    pub async fn powerdown(&mut self) -> Result<(), Self::Error> {
        // Return early if already in power down mode.
        if self.state == State::PowerDown {
            return Ok(())
        }

        // Check if in transmitting or receiving mode and disable CE.
        if (self.state == State::Transmitting) || (self.state == State::Receiving) {
            if let Err( hwe ) = self.ce.set_low().map_err( HardwareError::ChipEnable ) {
                return Err( ( Error::FailedModeSet(State::Standby), Some(hwe) ) );
            }

            // Set state to standby.
            self.state = State::Standby;
        }

        // Read the CONFIG register.
        let config = match self.readreg(Register::Config).await {
            Err( hwe ) => return Err( ( Error::FailedModeSet(State::PowerDown), Some(hwe) ) ),
            Ok(c) => c,
        };

        // Write the CONFIG register, disable PWR UP bit.
        if let Err( hwe ) = self.writereg(Register::Config, config & !(1 << 1)).await {
            return Err( ( Error::FailedModeSet(State::PowerDown), Some(hwe) ) );
        }

        // Set the state.
        self.state = State::PowerDown;

        Ok( () )
    }

    /// Sets the device in a RX ready state.
    /// In this state the device consumes the most energy but can begin
    /// transmitting in less time than when in standby.
    /// WARNING : The driver user must guarantee a delay depending on the mode
    /// transition that happenned:
    ///   - Standby     to RXMode : 130 microseconds
    ///   - TXMode      to RXMode : 130 microseconds
    ///   - Power down  to RXMode : 5 milliseconds
    pub async fn rxmode(&mut self) -> Result<(), Self::Error> {
        // Return early if already in TX mode.
        if self.state == State::Receiving {
            return Ok( () );
        }

        // Read the config register.
        let mut config = match self.readreg( Register::Config ).await {
            Err( hwe ) => return Err( (Error::FailedModeSet( State::Standby ), Some(hwe) ) ),
            Ok( c ) => c
        };

        // Clear all bits but CRC configuration.
        config &= 0b11 << 2;

        // Set PWR UP bit. Set PRX bit. Mask TX and RT IRQs.
        config |= (0b11 << 4) | (1 << 1) | 1;

        // Write the CONFIG register.
        if let Err( hwe ) = self.writereg( Register::Config, config ).await {
            return Err( (Error::FailedModeSet( State::Standby ), Some(hwe) ) );
        }

        // Set the standby mode.
        self.state = State::Standby;

        // Enable CE output.
        if let Err( hwe ) = self.ce.set_high().map_err( HardwareError::ChipEnable ) {
            return Err( (Error::FailedModeSet( State::Receiving ), Some( hwe )) );
        }

        // Set the TX mode.
        self.state = State::Receiving;

        Ok( () )
    }

    /// Sets the device in a TX ready state.
    /// In this state the device consumes the most energy but can begin
    /// transmitting in less time than when in standby.
    /// WARNING : The driver user must guarantee a delay depending on the mode
    /// transition that happenned:
    ///   - Standby     to TXMode : 130 microseconds
    ///   - RXMode      to TXMode : 130 microseconds
    ///   - Power down  to TXMode : 5 milliseconds
    pub async fn txmode(&mut self) -> Result<(), Self::Error> {
        // Return early if already in TX mode.
        if self.state == State::Transmitting {
            return Ok( () );
        }

        // Read the config register.
        let mut config = match self.readreg( Register::Config ).await {
            Err( hwe ) => return Err( (Error::FailedModeSet( State::Standby ), Some(hwe) ) ),
            Ok( c ) => c
        };

        // Clear all bits but CRC configuration.
        config &= 0b11 << 2;

        // Set PWR UP bit. Mask RX IRQ.
        config |= (1 << 6) | (1 << 1);

        // Write the CONFIG register.
        if let Err( hwe ) = self.writereg( Register::Config, config ).await {
            return Err( (Error::FailedModeSet( State::Standby ), Some(hwe) ) );
        }

        // Set the standby mode.
        self.state = State::Standby;

        // Enable CE output.
        if let Err( hwe ) = self.ce.set_high().map_err( HardwareError::ChipEnable ) {
            return Err( (Error::FailedModeSet( State::Transmitting ), Some( hwe )) );
        }

        // Set the TX mode.
        self.state = State::Transmitting;

        Ok( () )
    }

    /// Sets the device in a standby state.
    /// In this state the device consumes more energy than powered down but can
    /// begin transmitting or receiving faster.
    /// WARNING : The driver user must guarantee a delay depending on the mode
    /// transition that happenned:
    ///   - Power down  to Standby : 4.5 milliseconds
    pub async fn standby(&mut self) -> Result<(), Self::Error> {
        // Return early if already in standby mode.
        if self.state == State::Standby{
            return Ok( () );
        }

        // Check the state of the device.
        match self.state {
            // RX or TX mode.
            State::Receiving | State::Transmitting => {
                // Disable CE pin.
                if let Err( hwe ) = self.ce.set_low().map_err( HardwareError::ChipEnable ) {
                    return Err( (Error::FailedModeSet(State::Standby), Some(hwe) ) );
                }

                // Set the state.
                self.state = State::Standby;

                Ok(())
            },

            _ => {
                // Read the config register.
                let config = match self.readreg(Register::Config).await {
                    Err( hwe ) => return Err( (Error::FailedModeSet(State::Standby), Some(hwe)) ),
                    Ok(c) => c,
                };

                // Write the CONFIG register with enabled PWR UP bit.
                if let Err( hwe ) = self.writereg(Register::Config, config | (1 << 1)).await {
                    return Err( ( Error::FailedModeSet(State::Standby), Some(hwe) ) );
                }

                // Set the state.
                self.state = State::Standby;

                Ok( () )
            },
        }
    }
}


/// Basic methods for interacting with the NRF24 device.
/// Writing and reading buffers / commands / registers through SPI.
impl<SPI: SpiDevice, CE: OutputPin, IRQ: Wait> Driver<SPI, CE, IRQ> {
    /// Low level function to write a buffer to the device.
    /// This function correlates to the `write` SPI function.
    pub async fn writebuf(&mut self, buf: &[u8]) -> Result<(), Self::HWError> {
        self.spi.write(buf).await.map_err( HardwareError::Serial )
    }

    /// Low level function to write to a register.
    pub async fn writereg<R: Into<u8>>(&mut self, r: R, v: u8) -> Result<(), Self::HWError> {
        // Create the command buffer.
        let buf = [(1 << 5) | r.into(), v];

        // Write to the register.
        self.spi.write(&buf).await.map_err( HardwareError::Serial )
    }

    /// Low level function to read a register.
    pub async fn readreg<R: Into<u8>>(&mut self, r: R) -> Result<u8, Self::HWError> {
        // Create the command buffer.
        let write = [r.into(), 0x00];

        // Create the receive buffer.
        let mut read = [0u8; 2];

        // Perform the transfer.
        self.spi.transfer(&mut read, &write).await.map_err( HardwareError::Serial )?;

        Ok( read[1] )
    }

    /// Low level function to send a 1 byte command.
    pub async fn command(&mut self, c: Command) -> Result<(), Self::HWError> {
        self.spi.write(&[c.into()]).await.map_err( HardwareError::Serial )
    }
}


impl<SPI, CE, IRQ> Drop for Driver<SPI, CE, IRQ> {
    fn drop(&mut self) {
        // Orphan all the data pipes.
        self.txpipe.driver = DriverState::Orphaned;

        for pipe in &mut self.rxpipes {
            pipe.driver = DriverState::Orphaned;
        }
    }
}



/// External closure to read the status register.
/// Needed to avoid compiler complaints.
async fn status<SPI: SpiDevice, CE: OutputPin, IRQ: Wait>(irq: &mut Option<IRQ>, spi: &mut SPI) -> Result<u8, (Error, Option<HardwareError<SPI::Error, CE::Error, IRQ::Error>>)> {
    // Wait if the flag is set and IRQ pin is available.
    if let Some(ref mut irq) = irq {
        if let Err( hwe ) = irq.wait_for_low().await.map_err( HardwareError::Interrupt ) {
            return Err( ( Error::CouldNotReadStatus, Some(hwe) ) );
        }
    }

    // RX buffer.
    let mut read = [0u8];
    
    // Perform the SPI transfer.
    if let Err( hwe ) = spi.transfer(&mut read, &[0xFF]).await {
        return Err( ( Error::CouldNotReadStatus, Some( HardwareError::Serial(hwe) ) ) );
    }
    
    Ok( read[0] )    
}
