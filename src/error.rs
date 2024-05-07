//! Errors emitted NRF24L01(+) device drivers.
//! There are 2 types of errors, driver errors and hardware errors.
//! The driver will always return at least a driver error. The hardware errors
//! returned specify that the error originated from the hardware not the driver
//! or the NRF24L01(+) device.



use super::State;



/// A set of possible usage errors in the driver.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "log", derive(defmt::Format))]
pub enum Error {
    /// The driver could not read the status register.
    CouldNotReadStatus,

    /// The creation of the driver failed.
    /// Probably hardware error.
    FailedDriverCreation,

    /// The configuration of the device failed.
    FailedDriverConfiguration,

    /// The radio failed to get into a given state.
    FailedModeSet( State ),

    /// Failed to download a new payload from the device.
    FailedPacketDownload,

    /// Failed to upload a new payload to the device.
    FailedPacketUpload,

    /// Failed to modify a register.
    FailedRegisterWrite,

    /// The given pipe number is not allowed.
    IllegalPipeNumber( u8 ),

    /// A transaction reached the maximum number of retries without a response.
    MaxRetries,

    /// The driver disconnected from the pipe.
    Orphaned,

    /// The pipe disconnected from the driver.
    PipeClosed,

    /// The pipe is currently in use.
    PipeCurrentlyUsed,

    /// The pipe is not available.
    PipeNotAvailable,

    /// The device's RX FIFO has received more than 52 bytes.
    RXOverflow,

    /// Failed to create a RX pipe wrapper.
    RXPipeCreationFailed( u8 ),

    /// Failed to create a TX pipe wrapper.
    TXPipeCreationFailed,

    /// Could not read payload width for download process.
    UnknownPayloadWidth,
}

/*
#[cfg(feature = "log")]
impl defmt::Format for Error {
    fn format(&self, _: defmt::Formatter) {
        // String to format.
        let string = match self {
            Error::CouldNotReadStatus => defmt::intern!("Could not read device status"),
            Error::FailedDriverCreation => defmt::intern!("Failed to create radio driver"),
            Error::FailedDriverConfiguration => defmt::intern!("Failed to set driver configuration"),
            Error::FailedModeSet( state ) => match state {
                State::PowerDown => defmt::intern!("Failed to set radio to PowerDown"),
                State::Standby => defmt::intern!("Failed to set radio to Standby"),
                State::Transmitting => defmt::intern!("Failed to set radio to Transmit"),
                State::Receiving => defmt::intern!("Failed to set radio to Receive"),
            },
            Error::FailedPacketDownload => defmt::intern!("Failed to download a packet from the radio FIFO"),
            Error::FailedPacketUpload => defmt::intern!("Failed to upload a packet to the radio FIFO"),
            Error::FailedRegisterWrite => defmt::intern!("Failed to write to a register"),
            Error::IllegalPipeNumber( _ ) => defmt::intern!("Illegal pipe number found {=u8}"),
            Error::MaxRetries => defmt::intern!("Maximum retries reached while transmitting"),
            Error::Orphaned => defmt::intern!("A driver pipe was orphaned"),
            Error::PipeClosed => defmt::intern!("A driver pipe was closed"),
            Error::PipeCurrentlyUsed => defmt::intern!("The requested driver pipe is already in use"),
            Error::PipeNotAvailable => defmt::intern!("The requested driver pipe is not available"),
            Error::RXOverflow => defmt::intern!("Overflow in a RX pipe"),
            Error::RXPipeCreationFailed( _ ) => defmt::intern!("Failed to create RX pipe {=u8}"),
            Error::TXPipeCreationFailed => defmt::intern!("Failed to create a TX pipe"),
            Error::UnknownPayloadWidth => defmt::intern!("Unexpected payload width in a received packet"),
        };

        // Send the defmt string.
        defmt::export::istr(&string);

        // Check if the error needs additional data.
        match self {
            Error::IllegalPipeNumber( n ) => defmt::export::u8(n),
            Error::RXPipeCreationFailed( n ) => defmt::export::u8(n),

            _ => (),
        }
    }
}
*/



/// A set of possible hardware errors in the driver.
pub enum HardwareError<SPI, CE, IRQ> {
    /// An error with the SPI hardware.
    Serial( SPI ),

    /// An error with the IRQ pin.
    Interrupt( IRQ ),

    /// An error with the CE pin.
    ChipEnable( CE ),
}

#[cfg(feature = "log")]
impl<SPI, CE, IRQ> defmt::Format for HardwareError<SPI, CE, IRQ> {
    fn format(&self, _: defmt::Formatter) {
        // String to format.
        let string = match self {
            HardwareError::Serial( _ ) => defmt::intern!("SPI Serial Error"),
            HardwareError::Interrupt( _ ) => defmt::intern!("Interrupt Error"),
            HardwareError::ChipEnable( _ ) => defmt::intern!("Digital IO Error"),
        };

        // Send the defmt string.
        defmt::export::istr(&string);
    }
}
