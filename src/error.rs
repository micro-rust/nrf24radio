//! Errors emitted NRF24L01(+) device drivers.
//! There are 2 types of errors, driver errors and hardware errors.
//! The driver will always return at least a driver error. The hardware errors
//! returned specify that the error originated from the hardware not the driver
//! or the NRF24L01(+) device.



use super::State;



/// A set of possible usage errors in the driver.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
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

    /// The device's RX FIFO has received more than 52 bytes.
    RXOverflow,

    /// Failed to create a RX pipe wrapper.
    RXPipeCreationFailed( u8 ),

    /// Failed to create a TX pipe wrapper.
    TXPipeCreationFailed,

    /// Could not read payload width for download process.
    UnknownPayloadWidth,
}



/// A set of possible hardware errors in the driver.
pub enum HardwareError<SPI, CE, IRQ> {
    /// An error with the SPI hardware.
    Serial( SPI ),

    /// An error with the IRQ pin.
    Interrupt( IRQ ),

    /// An error with the CE pin.
    ChipEnable( CE ),
}
