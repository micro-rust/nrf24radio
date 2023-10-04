//! This example shows the functionality of the NRF24 radio driver with a single RP2040
//! The same RP2040 device has two NRF24 devices connected and uses one to send
//! data and one to receive data.
//! 
//! PINOUT :
//! 
//! NRF Device 0:
//!   - MISO : GPIO 0
//!   - CLK  : GPIO 2
//!   - MOSI : GPIO 3
//!   - CSN  : GPIO 4
//!   - CE   : GPIO 5
//!   - IRQ  : GPIO 6
//! 
//! NRF Device 1:
//!   - IRQ  : GPIO 10
//!   - CE   : GPIO 11
//!   - MISO : GPIO 12
//!   - CSN  : GPIO 13
//!   - CLK  : GPIO 14
//!   - MOSI : GPIO 15



#![no_std]
#![no_main]

#![feature(type_alias_impl_trait)]



// Core imports.
use core::mem::MaybeUninit;

// Setup the logging framework.
use defmt_rtt as _;

// Import the executor macros.
use embassy_executor::{
    main, task,

    Spawner,
};

// Import the RP2040 device peripherals.
use embassy_rp::{
    init,

    gpio::{
        Input, Level, Output, Pull,
    },

    peripherals::{
        SPI0, SPI1,

        PIN_4,  PIN_5,  PIN_6,
        PIN_10, PIN_11, PIN_13,
    },

    spi::{
        Async, Config, Spi,
    },
};

// Import the necessary SYNC abstractions.
use embassy_sync::{
    mutex::Mutex,
    blocking_mutex::raw::CriticalSectionRawMutex,
};

// Import the timer abstractions.
use embassy_time::{
    Duration, Instant, Timer,
};

// Import the SPI device abstraction.
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;

// Import the SPI device.
//use embedded_hal_async::spi::SpiDevice;

// Import radio.
use nrf24radio::{
    Error,

    pipe::{
        RXDataPipe, TXDataPipe,
    },
};

// Import the panic implementation.
use panic_probe as _;



/// SPI Global for the SPI0 Bus mutex.
/// This allows multiple tasks to access the SPI bus through `SpiDevice`s.
static mut SPI0GLOBAL: MaybeUninit<Mutex<CriticalSectionRawMutex, Spi<'static, SPI0, Async>>> = MaybeUninit::uninit();

/// SPI Global for the SPI1 Bus mutex.
/// This allows multiple tasks to access the SPI bus through `SpiDevice`s.
static mut SPI1GLOBAL: MaybeUninit<Mutex<CriticalSectionRawMutex, Spi<'static, SPI1, Async>>> = MaybeUninit::uninit();



#[main]
async fn main(spawner: Spawner) {
    defmt::info!("Main task started");

    // Get the peripherals. Default configuration.
    let p = init( Default::default() );

    // This block will spawn the RX task.
    {
        // These are the pins needed for the SPI interface.
        let miso = p.PIN_12;
        let clk  = p.PIN_14;
        let mosi = p.PIN_15;

        // The CS pin has to become an Output. This is because the Driver does
        // not use a `SpiBus` interface (which would block any other shared use
        // of the SPI peripheral) but instead uses a shared `SpiDevice`.
        let cs = Output::new( p.PIN_13, Level::High );

        // The CE pin is the Chip Enable pin of the NRF24L01(+).
        // This pin is active high, so we create it active low.
        let ce = Output::new( p.PIN_11, Level::Low );

        // The IRQ pin is optional but recommended. Providing this pin will increase
        // performance and reduce power consumption.
        // This is optional in order to reduce the use of pins in low pin count MCUs.
        let irq = Input::new( p.PIN_10, Pull::None );

        // Get the DMA channels.
        // The Async interface can also be done with interrupts if no channels are available.
        let tx_dma = p.DMA_CH2;
        let rx_dma = p.DMA_CH3;

        // Create the SPI configuration.
        // We set it at the maximum speed, but if you are getting an unstable driver
        // try to reduce the SPI speed.
        let mut config = Config::default();
        config.frequency = 10_000_000;

        // Create the SPI.
        let spi = Spi::new( p.SPI1, clk, mosi, miso, tx_dma, rx_dma, config );

        // Store the SPI1 global mutex.
        unsafe { SPI1GLOBAL.write( Mutex::new( spi ) ) };
        defmt::info!("Created RX SPI global");

        // Create the SPI device.
        // You can create more of these devices with the same global.
        let spidevice = SpiDevice::new( unsafe { &mut *SPI1GLOBAL.as_mut_ptr() }, cs );
        defmt::info!("Created RX shared SPI device");

        // Spawn the RX task.
        match spawner.spawn( rx(spidevice, ce, irq, spawner.clone()) ) {
            Err(_) => defmt::error!("Failed to spawn RX task"),
            _ => defmt::info!("Spawned RX task"),
        }
    }

    // This block will spawn the TX task.
    {
        // These are the pins needed for the SPI interface.
        let miso = p.PIN_0;
        let clk  = p.PIN_2;
        let mosi = p.PIN_3;

        // The CS pin has to become an Output. This is because the Driver does
        // not use a `SpiBus` interface (which would block any other shared use
        // of the SPI peripheral) but instead uses a shared `SpiDevice`.
        let cs = Output::new( p.PIN_4, Level::High );

        // The CE pin is the Chip Enable pin of the NRF24L01(+).
        // This pin is active high, so we create it active low.
        let ce = Output::new( p.PIN_5, Level::Low );

        // The IRQ pin is optional but recommended. Providing this pin will increase
        // performance and reduce power consumption.
        // This is optional in order to reduce the use of pins in low pin count MCUs.
        let irq = Input::new( p.PIN_6, Pull::None );

        // Get the DMA channels.
        // The Async interface can also be done with interrupts if no channels are available.
        let tx_dma = p.DMA_CH0;
        let rx_dma = p.DMA_CH1;

        // Create the SPI configuration.
        // We set it at the maximum speed, but if you are getting an unstable driver
        // try to reduce the SPI speed.
        let mut config = Config::default();
        config.frequency = 10_000_000;

        // Create the SPI.
        let spi = Spi::new( p.SPI0, clk, mosi, miso, tx_dma, rx_dma, config );

        // Store the SPI0 global mutex.
        unsafe { SPI0GLOBAL.write( Mutex::new( spi ) ) };
        defmt::info!("Created TX SPI global");

        // Create the SPI device.
        // You can create more of these devices with the same global.
        let spidevice = SpiDevice::new( unsafe { &mut *SPI0GLOBAL.as_mut_ptr() }, cs );
        defmt::info!("Created TX shared SPI device");

        // Spawn the RX task.
        match spawner.spawn( tx(spidevice, ce, irq, spawner.clone()) ) {
            Err(_) => defmt::error!("Failed to spawn TX task"),
            _ => defmt::info!("Spawned TX task"),
        }
    }
}


// This task will showcase how to handle a RX radio driver.
#[task]
async fn rx(spi: SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, Async>, Output<'static, PIN_13>>, ce: Output<'static, PIN_11>, irq: Input<'static, PIN_10>, spawner: Spawner) {
    use nrf24radio::{
        Config, Driver,

        common::*,

        pipe::{
            DataPipe, RXConfig,
        },
    };

    defmt::info!("RX : Hello from the RX driver task");

    // Static data pipes to use as buffers.
    // In the future these will be able to be optional in order to reduce memory footprint.

    // The TX pipe buffer.
    static mut TXPIPE: DataPipe = DataPipe::new();

    // The RX pipe buffers.
    static mut RXPIPE0: DataPipe = DataPipe::new();
    static mut RXPIPE1: DataPipe = DataPipe::new();
    static mut RXPIPE2: DataPipe = DataPipe::new();
    static mut RXPIPE3: DataPipe = DataPipe::new();
    static mut RXPIPE4: DataPipe = DataPipe::new();
    static mut RXPIPE5: DataPipe = DataPipe::new();

    // Collect into a slice.
    let rxpipes = unsafe { [
        &mut RXPIPE0, &mut RXPIPE1, &mut RXPIPE2,
        &mut RXPIPE3, &mut RXPIPE4, &mut RXPIPE5,
    ]};

    let txpipe = unsafe { &mut TXPIPE };

    // Create the radio driver.
    // Pay attention to the explicit type given here.
    // If you don't use the IRQ pin Rust will complain that it doesn't know the
    // type of the IRQ pin. To get around this, you have to provide a fake pin
    // type (the `Input<'_, PIN_10>` part). The pin number can be any.
    let mut radio: Driver<_, _, Input<'_, PIN_10>> = match Driver::create(spi, ce, Some(irq), rxpipes, txpipe).await {
        Err(_) => {
            defmt::error!("RX : Failed to create radio driver");
            return;
        },

        Ok(r) => r,
    };

    defmt::info!("RX : Created the radio driver");

    // Create the configuration of the radio.
    // Please note that this configuration MUST be the same as the TX configuration.
    // The only thing that can be different is the gain.
    // If you want this can be made a constant, as all `Config` methods are `const`.
    let config = Config::new()
        .channel( 0 )
        .datarate( DataRate::High )
        .gain( Gain::Max )
        .crc( CRCBytes::One );

    // Write the configuration.
    if let Err(_) = radio.configure( config ).await {
        defmt::error!("RX : Failed to configure the driver");
        return;
    }

    // Now we open a data pipe of the radio.
    // A data pipe is a stream of data that can be received on the radio.
    // The radios can receive data in up to 6 pipes with different addresses.
    // This will open Pipe 0.
    // Take into account that Pipe 0 has some restrictions on the configuration allowed
    let config = RXConfig::default();

    // Create the RX data Pipe 0.
    let pipe = match radio.openrx( 0, config ).await {
        Err(_) => {
            defmt::error!("RX : Failed to open data pipe");
            return;
        },
        Ok(p) => p,
    };

    // Now that we have a data pipe, we can send it to another task to do whatever it needs.
    // Please remember that at least one task has to handle the driver for the pipes to receive data.
    // The architecture of this driver is one task for driver handling and separate tasks for the pipes.
    if let Err(_) = spawner.spawn( rxdata(pipe, spawner.clone()) ) {
        defmt::error!("RX : Failed to spawn data pipe task");
        return;
    }

    // Now we have a NRF24 device in an unknown state (probably powered down or on standby).
    // So we have to wake it.
    // The NRF24L01 devices need at least 5 ms to go from POWERED DOWN to STANDBY and the driver
    // does not insert those delays (in order to be executor agnostic) so we have to do those manually.
    if let Err(_) = radio.standby().await {
        defmt::error!("RX : Failed to change to STANDBY mode");
        return;
    }

    // Now we wait the delay.
    Timer::after( Duration::from_millis(5) ).await;

    // Now that we are in standby mode, we have to move to the listening mode.
    // This change needs a delay of 130 us.
    if let Err(_) = radio.rxmode().await {
        defmt::error!("RX : Failed to change to RX mode");
        return;
    }

    // Now we wait the delay.
    Timer::after( Duration::from_micros(130) ).await;

    // Now we start the driver handling loop.
    loop {
        // Check for data.
        // Notice the `wait` boolean flag of the method.
        // This flag indicates if the driver wants to wait for incoming data or check and return immediately.
        // WARNING : If the wait flag is set the driver may wait forever if no data arrives.
        // But, on the other hand, waiting with an IRQ pin available (line 187) the driver will wait on the pin
        // and the device may go to sleep, reducing opwer consumption and latency to respond to messages.
        match radio.rxdata( true ).await {
            // If the radio received no data, or received data in a closed pipe, it will return None.
            Ok( maybe ) => match maybe {
                // If there was data received, the driver returns the number of the pipe that received data,
                // the number of bytes received and if there is more data immediately available to read.
                Some( (pipe, bytes, more) ) => defmt::trace!("RX : Received {} bytes on pipe {} (pending {})", bytes, pipe, more),
                _ => (),
            },

            // If there was an error the driver returns a tuple.
            // The first element is the driver error (what could not be accomplished)
            // The second element is the hardware error (the SPI failed, the IRQ failed, the CE failed)
            // This hardware error may or may not be set, depending on the reason for the first error.
            Err( (error, _) ) => match error {
                // RX OVerflow errors must be handled. These errors occur due to errors
                // in the ShockBurst protocol. For now, clear these manually, in the future
                // a clear or reset method may be provided.
                Error::RXOverflow => {
                    defmt::warn!("RX : Resetting RX device after overflow...");

                    // Set the device to power down. This needs no delay.
                    while let Err(_) = radio.powerdown().await {}

                    // Perform a RX FIFO flush.
                    while let Err(_) = radio.command( Command::FlushRX ).await {}

                    // Move to standby.
                    while let Err(_) = radio.standby().await {}

                    // Wait for 5 milliseconds.
                    Timer::after( Duration::from_millis( 5 ) ).await;

                    // Move to listen mode.
                    while let Err(_) = radio.rxmode().await {}

                    // Wait for 130 microseconds.
                    Timer::after( Duration::from_micros( 130 ) ).await;
                },

                _ => defmt::error!("RX : Generic driver error"),
            },
        }
    }
}



// This task will showcase how to handle a TX radio driver.
#[task]
async fn tx(spi: SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI0, Async>, Output<'static, PIN_4>>, ce: Output<'static, PIN_5>, irq: Input<'static, PIN_6>, spawner: Spawner) {
    use nrf24radio::{
        Config, Driver,

        common::*,

        pipe::{
            DataPipe, TXConfig,
        },
    };

    defmt::info!("TX : Hello from the TX driver task");

    // Static data pipes to use as buffers.
    // In the future these will be able to be optional in order to reduce memory footprint.

    // The TX pipe buffer.
    static mut TXPIPE: DataPipe = DataPipe::new();

    // The RX pipe buffers.
    static mut RXPIPE0: DataPipe = DataPipe::new();
    static mut RXPIPE1: DataPipe = DataPipe::new();
    static mut RXPIPE2: DataPipe = DataPipe::new();
    static mut RXPIPE3: DataPipe = DataPipe::new();
    static mut RXPIPE4: DataPipe = DataPipe::new();
    static mut RXPIPE5: DataPipe = DataPipe::new();

    // Collect into a slice.
    let rxpipes = unsafe { [
        &mut RXPIPE0, &mut RXPIPE1, &mut RXPIPE2,
        &mut RXPIPE3, &mut RXPIPE4, &mut RXPIPE5,
    ]};

    let txpipe = unsafe { &mut TXPIPE };

    // Create the radio driver.
    // Pay attention to the explicit type given here.
    // If you don't use the IRQ pin Rust will complain that it doesn't know the
    // type of the IRQ pin. To get around this, you have to provide a fake pin
    // type (the `Input<'_, PIN_10>` part). The pin number can be any.
    let mut radio: Driver<_, _, Input<'_, PIN_6>> = match Driver::create(spi, ce, Some(irq), rxpipes, txpipe).await {
        Err(_) => {
            defmt::error!("TX : Failed to create radio driver");
            return;
        },

        Ok(r) => r,
    };

    defmt::info!("TX : Created the radio driver");

    // Create the configuration of the radio.
    // Please note that this configuration MUST be the same as the RX configuration.
    // The only thing that can be different is the gain.
    // If you want this can be made a constant, as all `Config` methods are `const`.
    let config = Config::new()
        .channel( 0 )
        .datarate( DataRate::High )
        .gain( Gain::Max )
        .crc( CRCBytes::One );

    // Write the configuration.
    if let Err(_) = radio.configure( config ).await {
        defmt::error!("TX : Failed to configure the driver");
        return;
    }

    // Now we open a data pipe of the radio.
    // The TX data pipe can be configured on the number of retries and the delay between
    // retries (in 250 us increments) up to 15 retries and 4000 us.
    let config = TXConfig::new()
        .retries(15)
        .delay(16);

    // Create the RX data Pipe 0.
    let pipe = match radio.opentx( config ).await {
        Err(_) => {
            defmt::error!("TX : Failed to open data pipe");
            return;
        },
        Ok(p) => p,
    };

    // Now that we have a data pipe, we can send it to another task to do whatever it needs.
    // Please remember that at least one task has to handle the driver for the pipes to receive data.
    // The architecture of this driver is one task for driver handling and separate tasks for the pipes.
    if let Err(_) = spawner.spawn( txdata(pipe, spawner.clone()) ) {
        defmt::error!("TX : Failed to spawn data pipe task");
        return;
    }

    // Now we have a NRF24 device in an unknown state (probably powered down or on standby).
    // So we have to wake it.
    // The NRF24L01 devices need at least 5 ms to go from POWERED DOWN to STANDBY and the driver
    // does not insert those delays (in order to be executor agnostic) so we have to do those manually.
    if let Err(_) = radio.standby().await {
        defmt::error!("TX : Failed to change to STANDBY mode");
        return;
    }

    // Now we wait the delay.
    Timer::after( Duration::from_millis(5) ).await;

    // Now that we are in standby mode, we have to move to the transmitting mode.
    // This change needs a delay of 130 us.
    if let Err(_) = radio.txmode().await {
        defmt::error!("TX : Failed to change to RX mode");
        return;
    }

    // Now we wait the delay.
    Timer::after( Duration::from_micros(130) ).await;

    // Now we start the driver handling loop.
    loop {
        // Check for data to transmit.
        // Notice the `wait` boolean flag of the method.
        // This flag indicates if the driver wants to wait for outgoing data or check and return immediately.
        // WARNING : If the wait flag is set the driver may wait forever if no data is sent through the TX pipe.
        // But, on the other hand, waiting may allow the device to go to sleep, reducing power consumption and
        // latency to respond to messages.
        match radio.txdata(true).await {
            // If the radio found no data to send it will return None.
            Ok( maybe ) => match maybe {
                // If it sent data, it will return the number of bytes sent.
                Some( bytes ) => defmt::debug!("TX : Sent {} bytes", bytes),
                _ => (),
            },

            Err( (error, _) ) => match error {
                // At least 1 packet was lost during transmission after the maximum number of retries.
                // To fix this, reduce data rate of the config, increase gain, increase retries or retry delay.
                // All these fixes increase reliability at the cost of data rate.
                Error::MaxRetries => defmt::error!("TX : Max retries reached"),
                _ => defmt::error!("TX : Driver error"),
            },
        }
    }
}



/// RX Bitrate measurement flag.
static mut RXTRIGGER: bool = false;

/// RX Bitrate measurement flag.
static mut TXTRIGGER: bool = false;



/// This is a ficticious task that will read the data received on the RX pipe.
#[task]
async fn rxdata(mut pipe: RXDataPipe, spawner: Spawner) {
    defmt::info!("RX Pipe : Hello from the RX pipe task");

    // A buffer in which to read the data.
    // Must be at least 32 bytes to accommodate the maximum packet size.
    let mut buf = [0u8; 32];

    // This is a byte counter. We will use this to measure the real bitrate.
    let mut bytes = 0usize;

    // Create the start instant for bitrate measurements.
    let mut start = Instant::now();

    // Spawn the RX trigger task.
    if let Err(_) = spawner.spawn( trigger( unsafe { &mut RXTRIGGER } ) ) {
        defmt::error!("RX pipe : Failed to spawn bitrate measurement trigger task");
        return;
    }

    loop {
        // Check if it is time to report the bitrate.
        if unsafe { RXTRIGGER } {
            // Reset the flag.
            unsafe { RXTRIGGER = false }

            // Get the elapsed time.
            let elapsed = start.elapsed();

            // Measure bytes per second.
            let bps = (bytes as u64 * 1000) / elapsed.as_millis();

            // Log the bitrate.
            defmt::info!("RX pipe : Measured {} Bps / {} bps ({} bytes) ", bps, bps * 8, bytes);

            // Reset the variables.
            start = Instant::now();
            bytes = 0;
        }

        // Wait until there is data to be read.
        if let Err(_) = pipe.wait().await {
            defmt::error!("RX pipe : Error while waiting for data in the RX pipe");
            continue;
        }

        // Get the reader of the internal data.
        if let Some( reader ) = pipe.read() {
            // Index into our buffer.
            let mut i = 0;

            for byte in reader {
                buf[i] = byte;
                i += 1;
            }

            // Add the number of bytes of this packet.
            bytes += i;

            // Skip to the next iteration.
            continue;
        }

        defmt::error!("RX pipe : Failed to acquire RX pipe reader");
    }
}



/// This is a ficticious task that will send data on the TX pipe.
#[allow(dead_code)]
#[task]
async fn txdata(mut pipe: TXDataPipe, spawner: Spawner) {
    defmt::info!("TX Pipe : Hello from the RX pipe task");

    // Create the dummy data packets.
    #[link_section = ".data.RADIOTEST"]
    static SMALL: [u8; 6] = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];

    #[link_section = ".uninit.RADIOTEST"]
    static mut BIG: [u8; 65] = [0; 65];

    #[link_section = ".uninit.RADIOTEST"]
    static mut GIANT: [u8; 4096] = [0; 4096];

    // Initialize the buffers.
    for byte in unsafe { &mut BIG } {
        *byte = 0x99;
    }

    let mut i = 0;
    for byte in unsafe { &mut GIANT } {
        // Set the byte.
        *byte = i;

        // Increment i.
        match i {
            255 => i = 0,
            _ => i += 1,
        }
    }

    // This is a byte counter. We will use this to measure the real bitrate.
    let mut bytes = 0usize;

    // Create the start instant for bitrate measurements.
    let mut start = Instant::now();

    // Spawn the RX trigger task.
    if let Err(_) = spawner.spawn( trigger( unsafe { &mut TXTRIGGER } ) ) {
        defmt::error!("TX pipe : Failed to spawn bitrate measurement trigger task");
        return;
    }

    loop {
        // Check if it is time to report the bitrate.
        if unsafe { TXTRIGGER } {
            // Reset the flag.
            unsafe { TXTRIGGER = false }

            // Get the elapsed time.
            let elapsed = start.elapsed();

            // Measure bytes per second.
            let bps = (bytes as u64 * 1000) / elapsed.as_millis();

            // Log the bitrate.
            defmt::info!("TX pipe : Measured {} Bps / {} bps ({} bytes) ", bps, bps * 8, bytes);

            // Reset the variables.
            start = Instant::now();
            bytes = 0;
        }

        // Select here which kind of packet you want to send.
        // This will let you see the effects on bitrate of different buffer sizes.
        // You can also set if you want the receiver to acknowledge the reception of packets.
        if let Err(_) = pipe.send( unsafe { &GIANT }, false ).await {
            defmt::error!("TX pipe : Failed to send a packet");
        }

        // Increase the byte counter.
        bytes += unsafe { GIANT.len() };
    }
}



// Theses are independent tasks that will trigger the bitrate measurements.
#[task(pool_size = 2)]
async fn trigger(flag: &'static mut bool) {
    loop {
        // Wait 4 seconds.
        Timer::after( Duration::from_secs(4) ).await;

        // Set the trigger.
        *flag = true;
    }
}
