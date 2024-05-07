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
#![feature(impl_trait_in_assoc_type)]


#![allow(static_mut_refs)]

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
    },

    spi::{
        Async, Config, Spi,
    },
};

// Import the necessary SYNC abstractions.
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    mutex::Mutex,
    signal::Signal,
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
    Error, RadioPolicy, RXErrorPolicy, TXErrorPolicy,

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

/// Radio 0 policy control.
/// This allows to dynamically change the behaviour of the radio.
static R0BEHAVIOUR: Signal<CriticalSectionRawMutex, RadioPolicy> = Signal::new();

/// Radio 1 policy control.
/// This allows to dynamically change the behaviour of the radio.
static R1BEHAVIOUR: Signal<CriticalSectionRawMutex, RadioPolicy> = Signal::new();

/// Size of the packets
const PACKETSIZE: usize = 64;

/// Report interval of the bitrate.
const INTERVAL: Duration = Duration::from_secs(4);



// Create the configuration of the radio.
// Please note that this configuration MUST be the same as the TX configuration.
// The only thing that can be different is the gain.
// If you want this can be made a constant, as all `Config` methods are `const`.
const RADIOCONFIG: nrf24radio::Config = nrf24radio::Config::new()
        .channel( 64 )
        .datarate( nrf24radio::common::DataRate::High )
        .gain( nrf24radio::common::Gain::High )
        .crc( nrf24radio::common::CRCBytes::One );



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
async fn rx(spi: SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, Async>, Output<'static>>, ce: Output<'static>, irq: Input<'static>, spawner: Spawner) {
    use nrf24radio::{
        Driver,

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
    let mut radio: Driver<_, _, Input<'_>> = match Driver::create(spi, ce, Some(irq), rxpipes, txpipe).await {
        Err(_) => {
            defmt::error!("RX : Failed to create radio driver");
            return;
        },

        Ok(r) => r,
    };

    defmt::info!("RX : Created the radio driver");

    // Write the configuration.
    if let Err(_) = radio.configure( RADIOCONFIG ).await {
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

    core::sync::atomic::compiler_fence( core::sync::atomic::Ordering::AcqRel );

    // Set in RX mode.
    //radio.rxmode().await;

    // Set the radio behaviour.
    R0BEHAVIOUR.signal(RadioPolicy::Receive);

    // Start running the radio driver.
    radio.run(&R0BEHAVIOUR, RXErrorPolicy::Skip, TXErrorPolicy::Skip).await;
}



// This task will showcase how to handle a TX radio driver.
#[task]
async fn tx(spi: SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI0, Async>, Output<'static>>, ce: Output<'static>, irq: Input<'static>, spawner: Spawner) {
    use nrf24radio::{
        Driver,

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
    let mut radio: Driver<_, _, Input<'_>> = match Driver::create(spi, ce, Some(irq), rxpipes, txpipe).await {
        Err(_) => {
            defmt::error!("TX : Failed to create radio driver");
            return;
        },

        Ok(r) => r,
    };

    defmt::info!("TX : Created the radio driver");

    // Write the configuration.
    if let Err(_) = radio.configure( RADIOCONFIG ).await {
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

    // Set the radio behaviour.
    R1BEHAVIOUR.signal(RadioPolicy::Transmit);

    // Set in RX mode.
    //radio.txmode().await;

    // Start running the radio driver.
    radio.run(&R1BEHAVIOUR, RXErrorPolicy::Skip, TXErrorPolicy::Skip).await;
}



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

    loop {
        // Get the elapsed time.
        let elapsed = start.elapsed();

        // Check if it is time to report the bitrate.
        if elapsed > INTERVAL {
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
    let mut packet = [0; PACKETSIZE];


    // Initialize the buffer.
    let mut i = 0;
    for byte in &mut packet {
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

    loop {
        // Get the elapsed time.
        let elapsed = start.elapsed();

        // Check if it is time to report the bitrate.
        if elapsed > INTERVAL {
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
        if let Err( error ) = pipe.send( &packet, true ).await {
            //defmt::error!("TX pipe : Failed to send a packet: {}", error);
        }

        // Increase the byte counter.
        bytes += packet.len();

        embassy_futures::yield_now().await;
    }
}
