//! This example shows the receiver functionality of the NRF24 radio driver.
//! You can use and modify this example to test radio communication, reliability
//! and signal strength in various configurations.
//! 
//! PINOUT :
//! 
//! NRF Device:
//!   - MISO : GPIO 0
//!   - CLK  : GPIO 2
//!   - MOSI : GPIO 3
//!   - CSN  : GPIO 4
//!   - CE   : GPIO 5
//!   - IRQ  : GPIO 6



#![no_std]
#![no_main]

#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

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

    peripherals::SPI0,

    spi::{
        Async, Config, Spi,
    },
};

// Import the necessary SYNC abstractions.
use embassy_sync::{
    mutex::Mutex,
    blocking_mutex::raw::CriticalSectionRawMutex,
    signal::Signal,
};

// Import the timer abstractions.
use embassy_time::{
    Duration, Instant, Timer,
};

// Import the SPI device abstraction.
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;

// Import radio.
use nrf24radio::{
    RXErrorPolicy, TXErrorPolicy, RadioPolicy,

    pipe::RXDataPipe,
};

// Import the panic implementation.
use panic_probe as _;



/// SPI Global for the SPI0 Bus mutex.
/// This allows multiple tasks to access the SPI bus through `SpiDevice`s.
static mut SPI0GLOBAL: MaybeUninit<Mutex<CriticalSectionRawMutex, Spi<'static, SPI0, Async>>> = MaybeUninit::uninit();

/// Radio 0 policy control.
static R0BEHAVIOUR: Signal<CriticalSectionRawMutex, RadioPolicy> = Signal::new();

/// RX Bitrate measurement flag.
static mut RXTRIGGER: bool = false;

/// RX Bitrate measurement flag.
static mut TXTRIGGER: bool = false;


#[main]
async fn main(spawner: Spawner) {
    defmt::info!("Main task started");

    // Get the peripherals. Default configuration.
    let p = init( Default::default() );

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
        match spawner.spawn( rx(spidevice, ce, irq, spawner.clone()) ) {
            Err(_) => defmt::error!("Failed to spawn RX task"),
            _ => defmt::info!("Spawned RX task"),
        }

        // Spawn the LED task.
        match spawner.spawn( led( Output::new( p.PIN_25, Level::High ) ) ) {
            Err(_) => defmt::error!("Failed to spawn LED task"),
            _ => defmt::info!("Spawned LED task"),
        }
    }
}


#[task]
async fn led(mut led: Output<'static>) {
    loop {
        Timer::after( Duration::from_secs(1) ).await;

        led.toggle();
    }
}


#[task]
#[allow(warnings)]
async fn rx(spi: SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI0, Async>, Output<'static>>, ce: Output<'static>, irq: Input<'static>, spawner: Spawner) {
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
    let mut radio: Driver<_, _, Input<'_>> = match Driver::create(spi, ce, Some(irq), rxpipes, txpipe).await {
        Err(_) => {
            defmt::error!("RX : Failed to create radio driver");
            return;
        },

        Ok(r) => r,
    };

    defmt::info!("RX : Created the radio driver");


    if let Ok(status) = radio.status(true).await {
        defmt::info!("Device status: {:b}", status);
    }

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

    R0BEHAVIOUR.signal( RadioPolicy::ReceiveUntilTransmit );

    // Run the radio driver.
    radio.run(&R0BEHAVIOUR, RXErrorPolicy::Skip, TXErrorPolicy::SkipUnlessMaxRetries).await;
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
