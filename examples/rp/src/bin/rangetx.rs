//! Range testing with two RP2040, each with their own NRF24L01 radio.
//! This example contains the TX logic.
//! 
//! PINOUT :
//! 
//! NRF Device:
//!   - CE   : GPIO 0
//!   - CSN  : GPIO 1
//!   - SCLK : GPIO 2
//!   - MOSI : GPIO 3
//!   - MISO : GPIO 4
//!   - IRQ  : GPIO 5
//! 
//! ACK Switch:
//!   - IRQ  : GPIO 6



#![no_std]
#![no_main]

#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]


#![allow(static_mut_refs)]



// Core imports.
use core::mem::MaybeUninit;

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
        SPI0, PWM_SLICE4,
    },

    pwm::{
        Pwm, Config as PWMConfig,
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
    Duration, Timer,
};

// Import the SPI device abstraction.
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;

// Import the SPI device.
//use embedded_hal_async::spi::SpiDevice;

// Import radio.
use nrf24radio::{
    Driver, RadioPolicy, RXErrorPolicy, TXErrorPolicy,

    pipe::{
        DataPipe, TXDataPipe, TXConfig,
    },
};



// Setup the logging framework.
use defmt_rtt as _;

// Import the panic implementation.
use panic_probe as _;



/// WARNING : This is a quick hack for the example, you SHOULD NOT use raw statics.
/// Global ACK request configuration.
static mut ACKNOWLEDGE: bool = true;

/// WARNING : This is a quick hack for the example, you SHOULD NOT use raw statics.
/// SPI Global for the SPI1 Bus mutex.
/// This allows multiple tasks to access the SPI bus through `SpiDevice`s.
static mut SPIGLOBAL: MaybeUninit<Mutex<CriticalSectionRawMutex, Spi<'static, SPI0, Async>>> = MaybeUninit::uninit();

/// Radio policy control.
/// This allows to dynamically change the behaviour of the radio.
static BEHAVIOUR: Signal<CriticalSectionRawMutex, RadioPolicy> = Signal::new();

// Create the configuration of the radio.
// Please note that this configuration MUST be the same as the TX configuration.
// The only thing that can be different is the gain.
// If you want this can be made a constant, as all `Config` methods are `const`.
const RADIOCONFIG: nrf24radio::Config = nrf24radio::Config::new()
        .channel( 64 )
        .datarate( nrf24radio::common::DataRate::High )
        .gain( nrf24radio::common::Gain::High )
        .crc( nrf24radio::common::CRCBytes::One );



/// Main task of the TX range tester.
#[main]
async fn main(spawner: Spawner) {
    defmt::debug!("TX Ranger: Main task started");

    // Get the peripherals. Default configuration.
    let p = init( Default::default() );

    // These are the pins needed for the SPI interface.
    let sclk = p.PIN_2;
    let mosi = p.PIN_3;
    let miso = p.PIN_4;

    // The CS pin has to become an Output. This is because the Driver does
    // not use a `SpiBus` interface (which would block any other shared use
    // of the SPI peripheral) but instead uses a shared `SpiDevice`.
    let cs = Output::new( p.PIN_1, Level::High );

    // The CE pin is the Chip Enable pin of the NRF24L01(+).
    // This pin is active high, so we create it active low.
    let ce = Output::new( p.PIN_0, Level::Low );

    // The IRQ pin is optional but recommended. Providing this pin will increase
    // performance and reduce power consumption.
    // This is optional in order to reduce the use of pins in low pin count MCUs.
    let irq = Input::new( p.PIN_5, Pull::None );

    defmt::debug!("TX Ranger: All pins acquired");

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
    let spi = Spi::new( p.SPI0, sclk, mosi, miso, tx_dma, rx_dma, config );

    // Store the SPI1 global mutex.
    unsafe { SPIGLOBAL.write( Mutex::new( spi ) ) };
    defmt::debug!("TX Ranger: SPI Bus instance created");

    // Create the SPI device.
    // You can create more of these devices with the same global.
    let spidevice = SpiDevice::new( unsafe { &mut *SPIGLOBAL.as_mut_ptr() }, cs );
    defmt::debug!("TX Ranger: SPI Device acquired");

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

    defmt::debug!("TX Ranger: Data buffers created");

    // Create the radio driver.
    // Pay attention to the explicit type given here.
    // If you don't use the IRQ pin Rust will complain that it doesn't know the
    // type of the IRQ pin. To get around this, you have to provide a fake pin
    // type (the `Input<'_>` part).
    let mut radio: Driver<_, _, Input<'_>> = match Driver::create(spidevice, ce, Some(irq), rxpipes, txpipe).await {
        Err(_) => {
            defmt::error!("TX Ranger : Failed to create radio driver");
            return;
        },

        Ok(r) => r,
    };

    defmt::debug!("TX Ranger: NRF24 Radio Driver created");

    // Write the configuration.
    if let Err(_) = radio.configure( RADIOCONFIG ).await {
        defmt::error!("TX Ranger : Failed to configure the driver");
        return;
    }

    defmt::info!("TX Ranger: NRF24 Radio Driver configured");

    // Now we open a data pipe of the radio.
    // A data pipe is a stream of data that can be received on the radio.
    // The radios can receive data in up to 6 pipes with different addresses.
    // This will open Pipe 0.
    // Take into account that Pipe 0 has some restrictions on the configuration allowed
    let config = TXConfig::default();

    // Create the RX data Pipe 0.
    let pipe = match radio.opentx( config ).await {
        Err(_) => {
            defmt::error!("TX Ranger : Failed to open data pipe");
            return;
        },
        Ok(p) => p,
    };

    defmt::info!("TX Ranger: NRF24 Radio RX pipe created");

    // Create the LED PWM output.
    let pwm = Pwm::new_output_b( p.PWM_SLICE4, p.PIN_25, Default::default() );

    // Now that we have a data pipe, we can send it to another task to do whatever it needs.
    // Please remember that at least one task has to handle the driver for the pipes to receive data.
    // The architecture of this driver is one task for driver handling and separate tasks for the pipes.
    if let Err(_) = spawner.spawn( txdata(pipe, pwm) ) {
        defmt::error!("TX Ranger : Failed to spawn data pipe task");
        return;
    }

    // Create a task to switch acknowledge mode.
    if let Err(_) = spawner.spawn( switch( Input::new( p.PIN_6, Pull::Up ) ) ) {
        defmt::error!("TX Ranger : Failed to spawn ACK switch task");
        return;
    }

    //core::sync::atomic::compiler_fence( core::sync::atomic::Ordering::AcqRel );

    // Set the radio behaviour.
    BEHAVIOUR.signal(RadioPolicy::Transmit);

    // Start running the radio driver.
    radio.run(&BEHAVIOUR, RXErrorPolicy::Skip, TXErrorPolicy::SkipUnlessMaxRetries).await;
}



/// This task will send packets through the TX pipe.
#[task]
async fn txdata(mut pipe: TXDataPipe, mut led: Pwm<'static, PWM_SLICE4>) {
    defmt::debug!("TX Ranger : TX Pipe task started");

    // Create the dummy data packets.
    let mut packet = [0; 32];

    // Number of failed packets per iteration.
    let mut failed;

    // PWM configuration.
    let mut pwm = PWMConfig::default();
    pwm.top = 64;

    // Infinite loop.
    loop {
        // Read the ACKNOWLEDGE state for this cycle.
        let ack = unsafe { ACKNOWLEDGE };

        // Reset the failed packet count.
        failed = 0;

        // Send the 64 expected packets.
        for p in 0..64 {
            // Modify the current packet with the number of the packet in the sequence.
            packet.iter_mut().for_each(|byte| *byte = p );

            // Send the packet.
            if let Err( error ) = pipe.send( &packet, ack ).await {
                // Log the error.
                defmt::error!("TX Ranger : Failed to send a packet: {}", error);

                // Increase the number of failed packets.
                failed += 1;
            }
        }

        // Inform of the success rate.
        defmt::info!("Iteration done: {} / 64 packets sent", 64 - failed);

        // Modify the PWM duty cycle.
        pwm.compare_b = 64 - failed;

        // Set the LED light value.
        led.set_config( &pwm );

        // Wait for 1 second before the next round.
        Timer::after( Duration::from_secs( 1 ) ).await
    }
}



/// This task will change the ACK value.
#[task]
async fn switch(mut button: Input<'static>) {
    // Infinite loop.
    loop {
        // Wait for a falling edge.
        button.wait_for_low().await;

        // Change the value of the ACKNOWLEDGE state.
        unsafe { ACKNOWLEDGE = !ACKNOWLEDGE; }

        // Wait for 200 ms before checking again.
        Timer::after( Duration::from_millis( 200 ) ).await
    }
}
