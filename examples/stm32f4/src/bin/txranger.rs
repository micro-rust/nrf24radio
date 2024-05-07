//! This example allows the user to test the range limits of the NRF24 radio using two STM32F411s.
//! 
//! This examples is for the ranger transmitter. This device will continuously transmit
//! signed packets that the receiver will attempt to read while informing the user of
//! the link quality.
//! 
//! PINOUT : 
//! 
//! NRF Device :
//!   - IRQ  : PD7
//!   - CE   : PB7
//!   - CSN  : PB6
//!   - CLK  : PB3
//!   - MISO : PB4
//!   - MOSI : PB5



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
use embassy_stm32::{
    init,

    exti::ExtiInput,

    gpio::{
        Input, Level, Output, Pull, Speed,
    },

    peripherals::{
        SPI1,

        DMA2_CH2, DMA2_CH3,

        PB6, PB7, PD7, PD13,
    },

    spi::{
        Config, Spi,
    },

    time::Hertz,
};

// Import the necessary SYNC abstractions.
use embassy_sync::{
    mutex::Mutex,
    blocking_mutex::raw::CriticalSectionRawMutex,
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
    Error,

    pipe::TXDataPipe,
};

// Import the panic implementation.
use panic_probe as _;



/// SPI Global for the SPI1 Bus mutex.
/// This allows multiple tasks to access the SPI bus through `SpiDevice`s.
static mut SPIGLOBAL: MaybeUninit<Mutex<CriticalSectionRawMutex, Spi<'static, SPI1, DMA2_CH3, DMA2_CH2>>> = MaybeUninit::uninit();



#[main]
async fn main(spawner: Spawner) {
    defmt::info!("Main task started");

    // Get the peripherals. Default configuration.
    let p = init( Default::default() );

    // These are the pins needed for the SPI interface.
    let miso = p.PB4;
    let sck  = p.PB3;
    let mosi = p.PB5;

    // The CS pin has to become an Output. This is because the Driver does
    // not use a `SpiBus` interface (which would block any other shared use
    // of the SPI peripheral) but instead uses a shared `SpiDevice`.
    let cs = Output::new( p.PB6, Level::High, Speed::High );

    // The CE pin is the Chip Enable pin of the NRF24L01(+).
    // This pin is active high, so we create it active low.
    let ce = Output::new( p.PB7, Level::Low, Speed::High );

    // The IRQ pin is optional but recommended. Providing this pin will increase
    // performance and reduce power consumption.
    // This is optional in order to reduce the use of pins in low pin count MCUs.
    let irq = ExtiInput::new( Input::new( p.PD7, Pull::None ), p.EXTI7 );

    // Get the DMA channels.
    // The Async interface can also be done with interrupts if no channels are available.
    let tx_dma = p.DMA2_CH3;
    let rx_dma = p.DMA2_CH2;

    // Create the SPI configuration.
    // We set it at the maximum speed, but if you are getting an unstable driver
    // try to reduce the SPI speed.
    let mut config = Config::default();
    config.frequency = Hertz( 10_000_000 );

    // Create the SPI.
    let spi = Spi::new( p.SPI1, sck, mosi, miso, tx_dma, rx_dma, config );

    // Store the SPI0 global mutex.
    unsafe { SPIGLOBAL.write( Mutex::new( spi ) ) };
    defmt::info!("Created TX SPI global");

    // Create the SPI device.
    // You can create more of these devices with the same global.
    let spidevice = SpiDevice::new( unsafe { &mut *SPIGLOBAL.as_mut_ptr() }, cs );
    defmt::info!("Created TX shared SPI device");

    // Create the LED.
    let led = Output::new( p.PD13, Level::High, Speed::High );

    // Spawn the TX task.
    match spawner.spawn( tx(spidevice, ce, irq, led, spawner.clone()) ) {
        Err(_) => defmt::error!("Failed to spawn TX task"),
        _ => defmt::info!("Spawned TX task"),
    }
}



#[task]
async fn tx(spi: SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, DMA2_CH3, DMA2_CH2>, Output<'static, PB6>>, ce: Output<'static, PB7>, irq: ExtiInput<'static, PD7>, led: Output<'static, PD13>, spawner: Spawner) {
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
    let mut radio: Driver<_, _, ExtiInput<'_, PD7>> = match Driver::create(spi, ce, Some(irq), rxpipes, txpipe).await {
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
        .channel( 64 )
        .datarate( DataRate::High )
        .gain( Gain::Max )
        .crc( CRCBytes::Two );

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
    if let Err(_) = spawner.spawn( txdata(pipe, led) ) {
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
                //Error::MaxRetries => (),
                _ => defmt::error!("TX : Driver error"),
            },
        }
    }
}



/// This is a ficticious task that will send data on the TX pipe.
#[allow(dead_code)]
#[task]
async fn txdata(mut pipe: TXDataPipe, mut group: Output<'static, PD13>) {
    defmt::info!("TX Pipe : Hello from the RX pipe task");

    #[link_section = ".bss.RADIOTEST"]
    static mut STREAM: [u8; 4096] = [0; 4096];

    let mut i = 0;
    for byte in unsafe { &mut STREAM } {
        // Set the byte.
        *byte = i;

        // Increment i.
        match i {
            255 => i = 0,
            _ => i += 1,
        }
    }

    loop {
        // Send packets in 256 groups.
        for i in 0..=255u8 {
            defmt::info!("TX pipe : Sending packet group {}", i);

            // Indicate which group this is on the packets.
            for j in 0..(4096 / 32) {
                unsafe {
                    // Indicate the group number in the first position.
                    STREAM[(j * 32) +  0] = i;

                    // Indicate the group number in the last position.
                    STREAM[(j * 32) + 31] = i;
                }
            }

            // Select here which kind of packet you want to send.
            // This will let you see the effects on bitrate of different buffer sizes.
            // You can also set if you want the receiver to acknowledge the reception of packets.
            if let Err(_) = pipe.send( unsafe { &STREAM }, false ).await {
                defmt::error!("TX pipe : Failed to send a packet");
            }

            // Switch the LED state.
            group.toggle();
        }
    }
}
