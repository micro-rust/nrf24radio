# NRF24radio

`nrf24radio` is an async, `no-std` and executor agnostic device driver for NRF24L01 and NRF24L01+ devices.

It provides both high level and low level APIs to interact with the radio devices.

## How to install

To add this library to your project simple execute the `cargo` command 

`cargo add nrf24radio`

## Usage

The drivers are split into two main abstractions: driver handle and data pipes. This split decouples the driver logic from the data transmission logic, simplifying the development process.

Please see our showcase examples [here](./examples/) for some common use cases of this library.

To run an example navigate to the folder of the MCU you are using (e.g. `rp` for RP2040) and run the command

`cargo run --release --bin <example>`

to run the example using `probe-run` or

`cargo build --release --bin <example>`

to build it and the run it with your runner of choice.

### Driver handles

Driver handles contain all the logic to configure and interact with the NRF24 device. They can set the RF configuration, create data pipes and handle errors in the device, but do not interact with the actual data being transmitted.

Driver handle tasks are more complex tasks that require some care when interacting with the devices. There are examples of these tasks in the [examples](./examples/) folder.

### Data pipes

Data pipes are simplified abstraction over buffers that can send or receive data through the NRF24 devices. These pipes have a simple API to interact with this data and be notified of errors during communication.

```Rust
async fn send(mut pipe: TXDataPipe) {
    // Buffer to send.
    let buf = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F];

    // This flag indicates if the radio must expect and acknowledgement from the receiver.
    let ack = true;

    // Send the data packet.
    if let Err(_) = pipe.send( &buf, ack ).await {
        // If there was an error handle it here.
    }
}
```

## WIP

 * Better control over TX and RX0 configuration
 * Change TX address dynamically
 * Change RF channels dynamically
 * Noise detection 

## License and Contributing

This library and its examples are licensed under
 * Mozilla Public License 2.0 [LICENSE-MPL](LICENSE-MPL)

Contributions to this project will fall under the same license as defined above.
