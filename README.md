# BlackPill Embassy CDC Modbus

This example demonstrates using the `embassy` framework with the STM32F411CEUx microcontroller on the BlackPill board, operating with a 25 MHz HSE (High-Speed External) clock. The project configures the PLL (Phase-Locked Loop) to achieve the 48 MHz frequency required for USB communication and the CDC (Communications Device Class).

Additionally, a minimal Modbus server is implemented to assist users in rapidly prototyping Rust-based STM32 Modbus applications that utilize CDC.

## Getting Started

```
cargo run --bin blackpill-embassy-cdc-modbus --release --target thumbv7em-none-eabihf
```

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
