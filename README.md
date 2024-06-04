# Introduction to Rust programming on bare metal hardware

Rust code accompanying a presentation showing how to:

- compile and run firmware on RP2040
- make a USB CDC compatible device on RP2040
- communicate between host to RP2040 using a custom binary protocol
- introduction into RTIC v2

Video of the presentation: [Introduction to Rust programming on bare metal hardware](https://youtu.be/KECu_piSM5s)

## pre-requisites

Ater setting up a working [Rust environment](https://rustup.rs/) including nightly compiler:

```shell
# on Linux install build dependencies
apt install libudev-dev
# install compilation targets
rustup target add thumbv6m-none-eabi
rustup target add --toolchain nightly thumbv6m-none-eabi
# necessary linker
cargo install flip-link
# Useful to creating UF2 images for the RP2040 USB Bootloader
cargo install elf2uf2-rs --locked
# Useful for flashing over the SWD pins using a supported JTAG probe
cargo install probe-run
# most comfortable way of running and looking at logs
cargo install cargo-embed
```

for GDB debugging environment follow [OS Specific instructions](https://docs.rust-embedded.org/book/intro/install.html#os-specific-instructions)

for setting up the second RP2040 as debug probe (or use the [Raspberry Pi Debug Probe](https://www.raspberrypi.com/products/debug-probe/)) see the [Pico Getting Started Guide](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) "Appendix A: Using Picoprobe"

## overview of code

- `blink-host` host client for the serial port over USB protocol
- `blink-proto` the protocol implementation for host using `std` and RP2040 `no_std`
- `rp2040-blink` the classic hello world of embedded blinking a LED
- `rp2040-rtic-usb-serial-blinky` the usb-serial communication firmware using RTIC v2 alpha (needs rust nightly)
