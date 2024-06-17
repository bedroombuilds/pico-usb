# Introduction to Rust programming on bare metal hardware

Rust code accompanying a presentation showing how to:

- compile and run firmware on RP2040
- make a USB CDC compatible device on RP2040
- communicate between host to RP2040 using a custom binary protocol
- introduction into RTIC v2

Video of the presentation: [Introduction to Rust programming on bare metal hardware](https://youtu.be/KECu_piSM5s)

Code used in the Video is git-tagged with [`presentation-state`](https://github.com/bedroombuilds/pico-usb/tree/presentation-state)

## pre-requisites

After setting up a working [Rust environment](https://rustup.rs/):

```shell
# on Linux install build dependencies
apt install libudev-dev
# install compilation targets
rustup target add thumbv6m-none-eabi
# necessary linker
cargo install flip-link
# Useful to creating UF2 images for the RP2040 USB Bootloader
cargo install elf2uf2-rs --locked
# Useful for flashing over the SWD pins using a supported JTAG probe
# probe-rs-tools also contains cargo-embed a comfortable way of executing firmware and looking at logs
cargo install cargo-binstall
cargo binstall probe-rs-tools
```

For other ways to install `probe-rs` see their [installation docs](https://probe.rs/docs/getting-started/installation/)

for GDB debugging environment follow [OS Specific instructions](https://docs.rust-embedded.org/book/intro/install.html#os-specific-instructions)

for setting up the second RP2040 as debug probe (or use the [Raspberry Pi Debug Probe](https://www.raspberrypi.com/products/debug-probe/)) see the [Pico Getting Started Guide](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) "Appendix A: Using Picoprobe"

## using nightly compiler

Was needed on the day of the presentation, but rustc stable v1.75 handles RTIC v2.1.0 fine, no need anymore to use nightly.
If you still feel like using nightly, the following pre-requisite is needed:

```shell
# nightly does not come by default with core and alloc prebuilt
rustup target add --toolchain nightly thumbv6m-none-eabi
```

## overview of code

- `blink-host` host client for the serial port over USB protocol
- `blink-proto` the protocol implementation for host using `std` and RP2040 `no_std`
- `rp2040-blink` the classic hello world of embedded programming - blinking a LED
- `rp2040-rtic-usb-serial-blinky` the usb-serial communication firmware using RTIC v2
