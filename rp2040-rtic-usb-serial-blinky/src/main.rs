//! example of using RTIC to communicate over serial port and blink LED
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_rtt as _;
use panic_probe as _;

defmt::timestamp! {"{=u64}", {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n as u64
}
}

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {
    use core::mem::MaybeUninit;
    use defmt::*;
    use embedded_hal::digital::v2::ToggleableOutputPin;
    use rp_pico::{
        hal::{
            clocks::init_clocks_and_plls,
            gpio::{pin::bank0::*, Pin, Pins, PushPullOutput},
            usb::UsbBus,
            watchdog::Watchdog,
            Sio,
        },
        XOSC_CRYSTAL_FREQ,
    };
    use rtic_monotonics::rp2040::*;
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::SerialPort;

    type LedPin = Pin<Gpio25, PushPullOutput>;

    #[shared]
    struct Shared {
        led_blink: bool,
        led_pause: u32,
        msg_q: heapless::Deque<blink_proto::Message, 10>,
    }

    #[local]
    struct Local {
        led: LedPin,
        usb_dev: UsbDevice<'static, UsbBus>,
        serial: SerialPort<'static, UsbBus>,
        msg_buf: heapless::Vec<u8, { blink_proto::MSG_BUF_SIZE }>,
    }

    #[init(local = [usb_bus: MaybeUninit<UsbBusAllocator<UsbBus>> = MaybeUninit::uninit()])]
    fn init(c: init::Context) -> (Shared, Local) {
        let mut resets = c.device.RESETS;
        let monotoken = rtic_monotonics::create_rp2040_monotonic_token!();
        Timer::start(c.device.TIMER, &mut resets, monotoken);
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        let led = pins.gpio25.into_push_pull_output();

        let usb_bus = c.local.usb_bus;
        let usb_bus = usb_bus.write(UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        )));
        let serial = SerialPort::new(usb_bus);

        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Bedroom Builds")
            .product("Serial port")
            .serial_number("RTIC")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        let led_blink = true;

        led_blinker::spawn().ok();

        let msg_buf = heapless::Vec::<u8, { blink_proto::MSG_BUF_SIZE }>::new();
        let msg_q = heapless::Deque::<blink_proto::Message, 10>::new();

        info!("Send me a message!");
        (
            Shared {
                led_blink,
                led_pause: 500,
                msg_q,
            },
            Local {
                led,
                usb_dev,
                serial,
                msg_buf,
            },
        )
    }

    #[task(priority = 1, local = [led], shared = [led_blink, led_pause])]
    async fn led_blinker(mut ctx: led_blinker::Context) {
        loop {
            let mut pause = 500;
            (&mut ctx.shared.led_blink, &mut ctx.shared.led_pause).lock(|led_blink, led_pause| {
                if *led_blink {
                    ctx.local.led.toggle().ok();
                }
                pause = *led_pause;
            });
            Timer::delay((pause as u64).millis()).await;
        }
    }

    /// USB interrupt handler
    #[task(binds=USBCTRL_IRQ, local = [serial, usb_dev, msg_buf], shared = [led_blink, led_pause, msg_q])]
    fn on_usb(mut ctx: on_usb::Context) {
        let serial = ctx.local.serial;
        if !ctx.local.usb_dev.poll(&mut [serial]) {
            return;
        }
        let mut buf = [0u8; blink_proto::MSG_BUF_SIZE];
        let msg_buf = ctx.local.msg_buf;
        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                if msg_buf.extend_from_slice(&buf[..count]).is_err() {
                    error!("bufcopy {}", count);
                    msg_buf.clear();
                }
                use blink_proto::ParseResult::*;
                match blink_proto::parse(msg_buf) {
                    Found(msg) => {
                        if let blink_proto::Message::Ping { id } = msg {
                            info!("{:?}", msg);
                            let pong = blink_proto::Message::Pong { id };
                            ctx.shared.msg_q.lock(|q| q.push_back(pong).ok());
                        }
                        if let blink_proto::Message::Led {
                            id,
                            blinking,
                            pause,
                        } = msg
                        {
                            info!("{:?}", msg);
                            ctx.shared.led_blink.lock(|b| *b = blinking);
                            ctx.shared.led_pause.lock(|p| *p = pause);
                            // confirm execution with the same message
                            let pong = blink_proto::Message::Led {
                                id,
                                blinking,
                                pause,
                            };
                            ctx.shared.msg_q.lock(|q| q.push_back(pong).ok());
                        }
                        msg_buf.clear();
                    }
                    Need(b) => {
                        debug!("Need({})", b);
                    } // continue reading
                    HeaderInvalid | DataInvalid => {
                        debug!("invalid");
                        msg_buf.clear();
                    }
                }

                // write back to the host
                loop {
                    let msg = ctx.shared.msg_q.lock(|q| q.pop_front());
                    let bytes = match msg {
                        Some(msg) => match blink_proto::wrap_msg(msg) {
                            Ok(msg_bytes) => msg_bytes,
                            Err(_) => break,
                        },
                        None => break,
                    };
                    let mut wr_ptr = bytes.as_slice();
                    while !wr_ptr.is_empty() {
                        let _ = serial.write(wr_ptr).map(|len| {
                            wr_ptr = &wr_ptr[len..];
                        });
                    }
                }
            }
            _ => {}
        }
    }

    /// Task with least priority that only runs when nothing else is running.
    /// avoids rtic sending the board to sleep
    /*
    #[idle(local = [x: u32 = 0])]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }
    */
    #[idle]
    fn idle(_: idle::Context) -> ! {
        // debug::exit(debug::EXIT_SUCCESS);
        loop {
            // hprintln!("idle");
            cortex_m::asm::wfi(); // put the MCU in sleep mode until interrupt occurs
        }
    }
}
