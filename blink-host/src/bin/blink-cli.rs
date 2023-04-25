//! minimal client example for writing a client to talk to the embedded device
//! via serial port. shows how to send Ping messages and verify a Pong comes back
use std::env;
use std::io::{self, Read};
use std::time::Instant;

use std::sync::atomic::{AtomicU16, Ordering};
use std::thread;
use std::time::Duration;

use anyhow::Context;

use blink_host as client;
use blink_proto as rp;

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = env::args().skip(1).collect();

    if args.is_empty() {
        println!("USAGE: blink-cli /dev/tty true 800");
        anyhow::bail!("Please provide a serial port as argument (ex: /dev/ttyACM0)");
    }
    let port_name = &args[0];

    let blinking = if let Some(on_off) = args.get(1) {
        on_off.parse().unwrap()
    } else {
        false
    };
    let pause = if let Some(pause) = args.get(2) {
        pause.parse().unwrap()
    } else {
        500
    };

    let mut port = client::connect(port_name).context("Failed to open serial port")?;

    // Clone the port
    let mut cmd_port = port.try_clone().context("Failed to clone")?;
    let message_id = AtomicU16::new(0);

    // Send out a message at an interval
    thread::spawn(move || loop {
        let msg_id = message_id.fetch_add(1, Ordering::SeqCst);
        if msg_id == 5 {
            let led = rp::Message::Led {
                id: msg_id,
                blinking,
                pause,
            };
            if rp::write_msg(&mut cmd_port, led).is_err() {
                eprintln!("cannot write ping to serial port");
            }
            continue;
        }
        let ping = rp::Message::Ping { id: msg_id };
        if rp::write_msg(&mut cmd_port, ping).is_err() {
            eprintln!("cannot write ping to serial port");
        }
        thread::sleep(Duration::from_millis(500));
    });

    // Read bytes from the cloned port
    let mut msg_buf = Vec::<u8>::new();
    let start = Instant::now();
    let mut msgs_received = 0;
    loop {
        let mut buffer: [u8; 1] = [0; 1];
        match port.read(&mut buffer) {
            Ok(bytes) => {
                msg_buf.push(buffer[0]);
                if bytes != 1 {
                    println!("Impossible: {:?}", buffer);
                }
                match rp::parse(&msg_buf) {
                    rp::ParseResult::Found(msg) => {
                        msgs_received += 1;
                        let stop = start.elapsed();
                        match msg {
                            rp::Message::Pong { id } => println!("received Pong id: {id}"),
                            _ => println!("msg #{}: {:?} {}", msgs_received, msg, msg),
                        }
                        if stop.as_secs_f32() > 10. {
                            break;
                        }
                        msg_buf.clear();
                    }
                    rp::ParseResult::Need(_) => continue,
                    rp::ParseResult::HeaderInvalid | rp::ParseResult::DataInvalid => {
                        dbg!(&msg_buf);
                        msg_buf.clear();
                    }
                }
            }
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
            Err(e) => eprintln!("error {:?}", e),
        }
    }
    Ok(())
}
