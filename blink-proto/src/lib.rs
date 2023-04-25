//! Protocol for mcu to host communication
#![cfg_attr(not(feature = "std"), no_std)]
#[cfg(target_os = "none")]
use defmt::Format;
use heapless::Vec;
use postcard::{from_bytes, to_vec};
use serde::{Deserialize, Serialize};

pub const HEADER: &[u8; 4] = b"rZRH";
pub const MSG_BUF_SIZE: usize = 260;

/// Messages that will be exchanged between the Raspberry Pico
/// and the host computer via the USB CDC serial port
#[cfg_attr(target_os = "none", derive(Format))]
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub enum Message {
    Ping { id: u16 },
    Pong { id: u16 },
    Led { id: u16, blinking: bool, pause: u32 },
}

#[cfg(feature = "std")]
impl std::fmt::Display for Message {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Message::Ping { id } => write!(f, "Ping id: {}", id),
            Message::Pong { id } => write!(f, "Pong id: {}", id),
            Message::Led {
                id,
                blinking,
                pause,
            } => write!(f, "Led id: {} blink: {} pause: {}", id, blinking, pause),
        }
    }
}

fn encode(m: Message) -> Result<Vec<u8, 255>, postcard::Error> {
    to_vec(&m)
}

fn decode(bytes: &[u8]) -> Result<Message, postcard::Error> {
    from_bytes(bytes)
}

#[derive(Debug, Eq, PartialEq)]
pub enum ParseResult {
    Found(Message),
    Need(usize),
    HeaderInvalid,
    DataInvalid,
}

pub fn parse(msg_buf: &[u8]) -> ParseResult {
    if msg_buf[0] != HEADER[0] {
        return ParseResult::HeaderInvalid;
    }
    if msg_buf.len() > 4 {
        if &msg_buf[..4] == HEADER {
            let msg_len = msg_buf[4] as usize;
            let offset = HEADER.len() + 1;
            if msg_buf.len() >= offset + msg_len {
                let enc_msg = &msg_buf[offset..offset + msg_len];
                if let Ok(msg) = decode(enc_msg) {
                    return ParseResult::Found(msg);
                } else {
                    return ParseResult::DataInvalid;
                }
            } else {
                return ParseResult::Need((offset + msg_len).saturating_sub(msg_buf.len()));
            }
        } else {
            return ParseResult::HeaderInvalid;
        }
    }
    ParseResult::Need(HEADER.len())
}

#[cfg(feature = "std")]
pub fn write_msg<T: std::io::Write>(file: &mut T, msg: Message) -> std::io::Result<usize> {
    let buf = wrap_msg(msg).map_err(|_| std::io::ErrorKind::InvalidData)?;
    file.write_all(&buf)?;
    Ok(0)
}

pub fn wrap_msg(msg: Message) -> Result<Vec<u8, MSG_BUF_SIZE>, postcard::Error> {
    let data = encode(msg)?;
    let mut buf = Vec::<u8, MSG_BUF_SIZE>::new();
    {
        buf.extend_from_slice(HEADER).unwrap();
        buf.push(data.len() as u8).unwrap();
        buf.extend_from_slice(&data).unwrap();
    }
    Ok(buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse() -> Result<(), postcard::Error> {
        let mut msg_buf: Vec<u8, 16> = Vec::new();
        msg_buf.push(b'X').unwrap();
        assert_eq!(ParseResult::HeaderInvalid, parse(&msg_buf));
        msg_buf.clear();
        msg_buf.extend_from_slice(b"XYZX\x04asdf").unwrap();
        assert_eq!(ParseResult::HeaderInvalid, parse(&msg_buf));
        msg_buf.clear();
        msg_buf.extend_from_slice(HEADER).unwrap();
        assert_eq!(ParseResult::Need(4), parse(&msg_buf));
        msg_buf.push(2).unwrap();
        assert_eq!(ParseResult::Need(2), parse(&msg_buf));
        msg_buf
            .extend_from_slice(&encode(Message::Ping { id: 5 })?)
            .unwrap();
        assert_eq!(&msg_buf, &[114, 90, 82, 72, 2, 0, 5]);
        assert_eq!(ParseResult::Found(Message::Ping { id: 5 }), parse(&msg_buf));
        Ok(())
    }
}
