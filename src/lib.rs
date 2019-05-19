//! # Illyria
//!
//! Implements a stop-and-wait ARQ using postcard + COBS as a serialisation mechanism.
//!
//! See README.md for more details.
#![cfg_attr(not(test), no_std)]

/// Object for holding protocol state.
pub struct Illyria<TX>
where
    TX: embedded_hal::serial::Write<u8>,
    TX::Error: core::fmt::Debug,
{
    writer: TX,
    buffer: [u8; 66],
    state: State,
}

/// The possible errors Illyria can return
#[derive(Debug)]
pub enum Error<TE>
where
    TE: core::fmt::Debug,
{
    WouldBlock,
    MessageTooLarge,
    Postcard(postcard::Error),
    Transport(TE),
}
impl<TE> From<TE> for Error<TE> where TE: core::fmt::Debug {
    fn from(error: TE) -> Error<TE> {
        Error::Transport(error)
    }
}

enum State {
    Idle,
    SendingInitialZero { len: usize },
    Sending { sent: usize, len: usize },
    SendingFinalZero,
}

#[derive(Debug, Copy, Clone)]
struct Checksum(u16);

impl Checksum {
    fn generate(data: &[u8]) -> Checksum {
        let result = Checksum(crc::crc16::checksum_x25(data));
        result
    }

    fn validate(self, data: &[u8]) -> bool {
        crc::crc16::checksum_x25(data) == self.0
    }

    fn first_byte(self) -> u8 {
        (self.0 >> 8) as u8
    }

    fn second_byte(self) -> u8 {
        self.0 as u8
    }
}

impl<TX> Illyria<TX>
where
    TX: embedded_hal::serial::Write<u8>,
    TX::Error: core::fmt::Debug,
{
    const COBS_START_IDX: usize = 0;
    const FRAME_TYPE_IDX: usize = 1;
    const PAYLOAD_LENGTH_IDX: usize = 2;
    const DATA_IDX: usize = 3;

    /// We checksum the payload length, plus 2 bytes (the frame type and the
    /// length byte)
    const CHECKSUM_OVERHEAD: usize = 2;

    /// Frame overhead comprises the checksum overhead, plus two bytes of checksum.
    const FRAME_OVERHEAD: usize = Self::CHECKSUM_OVERHEAD + 2;

    /// Slice overhead is the frame overhead, plus the COBS byte
    const COBS_OVERHEAD: usize = Self::FRAME_OVERHEAD + 1;

    const IFRAME: u8 = 1;
    const ACK: u8 = 2;
    const NACK: u8 = 3;

    pub fn new(writer: TX) -> Illyria<TX> {
        Illyria {
            writer,
            buffer: [0u8; 66],
            state: State::Idle,
        }
    }

    pub fn space(&self) -> usize {
        self.buffer.len() - Self::COBS_OVERHEAD
    }

    pub fn send<M>(&mut self, message: &M) -> Result<(), Error<TX::Error>>
    where
        M: serde::ser::Serialize,
    {
        let _err = self.writer.flush();
        match postcard::to_slice(message, &mut self.buffer[Self::DATA_IDX..]).map(|buf| buf.len()) {
            Ok(payload_len) => {
                if payload_len <= self.space() {
                    // Build a complete frame
                    self.buffer[Self::FRAME_TYPE_IDX] = Self::IFRAME;
                    self.buffer[Self::PAYLOAD_LENGTH_IDX] = payload_len as u8;
                    let checksum = Checksum::generate(
                        &self.buffer[Self::FRAME_TYPE_IDX
                            ..Self::FRAME_TYPE_IDX + payload_len + Self::CHECKSUM_OVERHEAD],
                    );
                    self.buffer[Self::DATA_IDX + payload_len] = checksum.first_byte();
                    self.buffer[Self::DATA_IDX + payload_len + 1] = checksum.second_byte();
                    let slice_length = self.cobs_encode(payload_len + Self::FRAME_OVERHEAD);
                    self.state = State::SendingInitialZero { len: slice_length };
                    Ok(())
                } else {
                    // Message doesn't fit in the buffer when overheads are added
                    Err(Error::MessageTooLarge)
                }
            }
            Err(e) => Err(Error::Postcard(e)),
        }
    }

    /// This only works for payloads under 254 bytes in length - we can't handle
    /// the insertion of extra zeroes required when the buffer is longer than that.
    fn cobs_encode(&mut self, len: usize) -> usize {
        // Need to fill in our first byte as the offset to the first zero then
        // replace each zero with the offset to the next zero.
        let mut last_idx = Self::COBS_START_IDX;
        for i in (Self::COBS_START_IDX + 1)..len {
            let b = self.buffer[i];
            if b == 0 {
                let gap_to_zero = i - last_idx;
                self.buffer[last_idx] = gap_to_zero as u8;
                last_idx = i;
            }
        }
        let gap_to_zero = 1 + len - last_idx;
        self.buffer[last_idx] = gap_to_zero as u8;
        len + 1
    }

    pub fn reset(&mut self) {
        self.state = State::Idle;
    }

    fn writer_write(&mut self, byte: u8) -> Result<(), Error<TX::Error>> {
        match self.writer.write(byte) {
            Ok(()) => Ok(()),
            Err(nb::Error::WouldBlock) => {
                Err(Error::WouldBlock)
            }
            Err(nb::Error::Other(e)) => {
                Err(Error::Transport(e))
            }
        }
    }

    pub fn poll(&mut self) -> Result<(), Error<TX::Error>> {
        self.state = match self.state {
            State::Idle => {
                // Do nothing
                State::Idle
            }
            State::SendingInitialZero { len } => {
                self.writer_write(0x00)?;
                State::Sending { sent: 0, len }
            }
            State::Sending { sent, len } => {
                // Send the complete frame
                let b = self.buffer[sent];
                self.writer_write(b)?;
                let new_sent = sent + 1;
                if new_sent == len {
                    State::SendingFinalZero
                } else {
                    State::Sending {
                        sent: new_sent,
                        len,
                    }
                }
            }
            State::SendingFinalZero => {
                self.writer_write(0x00)?;
                State::Idle
            }
        };
        Ok(())
    }

    pub fn access_writer(&mut self) -> &mut TX {
        &mut self.writer
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nb;
    use serde::Serialize;

    #[derive(Debug)]
    struct TestWriter {
        out_buffer: Vec<u8>,
    }

    impl TestWriter {
        fn check(&self, expected: &[u8]) {
            assert_eq!(self.out_buffer, expected);
        }
    }

    #[derive(Serialize)]
    enum Message {
        A,
        B(u32),
        C(bool),
        D([u32; 16]),
        E([u32; 15]),
    }

    impl embedded_hal::serial::Write<u8> for TestWriter {
        type Error = u32;

        fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
            self.out_buffer.push(byte);
            Ok(())
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            self.out_buffer = Vec::new();
            Ok(())
        }
    }

    #[test]
    fn encode_a() {

        let t = TestWriter {
            out_buffer: Vec::new(),
        };

        let mut illyria = Illyria::new(t);

        illyria.send(&Message::A).unwrap();
        for _ in 0..100 {
            illyria.poll().unwrap();
        }
        illyria.access_writer().check(&[
            0,                                // COBS delimiter
            3,                                // Gap to next zero
            Illyria::<TestWriter>::IFRAME,    // Frame type
            1,                                // Length
            3,                                // Payload 0
            0x85,                             // Checksum 0
            0xC8,                             // Checksum 1
            0,                                // COBS delimiter
        ]);
    }

    #[test]
    fn encode_b() {
        let t = TestWriter {
            out_buffer: Vec::new(),
        };

        let mut illyria = Illyria::new(t);

        illyria.send(&Message::B(0x06070809)).unwrap();
        for _ in 0..100 {
            illyria.poll().unwrap();
        }
        illyria.access_writer().check(&[
            0,                                // COBS delimiter
            10,                               // Gap to next zero
            Illyria::<TestWriter>::IFRAME,    // Frame type
            5,                                // Length
            1,                                // Payload 0
            9,                                // Payload 1
            8,                                // Payload 2
            7,                                // Payload 3
            6,                                // Payload 4
            0x1B,                             // Checksum 0
            0xF9,                             // Checksum 1
            0,                                // COBS delimiter
        ]);
    }

    #[test]
    fn encode_c() {
        let t = TestWriter {
            out_buffer: Vec::new(),
        };

        let mut illyria = Illyria::new(t);

        illyria.send(&Message::C(true)).unwrap();
        for _ in 0..100 {
            illyria.poll().unwrap();
        }
        illyria.access_writer().check(&[
            0,                                // COBS delimiter
            7,                                // Gap to next zero
            Illyria::<TestWriter>::IFRAME, // Frame type
            2,                                // Length
            2,                                // Payload 0
            1,                                // Payload 1
            0x77,                             // Checksum 0
            0xE4,                             // Checksum 1
            0,                                // COBS delimiter
        ]);
    }

    #[test]
    fn encode_full() {
        let t = TestWriter {
            out_buffer: Vec::new(),
        };
        let mut illyria = Illyria::new(t);
        illyria.send(&Message::E([0; 15])).unwrap();
        for _ in 0..100 {
            illyria.poll().unwrap();
        }
        // Don't care what this looks like, just that it fits OK
    }

    #[test]
    fn encode_too_big() {
        let t = TestWriter {
            out_buffer: Vec::new(),
        };
        let mut illyria = Illyria::new(t);
        assert!(illyria.send(&Message::D([0; 16])).is_err());
    }
}
