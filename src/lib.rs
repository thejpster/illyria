//! # Illyria
//!
//! Implements a stop-and-wait ARQ using postcard + COBS as a serialisation mechanism.
//!
//! See README.md for more details.

/// Object for holding protocol state.
pub struct Illyria<T>
where
    T: embedded_hal::serial::Write<u8>,
{
    transport: T,
    buffer: [u8; 64],
    state: State,
}

enum State {
    Idle,
    SendingInitialZero { len: usize },
    Sending { sent: usize, len: usize },
}

#[derive(Debug, Copy, Clone)]
struct Checksum(u16);

impl Checksum {
    fn generate(data: &[u8]) -> Checksum {
        let result = Checksum(crc::crc16::checksum_x25(data));
        println!("Checksum of {:?} is {}/0x{:04x}", data, result.0, result.0);
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

impl<T> Illyria<T>
where
    T: embedded_hal::serial::Write<u8>,
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

    pub fn new(transport: T) -> Illyria<T> {
        Illyria {
            transport,
            buffer: [0u8; 64],
            state: State::Idle,
        }
    }

    pub fn space(&self) -> usize {
        self.buffer.len() - Self::COBS_OVERHEAD
    }

    pub fn send<M>(&mut self, message: &M) -> Result<(), ()>
    where
        M: serde::ser::Serialize,
    {
        let _err = self.transport.flush();
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
                    Err(())
                }
            }
            Err(_error) => Err(()),
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
                println!("Setting idx {} to {}", last_idx, gap_to_zero);
                last_idx = i;
            }
        }
        let gap_to_zero = 1 + len - last_idx;
        self.buffer[last_idx] = gap_to_zero as u8;
        println!("Setting idx {} to {}", last_idx, gap_to_zero);
        len + 1
    }

    pub fn reset(&mut self) {
        self.state = State::Idle;
    }

    pub fn poll(&mut self) -> nb::Result<(), ()> {
        match self.state {
            State::Idle => {
                // Do nothing
                Ok(())
            }
            State::SendingInitialZero { len } => {
                self.transmit_byte(0x00)?;
                self.state = State::Sending { sent: 0, len };
                Ok(())
            }
            State::Sending { sent, len } => {
                // Send the complete frame
                let b = self.buffer[sent];
                self.transmit_byte(b)?;
                let new_sent = sent + 1;
                if new_sent == len {
                    self.state = State::Idle;
                } else {
                    self.state = State::Sending {
                        sent: new_sent,
                        len,
                    };
                }
                Ok(())
            }
        }
    }

    fn transmit_byte(&mut self, byte: u8) -> nb::Result<(), ()> {
        match self.transport.write(byte) {
            Ok(_) => Ok(()),
            Err(nb::Error::WouldBlock) => {
                // Try again later
                Err(nb::Error::WouldBlock)
            }
            Err(nb::Error::Other(_e)) => Err(nb::Error::Other(())),
        }
    }

    pub fn access_transport(&mut self) -> &mut T {
        &mut self.transport
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nb;
    use serde::Serialize;

    struct TestTransport {
        out_buffer: Vec<u8>,
    }

    impl TestTransport {
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
    }

    impl embedded_hal::serial::Write<u8> for TestTransport {
        type Error = ();

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
        let t = TestTransport {
            out_buffer: Vec::new(),
        };

        let mut illyria = Illyria::new(t);

        illyria.send(&Message::A).unwrap();
        for _ in 0..100 {
            illyria.poll().unwrap();
        }
        illyria.access_transport().check(&[
            0,                                // COBS delimiter
            3,                                // Gap to next zero
            Illyria::<TestTransport>::IFRAME, // Frame type
            1,                                // Length
            3,                                // Payload 0
            0x85,                             // Checksum 0
            0xC8,                             // Checksum 1
        ]);
    }

    #[test]
    fn encode_b() {
        let t = TestTransport {
            out_buffer: Vec::new(),
        };

        let mut illyria = Illyria::new(t);

        illyria.send(&Message::B(0x06070809)).unwrap();
        for _ in 0..100 {
            illyria.poll().unwrap();
        }
        illyria.access_transport().check(&[
            0,                                // COBS delimiter
            10,                               // Gap to next zero
            Illyria::<TestTransport>::IFRAME, // Frame type
            5,                                // Length
            1,                                // Payload 0
            9,                                // Payload 1
            8,                                // Payload 2
            7,                                // Payload 3
            6,                                // Payload 4
            0x1B,                             // Checksum 0
            0xF9,                             // Checksum 1
        ]);
    }

    #[test]
    fn encode_c() {
        let t = TestTransport {
            out_buffer: Vec::new(),
        };

        let mut illyria = Illyria::new(t);

        illyria.send(&Message::C(true)).unwrap();
        for _ in 0..100 {
            illyria.poll().unwrap();
        }
        illyria.access_transport().check(&[
            0,                                // COBS delimiter
            7,                                // Gap to next zero
            Illyria::<TestTransport>::IFRAME, // Frame type
            2,                                // Length
            2,                                // Payload 0
            1,                                // Payload 1
            0x77,                             // Checksum 0
            0xE4,                             // Checksum 1
        ]);
    }

    #[test]
    fn encode_too_big() {
        let t = TestTransport {
            out_buffer: Vec::new(),
        };
        let mut illyria = Illyria::new(t);
        assert!(illyria.send(&Message::D([0; 16])).is_err());
    }
}
