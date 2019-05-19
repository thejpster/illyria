//! # Illyria
//!
//! Implements a stop-and-wait ARQ using postcard + COBS as a serialisation mechanism.
//!
//! See REEADME.md for more details.
#![cfg_attr(not(test), no_std)]

/// Object for holding protocol state.
pub struct Illyria<TX, RX>
where
    TX: embedded_hal::serial::Write<u8>,
    RX: embedded_hal::serial::Read<u8>,
    TX::Error: core::fmt::Debug,
    RX::Error: core::fmt::Debug,
{
    writer: TX,
    reader: RX,
    tx_buffer: [u8; 66],
    rx_buffer: [u8; 66],
    tx_state: TxState,
    rx_state: RxState,
}

/// The possible errors Illyria can return
#[derive(Debug)]
pub enum Error<TXE, RXE>
where
    TXE: core::fmt::Debug,
    RXE: core::fmt::Debug,
{
    WouldBlock,
    MessageTooLarge,
    Postcard(postcard::Error),
    Writer(TXE),
    Reader(RXE),
}

enum TxState {
    Idle,
    SendingInitialZero { len: usize },
    Sending { sent: usize, len: usize },
    SendingFinalZero { len: usize },
    WaitingForAckNack { len: usize, polls: u32 },
}

enum RxState {
    WantInitialZero,
    WantCobsHeader,
    WantFrameType {
        cobs: u8,
    },
    WantLength {
        cobs: u8,
        frame: u8,
    },
    WantPayload {
        cobs: u8,
        frame: u8,
        length: usize,
        received: usize,
    },
    WantChecksumFirst {
        cobs: u8,
        frame: u8,
        length: usize,
    },
    WantChecksumSecond {
        cobs: u8,
        frame: u8,
        length: usize,
        csum_first: u8,
    },
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

impl<TX, RX> Illyria<TX, RX>
where
    TX: embedded_hal::serial::Write<u8>,
    RX: embedded_hal::serial::Read<u8>,
    TX::Error: core::fmt::Debug,
    RX::Error: core::fmt::Debug,
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

    pub fn new(writer: TX, reader: RX) -> Illyria<TX, RX> {
        Illyria {
            writer,
            reader,
            tx_buffer: [0u8; 66],
            rx_buffer: [0u8; 66],
            tx_state: TxState::Idle,
            rx_state: RxState::WantInitialZero,
        }
    }

    pub fn space(&self) -> usize {
        self.tx_buffer.len() - Self::COBS_OVERHEAD
    }

    pub fn send<M>(&mut self, message: &M) -> Result<(), Error<TX::Error, RX::Error>>
    where
        M: serde::ser::Serialize,
    {
        let _err = self.writer.flush();
        match postcard::to_slice(message, &mut self.tx_buffer[Self::DATA_IDX..])
            .map(|buf| buf.len())
        {
            Ok(payload_len) => {
                if payload_len <= self.space() {
                    // Build a complete frame
                    self.tx_buffer[Self::FRAME_TYPE_IDX] = Self::IFRAME;
                    self.tx_buffer[Self::PAYLOAD_LENGTH_IDX] = payload_len as u8;
                    let checksum = Checksum::generate(
                        &self.tx_buffer[Self::FRAME_TYPE_IDX
                            ..Self::FRAME_TYPE_IDX + payload_len + Self::CHECKSUM_OVERHEAD],
                    );
                    self.tx_buffer[Self::DATA_IDX + payload_len] = checksum.first_byte();
                    self.tx_buffer[Self::DATA_IDX + payload_len + 1] = checksum.second_byte();
                    let slice_length = self.cobs_encode(payload_len + Self::FRAME_OVERHEAD);
                    self.tx_state = TxState::SendingInitialZero { len: slice_length };
                    Ok(())
                } else {
                    // Message doesn't fit in the tx_buffer when overheads are added
                    Err(Error::MessageTooLarge)
                }
            }
            Err(e) => Err(Error::Postcard(e)),
        }
    }

    /// This only works for payloads under 254 bytes in length - we can't handle
    /// the insertion of extra zeroes required when the tx_buffer is longer than that.
    fn cobs_encode(&mut self, len: usize) -> usize {
        // Need to fill in our first byte as the offset to the first zero then
        // replace each zero with the offset to the next zero.
        let mut last_idx = Self::COBS_START_IDX;
        for i in (Self::COBS_START_IDX + 1)..len {
            let b = self.tx_buffer[i];
            if b == 0 {
                let gap_to_zero = i - last_idx;
                self.tx_buffer[last_idx] = gap_to_zero as u8;
                last_idx = i;
            }
        }
        let gap_to_zero = 1 + len - last_idx;
        self.tx_buffer[last_idx] = gap_to_zero as u8;
        len + 1
    }

    pub fn reset(&mut self) {
        self.tx_state = TxState::Idle;
    }

    fn writer_write(&mut self, byte: u8) -> Result<(), Error<TX::Error, RX::Error>> {
        match self.writer.write(byte) {
            Ok(()) => Ok(()),
            Err(nb::Error::WouldBlock) => Err(Error::WouldBlock),
            Err(nb::Error::Other(e)) => Err(Error::Writer(e)),
        }
    }

    fn reader_read(&mut self) -> Result<u8, Error<TX::Error, RX::Error>> {
        match self.reader.read() {
            Ok(b) => Ok(b),
            Err(nb::Error::WouldBlock) => Err(Error::WouldBlock),
            Err(nb::Error::Other(e)) => Err(Error::Reader(e)),
        }
    }

    pub fn run_tx(&mut self) -> Result<(), Error<TX::Error, RX::Error>> {
        self.tx_state = match self.tx_state {
            TxState::Idle => {
                // Do nothing
                TxState::Idle
            }
            TxState::SendingInitialZero { len } => {
                self.writer_write(0x00)?;
                TxState::Sending { sent: 0, len }
            }
            TxState::Sending { sent, len } => {
                // Send the complete frame
                let b = self.tx_buffer[sent];
                self.writer_write(b)?;
                let new_sent = sent + 1;
                if new_sent == len {
                    TxState::SendingFinalZero { len }
                } else {
                    TxState::Sending {
                        sent: new_sent,
                        len,
                    }
                }
            }
            TxState::SendingFinalZero { len } => {
                self.writer_write(0x00)?;
                TxState::WaitingForAckNack { polls: 0, len }
            }
            TxState::WaitingForAckNack { polls, len } => {
                if polls >= 100 {
                    // Poll 100 times for ack/nack, else retry
                    TxState::SendingInitialZero { len }
                } else {
                    TxState::WaitingForAckNack {
                        polls: polls + 1,
                        len,
                    }
                }
            }
        };
        Ok(())
    }

    fn check_cobs(cobs: u8, next_byte: u8) -> (u8, u8) {
        if cobs == 1 {
            (next_byte, 0)
        } else {
            (cobs - 1, next_byte)
        }
    }

    pub fn run_rx(&mut self) -> Result<(), Error<TX::Error, RX::Error>> {
        let next_byte = self.reader_read()?;
        if next_byte == 0 {
            // Applies in any state
            self.rx_state = RxState::WantCobsHeader;
        } else {
            self.rx_state = match self.rx_state {
                RxState::WantInitialZero => RxState::WantInitialZero,
                RxState::WantCobsHeader => RxState::WantFrameType { cobs: next_byte },
                RxState::WantFrameType { cobs } => {
                    let (cobs, next_byte) = Self::check_cobs(cobs, next_byte);
                    RxState::WantLength {
                        cobs,
                        frame: next_byte,
                    }
                }
                RxState::WantLength { cobs, frame } => {
                    let (cobs, next_byte) = Self::check_cobs(cobs, next_byte);
                    if next_byte == 0 {
                        RxState::WantChecksumFirst {
                            cobs,
                            frame,
                            length: 0,
                        }
                    } else {
                        RxState::WantPayload {
                            cobs,
                            frame,
                            length: next_byte as usize,
                            received: 2,
                        }
                    }
                }
                RxState::WantPayload {
                    cobs,
                    frame,
                    length,
                    received,
                } => {
                    if received >= self.rx_buffer.len() {
                        // This packet is too long - drop it on the floor
                        RxState::WantInitialZero
                    } else {
                        let (cobs, next_byte) = Self::check_cobs(cobs, next_byte);
                        self.rx_buffer[received] = next_byte;
                        if received == length + Self::CHECKSUM_OVERHEAD {
                            RxState::WantChecksumFirst {
                                cobs,
                                frame,
                                length,
                            }
                        } else {
                            RxState::WantPayload {
                                cobs,
                                frame,
                                length,
                                received: received + 1,
                            }
                        }
                    }
                }
                RxState::WantChecksumFirst {
                    cobs,
                    frame,
                    length,
                } => {
                    let (cobs, next_byte) = Self::check_cobs(cobs, next_byte);
                    RxState::WantChecksumSecond {
                        cobs,
                        frame,
                        length,
                        csum_first: next_byte,
                    }
                }
                RxState::WantChecksumSecond {
                    cobs,
                    frame,
                    length,
                    csum_first,
                } => {
                    // process packet here
                    let (_cobs, next_byte) = Self::check_cobs(cobs, next_byte);
                    let csum = Checksum(((csum_first as u16) << 8) | next_byte as u16);
                    self.rx_buffer[0] = frame;
                    self.rx_buffer[1] = length as u8;
                    if csum.validate(&self.rx_buffer[0..length + 2]) {
                        // Good packet
                        // 1. Schedule an ACK
                        // 2. Return the good packet to the caller
                    } else {
                        // Bad packet
                        // 1. Schedule an NACK
                    }
                    // Now start over
                    RxState::WantInitialZero
                }
            };
        }
        Ok(())
    }

    pub fn access_writer(&mut self) -> &mut TX {
        &mut self.writer
    }

    pub fn access_reader(&mut self) -> &mut RX {
        &mut self.reader
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nb;
    use serde::Serialize;
    use std::collections::VecDeque;

    #[derive(Debug)]
    struct TestWriter {
        out_tx_buffer: Vec<u8>,
    }

    #[derive(Debug)]
    struct TestReader {
        source: VecDeque<u8>,
    }

    impl TestWriter {
        fn check(&self, expected: &[u8]) {
            assert_eq!(self.out_tx_buffer, expected);
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
        type Error = ();

        fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
            self.out_tx_buffer.push(byte);
            Ok(())
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            self.out_tx_buffer = Vec::new();
            Ok(())
        }
    }

    impl embedded_hal::serial::Read<u8> for TestReader {
        type Error = ();

        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            match self.source.pop_front() {
                Some(b) => Ok(b),
                None => Err(nb::Error::WouldBlock),
            }
        }
    }

    #[test]
    fn encode_a() {
        let t = TestWriter {
            out_tx_buffer: Vec::new(),
        };

        let r = TestReader {
            source: VecDeque::new(),
        };

        let mut illyria = Illyria::new(t, r);

        illyria.send(&Message::A).unwrap();
        for _ in 0..100 {
            illyria.run_tx().unwrap();
        }
        illyria.access_writer().check(&[
            0,    // COBS delimiter
            3,    // Gap to next zero
            1,    // Frame type
            1,    // Length
            3,    // Payload 0
            0x85, // Checksum 0
            0xC8, // Checksum 1
            0,    // COBS delimiter
        ]);
    }

    #[test]
    fn encode_b() {
        let t = TestWriter {
            out_tx_buffer: Vec::new(),
        };

        let r = TestReader {
            source: VecDeque::new(),
        };

        let mut illyria = Illyria::new(t, r);

        illyria.send(&Message::B(0x06070809)).unwrap();
        for _ in 0..100 {
            illyria.run_tx().unwrap();
        }
        illyria.access_writer().check(&[
            0,    // COBS delimiter
            10,   // Gap to next zero
            1,    // Frame type
            5,    // Length
            1,    // Payload 0
            9,    // Payload 1
            8,    // Payload 2
            7,    // Payload 3
            6,    // Payload 4
            0x1B, // Checksum 0
            0xF9, // Checksum 1
            0,    // COBS delimiter
        ]);
    }

    #[test]
    fn encode_c() {
        let t = TestWriter {
            out_tx_buffer: Vec::new(),
        };

        let r = TestReader {
            source: VecDeque::new(),
        };

        let mut illyria = Illyria::new(t, r);

        illyria.send(&Message::C(true)).unwrap();
        for _ in 0..100 {
            illyria.run_tx().unwrap();
        }
        illyria.access_writer().check(&[
            0,    // COBS delimiter
            7,    // Gap to next zero
            1,    // Frame type
            2,    // Length
            2,    // Payload 0
            1,    // Payload 1
            0x77, // Checksum 0
            0xE4, // Checksum 1
            0,    // COBS delimiter
        ]);
    }

    #[test]
    fn encode_full() {
        let t = TestWriter {
            out_tx_buffer: Vec::new(),
        };

        let r = TestReader {
            source: VecDeque::new(),
        };

        let mut illyria = Illyria::new(t, r);
        illyria.send(&Message::E([0; 15])).unwrap();
        for _ in 0..100 {
            illyria.run_tx().unwrap();
        }
        // Don't care what this looks like, just that it fits OK
    }

    #[test]
    fn encode_too_big() {
        let t = TestWriter {
            out_tx_buffer: Vec::new(),
        };

        let r = TestReader {
            source: VecDeque::new(),
        };

        let mut illyria = Illyria::new(t, r);
        assert!(illyria.send(&Message::D([0; 16])).is_err());
    }
}
