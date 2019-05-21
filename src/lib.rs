//! # Illyria
//!
//! Implements a stop-and-wait ARQ using postcard + COBS as a serialisation mechanism.
//!
//! See README.md for more details.
//#![cfg_attr(not(test), no_std)]

/// Object for holding protocol state.
pub struct Illyria<TX, RX, TXLEN, RXLEN>
where
    TX: embedded_hal::serial::Write<u8>,
    RX: embedded_hal::serial::Read<u8>,
    TX::Error: core::fmt::Debug,
    RX::Error: core::fmt::Debug,
    RXLEN: heapless::ArrayLength<u8>,
    TXLEN: heapless::ArrayLength<u8>,
{
    poll_limit: u32,
    writer: TX,
    reader: RX,
    tx_buffer: heapless::Vec<u8, TXLEN>,
    sframe_pending: Option<&'static [u8]>,
    rx_buffer: heapless::Vec<u8, RXLEN>,
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
    TransportWouldBlock,
    PacketInFlight,
    MessageTooLarge,
    Postcard(postcard::Error),
    Writer(TXE),
    Reader(RXE),
}

#[derive(Debug, Copy, Clone)]
enum Payload {
    IFrame,
    SFrame(&'static [u8]),
}

#[derive(Debug)]
enum TxState {
    Idle,
    SendingDelimiterStart { payload: Payload },
    SendingCobsHeader { payload: Payload },
    SendingPayload { payload: Payload, sent: usize },
    SendingDelimiterEnd { payload: Payload },
    WaitingForAckNack { num_polls: u32 },
}

#[derive(Debug)]
enum RxState {
    WantFrameDelimiter,
    WantCobsHeader,
    WantFrameType { cobs: u8 },
    WantLength { cobs: u8, frame: u8 },
    WantPayload { cobs: u8, frame: u8, length: usize },
    WantChecksumFirst { cobs: u8, frame: u8 },
    WantChecksumSecond { cobs: u8, frame: u8, csum_first: u8 },
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

impl<TX, RX, TXLEN, RXLEN> Illyria<TX, RX, TXLEN, RXLEN>
where
    TX: embedded_hal::serial::Write<u8>,
    RX: embedded_hal::serial::Read<u8>,
    TX::Error: core::fmt::Debug,
    RX::Error: core::fmt::Debug,
    RXLEN: heapless::ArrayLength<u8>,
    TXLEN: heapless::ArrayLength<u8>,
{
    const FRAME_TYPE_IDX: usize = 0;
    const PAYLOAD_LENGTH_IDX: usize = 1;
    const DATA_IDX: usize = 2;

    /// We checksum the payload length, plus 2 bytes (the frame type and the
    /// length byte)
    const CHECKSUM_OVERHEAD: usize = 2;

    /// Frame overhead comprises the checksum overhead, plus two bytes of
    /// checksum.
    const FRAME_OVERHEAD: usize = Self::CHECKSUM_OVERHEAD + 2;

    const HEADER_IFRAME: u8 = 1;
    const HEADER_ACK: u8 = 2;
    const HEADER_NACK: u8 = 3;

    /// Manually encoded ACK packet, which never changes. We could render it
    /// into the tx_buffer but keeping it separate lets us cache a packet for
    /// TX while we send an ACK.
    const SFRAME_ACK: [u8; 4] = [Self::HEADER_ACK, 0, 0x3C, 0xF7];

    /// Manually encoded NACK packet, which never changes. We could render it
    /// into the tx_buffer but keeping it separate lets us cache a packet for
    /// TX while we send an ACK.
    const SFRAME_NACK: [u8; 4] = [Self::HEADER_NACK, 0, 0x25, 0x2F];

    pub fn new(writer: TX, reader: RX, poll_limit: u32) -> Illyria<TX, RX, TXLEN, RXLEN> {
        Illyria {
            poll_limit,
            writer,
            reader,
            tx_buffer: heapless::Vec::new(),
            sframe_pending: None,
            rx_buffer: heapless::Vec::new(),
            tx_state: TxState::Idle,
            rx_state: RxState::WantFrameDelimiter,
        }
    }

    pub fn space(&self) -> usize {
        self.tx_buffer.capacity()
    }

    pub fn send<M>(&mut self, message: &M) -> Result<(), Error<TX::Error, RX::Error>>
    where
        M: serde::ser::Serialize,
    {
        if self.tx_buffer.len() != 0 {
            return Err(Error::PacketInFlight);
        }
        match self.tx_state {
            TxState::Idle
            | TxState::SendingDelimiterStart {
                payload: Payload::SFrame(_),
                ..
            }
            | TxState::SendingPayload {
                payload: Payload::SFrame(_),
                ..
            }
            | TxState::SendingDelimiterEnd {
                payload: Payload::SFrame(_),
                ..
            } => {
                let _err = self.writer.flush();
                self.tx_buffer
                    .resize_default(self.tx_buffer.capacity())
                    .unwrap();
                match postcard::to_slice(message, &mut self.tx_buffer[Self::DATA_IDX..])
                    .map(|buf| buf.len())
                {
                    Ok(payload_len) => {
                        if payload_len <= self.space() {
                            // Build a complete frame
                            self.tx_buffer[Self::FRAME_TYPE_IDX] = Self::HEADER_IFRAME;
                            self.tx_buffer[Self::PAYLOAD_LENGTH_IDX] = payload_len as u8;
                            let checksum = Checksum::generate(
                                &self.tx_buffer[Self::FRAME_TYPE_IDX
                                    ..Self::FRAME_TYPE_IDX + payload_len + Self::CHECKSUM_OVERHEAD],
                            );
                            self.tx_buffer[Self::DATA_IDX + payload_len] = checksum.first_byte();
                            self.tx_buffer[Self::DATA_IDX + payload_len + 1] =
                                checksum.second_byte();
                            self.tx_buffer.truncate(payload_len + Self::FRAME_OVERHEAD);
                            Ok(())
                        } else {
                            // Message doesn't fit in the tx_buffer when overheads are added
                            Err(Error::MessageTooLarge)
                        }
                    }
                    Err(e) => Err(Error::Postcard(e)),
                }
            }
            _ => Err(Error::PacketInFlight),
        }
    }

    pub fn reset(&mut self) {
        self.tx_state = TxState::Idle;
    }

    fn writer_write(&mut self, byte: u8) -> Result<(), Error<TX::Error, RX::Error>> {
        match self.writer.write(byte) {
            Ok(()) => Ok(()),
            Err(nb::Error::WouldBlock) => Err(Error::TransportWouldBlock),
            Err(nb::Error::Other(e)) => Err(Error::Writer(e)),
        }
    }

    fn reader_read(&mut self) -> Result<u8, Error<TX::Error, RX::Error>> {
        match self.reader.read() {
            Ok(b) => Ok(b),
            Err(nb::Error::WouldBlock) => Err(Error::TransportWouldBlock),
            Err(nb::Error::Other(e)) => Err(Error::Reader(e)),
        }
    }

    pub fn cobs_find_zero(&self, source: &[u8]) -> usize {
        println!("Finding next 0 in {:?}", self.tx_buffer);
        let mut num = source.len();
        for (i, &b) in source.iter().enumerate() {
            if b == 0 {
                num = i;
                break;
            } else if i == 254 {
                num = 254;
                break;
            }
        }
        println!("Found at {}", num);
        num
    }

    pub fn run_tx(&mut self) -> Result<(), Error<TX::Error, RX::Error>> {
        self.tx_state = match self.tx_state {
            TxState::Idle => {
                // Do nothing
                if self.tx_buffer.len() != 0 {
                    TxState::SendingDelimiterStart {
                        payload: Payload::IFrame,
                    }
                } else if let Some(frame) = self.sframe_pending.take() {
                    TxState::SendingDelimiterStart {
                        payload: Payload::SFrame(frame),
                    }
                } else {
                    TxState::Idle
                }
            }
            TxState::SendingDelimiterStart { payload } => {
                self.writer_write(0x00)?;
                TxState::SendingCobsHeader { payload }
            }
            TxState::SendingCobsHeader { payload } => {
                // Count how many bytes up to the first zero byte.
                // And send that number
                let num = match payload {
                    Payload::IFrame => self.cobs_find_zero(&self.tx_buffer),
                    Payload::SFrame(frame) => self.cobs_find_zero(frame),
                };
                self.writer_write(num as u8 + 1)?;
                TxState::SendingPayload { payload, sent: 0 }
            }
            TxState::SendingPayload { payload, sent } => {
                // Send the complete frame
                let source = match payload {
                    Payload::IFrame => &self.tx_buffer,
                    Payload::SFrame(frame) => frame,
                };
                let len = source.len();
                let mut b = source[sent];
                if b == 0 {
                    // Can't send zeros - send gap to next zero instead
                    let num = self.cobs_find_zero(&source[sent + 1..]);
                    b = num as u8 + 1;
                }
                self.writer_write(b)?;
                let new_sent = sent + 1;
                if new_sent == len {
                    TxState::SendingDelimiterEnd { payload }
                } else {
                    TxState::SendingPayload {
                        payload,
                        sent: new_sent,
                    }
                }
            }
            TxState::SendingDelimiterEnd { payload } => {
                self.writer_write(0x00)?;
                match payload {
                    Payload::IFrame => TxState::WaitingForAckNack { num_polls: 0 },
                    Payload::SFrame { .. } => TxState::Idle,
                }
            }
            TxState::WaitingForAckNack { num_polls } => {
                if num_polls >= self.poll_limit {
                    // Poll N times for ack/nack, else retry
                    TxState::SendingDelimiterStart {
                        payload: Payload::IFrame,
                    }
                } else {
                    TxState::WaitingForAckNack {
                        num_polls: num_polls + 1,
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
                RxState::WantFrameDelimiter => RxState::WantFrameDelimiter,
                RxState::WantCobsHeader => RxState::WantFrameType { cobs: next_byte },
                RxState::WantFrameType { cobs } => {
                    let (cobs, next_byte) = Self::check_cobs(cobs, next_byte);
                    self.rx_buffer.push(next_byte).unwrap();
                    RxState::WantLength {
                        cobs,
                        frame: next_byte,
                    }
                }
                RxState::WantLength { cobs, frame } => {
                    let (cobs, next_byte) = Self::check_cobs(cobs, next_byte);
                    self.rx_buffer.push(next_byte).unwrap();
                    if next_byte == 0 {
                        // Zero length - skip the payload
                        RxState::WantChecksumFirst { cobs, frame }
                    } else {
                        // Collect a payload first
                        RxState::WantPayload {
                            cobs,
                            frame,
                            length: next_byte as usize,
                        }
                    }
                }
                RxState::WantPayload {
                    cobs,
                    frame,
                    length,
                } => {
                    if self.rx_buffer.len() == self.rx_buffer.capacity() {
                        // This packet is too long - drop it on the floor
                        RxState::WantFrameDelimiter
                    } else {
                        let (cobs, next_byte) = Self::check_cobs(cobs, next_byte);
                        self.rx_buffer.push(next_byte).unwrap();
                        if self.rx_buffer.len() == length + Self::CHECKSUM_OVERHEAD {
                            RxState::WantChecksumFirst { cobs, frame }
                        } else {
                            RxState::WantPayload {
                                cobs,
                                frame,
                                length,
                            }
                        }
                    }
                }
                RxState::WantChecksumFirst { cobs, frame } => {
                    let (cobs, next_byte) = Self::check_cobs(cobs, next_byte);
                    RxState::WantChecksumSecond {
                        cobs,
                        frame,
                        csum_first: next_byte,
                    }
                }
                RxState::WantChecksumSecond {
                    cobs,
                    frame,
                    csum_first,
                } => {
                    // process packet here
                    let (_cobs, next_byte) = Self::check_cobs(cobs, next_byte);
                    let csum = Checksum(((csum_first as u16) << 8) | next_byte as u16);
                    if csum.validate(&self.rx_buffer) {
                        // Good packet
                        match frame {
                            Self::HEADER_IFRAME => {
                                // 1. Schedule an ACK
                                self.sframe_pending = Some(&Self::SFRAME_ACK);
                                // 2. Return the good packet to the caller
                                // TODO
                            }
                            Self::HEADER_ACK => {
                                if let TxState::WaitingForAckNack { .. } = self.tx_state {
                                    self.tx_state = TxState::Idle;
                                    self.tx_buffer.truncate(0);
                                }
                            }
                            Self::HEADER_NACK => {
                                if let TxState::WaitingForAckNack { .. } = self.tx_state {
                                    self.tx_state = TxState::Idle;
                                    // leave contents in tx_buffer so we re-send
                                }
                            }
                            _ => {
                                // Valid, but not understood. This is a protocol error.
                            }
                        }
                    } else {
                        // Bad packet
                        self.sframe_pending = Some(&Self::SFRAME_NACK);
                    }
                    // Now start over
                    RxState::WantFrameDelimiter
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
    fn timeout_message() {
        let t = TestWriter {
            out_tx_buffer: Vec::new(),
        };

        let r = TestReader {
            source: VecDeque::new(),
        };

        let mut illyria: Illyria<_, _, heapless::consts::U66, heapless::consts::U66> =
            Illyria::new(t, r, 10);

        illyria.send(&Message::A).unwrap();
        for _ in 0..17 {
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
        illyria.access_writer().out_tx_buffer.truncate(0);
        // This should cause a retry
        for _ in 0..11 {
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
    fn rx_message() {
        let t = TestWriter {
            out_tx_buffer: Vec::new(),
        };

        let r = TestReader {
            source: VecDeque::new(),
        };

        let mut illyria: Illyria<_, _, heapless::consts::U66, heapless::consts::U66> =
            Illyria::new(t, r, 10);

        illyria.access_reader().source.push_back(0); // COBS delimiter
        illyria.access_reader().source.push_back(3); // Gap to next zero
        illyria.access_reader().source.push_back(1); // Frame type
        illyria.access_reader().source.push_back(1); // Length
        illyria.access_reader().source.push_back(3); // Payload 0
        illyria.access_reader().source.push_back(0x85); // Checksum 0
        illyria.access_reader().source.push_back(0xC8); // Checksum 1
        illyria.access_reader().source.push_back(0); // COBS delimiter

        for _ in 0..20 {
            illyria.run_tx().unwrap();
            match illyria.run_rx() {
                Ok(()) => {}
                Err(Error::TransportWouldBlock) => {}
                Err(e) => {
                    panic!("Got error {:?}", e);
                }
            }
        }

        illyria.access_writer().check(&[
            0,    // COBS delimiter
            2,    // Gap to next zero
            2,    // Frame type
            3,    // Length
            0x3C, // Checksum 0
            0xF7, // Checksum 1
            0,    // COBS delimiter
        ]);
    }

    #[test]
    fn rx_bad_message() {
        let t = TestWriter {
            out_tx_buffer: Vec::new(),
        };

        let r = TestReader {
            source: VecDeque::new(),
        };

        let mut illyria: Illyria<_, _, heapless::consts::U66, heapless::consts::U66> =
            Illyria::new(t, r, 10);

        illyria.access_reader().source.push_back(0); // COBS delimiter
        illyria.access_reader().source.push_back(3); // Gap to next zero
        illyria.access_reader().source.push_back(1); // Frame type
        illyria.access_reader().source.push_back(1); // Length
        illyria.access_reader().source.push_back(3); // Payload 0
        illyria.access_reader().source.push_back(0xFF); // Checksum 0
        illyria.access_reader().source.push_back(0xC8); // Checksum 1
        illyria.access_reader().source.push_back(0); // COBS delimiter

        for _ in 0..20 {
            illyria.run_tx().unwrap();
            match illyria.run_rx() {
                Ok(()) => {}
                Err(Error::TransportWouldBlock) => {}
                Err(e) => {
                    panic!("Got error {:?}", e);
                }
            }
        }

        illyria.access_writer().check(&[
            0,    // COBS delimiter
            2,    // Gap to next zero
            3,    // Frame type
            3,    // Length
            0x25, // Checksum 0
            0x2F, // Checksum 1
            0,    // COBS delimiter
        ]);
    }

    #[test]
    fn ack_message() {
        let t = TestWriter {
            out_tx_buffer: Vec::new(),
        };

        let r = TestReader {
            source: VecDeque::new(),
        };

        let mut illyria: Illyria<_, _, heapless::consts::U66, heapless::consts::U66> =
            Illyria::new(t, r, 50);

        illyria.send(&Message::A).unwrap();
        for _ in 0..17 {
            illyria.run_tx().unwrap();
            match illyria.run_rx() {
                Ok(()) => {}
                Err(Error::TransportWouldBlock) => {}
                Err(e) => {
                    panic!("Got error {:?}", e);
                }
            }
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
        // Send an ACK
        illyria.access_reader().source.push_back(0);
        illyria.access_reader().source.push_back(2);
        illyria.access_reader().source.push_back(2);
        illyria.access_reader().source.push_back(3);
        illyria.access_reader().source.push_back(0x3C);
        illyria.access_reader().source.push_back(0xF7);
        illyria.access_writer().out_tx_buffer.truncate(0);
        // This should not cause a retry because it's been acked
        for _ in 0..50 {
            illyria.run_tx().unwrap();
            match illyria.run_rx() {
                Ok(()) => {}
                Err(Error::TransportWouldBlock) => {}
                Err(e) => {
                    panic!("Got error {:?}", e);
                }
            }
        }
        illyria.access_writer().check(&[]);
    }

    #[test]
    fn nack_message() {
        let t = TestWriter {
            out_tx_buffer: Vec::new(),
        };

        let r = TestReader {
            source: VecDeque::new(),
        };

        let mut illyria: Illyria<_, _, heapless::consts::U66, heapless::consts::U66> =
            Illyria::new(t, r, 50);

        illyria.send(&Message::A).unwrap();
        for _ in 0..17 {
            illyria.run_tx().unwrap();
            match illyria.run_rx() {
                Ok(()) => {}
                Err(Error::TransportWouldBlock) => {}
                Err(e) => {
                    panic!("Got error {:?}", e);
                }
            }
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
        // Send a NACK
        illyria.access_reader().source.push_back(0);
        illyria.access_reader().source.push_back(2);
        illyria.access_reader().source.push_back(3);
        illyria.access_reader().source.push_back(3);
        illyria.access_reader().source.push_back(0x25);
        illyria.access_reader().source.push_back(0x2F);
        illyria.access_writer().out_tx_buffer.truncate(0);
        // This should cause a retry because it's been nacked
        for _ in 0..50 {
            illyria.run_tx().unwrap();
            match illyria.run_rx() {
                Ok(()) => {}
                Err(Error::TransportWouldBlock) => {}
                Err(e) => {
                    panic!("Got error {:?}", e);
                }
            }
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
    fn encode_a() {
        let t = TestWriter {
            out_tx_buffer: Vec::new(),
        };

        let r = TestReader {
            source: VecDeque::new(),
        };

        let mut illyria: Illyria<_, _, heapless::consts::U66, heapless::consts::U66> =
            Illyria::new(t, r, 100);

        illyria.send(&Message::A).unwrap();
        for _ in 0..50 {
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

        let mut illyria: Illyria<_, _, heapless::consts::U66, heapless::consts::U66> =
            Illyria::new(t, r, 100);

        illyria.send(&Message::B(0x06070809)).unwrap();
        for _ in 0..50 {
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

        let mut illyria: Illyria<_, _, heapless::consts::U66, heapless::consts::U66> =
            Illyria::new(t, r, 100);

        illyria.send(&Message::C(true)).unwrap();
        for _ in 0..50 {
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

        let mut illyria: Illyria<_, _, heapless::consts::U66, heapless::consts::U66> =
            Illyria::new(t, r, 100);
        illyria.send(&Message::E([0; 15])).unwrap();
        for _ in 0..50 {
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

        let mut illyria: Illyria<_, _, heapless::consts::U66, heapless::consts::U66> =
            Illyria::new(t, r, 10);
        assert!(illyria.send(&Message::D([0; 16])).is_err());
    }
}
