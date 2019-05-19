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
    Sending(usize, usize),
}

impl<T> Illyria<T>
where
    T: embedded_hal::serial::Write<u8>,
{
    pub fn new(transport: T) -> Illyria<T> {
        Illyria {
            transport,
            buffer: [0u8; 64],
            state: State::Idle,
        }
    }

    pub fn send<M>(&mut self, message: &M) -> Result<(), ()>
    where
        M: serde::ser::Serialize,
    {
        let _err = self.transport.flush();
        match postcard::to_slice(message, &mut self.buffer) {
            Ok(buf) => {
                self.state = State::Sending(0, buf.len());
                Ok(())
            }
            Err(_error) => Err(()),
        }
    }

    pub fn poll(&mut self) -> nb::Result<(), ()> {
        match self.state {
            State::Idle => {
                // Do nothing
                Ok(())
            }
            State::Sending(n, len) => {
                match self.transport.write(self.buffer[n]) {
                    Ok(_) => {
                        let new_n = n + 1;
                        if new_n == len {
                            self.state = State::Idle;
                        } else {
                            self.state = State::Sending(n + 1, len);
                        }
                        Ok(())
                    }
                    Err(nb::Error::WouldBlock) => {
                        // Try again later
                        Err(nb::Error::WouldBlock)
                    }
                    Err(nb::Error::Other(_e)) => Err(nb::Error::Other(())),
                }
            }
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
    fn encode() {
        let t = TestTransport {
            out_buffer: Vec::new(),
        };

        let mut illyria = Illyria::new(t);

        illyria.send(&Message::A).unwrap();
        for _ in 0..100 {
            illyria.poll().unwrap();
        }
        illyria.access_transport().check(&[0]);

        illyria.send(&Message::B(0)).unwrap();
        for _ in 0..100 {
            illyria.poll().unwrap();
        }
        illyria.access_transport().check(&[1, 0, 0, 0, 0]);

        illyria.send(&Message::C(true)).unwrap();
        for _ in 0..100 {
            illyria.poll().unwrap();
        }
        illyria.access_transport().check(&[2, 1]);
    }
}
