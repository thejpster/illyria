# Illyria - a stop-and-wait ARQ using postcard + COBS as a serialisation mechanism

Imagine you have two systems connected via a byte-pipe, e.g. a standard UART.
If you want to send messages between those two systems, the message need to be
converted to and from bytes in a way both sides agree on. You'll also need a
way to unambiguously mark the start and/or end of each message, to avoid
confusion. Further, if your link happens to be a bit 'lossy' or prone to
corruption, you need some mechanism to detect errors and cause bad messages to
be re-sent.

In the past, protocols such as SLIP have handled the messaging, but not the
retries. TCP is a protocol for handling the retries, but is only designed to
work over IP, not a raw UART link (and sending IP frames over a UART between
two systems is not an efficient use of bandwidth).

Illyria is a Rust crate which takes a layered approach to this problem.

## The Layers

### Overview

Illyria is a stack comprised of three layers (plus an Application on top and a
Transport at the bottom). It is symmetrical - there no primary or secondary
side - and so either side is free to send messages as they see fit. Of course,
for your Application it may make sense to have a primary side and a secondary
side (for example where the primary side speaks first, the secondary side only
responds when spoken to), but Illyria doesn't mandate this.

```
    +-----------------------------------+
    | Application                       |
    |                                   |
    |                                   |
    +-----------------------------------+
      | Send(Message)                 ^
      v              Receive(Message) |
 +=========================================+
||    |          Illyria:             |    ||
||  +-----------------------------------+  ||
||  | Layer 3 - Postcard                |  ||
||  |                                   |  ||
||  |                                   |  ||
||  +-----------------------------------+  ||
||    | Send(Byte Slice)              ^    ||
||    v              Recv(Byte Slice) |    ||
||  +-----------------------------------+  ||
||  | Layer 2 - Burkle                  |  ||
||  |                                   |  ||
||  |                                   |  ||
||  +-----------------------------------+  ||
||    | Send(Frame)                   ^    ||
||    v                   Recv(Frame) |    ||
||  +-----------------------------------+  ||
||  | Layer 1 - COBS                    |  ||
||  |                                   |  ||
||  |                                   |  ||
||  +-----------------------------------+  ||
||    |                               |    ||
 +=========================================+
      | Send(Byte)                    ^
      v                    Recv(Byte) |
    +-----------------------------------+
    | Layer 0 - Serial Transport        |
    |                                   |
    |                                   |
    +-----------------------------------+
```

### Layer 0 - Serial Transport

Layer 0 is our transport. For Illyria, it must be something that implements
the `embedded-hal::Serial` trait.

```
trait Serial {
	type Error;

	fn read(&mut self) -> nb::Result<u8, Self::Error>;
	fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error>;
}
```

For now, Illyria must poll the transport to see if there are any new bytes
available, or if there is any space to send some bytes. In the future, we
might have some fancier scheme for determining when it is appropriate to poll
the transport (e.g. by receiving an interrupt from the Serial hardware).

### Layer 1 - COBS Framing

Layer 1 takes in byte slices from above and emits bytes to the transport, one
at a time. Conversely, it takes in individual bytes (which may arrive some
time apart), and provides a complete byte slice to the layer above when the
message is complete. Layer 1 requires some storage space to assemble the
incoming message, which can be setup when your Illyria object is created.
Framing is perfomed using [Consistent Overhead Byte
Stuffing](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing),
which is a much more efficient scheme than the escaping mechanism used in SLIP
and HDLC.

### Layer 2 - Burkle Stop-and-Wait ARQ

Layer 2 organises the retries and acknowledgements. It takes byte slices, and
sends down those byte slices to Layer 1 (perhaps more than once). Conversely
it takes in byte slices from Layer 1 and sends some subset of those up to
Layer 3. It can send/receive two sorts of frame:

* I-Frame - these frames contain information (a payload formed of zero or more
  bytes)
* S-Frame - these frames contain metadata about the link (such as ACK
  or NAK)

These names are taken from
[HDLC](https://en.wikipedia.org/wiki/High-Level_Data_Link_Control), but the
implementation is different, and absolutely not HDLC compatible.

Note: This isn't a standard - it's something I've just made up.

#### Frame Format

The S/I-Frame format is as follows, where each `[ x ]` is a byte:

```
[ header ] [ length ] [ payload0 ] [ payload1 ] ... [ payloadN ] [ checksumUpper ] [ checksumLower ]
```

The header bytes are:

1. I-Frame - contains payload data
2. ACK S-Frame - confirms that the most recent I-Frame received by the sender was valid
3. NACK S-Frame - indicates that the most recent I-Frame received by the send was corrupted and should be re-sent

The length is a value from 0 to 255, and indicates how many payload bytes
follow (`0` to `N`, where `N` is `length - 1`).

The checksum is the X25 CRC16, expressed as two bytes in big-endian fashion.
The checksum is over the entire Burkle payload, from the `[header]` to the
last payload byte (if any).

#### Timeouts

Of course, it is possible for an ACK or NACK S-Frame to be lost or corrupted. The sender of an I-Frame will therefore wait for a period of time for an ACK to be received. If a NACK is received, or too long is spent waiting, the I-Frame will be resent. This continues indefinitely.

#### Example Frame 1

This is an I-Frame, length 3, with the ASCII/UTF-8 payload "123".

```
[ 0x01, 0x03, 0x31, 0x32, 0x33, 0xBB, 0x86 ]
```

#### Example Frame 2

This is an ACK S-Frame, length 0.

```
[ 0x02, 0x00, 0x3C, 0xF7 ]
```

### Layer 3

Layer 3 is the public API to the library. It takes in messages which implement
`serdes::Serialise` and passes up messages which implement
`serdes::Deserialise`. Byte slices are created for sending down to Layer 3
using the [postcard](https://docs.rs/postcard) crate.

## Using Illyria

Illyria has two main methods `run_rx` and `run_tx`. Both must be called regularly. If you want a basic system running, just call them both in your main loop:

```
fn main() -> ! {
    let mut i = Illyria::new(rx, tx, 100);
    loop {
        let _ = i.run_tx();
        let _ = i.run_rx();
    }
}
```

If you want to send a message, call the `send` method. As long as the type you pass implements `Serializable` from `serdes`, Illyria can convert it into bytes using `postcard` and attempt to deliver it reliably.

```
let mut illyria = Illyria::new(t, r, 10);
illyria.send(&Message::A).unwrap();
for _ in 0..20 {
    illyria.run_tx().unwrap();
}
```

## Memory

Illyria is a `#![no_std]` crate and does not require `alloc`. When an Illyria
object is created it takes in a mutable byte slice for storage. For a given
slice length `N`, it can send messages up to length `N-4` (or 255, whichever is
the smaller).

Future versions of Illyria might support queuing multiple messages for
transmission, and perhaps even having multiple messages in-flight at one time
(which is better if your transport has a high bandwidth-latency product, e.g.
you are using a modem).

## Change History

### TODO

* More tests
* Example which uses a pair of serial ports with flow control
* Example which uses a pair of serial ports with random byte loss
* Example which uses a pair of serial ports with random byte corruption
* Example which uses a loopback TCP socket
* Colouring packets (Red, Blue or Purple) so retries can be detected and reboots can be tolerated
* Add a function which says whether Illyria is waiting for an ACK or if the last message was sent OK. You can then spin on this if you just want to send a message in a blocking fashion.

### Unreleased Changes

* Can serialise to a transport, with retries.
* Parser can receive and de-COBS, but doesn't run the bytes through Postcard (yet).

## Trivia

Illyria was a demon in the 1999-2004 TV series 'Angel'. Illyria inhabited the
body of the character Winifred 'Fred' Burkle, and both were played by Amy Acker.
