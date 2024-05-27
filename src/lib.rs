#![deny(unsafe_code)]
#![warn(unused_results)]
#![warn(private_interfaces)]
#![warn(private_bounds)]
#![warn(clippy::empty_line_after_outer_attr)]
#![warn(clippy::manual_filter_map)]
#![warn(clippy::if_not_else)]
#![warn(clippy::mut_mut)]
#![warn(clippy::non_ascii_literal)]
#![warn(clippy::map_unwrap_or)]
#![warn(clippy::use_self)]
#![warn(clippy::used_underscore_binding)]
#![warn(clippy::print_stdout)]
#![warn(clippy::print_stderr)]
#![warn(clippy::map_flatten)]
#![warn(clippy::wildcard_dependencies)]
#![warn(clippy::cargo_common_metadata)]
#![warn(clippy::wildcard_enum_match_arm)]
#![warn(clippy::missing_const_for_fn)]
#![warn(clippy::dbg_macro)]
#![warn(clippy::path_buf_push_overwrite)]
#![warn(clippy::manual_find_map)]
#![warn(clippy::filter_map_next)]
#![warn(clippy::checked_conversions)]
#![warn(clippy::type_repetition_in_bounds)]

// TODO: Implement more features from the original Windows app.
// TODO: How serial baud rate works and why there is a "Reset" button?
// TODO: Does the adapter set the ACL bit in the CAN frame on receive? Test with a second adapter.

use core::panic;
use std::{
    borrow::Cow,
    fmt::{self, Display},
    io::{self, Read},
    result,
    thread::sleep,
    time::Duration,
};

use embedded_can::{blocking, ExtendedId, Frame as _, Id, StandardId};
use serialport::{ClearBuffer, DataBits, SerialPort, StopBits};
use thiserror::Error;
use tracing::{debug, trace};

pub const DEFAULT_SERIAL_BAUD_RATE: u32 = 2000000;
pub const DEFAULT_SERIAL_RECEIVE_TIMEOUT: Duration = Duration::from_millis(1000);

#[derive(Error, Debug)]
pub enum Error {
    #[error("Serial port error: {}", .0.description)]
    Serial(#[from] serialport::Error),

    #[error("Serial IO error: {0}")]
    SerialIO(#[from] io::Error),

    #[error("Received unexpected data: {0}")]
    RecvUnexpected(String),

    #[error("Trying to send invalid frame: {0}")]
    SendingInvalidFrame(String),
}

impl embedded_can::Error for Error {
    fn kind(&self) -> embedded_can::ErrorKind {
        // We don't have a way to distinguish between different kinds of errors.
        embedded_can::ErrorKind::Other
    }
}

pub type Result<T> = result::Result<T, Error>;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Usb2CanBuilder {
    path: String,
    serial_baud_rate: u32,
    serial_receive_timeout: Duration,
    can_baud_rate: CanBaudRate,
    extended_frame: bool,
    loopback: bool,
    silent: bool,
    automatic_retransmission: bool,
}

impl Usb2CanBuilder {
    #[must_use]
    pub fn path<'a>(mut self, path: impl Into<std::borrow::Cow<'a, str>>) -> Self {
        self.path = path.into().to_string();
        self
    }

    #[must_use]
    pub const fn serial_baud_rate(mut self, serial_baud_rate: u32) -> Self {
        self.serial_baud_rate = serial_baud_rate;
        self
    }

    #[must_use]
    pub const fn serial_receive_timeout(mut self, serial_receive_timeout: Duration) -> Self {
        self.serial_receive_timeout = serial_receive_timeout;
        self
    }

    #[must_use]
    pub const fn can_baud_rate(mut self, can_baud_rate: CanBaudRate) -> Self {
        self.can_baud_rate = can_baud_rate;
        self
    }

    #[must_use]
    pub const fn extended_frame(mut self, extended_frame: bool) -> Self {
        self.extended_frame = extended_frame;
        self
    }

    #[must_use]
    pub const fn loopback(mut self, loopback: bool) -> Self {
        self.loopback = loopback;
        self
    }

    #[must_use]
    pub const fn silent(mut self, silent: bool) -> Self {
        self.silent = silent;
        self
    }

    #[must_use]
    pub const fn automatic_retransmission(mut self, automatic_retransmission: bool) -> Self {
        self.automatic_retransmission = automatic_retransmission;
        self
    }

    pub fn open(self) -> Result<Usb2Can> {
        debug!("Opening USB2CAN with configuration {:?}", self);

        let serial = serialport::new(&self.path, self.serial_baud_rate)
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::Two)
            .timeout(self.serial_receive_timeout);
        let serial = serial.open()?;
        debug!("Serial port opened: {:?}", serial.name());

        let mut usb2can = Usb2Can {
            serial,
            extended_frame: self.extended_frame,
        };
        usb2can.configure(
            self.can_baud_rate,
            self.extended_frame,
            self.loopback,
            self.silent,
            self.automatic_retransmission,
        )?;
        Ok(usb2can)
    }
}

pub fn new<'a>(path: impl Into<Cow<'a, str>>, can_baud_rate: CanBaudRate) -> Usb2CanBuilder {
    Usb2CanBuilder {
        path: path.into().into_owned(),
        serial_baud_rate: DEFAULT_SERIAL_BAUD_RATE,
        serial_receive_timeout: DEFAULT_SERIAL_RECEIVE_TIMEOUT,
        can_baud_rate,
        extended_frame: false,
        loopback: false,
        silent: false,
        automatic_retransmission: true,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanBaudRate {
    Kbitps5,
    Kbitps10,
    Kbitps20,
    Kbitps50,
    Kbitps100,
    Kbitps125,
    Kbitps200,
    Kbitps250,
    Kbitps400,
    Kbitps500,
    Kbitps800,
    Kbitps1000,
}

impl CanBaudRate {
    pub const fn to_baud_rate(&self) -> u32 {
        match self {
            Self::Kbitps5 => 5_000,
            Self::Kbitps10 => 10_000,
            Self::Kbitps20 => 20_000,
            Self::Kbitps50 => 50_000,
            Self::Kbitps100 => 100_000,
            Self::Kbitps125 => 125_000,
            Self::Kbitps200 => 200_000,
            Self::Kbitps250 => 250_000,
            Self::Kbitps400 => 400_000,
            Self::Kbitps500 => 500_000,
            Self::Kbitps800 => 800_000,
            Self::Kbitps1000 => 1_000_000,
        }
    }

    const fn to_config_value(self) -> u8 {
        match self {
            Self::Kbitps5 => 0x0c,
            Self::Kbitps10 => 0x0b,
            Self::Kbitps20 => 0x0a,
            Self::Kbitps50 => 0x09,
            Self::Kbitps100 => 0x08,
            Self::Kbitps125 => 0x07,
            Self::Kbitps200 => 0x06,
            Self::Kbitps250 => 0x05,
            Self::Kbitps400 => 0x04,
            Self::Kbitps500 => 0x03,
            Self::Kbitps800 => 0x02,
            Self::Kbitps1000 => 0x01,
        }
    }
}

impl Display for CanBaudRate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} bits/s", self.to_baud_rate())
    }
}

const PROTO_HEADER: u8 = 0xaa;
const PROTO_HEADER_FIXED: u8 = 0x55;

const PROTO_TYPE_CFG_SET: u8 = 0b00000010;
const PROTO_TYPE_CFG_SET_VARIABLE: u8 = 0b00010000;

const PROTO_CFG_MODE_FLAG_LOOPBACK: u8 = 0b00000001;
const PROTO_CFG_MODE_FLAG_SILENT: u8 = 0b00000010;

const PROTO_CFG_FRAME_NORMAL: u8 = 0x01;
const PROTO_CFG_FRAME_EXTENDED: u8 = 0x02;

const PROTO_CFG_RETRANSMISSION_ENABLED: u8 = 0x00;
const PROTO_CFG_RETRANSMISSION_DISABLED: u8 = 0x01;

const PROTO_TYPE_VARIABLE: u8 = 0b11000000;
const PROTO_TYPE_VARIABLE_MASK: u8 = PROTO_TYPE_VARIABLE;

const PROTO_TYPE_FLAG_EXTENDED: u8 = 0b00100000;
const PROTO_TYPE_FLAG_REMOTE: u8 = 0b00010000;

const PROTO_END: u8 = 0x55;

pub struct Usb2Can {
    serial: Box<dyn SerialPort>,
    extended_frame: bool,
}

impl Usb2Can {
    const CONFIGURATION_DELAY: Duration = Duration::from_millis(100);

    fn configure(
        &mut self,
        can_baud_rate: CanBaudRate,
        extended_frame: bool,
        loopback: bool,
        silent: bool,
        automatic_retransmission: bool,
    ) -> Result<()> {
        let mut config_message: [u8; 20] = [
            // Header
            PROTO_HEADER,
            PROTO_HEADER_FIXED,
            // Use variable length protocol to send and receive data
            // TODO: Make configurable.
            PROTO_TYPE_CFG_SET | PROTO_TYPE_CFG_SET_VARIABLE,
            // CAN bus speed
            can_baud_rate.to_config_value(),
            // Frame type
            if extended_frame {
                PROTO_CFG_FRAME_EXTENDED
            } else {
                PROTO_CFG_FRAME_NORMAL
            },
            // Filter (not used)
            // TODO: Use filters.
            0x00,
            0x00,
            0x00,
            0x00,
            // Mask (not used)
            0x00,
            0x00,
            0x00,
            0x00,
            // CAN adapter mode
            if loopback {
                PROTO_CFG_MODE_FLAG_LOOPBACK
            } else {
                0x00
            } | if silent {
                PROTO_CFG_MODE_FLAG_SILENT
            } else {
                0x00
            },
            // Automatic retransmission
            if automatic_retransmission {
                PROTO_CFG_RETRANSMISSION_ENABLED
            } else {
                PROTO_CFG_RETRANSMISSION_DISABLED
            },
            // Reserved
            0x00,
            0x00,
            0x00,
            0x00,
            // Checksum (will be filled in later)
            0x00,
        ];
        Self::fill_checksum(&mut config_message);

        self.serial.clear(ClearBuffer::All)?;
        self.serial_write(&config_message)?;

        // Adapter needs some time to process the configuration change.
        sleep(Self::CONFIGURATION_DELAY);

        // TODO: Receive all remaining data before returning.
        // TODO: Check that configuration was successful by first using loopback mode?
        Ok(())
    }

    fn fill_checksum(message: &mut [u8]) {
        message[message.len() - 1] = Self::generate_checksum(message);
    }

    fn generate_checksum(message: &mut [u8]) -> u8 {
        message[2..message.len() - 1]
            .iter()
            .fold(0u8, |acc, &x| acc.wrapping_add(x))
    }

    fn serial_write(&mut self, message: &[u8]) -> Result<()> {
        trace!("Writing into serial: {:?}", message);
        self.serial.write_all(message)?;
        self.serial.flush()?;
        Ok(())
    }

    fn serial_read_byte(&mut self) -> Result<u8> {
        // TODO: Buffered read.
        let mut byte = [0];
        self.serial.read_exact(&mut byte)?;
        trace!("Byte read from serial: {}", byte[0]);
        Ok(byte[0])
    }

    pub fn try_clone(&self) -> Result<Self> {
        let serial = self.serial.try_clone()?;
        Ok(Self {
            serial,
            extended_frame: self.extended_frame,
        })
    }
}

// TODO: Nonblocking.
impl blocking::Can for Usb2Can {
    type Frame = Frame;
    type Error = Error;

    fn transmit(&mut self, frame: &Frame) -> Result<()> {
        trace!("Transmitting frame: {:?}", frame);

        if frame.is_extended() {
            if !self.extended_frame {
                return Err(Error::SendingInvalidFrame(
                    "Trying to send extended frame but adapter configured for standard".into(),
                ));
            }
        } else if self.extended_frame {
            return Err(Error::SendingInvalidFrame(
                "Trying to send standard frame but adapter configured for extended".into(),
            ));
        }

        self.serial_write(&frame.to_message())
    }

    fn receive(&mut self) -> Result<Self::Frame> {
        ReceiverState::read_frame(&mut || self.serial_read_byte()).map(|frame| {
            trace!("Received frame: {:?}", frame);
            frame
        })
    }
}

enum ReceiverState {
    Header,
    Type,
    Id { bytes_left: usize, frame: Frame },
    Data { byte_n: usize, frame: Frame },
    SkipRemote { bytes_left: usize, frame: Frame },
    End(Frame),
    Finished(Frame),
}

impl ReceiverState {
    fn read_frame(read_byte: &mut impl FnMut() -> Result<u8>) -> Result<Frame> {
        let mut state = Self::Header;

        Ok(loop {
            state = state.advance(read_byte()?)?;
            if let Self::Finished(frame) = state {
                break frame;
            }
        })
    }

    fn advance(self, byte: u8) -> Result<Self> {
        let next_state = match (self, byte) {
            (Self::Header, PROTO_HEADER) => Self::Type,
            (Self::Header, byte) => {
                return Err(Error::RecvUnexpected(format!(
                    "Expected header, received 0x{:02x} (!= 0x{:02x})",
                    byte, PROTO_HEADER
                )))
            }

            (Self::Type, byte) => {
                if byte & PROTO_TYPE_VARIABLE_MASK != PROTO_TYPE_VARIABLE {
                    return Err(Error::RecvUnexpected(format!(
                        "Expected type, received 0x{:02x} (0x{:02x} not set)",
                        byte, PROTO_TYPE_VARIABLE
                    )));
                }

                let zero_id = if byte & PROTO_TYPE_FLAG_EXTENDED == 0 {
                    Id::Standard(StandardId::ZERO)
                } else {
                    Id::Extended(ExtendedId::ZERO)
                };

                let data_len = (byte & 0b00001111) as usize;
                if data_len > 8 {
                    return Err(Error::RecvUnexpected(format!(
                        "Expected type, received 0x{:02x} (data length > 8)",
                        byte
                    )));
                }

                let frame = if byte & PROTO_TYPE_FLAG_REMOTE == 0 {
                    Frame {
                        id: zero_id,
                        data: FrameData::Data(vec![0; data_len]),
                    }
                } else {
                    Frame {
                        id: zero_id,
                        data: FrameData::Remote(data_len),
                    }
                };

                Self::Id {
                    bytes_left: if frame.is_extended() { 3 } else { 1 },
                    frame,
                }
            }

            (
                Self::Id {
                    bytes_left,
                    mut frame,
                },
                byte,
            ) => {
                frame.id = match frame.id {
                    Id::Standard(standard_id) => {
                        let standard_id =
                            standard_id.as_raw() | (byte as u16) << ((1 - bytes_left) * 8);
                        Id::Standard(StandardId::new(standard_id).ok_or_else(|| {
                            Error::RecvUnexpected(format!(
                                "Invalid standard ID: 0x{:04x}",
                                standard_id
                            ))
                        })?)
                    }

                    Id::Extended(extended_id) => {
                        let extended_id =
                            extended_id.as_raw() | (byte as u32) << ((3 - bytes_left) * 8);
                        Id::Extended(ExtendedId::new(extended_id).ok_or_else(|| {
                            Error::RecvUnexpected(format!(
                                "Invalid extended ID: 0x{:08x}",
                                extended_id
                            ))
                        })?)
                    }
                };

                if bytes_left == 0 {
                    if frame.dlc() == 0 {
                        Self::End(frame)
                    } else if frame.is_remote_frame() {
                        Self::SkipRemote {
                            bytes_left: frame.dlc(),
                            frame,
                        }
                    } else {
                        Self::Data { byte_n: 0, frame }
                    }
                } else {
                    Self::Id {
                        bytes_left: bytes_left - 1,
                        frame,
                    }
                }
            }

            (
                Self::Data {
                    mut byte_n,
                    mut frame,
                },
                byte,
            ) => match frame.data {
                FrameData::Data(ref mut data) => {
                    data[byte_n] = byte;

                    byte_n += 1;
                    if byte_n == data.len() {
                        Self::End(frame)
                    } else {
                        Self::Data { byte_n, frame }
                    }
                }

                FrameData::Remote(_) => {
                    unreachable!("Logic error: trying to receive data for remote frame")
                }
            },

            (
                Self::SkipRemote {
                    mut bytes_left,
                    frame,
                },
                _, // Adapter inserts bogus data instead of zeros for remote frames, just skip it.
            ) => {
                bytes_left -= 1;
                if bytes_left == 0 {
                    Self::End(frame)
                } else {
                    Self::SkipRemote { bytes_left, frame }
                }
            }

            (Self::End(frame), PROTO_END) => Self::Finished(frame),
            (Self::End(_), byte) => {
                return Err(Error::RecvUnexpected(format!(
                    "Expected end, received 0x{:02x} (!= 0x{:02x})",
                    byte, PROTO_END
                )))
            }

            (Self::Finished(_), _) => {
                panic!("Logic error: trying to advance finished state")
            }
        };

        Ok(next_state)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    id: Id,
    data: FrameData,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum FrameData {
    Data(Vec<u8>),
    Remote(usize),
}

impl Frame {
    const MAX_SEND_MESSAGE_SIZE: usize = 15;

    fn to_message(&self) -> Vec<u8> {
        // TODO: Support fixed length encoding.

        let mut message = Vec::with_capacity(Self::MAX_SEND_MESSAGE_SIZE);

        // Header
        message.push(PROTO_HEADER);

        // Type
        let mut message_type = PROTO_TYPE_VARIABLE;
        if self.is_extended() {
            message_type |= PROTO_TYPE_FLAG_EXTENDED;
        }
        if self.is_remote_frame() {
            message_type |= PROTO_TYPE_FLAG_REMOTE;
        }
        message_type |= self.dlc() as u8;
        message.push(message_type);

        // Frame ID
        match self.id() {
            Id::Standard(standard_id) => message.extend(standard_id.as_raw().to_le_bytes()),
            Id::Extended(extended_id) => message.extend(extended_id.as_raw().to_le_bytes()),
        }

        // Data
        if self.is_remote_frame() {
            message.resize(message.len() + self.dlc(), 0);
        } else {
            message.extend(self.data());
        }

        // End code
        message.push(PROTO_END);

        message
    }
}

impl embedded_can::Frame for Frame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            None
        } else {
            Some(Self {
                id: id.into(),
                data: FrameData::Data(data.to_vec()),
            })
        }
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        if dlc > 8 {
            None
        } else {
            Some(Self {
                id: id.into(),
                data: FrameData::Remote(dlc),
            })
        }
    }

    fn is_extended(&self) -> bool {
        matches!(self.id, Id::Extended(_))
    }

    fn is_remote_frame(&self) -> bool {
        matches!(self.data, FrameData::Remote(_))
    }

    fn id(&self) -> Id {
        self.id
    }

    fn dlc(&self) -> usize {
        match &self.data {
            FrameData::Data(data) => data.len(),
            FrameData::Remote(dlc) => *dlc,
        }
    }

    fn data(&self) -> &[u8] {
        match &self.data {
            FrameData::Data(data) => data,
            FrameData::Remote(_) => &[],
        }
    }
}

#[cfg(test)]
mod tests {
    use embedded_can::{ExtendedId, StandardId};

    use super::*;

    #[test]
    fn small_send_message_size() {
        // Smallest message.
        let frame = Frame::new_remote(StandardId::MAX, 0).unwrap();
        let message = frame.to_message();
        assert!(message.len() < Frame::MAX_SEND_MESSAGE_SIZE);
    }

    #[test]
    fn max_send_message_size() {
        // Largest message.
        let frame = Frame::new(ExtendedId::MAX, &[0, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        let message = frame.to_message();
        assert_eq!(message.len(), Frame::MAX_SEND_MESSAGE_SIZE);
    }

    #[test]
    fn standard_id_endianness() {
        let frame = Frame::new_remote(StandardId::MAX, 0).unwrap();
        let message = frame.to_message();
        assert_eq!(message[2], 0xff);
        assert_eq!(message[3], 0x07);
    }

    #[test]
    fn extended_id_endianness() {
        let frame = Frame::new_remote(ExtendedId::MAX, 0).unwrap();
        let message = frame.to_message();
        assert_eq!(message[2], 0xff);
        assert_eq!(message[3], 0xff);
        assert_eq!(message[4], 0xff);
        assert_eq!(message[5], 0x1f);
    }

    #[test]
    fn serde_frame_standard() {
        let frame = Frame::new(StandardId::MAX, &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();
        check_serialize_deserialize_frame(&frame);

        let frame = Frame::new(StandardId::MAX, &[]).unwrap();
        check_serialize_deserialize_frame(&frame);
    }

    #[test]
    fn serde_frame_extended() {
        let frame = Frame::new(ExtendedId::MAX, &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();
        check_serialize_deserialize_frame(&frame);

        let frame = Frame::new(ExtendedId::MAX, &[]).unwrap();
        check_serialize_deserialize_frame(&frame);
    }

    #[test]
    fn serde_frame_remote_standard() {
        let frame = Frame::new_remote(StandardId::MAX, 8).unwrap();
        check_serialize_deserialize_frame(&frame);

        let frame = Frame::new_remote(StandardId::MAX, 0).unwrap();
        check_serialize_deserialize_frame(&frame);
    }

    #[test]
    fn serde_frame_remote_extended() {
        let frame = Frame::new_remote(ExtendedId::MAX, 8).unwrap();
        check_serialize_deserialize_frame(&frame);

        let frame = Frame::new_remote(ExtendedId::MAX, 0).unwrap();
        check_serialize_deserialize_frame(&frame);
    }

    // TODO: Test serialization and deserialization with injected errors.

    fn check_serialize_deserialize_frame(frame: &Frame) {
        let message = frame.to_message();
        let mut reader_fn = into_reader_fn(message);
        let received_frame = ReceiverState::read_frame(&mut reader_fn).unwrap();
        assert!(reader_fn().is_err());
        assert_eq!(frame, &received_frame);
    }

    fn into_reader_fn(data: Vec<u8>) -> impl FnMut() -> Result<u8> {
        let mut iter = data.into_iter();
        move || {
            iter.next()
                .ok_or_else(|| Error::RecvUnexpected("Unexpected end of data".into()))
        }
    }
}
