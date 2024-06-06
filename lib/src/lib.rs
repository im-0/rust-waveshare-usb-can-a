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

// TODO: Implement manual CAN bus baudrate selection (SJW, BS1, BS2, prescale).
// TODO: Implement ID filtering configuration.
// TODO: Implement configuration change on the fly.
// TODO: Throttle transmitting to the CAN baud rate.

use core::panic;
use std::{
    borrow::Cow,
    fmt::{self, Display},
    io::{self, BufReader, Read},
    result,
    thread::sleep,
    time::Duration,
};

use embedded_can::{blocking, ExtendedId, Frame as _, Id, StandardId};
use serialport::{ClearBuffer, DataBits, SerialPort, StopBits};
use thiserror::Error;
use tracing::{debug, trace};

pub const DEFAULT_SERIAL_BAUD_RATE: SerialBaudRate = SerialBaudRate::R2000000Bd;
pub const DEFAULT_SERIAL_RECEIVE_TIMEOUT: Duration = Duration::from_millis(1000);

pub const MAX_DATA_LENGTH: usize = 8;

pub const BASE_ID_BITS: usize = 11;
pub const EXTENDED_ID_BITS: usize = 29;
pub const EXTENDED_ID_EXTRA_BITS: usize = EXTENDED_ID_BITS - BASE_ID_BITS;

#[derive(Error, Debug)]
pub enum Error {
    #[error("Configuration error: {0}")]
    Configuration(String),

    #[error("Serial port error: {}", .0.description)]
    Serial(#[from] serialport::Error),

    #[error("Serial read timed out")]
    SerialReadTimedOut,

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
    serial_baud_rate: SerialBaudRate,
    serial_receive_timeout: Duration,
    can_baud_rate: CanBaudRate,
    receive_only_extended_frames: bool,
    filter: Id,
    mask: Id,
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
    pub const fn serial_baud_rate(mut self, serial_baud_rate: SerialBaudRate) -> Self {
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
    pub const fn receive_only_extended_frames(mut self, extended_frame: bool) -> Self {
        self.receive_only_extended_frames = extended_frame;
        self
    }

    pub fn filter(mut self, filter: Id, mask: Id) -> Result<Self> {
        match (&filter, &mask) {
            (Id::Standard(_), Id::Extended(_)) | (Id::Extended(_), Id::Standard(_)) => {
                return Err(Error::Configuration(
                    "Filter and mask must have the same type (standard or extended)".into(),
                ));
            }
            _ => {}
        }

        self.filter = filter;
        self.mask = mask;
        Ok(self)
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

        let serial = serialport::new(&self.path, u32::from(self.serial_baud_rate))
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::Two)
            .timeout(self.serial_receive_timeout);
        let serial = serial.open()?;
        debug!("Serial port opened: {:?}", serial.name());

        let mut usb2can = Usb2Can {
            can_baud_rate: self.can_baud_rate,
            serial,
            read_buffer: None,
            tried_to_initialize_read_buffer: false,
            sync_attempts_left: SYNC_ATTEMPTS,
        };
        usb2can.configure(&self)?;
        Ok(usb2can)
    }
}

pub fn new<'a>(path: impl Into<Cow<'a, str>>, can_baud_rate: CanBaudRate) -> Usb2CanBuilder {
    Usb2CanBuilder {
        path: path.into().into_owned(),
        serial_baud_rate: DEFAULT_SERIAL_BAUD_RATE,
        serial_receive_timeout: DEFAULT_SERIAL_RECEIVE_TIMEOUT,
        can_baud_rate,
        receive_only_extended_frames: false,
        filter: ExtendedId::ZERO.into(),
        mask: ExtendedId::ZERO.into(),
        loopback: false,
        silent: false,
        automatic_retransmission: true,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialBaudRate {
    R9600Bd,
    R19200Bd,
    R38400Bd,
    R115200Bd,
    R1228800Bd,
    R2000000Bd,
}

impl SerialBaudRate {
    const BLINK_DELAY: Duration = Duration::from_millis(700);

    const fn to_config_value(self) -> u8 {
        match self {
            Self::R9600Bd => 0x05,
            Self::R19200Bd => 0x04,
            Self::R38400Bd => 0x03,
            Self::R115200Bd => 0x02,
            Self::R1228800Bd => 0x01,
            Self::R2000000Bd => 0x00,
        }
    }

    fn to_blink_delay(self) -> Duration {
        let blinks = match self {
            Self::R9600Bd => 6,
            Self::R19200Bd => 5,
            Self::R38400Bd => 4,
            Self::R115200Bd => 3,
            Self::R1228800Bd => 2,
            Self::R2000000Bd => 1,
        };

        Self::BLINK_DELAY * blinks
    }
}

impl Display for SerialBaudRate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} Bd", u32::from(*self))
    }
}

impl TryFrom<u32> for SerialBaudRate {
    type Error = Error;

    fn try_from(value: u32) -> result::Result<Self, Self::Error> {
        match value {
            9_600 => Ok(Self::R9600Bd),
            19_200 => Ok(Self::R19200Bd),
            38_400 => Ok(Self::R38400Bd),
            115_200 => Ok(Self::R115200Bd),
            1_228_800 => Ok(Self::R1228800Bd),
            2_000_000 => Ok(Self::R2000000Bd),
            _ => Err(Error::Configuration(format!(
                "Unsupported serial baud rate value: {}",
                value
            ))),
        }
    }
}

impl From<SerialBaudRate> for u32 {
    fn from(value: SerialBaudRate) -> Self {
        match value {
            SerialBaudRate::R9600Bd => 9_600,
            SerialBaudRate::R19200Bd => 19_200,
            SerialBaudRate::R38400Bd => 38_400,
            SerialBaudRate::R115200Bd => 115_200,
            SerialBaudRate::R1228800Bd => 1_228_800,
            SerialBaudRate::R2000000Bd => 2_000_000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanBaudRate {
    R5kBd,
    R10kBd,
    R20kBd,
    R50kBd,
    R100kBd,
    R125kBd,
    R200kBd,
    R250kBd,
    R400kBd,
    R500kBd,
    R800kBd,
    R1000kBd,
}

impl CanBaudRate {
    const fn to_config_value(self) -> u8 {
        match self {
            Self::R5kBd => 0x0c,
            Self::R10kBd => 0x0b,
            Self::R20kBd => 0x0a,
            Self::R50kBd => 0x09,
            Self::R100kBd => 0x08,
            Self::R125kBd => 0x07,
            Self::R200kBd => 0x06,
            Self::R250kBd => 0x05,
            Self::R400kBd => 0x04,
            Self::R500kBd => 0x03,
            Self::R800kBd => 0x02,
            Self::R1000kBd => 0x01,
        }
    }
}

impl Display for CanBaudRate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::R5kBd
            | Self::R10kBd
            | Self::R20kBd
            | Self::R50kBd
            | Self::R100kBd
            | Self::R125kBd
            | Self::R200kBd
            | Self::R250kBd
            | Self::R400kBd
            | Self::R500kBd
            | Self::R800kBd => write!(f, "{} kBd", u32::from(*self) / 1000),

            Self::R1000kBd => write!(f, "1 MBd"),
        }
    }
}

impl TryFrom<u32> for CanBaudRate {
    type Error = Error;

    fn try_from(value: u32) -> result::Result<Self, Self::Error> {
        match value {
            5_000 => Ok(Self::R5kBd),
            10_000 => Ok(Self::R10kBd),
            20_000 => Ok(Self::R20kBd),
            50_000 => Ok(Self::R50kBd),
            100_000 => Ok(Self::R100kBd),
            125_000 => Ok(Self::R125kBd),
            200_000 => Ok(Self::R200kBd),
            250_000 => Ok(Self::R250kBd),
            400_000 => Ok(Self::R400kBd),
            500_000 => Ok(Self::R500kBd),
            800_000 => Ok(Self::R800kBd),
            1_000_000 => Ok(Self::R1000kBd),
            _ => Err(Error::Configuration(format!(
                "Unsupported CAN baud rate value: {}",
                value
            ))),
        }
    }
}

impl From<CanBaudRate> for u32 {
    fn from(value: CanBaudRate) -> Self {
        match value {
            CanBaudRate::R5kBd => 5_000,
            CanBaudRate::R10kBd => 10_000,
            CanBaudRate::R20kBd => 20_000,
            CanBaudRate::R50kBd => 50_000,
            CanBaudRate::R100kBd => 100_000,
            CanBaudRate::R125kBd => 125_000,
            CanBaudRate::R200kBd => 200_000,
            CanBaudRate::R250kBd => 250_000,
            CanBaudRate::R400kBd => 400_000,
            CanBaudRate::R500kBd => 500_000,
            CanBaudRate::R800kBd => 800_000,
            CanBaudRate::R1000kBd => 1_000_000,
        }
    }
}

const MAX_FIXED_MESSAGE_SIZE: usize = 20;
const MAX_VARIABLE_MESSAGE_SIZE: usize = 15;
// Should be enough to sync if leftovers of a single frame are left somewhere in the buffers.
const SYNC_ATTEMPTS: usize = const_fn_max(MAX_FIXED_MESSAGE_SIZE, MAX_VARIABLE_MESSAGE_SIZE) - 1;

const PROTO_HEADER: u8 = 0xaa;
const PROTO_HEADER_FIXED: u8 = 0x55;

const PROTO_TYPE_CFG_SET: u8 = 0b00000010;
const PROTO_TYPE_CFG_SET_VARIABLE: u8 = 0b00010000;
const PROTO_TYPE_CFG_SET_SERIAL_BAUD_RATE: u8 = 0b00000100;

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

const PROTO_TYPE_SIZE_MASK: u8 = 0b00001111;

const PROTO_END: u8 = 0x55;

const fn const_fn_max(a: usize, b: usize) -> usize {
    [a, b][(a < b) as usize]
}

pub struct Usb2Can {
    can_baud_rate: CanBaudRate,
    serial: Box<dyn SerialPort>,
    read_buffer: Option<BufReader<Box<dyn SerialPort>>>,
    tried_to_initialize_read_buffer: bool,
    sync_attempts_left: usize,
}

impl Usb2Can {
    const CONFIGURATION_DELAY: Duration = Duration::from_millis(100);

    pub fn name(&self) -> Option<String> {
        self.serial.name()
    }

    pub const fn can_baud_rate(&self) -> CanBaudRate {
        self.can_baud_rate
    }

    pub fn serial_receive_timeout(&self) -> Duration {
        self.serial.timeout()
    }

    pub fn set_serial_receive_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.serial.set_timeout(timeout).map_err(|error| {
            Error::Configuration(format!("Failed to set serial receive timeout: {}", error))
        })
    }

    fn configure(&mut self, configuration: &Usb2CanBuilder) -> Result<()> {
        let filter = id_to_bytes(configuration.filter);
        let mask = id_to_bytes(configuration.mask);

        let mut config_message: [u8; MAX_FIXED_MESSAGE_SIZE] = [
            // Header
            PROTO_HEADER,
            PROTO_HEADER_FIXED,
            // Use variable length protocol to send and receive data
            // TODO: Make configurable.
            PROTO_TYPE_CFG_SET | PROTO_TYPE_CFG_SET_VARIABLE,
            // CAN bus speed
            configuration.can_baud_rate.to_config_value(),
            // Frame type
            if configuration.receive_only_extended_frames {
                PROTO_CFG_FRAME_EXTENDED
            } else {
                PROTO_CFG_FRAME_NORMAL
            },
            // Filter
            filter[0],
            filter[1],
            filter[2],
            filter[3],
            // Mask
            mask[0],
            mask[1],
            mask[2],
            mask[3],
            // CAN adapter mode
            if configuration.loopback {
                PROTO_CFG_MODE_FLAG_LOOPBACK
            } else {
                0x00
            } | if configuration.silent {
                PROTO_CFG_MODE_FLAG_SILENT
            } else {
                0x00
            },
            // Automatic retransmission
            if configuration.automatic_retransmission {
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
        self.fill_checksum_and_transmit_configuration_message(&mut config_message)
    }

    pub fn serial_baud_rate(&self) -> Result<SerialBaudRate> {
        self.serial
            .baud_rate()
            .map_err(|error| {
                Error::Configuration(format!("Failed to get serial baud rate: {}", error))
            })
            .and_then(|value| {
                SerialBaudRate::try_from(value).map_err(|error| {
                    Error::Configuration(format!(
                        "Unsupported baud rate on underlying serial interface: {}",
                        error
                    ))
                })
            })
    }

    pub fn set_serial_baud_rate(&mut self, serial_baud_rate: SerialBaudRate) -> Result<()> {
        let mut config_message: [u8; MAX_FIXED_MESSAGE_SIZE] = [
            // Header
            PROTO_HEADER,
            PROTO_HEADER_FIXED,
            // Set serial baud rate
            PROTO_TYPE_CFG_SET | PROTO_TYPE_CFG_SET_SERIAL_BAUD_RATE,
            // Serial baud rate
            serial_baud_rate.to_config_value(),
            // Reserved
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            // Checksum (will be filled in later)
            0x00,
        ];
        self.fill_checksum_and_transmit_configuration_message(&mut config_message)?;

        // Adapater does not respond while blinking.
        let blink_delay = serial_baud_rate.to_blink_delay();
        debug!(
            "Waiting {}s while adapter is blinking",
            blink_delay.as_secs_f64()
        );
        sleep(blink_delay);

        // Set the new baud rate on underlying serial interface.
        self.serial
            .set_baud_rate(u32::from(serial_baud_rate))
            .map_err(|error| {
                Error::Configuration(format!("Failed to set serial baud rate: {}", error))
            })
    }

    fn fill_checksum_and_transmit_configuration_message(
        &mut self,
        config_message: &mut [u8],
    ) -> Result<()> {
        Self::fill_checksum(config_message);

        self.serial.clear(ClearBuffer::All)?;
        self.serial_write(config_message)?;

        // Adapter needs some time to process the configuration change.
        debug!(
            "Waiting {}s for configuration change to take effect",
            Self::CONFIGURATION_DELAY.as_secs_f64()
        );
        sleep(Self::CONFIGURATION_DELAY);

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
        if !self.tried_to_initialize_read_buffer {
            self.read_buffer = self
                .serial
                .try_clone()
                .map_err(|error| debug!("No read buffer, failed to clone serial port: {}", error))
                .map(BufReader::new)
                .ok();
            self.tried_to_initialize_read_buffer = true;
        }

        let mut byte = [0];
        if let Some(read_buffer) = &mut self.read_buffer {
            read_buffer.read_exact(&mut byte)
        } else {
            self.serial.read_exact(&mut byte)
        }
        .map_err(|error| {
            if error.kind() == io::ErrorKind::TimedOut {
                Error::SerialReadTimedOut
            } else {
                Error::from(error)
            }
        })?;

        trace!("Byte read from serial: {}", byte[0]);
        Ok(byte[0])
    }

    fn receive_inner(&mut self) -> Result<Frame> {
        ReceiverState::read_frame(&mut || self.serial_read_byte())
    }

    pub fn try_clone(&self) -> Result<Self> {
        let serial = self.serial.try_clone()?;

        Ok(Self {
            can_baud_rate: self.can_baud_rate,
            serial,
            read_buffer: None,
            tried_to_initialize_read_buffer: false,

            sync_attempts_left: if self.sync_attempts_left == SYNC_ATTEMPTS {
                SYNC_ATTEMPTS
            } else {
                0
            },
        })
    }
}

// TODO: Nonblocking.
impl blocking::Can for Usb2Can {
    type Frame = Frame;
    type Error = Error;

    fn transmit(&mut self, frame: &Frame) -> Result<()> {
        trace!("Transmitting frame: {:?}", frame);
        self.serial_write(&frame.to_message())
    }

    fn receive(&mut self) -> Result<Self::Frame> {
        if self.sync_attempts_left == 0 {
            self.receive_inner()
        } else {
            loop {
                match self.receive_inner() {
                    Ok(frame) => {
                        self.sync_attempts_left = 0;
                        break Ok(frame);
                    }

                    error @ Err(Error::SerialReadTimedOut) => break error,
                    Err(error) => debug!("Failed to sync: {}", error),
                }

                if self.sync_attempts_left == 0 {
                    break Err(Error::RecvUnexpected(
                        "Failed to sync, no attempts left".into(),
                    ));
                }
                self.sync_attempts_left -= 1;
            }
        }
        .map(|frame| {
            trace!("Received frame: {:?}", frame);
            frame
        })
    }
}

enum ReceiverState {
    EndOrHeader,
    Header,
    Type,
    Id { bytes_left: usize, frame: Frame },
    Data { byte_n: usize, frame: Frame },
    SkipRemote { bytes_left: usize, frame: Frame },
    Finished(Frame),
}

impl ReceiverState {
    fn read_frame(read_byte: &mut impl FnMut() -> Result<u8>) -> Result<Frame> {
        let mut state = Self::EndOrHeader;

        Ok(loop {
            state = state.advance(read_byte()?)?;
            if let Self::Finished(frame) = state {
                break frame;
            }
        })
    }

    fn advance(self, byte: u8) -> Result<Self> {
        let next_state = match (self, byte) {
            (Self::EndOrHeader, PROTO_END) => Self::Header,
            (Self::EndOrHeader, PROTO_HEADER) => Self::Type,
            (Self::EndOrHeader, byte) => {
                return Err(Error::RecvUnexpected(format!(
                    "Expected end or header, received 0x{:02x} (!= {{0x{:02x}, 0x{:02x}}})",
                    byte, PROTO_END, PROTO_HEADER
                )))
            }

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

                let data_len = (byte & PROTO_TYPE_SIZE_MASK) as usize;
                if data_len > MAX_DATA_LENGTH {
                    return Err(Error::RecvUnexpected(format!(
                        "Expected type, received 0x{:02x} (data length > {})",
                        byte, MAX_DATA_LENGTH
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
                        Self::Finished(frame)
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
                        Self::Finished(frame)
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
                    Self::Finished(frame)
                } else {
                    Self::SkipRemote { bytes_left, frame }
                }
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
    fn to_message(&self) -> Vec<u8> {
        // TODO: Support fixed length encoding.

        let mut message = Vec::with_capacity(MAX_VARIABLE_MESSAGE_SIZE);

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
            Id::Standard(standard_id) => message.extend(standard_id.to_bytes()),
            Id::Extended(extended_id) => message.extend(extended_id.to_bytes()),
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
        if data.len() > MAX_DATA_LENGTH {
            None
        } else {
            Some(Self {
                id: id.into(),
                data: FrameData::Data(data.to_vec()),
            })
        }
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        if dlc > MAX_DATA_LENGTH {
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

impl Display for Frame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_standard() {
            write!(f, "      ")?;
        }

        // Type and data length.
        if self.is_remote_frame() {
            write!(f, "r:")?;
        } else {
            write!(f, "d:")?;
        }

        // Identifier.
        let (base_id, extended_id) = {
            match self.id {
                Id::Standard(standard_id) => (standard_id.as_raw(), None),

                Id::Extended(extended_id) => (
                    extended_id.standard_id().as_raw(),
                    Some(extended_id.as_raw() & ((1 << EXTENDED_ID_EXTRA_BITS) - 1)), // Bits ID-17 to ID-0
                ),
            }
        };
        if let Some(extended_id) = extended_id {
            write!(f, "{:05x}.", extended_id)?;
        }
        write!(f, "{:03x}", base_id)?;
        write!(f, ":")?;

        // Data or remote length.
        if self.is_remote_frame() {
            write!(f, "{}", self.dlc())?;
            write!(f, "                        ")?;
        } else {
            // Print as hex.
            for byte in self.data() {
                write!(f, "{:02x}", byte)?;
            }
            for _ in self.dlc()..8 {
                write!(f, "  ")?;
            }

            // Print printable characters as ASCII.
            write!(f, " ")?;
            for byte in self.data() {
                let char = char::from_u32(*byte as u32).map_or('.', |char| {
                    if char.is_ascii_graphic() {
                        char
                    } else {
                        '.'
                    }
                });
                write!(f, "{}", char)?;
            }
            for _ in self.dlc()..8 {
                write!(f, " ")?;
            }
        }

        Ok(())
    }
}

trait IdToBytes<const N: usize> {
    fn to_bytes(self) -> [u8; N];
}

impl IdToBytes<2> for StandardId {
    fn to_bytes(self) -> [u8; 2] {
        self.as_raw().to_le_bytes()
    }
}

impl IdToBytes<4> for ExtendedId {
    fn to_bytes(self) -> [u8; 4] {
        self.as_raw().to_le_bytes()
    }
}

fn id_to_bytes(id: Id) -> [u8; 4] {
    match id {
        Id::Standard(standard_id) => {
            let bytes = standard_id.to_bytes();
            [bytes[0], bytes[1], 0, 0]
        }

        Id::Extended(extended_id) => extended_id.to_bytes(),
    }
}

#[cfg(test)]
mod tests {
    use embedded_can::{ExtendedId, StandardId};

    use super::*;

    #[test]
    fn invalid_filter_conf() {
        let builder = new("/dev/ttyUSB123", CanBaudRate::R1000kBd);
        let builder = builder.filter(StandardId::ZERO.into(), ExtendedId::ZERO.into());
        assert!(matches!(builder, Err(Error::Configuration(_))));

        let builder = new("/dev/ttyUSB123", CanBaudRate::R1000kBd);
        let builder = builder.filter(ExtendedId::ZERO.into(), StandardId::ZERO.into());
        assert!(matches!(builder, Err(Error::Configuration(_))));
    }

    #[test]
    fn small_send_message_size() {
        // Smallest message.
        let frame = Frame::new_remote(StandardId::MAX, 0).unwrap();
        let message = frame.to_message();
        assert!(message.len() < MAX_VARIABLE_MESSAGE_SIZE);
    }

    #[test]
    fn max_send_message_size() {
        // Largest message.
        let frame = Frame::new(ExtendedId::MAX, &[0, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        let message = frame.to_message();
        assert_eq!(message.len(), MAX_VARIABLE_MESSAGE_SIZE);
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
    fn data_frame_data_too_long() {
        let frame = Frame::new(StandardId::ZERO, &[0; 9]);
        assert!(frame.is_none());
    }

    #[test]
    fn remote_frame_data_too_long() {
        let frame = Frame::new_remote(StandardId::ZERO, 9);
        assert!(frame.is_none());
    }

    #[test]
    fn standard_data_frame_short() {
        let frame = Frame::new(StandardId::MAX, &[]).unwrap();
        let message = frame.to_message();
        assert_eq!(
            message,
            [
                0xaa, // Header
                0xc0, // Type | flags | length
                0xff, 0x07, // ID
                0x55, // End
            ]
        );
    }

    #[test]
    fn standard_data_frame_long() {
        let frame = Frame::new(
            StandardId::MAX,
            &[0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef],
        )
        .unwrap();
        let message = frame.to_message();
        assert_eq!(
            message,
            [
                0xaa, // Header
                0xc8, // Type | flags | length
                0xff, 0x07, // ID
                0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, // Data
                0x55, // End
            ]
        );
    }

    #[test]
    fn extended_data_frame_short() {
        let frame = Frame::new(ExtendedId::MAX, &[]).unwrap();
        let message = frame.to_message();
        assert_eq!(
            message,
            [
                0xaa, // Header
                0xe0, // Type | flags | length
                0xff, 0xff, 0xff, 0x1f, // Id
                0x55, // End
            ]
        );
    }

    #[test]
    fn extended_data_frame_long() {
        let frame = Frame::new(
            ExtendedId::MAX,
            &[0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef],
        )
        .unwrap();
        let message = frame.to_message();
        assert_eq!(
            message,
            [
                0xaa, // Header
                0xe8, // Type | flags | length
                0xff, 0xff, 0xff, 0x1f, // Id
                0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, // Data
                0x55, // End
            ]
        );
    }

    #[test]
    fn standard_remote_frame_short() {
        let frame = Frame::new_remote(StandardId::MAX, 0).unwrap();
        let message = frame.to_message();
        assert_eq!(
            message,
            [
                0xaa, // Header
                0xd0, // Type | flags | length
                0xff, 0x07, // ID
                0x55, // End
            ]
        );
    }

    #[test]
    fn standard_remote_frame_long() {
        let frame = Frame::new_remote(StandardId::MAX, MAX_DATA_LENGTH).unwrap();
        let message = frame.to_message();
        assert_eq!(
            message,
            [
                0xaa, // Header
                0xd8, // Type | flags | length
                0xff, 0x07, // ID
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Data placeholder
                0x55, // End
            ]
        );
    }

    #[test]
    fn extended_remote_frame_short() {
        let frame = Frame::new_remote(ExtendedId::MAX, 0).unwrap();
        let message = frame.to_message();
        assert_eq!(
            message,
            [
                0xaa, // Header
                0xf0, // Type | flags | length
                0xff, 0xff, 0xff, 0x1f, // Id
                0x55, // End
            ]
        );
    }

    #[test]
    fn extended_remote_frame_long() {
        let frame = Frame::new_remote(ExtendedId::MAX, MAX_DATA_LENGTH).unwrap();
        let message = frame.to_message();
        assert_eq!(
            message,
            [
                0xaa, // Header
                0xf8, // Type | flags | length
                0xff, 0xff, 0xff, 0x1f, // Id
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Data placeholder
                0x55, // End
            ]
        );
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
        let frame = Frame::new_remote(StandardId::MAX, MAX_DATA_LENGTH).unwrap();
        check_serialize_deserialize_frame(&frame);

        let frame = Frame::new_remote(StandardId::MAX, 0).unwrap();
        check_serialize_deserialize_frame(&frame);
    }

    #[test]
    fn serde_frame_remote_extended() {
        let frame = Frame::new_remote(ExtendedId::MAX, MAX_DATA_LENGTH).unwrap();
        check_serialize_deserialize_frame(&frame);

        let frame = Frame::new_remote(ExtendedId::MAX, 0).unwrap();
        check_serialize_deserialize_frame(&frame);
    }

    // TODO: Test serialization and deserialization with injected errors.

    fn check_serialize_deserialize_frame(frame: &Frame) {
        let message = frame.to_message();
        let mut reader_fn = into_reader_fn(message);

        let received_frame = ReceiverState::read_frame(&mut reader_fn).unwrap();

        assert_eq!(reader_fn().unwrap(), PROTO_END);
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
