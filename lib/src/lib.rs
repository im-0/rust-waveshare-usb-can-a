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
    collections::VecDeque,
    fmt::{self, Display},
    io::{self, BufReader, Read},
    result,
    sync::{Arc, Mutex, MutexGuard},
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

const MAX_FIXED_MESSAGE_SIZE: usize = 20;
const MAX_VARIABLE_MESSAGE_SIZE: usize = 15;
const MAX_MESSAGE_SIZE: usize = const_fn_max(MAX_FIXED_MESSAGE_SIZE, MAX_VARIABLE_MESSAGE_SIZE);
// Should be enough to sync as leftovers of a single frame are kept in the buffers.
const SYNC_ATTEMPTS: usize = MAX_MESSAGE_SIZE - 1;

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

#[derive(Error, Debug)]
pub enum Error {
    #[error("Configuration error: {0}")]
    SetConfiguration(String),

    #[error("Configuration reading error: {0}")]
    GetConfiguration(String),

    #[error("Locking error: {0}")]
    Locking(String),

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
    adapter_configuration: Usb2CanConfiguration,
}

impl Usb2CanBuilder {
    pub fn path(&self) -> &str {
        &self.path
    }

    #[must_use]
    pub fn set_path<'a>(mut self, path: impl Into<std::borrow::Cow<'a, str>>) -> Self {
        self.path = path.into().to_string();
        self
    }

    pub const fn serial_baud_rate(&self) -> SerialBaudRate {
        self.serial_baud_rate
    }

    #[must_use]
    pub const fn set_serial_baud_rate(mut self, serial_baud_rate: SerialBaudRate) -> Self {
        self.serial_baud_rate = serial_baud_rate;
        self
    }

    pub const fn serial_receive_timeout(&self) -> Duration {
        self.serial_receive_timeout
    }

    #[must_use]
    pub const fn set_serial_receive_timeout(mut self, serial_receive_timeout: Duration) -> Self {
        self.serial_receive_timeout = serial_receive_timeout;
        self
    }

    pub const fn adapter_configuration(&self) -> &Usb2CanConfiguration {
        &self.adapter_configuration
    }

    #[must_use]
    pub const fn set_adapter_configuration(
        mut self,
        adapter_configuration: Usb2CanConfiguration,
    ) -> Self {
        self.adapter_configuration = adapter_configuration;
        self
    }

    pub fn open(&self) -> Result<Usb2Can> {
        self.open_inner(false)
    }

    pub fn open_with_blink_delay(&self) -> Result<Usb2Can> {
        self.open_inner(true)
    }

    fn open_inner(&self, with_blink_delay: bool) -> Result<Usb2Can> {
        debug!("Opening USB2CAN with configuration {:?}", self);

        let serial = serialport::new(&self.path, u32::from(self.serial_baud_rate))
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::Two);
        let serial = serial.open()?;
        debug!("Serial port opened: {:?}", serial.name());

        let mut receiver = Receiver::new(serial.try_clone()?);
        receiver.set_receive_timeout(self.serial_receive_timeout)?;

        let mut usb2can = Usb2Can {
            receiver: Arc::new(Mutex::new(receiver)),
            transmitter: Arc::new(Mutex::new(Transmitter::new(serial))),
            configuration: Arc::new(Mutex::new(self.adapter_configuration.clone())),
        };

        if with_blink_delay {
            self.serial_baud_rate.sleep_while_blinking();
        }
        usb2can.set_configuration_inner()?;

        Ok(usb2can)
    }
}

pub fn new<'a>(
    path: impl Into<Cow<'a, str>>,
    adapter_configuration: &Usb2CanConfiguration,
) -> Usb2CanBuilder {
    Usb2CanBuilder {
        path: path.into().into_owned(),
        serial_baud_rate: DEFAULT_SERIAL_BAUD_RATE,
        serial_receive_timeout: DEFAULT_SERIAL_RECEIVE_TIMEOUT,
        adapter_configuration: adapter_configuration.clone(),
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Usb2CanConfiguration {
    can_baud_rate: CanBaudRate,
    receive_only_extended_frames: bool,
    filter: Id,
    mask: Id,
    loopback: bool,
    silent: bool,
    automatic_retransmission: bool,
}

impl Usb2CanConfiguration {
    pub fn new(can_baud_rate: CanBaudRate) -> Self {
        Self {
            can_baud_rate,
            receive_only_extended_frames: false,
            filter: ExtendedId::ZERO.into(),
            mask: ExtendedId::ZERO.into(),
            loopback: false,
            silent: false,
            automatic_retransmission: true,
        }
    }

    pub const fn can_baud_rate(&self) -> CanBaudRate {
        self.can_baud_rate
    }

    #[must_use]
    pub const fn set_can_baud_rate(mut self, can_baud_rate: CanBaudRate) -> Self {
        self.can_baud_rate = can_baud_rate;
        self
    }

    pub const fn receive_only_extended_frames(&self) -> bool {
        self.receive_only_extended_frames
    }

    #[must_use]
    pub const fn set_receive_only_extended_frames(mut self, extended_frame: bool) -> Self {
        self.receive_only_extended_frames = extended_frame;
        self
    }

    pub const fn filter(&self) -> (Id, Id) {
        (self.filter, self.mask)
    }

    pub fn set_filter(mut self, filter: Id, mask: Id) -> Result<Self> {
        match (&filter, &mask) {
            (Id::Standard(_), Id::Extended(_)) | (Id::Extended(_), Id::Standard(_)) => {
                return Err(Error::SetConfiguration(
                    "Filter and mask must have the same type (standard or extended)".into(),
                ));
            }
            _ => {}
        }

        self.filter = filter;
        self.mask = mask;
        Ok(self)
    }

    pub const fn loopback(&self) -> bool {
        self.loopback
    }

    #[must_use]
    pub const fn set_loopback(mut self, loopback: bool) -> Self {
        self.loopback = loopback;
        self
    }

    pub const fn silent(&self) -> bool {
        self.silent
    }

    #[must_use]
    pub const fn set_silent(mut self, silent: bool) -> Self {
        self.silent = silent;
        self
    }

    pub const fn automatic_retransmission(&self) -> bool {
        self.automatic_retransmission
    }

    #[must_use]
    pub const fn set_automatic_retransmission(mut self, automatic_retransmission: bool) -> Self {
        self.automatic_retransmission = automatic_retransmission;
        self
    }

    fn to_configuration_message(&self) -> [u8; MAX_FIXED_MESSAGE_SIZE] {
        let filter = id_to_bytes(self.filter);
        let mask = id_to_bytes(self.mask);

        [
            // Header
            PROTO_HEADER,
            PROTO_HEADER_FIXED,
            // Use variable length protocol to send and receive data
            // TODO: Make configurable.
            PROTO_TYPE_CFG_SET | PROTO_TYPE_CFG_SET_VARIABLE,
            // CAN bus speed
            self.can_baud_rate.to_config_value(),
            // Frame type
            if self.receive_only_extended_frames {
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
            if self.loopback {
                PROTO_CFG_MODE_FLAG_LOOPBACK
            } else {
                0x00
            } | if self.silent {
                PROTO_CFG_MODE_FLAG_SILENT
            } else {
                0x00
            },
            // Automatic retransmission
            if self.automatic_retransmission {
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
        ]
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

    fn sleep_while_blinking(self) {
        let blink_delay = self.to_blink_delay();
        debug!(
            "Waiting {:.03}s while adapter is blinking",
            blink_delay.as_secs_f64()
        );
        sleep(blink_delay);
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

    const fn to_configuration_message(self) -> [u8; MAX_FIXED_MESSAGE_SIZE] {
        [
            // Header
            PROTO_HEADER,
            PROTO_HEADER_FIXED,
            // Set serial baud rate
            PROTO_TYPE_CFG_SET | PROTO_TYPE_CFG_SET_SERIAL_BAUD_RATE,
            // Serial baud rate
            self.to_config_value(),
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
        ]
    }

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
            _ => Err(Error::SetConfiguration(format!(
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
            _ => Err(Error::SetConfiguration(format!(
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

const fn const_fn_max(a: usize, b: usize) -> usize {
    [a, b][(a < b) as usize]
}

#[derive(Clone)]
pub struct Usb2Can {
    receiver: Arc<Mutex<Receiver>>,
    transmitter: Arc<Mutex<Transmitter>>,
    configuration: Arc<Mutex<Usb2CanConfiguration>>,
}

impl Usb2Can {
    const CONFIGURATION_DELAY: Duration = Duration::from_millis(100);

    pub fn name(&self) -> Result<Option<String>> {
        self.lock_transmitter()
            .map(|transmitter| transmitter.name())
    }

    pub fn serial_receive_timeout(&self) -> Result<Duration> {
        self.lock_receiver()
            .map(|receiver| receiver.receive_timeout())
    }

    pub fn set_serial_receive_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.lock_receiver()
            .and_then(|mut receiver| receiver.set_receive_timeout(timeout))
    }

    pub fn configuration(&self) -> Result<Usb2CanConfiguration> {
        Ok(self.lock_configuration()?.clone())
    }

    pub fn set_configuration(&mut self, configuration: &Usb2CanConfiguration) -> Result<()> {
        *self.lock_configuration()? = configuration.clone();
        self.set_configuration_inner()
    }

    fn set_configuration_inner(&mut self) -> Result<()> {
        let mut config_message = self.lock_configuration()?.to_configuration_message();
        self.fill_checksum_and_transmit_configuration_message(&mut config_message)
    }

    pub fn serial_baud_rate(&self) -> Result<SerialBaudRate> {
        self.lock_transmitter()?.serial_baud_rate()
    }

    pub fn set_serial_baud_rate(&mut self, serial_baud_rate: SerialBaudRate) -> Result<()> {
        let mut config_message = serial_baud_rate.to_configuration_message();
        self.fill_checksum_and_transmit_configuration_message(&mut config_message)?;

        // Adapater does not respond while blinking.
        serial_baud_rate.sleep_while_blinking();

        // Set the new baud rate on underlying serial interface.
        self.lock_transmitter()?
            .set_serial_baud_rate(serial_baud_rate)
    }

    fn fill_checksum_and_transmit_configuration_message(
        &mut self,
        config_message: &mut [u8],
    ) -> Result<()> {
        Self::fill_checksum(config_message);

        self.lock_receiver()?.clear()?;
        self.lock_transmitter()?.transmit_all(config_message)?;

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

    fn lock_configuration(&self) -> Result<MutexGuard<Usb2CanConfiguration>> {
        self.configuration
            .lock()
            .map_err(|error| Error::Locking(format!("{} (configuration)", error)))
    }

    fn lock_transmitter(&self) -> Result<MutexGuard<Transmitter>> {
        self.transmitter
            .lock()
            .map_err(|error| Error::Locking(format!("{} (transmitter)", error)))
    }

    fn lock_receiver(&self) -> Result<MutexGuard<Receiver>> {
        self.receiver
            .lock()
            .map_err(|error| Error::Locking(format!("{} (receiver)", error)))
    }
}

// TODO: Nonblocking.
impl blocking::Can for Usb2Can {
    type Frame = Frame;
    type Error = Error;

    fn transmit(&mut self, frame: &Frame) -> Result<()> {
        self.lock_transmitter()?.transmit_frame(frame)
    }

    fn receive(&mut self) -> Result<Self::Frame> {
        self.lock_receiver()?.receive_frame()
    }
}

struct Transmitter {
    serial: Box<dyn SerialPort>,
}

impl Transmitter {
    fn new(serial: Box<dyn SerialPort>) -> Self {
        Self { serial }
    }

    fn name(&self) -> Option<String> {
        self.serial.name()
    }

    fn serial_baud_rate(&self) -> Result<SerialBaudRate> {
        self.serial
            .baud_rate()
            .map_err(|error| {
                Error::GetConfiguration(format!("Failed to get serial baud rate: {}", error))
            })
            .and_then(|value| {
                SerialBaudRate::try_from(value).map_err(|error| {
                    Error::GetConfiguration(format!(
                        "Unsupported baud rate on underlying serial interface: {}",
                        error
                    ))
                })
            })
    }

    fn set_serial_baud_rate(&mut self, serial_baud_rate: SerialBaudRate) -> Result<()> {
        self.serial
            .set_baud_rate(u32::from(serial_baud_rate))
            .map_err(|error| {
                Error::SetConfiguration(format!("Failed to set serial baud rate: {}", error))
            })
    }

    fn transmit_frame(&mut self, frame: &Frame) -> Result<()> {
        trace!("Transmitting frame: {:?}", frame);
        self.transmit_all(&frame.to_message())
    }

    fn transmit_all(&mut self, message: &[u8]) -> Result<()> {
        trace!("Writing into serial: {:?}", message);
        self.serial.write_all(message)?;
        self.serial.flush()?;
        Ok(())
    }
}

struct Receiver {
    read_buffer: BufReader<Box<dyn SerialPort>>,
    leftover: VecDeque<u8>,
    sync_attempts_left: usize,
}

impl Receiver {
    fn new(serial: Box<dyn SerialPort>) -> Self {
        Self {
            read_buffer: BufReader::new(serial),
            leftover: VecDeque::with_capacity(MAX_MESSAGE_SIZE),
            sync_attempts_left: SYNC_ATTEMPTS,
        }
    }

    fn receive_timeout(&self) -> Duration {
        self.read_buffer.get_ref().timeout()
    }

    fn set_receive_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.read_buffer
            .get_mut()
            .set_timeout(timeout)
            .map_err(|error| {
                Error::SetConfiguration(format!("Failed to set receive timeout: {}", error))
            })
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        if self.sync_attempts_left == 0 {
            self.receive_frame_no_sync()
        } else {
            loop {
                match self.receive_frame_no_sync() {
                    Ok(frame) => {
                        self.sync_attempts_left = 0;
                        break Ok(frame);
                    }

                    error @ Err(Error::SerialReadTimedOut) => break error,

                    Err(error) => {
                        debug!("Failed to sync: {}", error);

                        if self.sync_attempts_left == 0 {
                            break Err(Error::RecvUnexpected(format!(
                                "Failed to sync, no attempts left: {}",
                                error,
                            )));
                        }

                        self.sync_attempts_left -= 1;
                    }
                }
            }
        }
        .map(|frame| {
            trace!("Received frame: {:?}", frame);
            frame
        })
    }

    fn receive_frame_no_sync(&mut self) -> Result<Frame> {
        let mut received_bytes = Vec::with_capacity(MAX_MESSAGE_SIZE);

        let result = FrameReceiveState::read_frame(&mut || {
            self.receive_byte().map(|byte| {
                received_bytes.push(byte);
                byte
            })
        });

        if result.is_err() && !received_bytes.is_empty() {
            self.push_back_unused_bytes(&received_bytes[1..]);
        }

        result
    }

    fn receive_byte(&mut self) -> Result<u8> {
        if self.leftover.is_empty() {
            let mut byte = [0];
            self.read_buffer.read_exact(&mut byte).map_err(|error| {
                if error.kind() == io::ErrorKind::TimedOut {
                    Error::SerialReadTimedOut
                } else {
                    Error::from(error)
                }
            })?;

            trace!("Byte read from serial: {}", byte[0]);
            Ok(byte[0])
        } else {
            let byte = self
                .leftover
                .pop_front()
                .expect("Logic error: leftover is empty");
            trace!("Reusing leftover byte: {}", byte);
            Ok(byte)
        }
    }

    fn push_back_unused_bytes(&mut self, bytes: &[u8]) {
        self.leftover.extend(bytes);
    }

    fn clear(&mut self) -> Result<()> {
        self.read_buffer.get_mut().clear(ClearBuffer::All)?;
        self.leftover.clear();
        self.sync_attempts_left = SYNC_ATTEMPTS;
        Ok(())
    }
}

enum FrameReceiveState {
    EndOrHeader,
    Header,
    Type,
    Id { bytes_left: usize, frame: Frame },
    Data { byte_n: usize, frame: Frame },
    SkipRemote { bytes_left: usize, frame: Frame },
    Finished(Frame),
}

impl FrameReceiveState {
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
    use std::panic;

    use embedded_can::{ExtendedId, StandardId};
    use proptest::{arbitrary::any, collection::vec, proptest};

    use super::*;

    #[test]
    fn invalid_filter_conf() {
        let conf = Usb2CanConfiguration::new(CanBaudRate::R1000kBd);
        let conf = conf.set_filter(StandardId::ZERO.into(), ExtendedId::ZERO.into());
        assert!(matches!(conf, Err(Error::SetConfiguration(_))));

        let conf = Usb2CanConfiguration::new(CanBaudRate::R1000kBd);
        let conf = conf.set_filter(ExtendedId::ZERO.into(), StandardId::ZERO.into());
        assert!(matches!(conf, Err(Error::SetConfiguration(_))));
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

    proptest! {
        #[test]
        fn random_send_message_size_standard(
                id in StandardId::ZERO.as_raw()..=StandardId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = StandardId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new(id, &data).unwrap();
            let message = frame.to_message();
            assert!(message.len() <= MAX_VARIABLE_MESSAGE_SIZE);
        }

        #[test]
        fn random_send_message_size_extended(
                id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid extended ID");
            let frame = Frame::new(id, &data).unwrap();
            let message = frame.to_message();
            assert!(message.len() <= MAX_VARIABLE_MESSAGE_SIZE);
        }
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

    proptest! {
        #[test]
        fn random_serde_frame_standard(
                id in StandardId::ZERO.as_raw()..=StandardId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = StandardId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new(id, &data).unwrap();
            check_serialize_deserialize_frame(&frame);
        }

        #[test]
        fn random_serde_frame_extended(
                id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new(id, &data).unwrap();
            check_serialize_deserialize_frame(&frame);
        }

        fn random_serde_frame_remote_standard(
            id in StandardId::ZERO.as_raw()..=StandardId::MAX.as_raw(),
            dlc in 0..=MAX_DATA_LENGTH,
        ) {
            let id = StandardId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new_remote(id, dlc).unwrap();
            check_serialize_deserialize_frame(&frame);
        }

        fn random_serde_frame_remote_extended(
            id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
            dlc in 0..=MAX_DATA_LENGTH,
        ) {
            let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new_remote(id, dlc).unwrap();
            check_serialize_deserialize_frame(&frame);
        }
    }

    // TODO: Test serialization and deserialization with injected errors.

    fn check_serialize_deserialize_frame(frame: &Frame) {
        let message = frame.to_message();
        let mut reader_fn = into_reader_fn(message);

        let received_frame = FrameReceiveState::read_frame(&mut reader_fn).unwrap();

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
