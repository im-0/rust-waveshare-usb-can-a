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

use std::{
    borrow::Cow,
    collections::VecDeque,
    fmt::{self, Display},
    io::{self, BufReader, Read},
    result,
    sync::{Arc, Mutex, MutexGuard},
    thread::sleep,
    time::{Duration, Instant},
};

use embedded_can::{blocking, ExtendedId, Frame as _, Id, StandardId};
use serialport::{ClearBuffer, DataBits, SerialPort, StopBits};
use thiserror::Error;
use tracing::{debug, trace};

// Not really infinite...
// TODO: Fix rust-serialport to allow blocking mode.
const INFINITE_TIMEOUT: Duration = Duration::from_secs(60 * 60 * 24 * 365 * 100);

pub const DEFAULT_SERIAL_BAUD_RATE: SerialBaudRate = SerialBaudRate::R2000000Bd;
pub const DEFAULT_SERIAL_RECEIVE_TIMEOUT: Duration = Duration::from_millis(1000);
pub const DEFAULT_FRAME_DELAY_MULTIPLIER: f64 = 1.05;

pub const MAX_DATA_LENGTH: usize = 8;

pub const BASE_ID_BITS: usize = 11;
pub const EXTENDED_ID_BITS: usize = 29;
pub const EXTENDED_ID_EXTRA_BITS: usize = EXTENDED_ID_BITS - BASE_ID_BITS;

const CONFIGURATION_DELAY: Duration = Duration::from_millis(100);
const BLINK_DELAY: Duration = Duration::from_millis(700);

const MAX_FIXED_MESSAGE_SIZE: usize = 20;
const MAX_VARIABLE_MESSAGE_SIZE: usize = 15;
const MAX_MESSAGE_SIZE: usize = const_fn_max(MAX_FIXED_MESSAGE_SIZE, MAX_VARIABLE_MESSAGE_SIZE);
// Should be enough to sync as leftovers of a single frame are kept in the buffers.
const SYNC_ATTEMPTS: usize = MAX_MESSAGE_SIZE - 1;

// Protocol constants.
const PROTO_HEADER: u8 = 0b10101010;
const PROTO_HEADER_TYPE_FIXED: u8 = 0b01010101;

const PROTO_HEADER_TYPE_VARIABLE_FRAME_FLAG: u8 = 0b11000000;
const PROTO_HEADER_TYPE_VARIABLE_FRAME_MASK: u8 = PROTO_HEADER_TYPE_VARIABLE_FRAME_FLAG;
const PROTO_HEADER_TYPE_VARIABLE_FRAME_EXTENDED_FLAG: u8 = 0b00100000;
const PROTO_HEADER_TYPE_VARIABLE_FRAME_REMOTE_FLAG: u8 = 0b00010000;

const PROTO_FIXED_TYPE_FRAME_FLAG: u8 = 0b00000001;
const PROTO_FIXED_TYPE_CFG_FLAG: u8 = 0b00000010;
const PROTO_FIXED_TYPE_CFG_SET_SERIAL_BAUD_RATE_FLAG: u8 = 0b00000100;
const PROTO_FIXED_TYPE_CFG_SET_VARIABLE_FLAG: u8 = 0b00010000;

const PROTO_FIXED_TYPE_CFG_SET_STORED_ID_FILTER_FLAG: u8 = 0b00010000;
const PROTO_FIXED_TYPE_CFG_SET_STORED_ID_FILTER_BLOCKLIST_FLAG: u8 = 0b00000001;

const PROTO_FIXED_CFG_MODE_LOOPBACK_FLAG: u8 = 0b00000001;
const PROTO_FIXED_CFG_MODE_SILENT_FLAG: u8 = 0b00000010;

const PROTO_FIXED_ANY_STANDARD: u8 = 0x01;
const PROTO_FIXED_ANY_EXTENDED: u8 = 0x02;

const PROTO_FIXED_CFG_RETRANSMISSION_ENABLED: u8 = 0x00;
const PROTO_FIXED_CFG_RETRANSMISSION_DISABLED: u8 = 0x01;

const PROTO_FIXED_FRAME_DATA: u8 = 0x01;
const PROTO_FIXED_FRAME_REMOTE: u8 = 0x02;

const PROTO_VARIABLE_FRAME_LENGTH_MASK: u8 = 0b00001111;

const PROTO_VARIABLE_END: u8 = 0b01010101;

const fn const_fn_max(a: usize, b: usize) -> usize {
    [a, b][(a < b) as usize]
}

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
}

impl embedded_can::Error for Error {
    fn kind(&self) -> embedded_can::ErrorKind {
        // We don't have a way to distinguish between different kinds of errors.
        embedded_can::ErrorKind::Other
    }
}

pub type Result<T> = result::Result<T, Error>;

#[derive(Debug, Clone, PartialEq)]
pub struct Usb2CanBuilder {
    path: String,
    serial_baud_rate: SerialBaudRate,
    serial_receive_timeout: Duration,
    frame_delay_multiplier: f64,
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

    pub const fn frame_delay_multiplier(&self) -> f64 {
        self.frame_delay_multiplier
    }

    pub fn set_frame_delay_multiplier(mut self, frame_delay_multiplier: f64) -> Result<Self> {
        if frame_delay_multiplier.is_finite() && frame_delay_multiplier.is_sign_positive() {
            self.frame_delay_multiplier = frame_delay_multiplier;
            Ok(self)
        } else {
            Err(Error::SetConfiguration(
                "Frame delay multiplier must be a positive finite number".into(),
            ))
        }
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

    pub fn open_without_blink_delay(&self) -> Result<Usb2Can> {
        self.open_inner(false)
    }

    pub fn open(&self) -> Result<Usb2Can> {
        self.open_inner(true)
    }

    fn open_inner(&self, with_blink_delay: bool) -> Result<Usb2Can> {
        debug!("Opening USB2CAN with configuration {:?}", self);

        let serial = serialport::new(&self.path, u32::from(self.serial_baud_rate))
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::Two);
        let mut serial = serial.open()?;
        debug!("Serial port opened: {:?}", serial.name());

        let mut receiver = Receiver::new(serial.try_clone()?);
        receiver.set_receive_timeout(self.serial_receive_timeout)?;

        // Set timeout for writing to "infinite" value to make every write blocking.
        serial.set_timeout(INFINITE_TIMEOUT).map_err(|error| {
            Error::SetConfiguration(format!("Failed to set write timeout to MAX: {}", error))
        })?;
        let mut transmitter = Transmitter::new(serial);
        transmitter.set_frame_delay_multiplier(self.frame_delay_multiplier)?;

        if with_blink_delay {
            let delay = self.serial_baud_rate.to_blink_delay();
            debug!("Initial blink delay enabled: {:.03}s", delay.as_secs_f64());
            receiver.add_delay(delay);
            transmitter.add_delay(delay);
        };

        let mut usb2can = Usb2Can {
            receiver: Arc::new(Mutex::new(receiver)),
            transmitter: Arc::new(Mutex::new(transmitter)),
            configuration: Arc::new(Mutex::new(self.adapter_configuration.clone())),
        };

        usb2can.set_configuration_inner(None)?;

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
        frame_delay_multiplier: DEFAULT_FRAME_DELAY_MULTIPLIER,
        adapter_configuration: adapter_configuration.clone(),
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Usb2CanConfiguration {
    variable_encoding: bool,
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
            variable_encoding: false,
            can_baud_rate,
            receive_only_extended_frames: false,
            filter: ExtendedId::ZERO.into(),
            mask: ExtendedId::ZERO.into(),
            loopback: false,
            silent: false,
            automatic_retransmission: true,
        }
    }

    pub const fn variable_encoding(&self) -> bool {
        self.variable_encoding
    }

    #[must_use]
    pub const fn set_variable_encoding(mut self, variable_encoding: bool) -> Self {
        self.variable_encoding = variable_encoding;
        self
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

        let mut message = [
            // Header
            PROTO_HEADER,
            PROTO_HEADER_TYPE_FIXED,
            // Encoding type
            PROTO_FIXED_TYPE_CFG_FLAG
                | if self.variable_encoding {
                    // Use variable length protocol to send and receive data
                    PROTO_FIXED_TYPE_CFG_SET_VARIABLE_FLAG
                } else {
                    // Use fixed length protocol to send and receive data
                    0
                },
            // CAN bus speed
            self.can_baud_rate.to_config_value(),
            // Frame type
            if self.receive_only_extended_frames {
                PROTO_FIXED_ANY_EXTENDED
            } else {
                PROTO_FIXED_ANY_STANDARD
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
                PROTO_FIXED_CFG_MODE_LOOPBACK_FLAG
            } else {
                0x00
            } | if self.silent {
                PROTO_FIXED_CFG_MODE_SILENT_FLAG
            } else {
                0x00
            },
            // Automatic retransmission
            if self.automatic_retransmission {
                PROTO_FIXED_CFG_RETRANSMISSION_ENABLED
            } else {
                PROTO_FIXED_CFG_RETRANSMISSION_DISABLED
            },
            // Reserved
            0x00,
            0x00,
            0x00,
            0x00,
            // Checksum (will be filled in later)
            0x00,
        ];

        fill_checksum(&mut message);
        message
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
    fn to_blink_delay(self) -> Duration {
        let blinks = match self {
            Self::R9600Bd => 6,
            Self::R19200Bd => 5,
            Self::R38400Bd => 4,
            Self::R115200Bd => 3,
            Self::R1228800Bd => 2,
            Self::R2000000Bd => 1,
        };

        BLINK_DELAY * blinks
    }

    fn to_configuration_message(self) -> [u8; MAX_FIXED_MESSAGE_SIZE] {
        let mut message = [
            // Header
            PROTO_HEADER,
            PROTO_HEADER_TYPE_FIXED,
            // Set serial baud rate
            PROTO_FIXED_TYPE_CFG_FLAG | PROTO_FIXED_TYPE_CFG_SET_SERIAL_BAUD_RATE_FLAG,
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
        ];

        fill_checksum(&mut message);
        message
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

    fn to_bit_length(self) -> Duration {
        Duration::from_secs(1) / u32::from(self)
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

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StoredIdFilter {
    Disabled,
    AllowList(Vec<Id>),
    BlockList(Vec<Id>),
}

impl StoredIdFilter {
    fn to_configuration_message(&self) -> Result<Vec<u8>> {
        if self.to_ids().len() > u8::MAX as usize {
            return Err(Error::SetConfiguration(format!(
                "Stored ID filter list is too long (max {} IDs)",
                u8::MAX
            )));
        }

        let mut message = vec![
            // Header
            PROTO_HEADER,
            PROTO_HEADER_TYPE_FIXED,
            // Message type
            self.to_config_value(),
            // Number of IDs
            self.to_ids().len() as u8,
        ];

        // Place IDs
        for id in self.to_ids() {
            let bytes = id_to_bytes(*id);
            message.extend_from_slice(&bytes);
        }

        // Checksum (will be filled in later)
        message.push(0x00);

        fill_checksum(&mut message);
        Ok(message)
    }

    const fn to_config_value(&self) -> u8 {
        match self {
            Self::Disabled | Self::BlockList(_) => {
                PROTO_FIXED_TYPE_CFG_SET_STORED_ID_FILTER_FLAG
                    | PROTO_FIXED_TYPE_CFG_SET_STORED_ID_FILTER_BLOCKLIST_FLAG
            }

            Self::AllowList(_) => PROTO_FIXED_TYPE_CFG_SET_STORED_ID_FILTER_FLAG,
        }
    }

    fn to_ids(&self) -> &[Id] {
        match self {
            Self::Disabled => &[],
            Self::AllowList(ids) | Self::BlockList(ids) => ids,
        }
    }
}

#[derive(Clone)]
pub struct Usb2Can {
    receiver: Arc<Mutex<Receiver>>,
    transmitter: Arc<Mutex<Transmitter>>,
    configuration: Arc<Mutex<Usb2CanConfiguration>>,
}

impl Usb2Can {
    pub fn name(&self) -> Result<String> {
        self.lock_transmitter()
            .and_then(|transmitter| transmitter.name())
    }

    pub fn serial_receive_timeout(&self) -> Result<Duration> {
        self.lock_receiver()
            .map(|receiver| receiver.receive_timeout())
    }

    pub fn set_serial_receive_timeout(&mut self, timeout: Duration) -> Result<()> {
        debug!(
            "Changing serial receive timeout to {:.03}s",
            timeout.as_secs_f64()
        );
        self.lock_receiver()
            .and_then(|mut receiver| receiver.set_receive_timeout(timeout))
    }

    pub fn configuration(&self) -> Result<Usb2CanConfiguration> {
        Ok(self.lock_configuration()?.clone())
    }

    pub fn set_configuration(&mut self, configuration: &Usb2CanConfiguration) -> Result<()> {
        self.set_configuration_inner(Some(configuration.clone()))
    }

    fn set_configuration_inner(
        &mut self,
        configuration: Option<Usb2CanConfiguration>,
    ) -> Result<()> {
        debug!("Changing adapter configuration to {:?}...", configuration);

        let mut receiver_guard = self.lock_receiver()?;
        let mut transmitter_guard = self.lock_transmitter()?;
        let mut configuration_guard = self.lock_configuration()?;

        let config_message = if let Some(configuration) = configuration {
            let config_message = configuration.to_configuration_message();
            *configuration_guard = configuration;
            config_message
        } else {
            configuration_guard.to_configuration_message()
        };

        receiver_guard.clear()?;
        transmitter_guard.transmit_all(&config_message)?;

        // Adapter needs some time to process the configuration change.
        receiver_guard.add_delay(CONFIGURATION_DELAY);
        transmitter_guard.add_delay(CONFIGURATION_DELAY);

        debug!("Done changing adapter configuration!");
        Ok(())
    }

    pub fn serial_baud_rate(&self) -> Result<SerialBaudRate> {
        self.lock_transmitter()?.serial_baud_rate()
    }

    pub fn set_serial_baud_rate(&mut self, serial_baud_rate: SerialBaudRate) -> Result<()> {
        debug!("Changing serial baud rate to {}...", serial_baud_rate);

        let mut receiver_guard = self.lock_receiver()?;
        let mut transmitter_guard = self.lock_transmitter()?;

        let config_message = serial_baud_rate.to_configuration_message();
        transmitter_guard.transmit_all(&config_message)?;

        // Adapater does not respond while blinking.
        let delay = serial_baud_rate.to_blink_delay();

        receiver_guard.add_delay(delay);
        transmitter_guard.add_delay(delay);

        // Set the new baud rate on underlying serial interface.
        transmitter_guard.set_serial_baud_rate(serial_baud_rate)?;
        receiver_guard.clear()?;

        debug!("Done changing serial baud rate!");
        Ok(())
    }

    pub fn frame_delay_multiplier(&self) -> Result<f64> {
        self.lock_transmitter()
            .map(|transmitter| transmitter.frame_delay_multiplier())
    }

    pub fn set_frame_delay_multiplier(&mut self, frame_delay_multiplier: f64) -> Result<()> {
        debug!(
            "Changing frame delay multiplier to {:.03}...",
            frame_delay_multiplier
        );
        self.lock_transmitter().and_then(|mut transmitter| {
            transmitter.set_frame_delay_multiplier(frame_delay_multiplier)
        })
    }

    pub fn set_stored_id_filter(&mut self, filter: &StoredIdFilter) -> Result<()> {
        debug!("Changing stored ID filter to {:?}...", filter);

        let mut receiver_guard = self.lock_receiver()?;
        let mut transmitter_guard = self.lock_transmitter()?;

        let config_message = filter.to_configuration_message()?;
        transmitter_guard.transmit_all(&config_message)?;
        transmitter_guard.add_delay(CONFIGURATION_DELAY);

        receiver_guard.clear()?;

        debug!("Done changing stored ID filter!");
        Ok(())
    }

    pub fn delay(&self) -> Result<()> {
        debug!("Sleeping to remove receiver and transmitter delays...");

        let receiver_guard = self.lock_receiver()?;
        let transmitter_guard = self.lock_transmitter()?;

        receiver_guard.delay();
        transmitter_guard.delay();

        Ok(())
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

// TODO: Add async variant.
impl blocking::Can for Usb2Can {
    type Frame = Frame;
    type Error = Error;

    fn transmit(&mut self, frame: &Frame) -> Result<()> {
        trace!("Transmitting frame: {:?}", frame);

        let mut transmitter_guard = self.lock_transmitter()?;
        let configuration_guard = self.lock_configuration()?;

        if configuration_guard.variable_encoding() {
            transmitter_guard.transmit_all(&frame.to_message_variable())?;
        } else {
            transmitter_guard.transmit_all(&frame.to_message_fixed())?;
        }

        let delay_multipyer = transmitter_guard.frame_delay_multiplier();
        if delay_multipyer > f64::EPSILON {
            let delay =
                configuration_guard.can_baud_rate.to_bit_length() * frame.length_bound_in_bits();
            let delay = delay.mul_f64(delay_multipyer);
            transmitter_guard.add_delay(delay);
        }

        Ok(())
    }

    fn receive(&mut self) -> Result<Self::Frame> {
        let mut receiver_guard = self.lock_receiver()?;
        let configuration_guard = self.lock_configuration()?;

        let result = receiver_guard.receive_frame(configuration_guard.variable_encoding());

        if let Ok(frame) = &result {
            trace!("Received frame: {:?}", frame);
        } else if matches!(result, Err(Error::RecvUnexpected(_))) {
            receiver_guard.resync();
        };

        result
    }
}

struct Transmitter {
    serial: Box<dyn SerialPort>,
    ready_at: Instant,
    frame_delay_multiplier: f64,
}

impl Transmitter {
    fn new(serial: Box<dyn SerialPort>) -> Self {
        Self {
            serial,
            ready_at: Instant::now(),
            frame_delay_multiplier: 0.0,
        }
    }

    fn name(&self) -> Result<String> {
        self.serial
            .name()
            .ok_or_else(|| Error::GetConfiguration("Failed to get serial port name".into()))
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
        self.delay();
        self.serial
            .set_baud_rate(u32::from(serial_baud_rate))
            .map_err(|error| {
                Error::SetConfiguration(format!("Failed to set serial baud rate: {}", error))
            })
    }

    const fn frame_delay_multiplier(&self) -> f64 {
        self.frame_delay_multiplier
    }

    fn set_frame_delay_multiplier(&mut self, frame_delay_multiplier: f64) -> Result<()> {
        if frame_delay_multiplier.is_finite() && frame_delay_multiplier.is_sign_positive() {
            self.frame_delay_multiplier = frame_delay_multiplier;
            Ok(())
        } else {
            Err(Error::SetConfiguration(
                "Frame delay multiplier must be a positive finite number".into(),
            ))
        }
    }

    fn transmit_all(&mut self, message: &[u8]) -> Result<()> {
        self.delay();
        trace!("Writing into serial: {:?}", message);
        self.serial.write_all(message)?;
        self.serial.flush()?;
        Ok(())
    }

    fn delay(&self) {
        let now = Instant::now();
        if now < self.ready_at {
            let delay = self.ready_at - now;
            trace!(
                "Sleeping for {:.03}s before transmitting anything...",
                delay.as_secs_f64()
            );
            sleep(delay);
        }
    }

    fn add_delay(&mut self, delay: Duration) {
        let now = Instant::now();
        if now > self.ready_at {
            self.ready_at = now + delay;
        } else {
            self.ready_at += delay;
        }
    }
}

fn fill_checksum(message: &mut [u8]) {
    message[message.len() - 1] = generate_checksum(message);
}

fn generate_checksum(message: &mut [u8]) -> u8 {
    message[2..message.len() - 1]
        .iter()
        .fold(0u8, |acc, &x| acc.wrapping_add(x))
}

struct Receiver {
    read_buffer: BufReader<Box<dyn SerialPort>>,
    leftover: VecDeque<u8>,
    sync_attempts_left: usize,
    ready_at: Instant,
}

impl Receiver {
    fn new(serial: Box<dyn SerialPort>) -> Self {
        Self {
            read_buffer: BufReader::new(serial),
            leftover: VecDeque::with_capacity(MAX_MESSAGE_SIZE),
            sync_attempts_left: SYNC_ATTEMPTS,
            ready_at: Instant::now(),
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

    fn receive_frame(&mut self, variable: bool) -> Result<Frame> {
        if self.sync_attempts_left == 0 {
            self.receive_frame_no_sync(variable)
        } else {
            loop {
                match self.receive_frame_no_sync(variable) {
                    Ok(frame) => {
                        self.sync_attempts_left = 0;
                        break Ok(frame);
                    }

                    Err(error @ Error::RecvUnexpected(_)) => {
                        debug!("Failed to sync: {}", error);

                        if self.sync_attempts_left == 0 {
                            break Err(Error::RecvUnexpected(format!(
                                "Failed to sync, no attempts left: {}",
                                error,
                            )));
                        }

                        self.sync_attempts_left -= 1;
                    }

                    Err(error) => break Err(error),
                }
            }
        }
    }

    fn receive_frame_no_sync(&mut self, variable: bool) -> Result<Frame> {
        let mut received_bytes = Vec::with_capacity(MAX_MESSAGE_SIZE);

        let mut read_byte = || {
            self.receive_byte().map(|byte| {
                received_bytes.push(byte);
                byte
            })
        };

        let result = if variable {
            VariableFrameReceiveState::read_frame(&mut read_byte)
        } else {
            FixedFrameReceiveState::read_frame(&mut read_byte)
        };

        if !received_bytes.is_empty() {
            match result {
                Err(Error::RecvUnexpected(_)) => self.push_back_unused_bytes(&received_bytes[1..]),
                Err(_) => self.push_back_unused_bytes(&received_bytes),
                Ok(_) => {}
            }
        }

        result
    }

    fn receive_byte(&mut self) -> Result<u8> {
        if self.leftover.is_empty() {
            self.delay();

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
        self.resync();
        Ok(())
    }

    fn resync(&mut self) {
        self.sync_attempts_left = SYNC_ATTEMPTS;
    }

    fn delay(&self) {
        let now = Instant::now();
        if now < self.ready_at {
            let delay = self.ready_at - now;
            debug!(
                "Sleeping for {:.03}s before trying to receive anything...",
                delay.as_secs_f64()
            );
            sleep(delay);
        }
    }

    fn add_delay(&mut self, delay: Duration) {
        let now = Instant::now();
        if now > self.ready_at {
            self.ready_at = now + delay;
        } else {
            self.ready_at += delay;
        }
    }
}

enum FixedFrameReceiveState {
    HeaderOrHeaderFixed,
    HeaderFixed,
    HeaderFrame,
    StandardOrExtended,
    DataOrRemote { extended: bool },
    Id { bytes_left: usize, frame: Frame },
    SkipUnusedId { bytes_left: usize, frame: Frame },
    DataLength(Frame),
    Data { byte_n: usize, frame: Frame },
    Skip { bytes_left: usize, frame: Frame },
    Reserved(Frame),
    Checksum(Frame),
    Finished { checksum: u8, frame: Frame },
}

impl FixedFrameReceiveState {
    fn read_frame(read_byte: &mut impl FnMut() -> Result<u8>) -> Result<Frame> {
        let mut state = Self::HeaderOrHeaderFixed;

        let mut our_checksum = 0u8;

        loop {
            let byte = read_byte()?;

            match state {
                Self::HeaderFrame
                | Self::StandardOrExtended
                | Self::DataOrRemote { .. }
                | Self::Id { .. }
                | Self::SkipUnusedId { .. }
                | Self::DataLength(_)
                | Self::Data { .. }
                | Self::Skip { .. }
                | Self::Reserved { .. } => {
                    our_checksum = our_checksum.wrapping_add(byte);
                }

                Self::HeaderOrHeaderFixed
                | Self::HeaderFixed
                | Self::Checksum(_)
                | Self::Finished { .. } => {}
            }

            state = state.advance(byte)?;
            if let Self::Finished { checksum, frame } = state {
                if checksum == our_checksum {
                    break Ok(frame);
                } else {
                    break Err(Error::RecvUnexpected(format!(
                        "Wrong checksum, message has 0x{:02x} (calculated 0x{:02x})",
                        checksum, our_checksum
                    )));
                }
            }
        }
    }

    fn advance(self, byte: u8) -> Result<Self> {
        let next_state = match (self, byte) {
            (Self::HeaderOrHeaderFixed, PROTO_HEADER) => Self::HeaderFixed,
            (Self::HeaderOrHeaderFixed, PROTO_HEADER_TYPE_FIXED) => Self::HeaderFrame,
            (Self::HeaderOrHeaderFixed, byte) => {
                return Err(Error::RecvUnexpected(format!(
                    "Expected header, received 0x{:02x} (!= 0x{:02x})",
                    byte, PROTO_HEADER
                )))
            }

            (Self::HeaderFixed, PROTO_HEADER_TYPE_FIXED) => Self::HeaderFrame,
            (Self::HeaderFixed, byte) => {
                return Err(Error::RecvUnexpected(format!(
                    "Expected fixed header, received 0x{:02x} (!= 0x{:02x})",
                    byte, PROTO_HEADER_TYPE_FIXED
                )))
            }

            (Self::HeaderFrame, PROTO_FIXED_TYPE_FRAME_FLAG) => Self::StandardOrExtended,
            (Self::HeaderFrame, byte) => {
                return Err(Error::RecvUnexpected(format!(
                    "Expected frame header, received 0x{:02x} (!= 0x{:02x})",
                    byte, PROTO_FIXED_TYPE_FRAME_FLAG
                )))
            }

            (Self::StandardOrExtended, PROTO_FIXED_ANY_STANDARD) => Self::DataOrRemote { extended: false },
            (Self::StandardOrExtended, PROTO_FIXED_ANY_EXTENDED) => Self::DataOrRemote { extended: true },
            (Self::StandardOrExtended, byte) => {
                return Err(Error::RecvUnexpected(format!(
                    "Expected frame type (standard or extended), received 0x{:02x} (!= {{0x{:02x}, 0x{:02x}}})",
                    byte, PROTO_FIXED_ANY_STANDARD, PROTO_FIXED_ANY_EXTENDED
                )))
            }

            (Self::DataOrRemote { extended }, PROTO_FIXED_FRAME_DATA) => if extended {
                Self::Id { bytes_left: 3, frame: Frame {
                    id: ExtendedId::ZERO.into(),
                    data: FrameData::Data(Vec::with_capacity(MAX_DATA_LENGTH)),
                }}
            } else {
                Self::Id { bytes_left: 1, frame: Frame {
                    id: StandardId::ZERO.into(),
                    data: FrameData::Data(Vec::with_capacity(MAX_DATA_LENGTH)),
                }}
            }
            (Self::DataOrRemote { extended }, PROTO_FIXED_FRAME_REMOTE) => if extended {
                Self::Id { bytes_left: 3, frame: Frame {
                    id: ExtendedId::ZERO.into(),
                    data: FrameData::Remote(0),
                }}
            } else {
                Self::Id { bytes_left: 1, frame: Frame {
                    id: StandardId::ZERO.into(),
                    data: FrameData::Remote(0),
                }}
            }
            (Self::DataOrRemote { .. }, byte) => {
                return Err(Error::RecvUnexpected(format!(
                    "Expected frame type (data or remote), received 0x{:02x} (!= {{0x{:02x}, 0x{:02x}}})",
                    byte, PROTO_FIXED_FRAME_DATA, PROTO_FIXED_FRAME_REMOTE
                )))
            }

            (
                Self::Id {
                    bytes_left,
                    mut frame,
                },
                byte,
            ) => {
                frame.add_byte_to_id(bytes_left, byte)?;

                if bytes_left == 0 {
                    if frame.is_extended() {
                        Self::DataLength(frame)
                    } else {
                        Self::SkipUnusedId { bytes_left: 2, frame }
                    }
                } else {
                    Self::Id {
                        bytes_left: bytes_left - 1,
                        frame,
                    }
                }
            }

            (
                Self::SkipUnusedId {
                    mut bytes_left,
                    frame,
                },
                0, // Unused fields: standard frame, thus need to skip unused two zeros.
            ) => {
                bytes_left -= 1;
                if bytes_left == 0 {
                    Self::DataLength(frame)
                } else {
                    Self::SkipUnusedId { bytes_left, frame }
                }
            }
            (Self::SkipUnusedId { .. }, byte) => {
                return Err(Error::RecvUnexpected(format!(
                    "Expected unused ID byte, received 0x{:02x} (!= 0x00)",
                    byte
                )))
            }

            (
                Self::DataLength(mut frame),
                byte,
            ) if byte as usize <= MAX_DATA_LENGTH => {
                match frame.data {
                    FrameData::Data(ref mut data) => {
                        if byte == 0 {
                            Self::Skip { bytes_left: MAX_DATA_LENGTH, frame }
                        } else {
                            data.resize(byte as usize, 0);
                            Self::Data {
                                byte_n: 0,
                                frame,
                            }
                        }
                    }

                    FrameData::Remote(ref mut dlc) => {
                        *dlc = byte as usize;
                        Self::Skip {
                            bytes_left: MAX_DATA_LENGTH,
                            frame,
                        }
                    }
                }
            }
            (
                Self::DataLength { .. },
                byte,
            ) => {
                return Err(Error::RecvUnexpected(format!(
                    "Expected data length, received 0x{:02x} (data length > {})",
                    byte, MAX_DATA_LENGTH
                )));
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
                        if data.len() == MAX_DATA_LENGTH {
                            Self::Reserved(frame)
                        } else {
                            Self::Skip { bytes_left: MAX_DATA_LENGTH - data.len(), frame }
                        }
                    } else {
                        Self::Data { byte_n, frame }
                    }
                }

                FrameData::Remote(_) => {
                    unreachable!("Logic error: trying to receive data for remote frame")
                }
            },

            (
                Self::Skip {
                    mut bytes_left,
                    frame,
                },
                _, // Unused fields: remote frame or less than 8 bytes of data.
            ) => {
                bytes_left -= 1;
                if bytes_left == 0 {
                    Self::Reserved(frame)
                } else {
                    Self::Skip { bytes_left, frame }
                }
            }

            (Self::Reserved(frame), 0) => Self::Checksum(frame),
            (Self::Reserved(_), byte) => {
                return Err(Error::RecvUnexpected(format!(
                    "Expected reserved, received 0x{:02x} (!= 0x00)",
                    byte
                )))
            }

            (Self::Checksum(frame), byte) => Self::Finished { checksum: byte, frame },

            (Self::Finished { .. }, _) => {
                unreachable!("Logic error: trying to advance finished state")
            }
        };

        Ok(next_state)
    }
}

enum VariableFrameReceiveState {
    EndOrHeader,
    Header,
    Type,
    Id { bytes_left: usize, frame: Frame },
    Data { byte_n: usize, frame: Frame },
    SkipRemote { bytes_left: usize, frame: Frame },
    Finished(Frame),
}

impl VariableFrameReceiveState {
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
            (Self::EndOrHeader, PROTO_VARIABLE_END) => Self::Header,
            (Self::EndOrHeader, PROTO_HEADER) => Self::Type,
            (Self::EndOrHeader, byte) => {
                return Err(Error::RecvUnexpected(format!(
                    "Expected end or header, received 0x{:02x} (!= {{0x{:02x}, 0x{:02x}}})",
                    byte, PROTO_VARIABLE_END, PROTO_HEADER
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
                if byte & PROTO_HEADER_TYPE_VARIABLE_FRAME_MASK
                    != PROTO_HEADER_TYPE_VARIABLE_FRAME_FLAG
                {
                    return Err(Error::RecvUnexpected(format!(
                        "Expected type, received 0x{:02x} (0x{:02x} not set)",
                        byte, PROTO_HEADER_TYPE_VARIABLE_FRAME_FLAG
                    )));
                }

                let zero_id = if byte & PROTO_HEADER_TYPE_VARIABLE_FRAME_EXTENDED_FLAG == 0 {
                    Id::Standard(StandardId::ZERO)
                } else {
                    Id::Extended(ExtendedId::ZERO)
                };

                let data_len = (byte & PROTO_VARIABLE_FRAME_LENGTH_MASK) as usize;
                if data_len > MAX_DATA_LENGTH {
                    return Err(Error::RecvUnexpected(format!(
                        "Expected type, received 0x{:02x} (data length > {})",
                        byte, MAX_DATA_LENGTH
                    )));
                }

                let frame = if byte & PROTO_HEADER_TYPE_VARIABLE_FRAME_REMOTE_FLAG == 0 {
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
                frame.add_byte_to_id(bytes_left, byte)?;

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
                unreachable!("Logic error: trying to advance finished state")
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
    fn to_message_fixed(&self) -> [u8; MAX_FIXED_MESSAGE_SIZE] {
        let id = id_to_bytes(self.id);

        #[allow(clippy::get_first)]
        let mut message = [
            // Header
            PROTO_HEADER,
            PROTO_HEADER_TYPE_FIXED,
            // This is normal data or remote frame
            PROTO_FIXED_TYPE_FRAME_FLAG,
            // Frame type: standard or extended
            if self.is_extended() {
                PROTO_FIXED_ANY_EXTENDED
            } else {
                PROTO_FIXED_ANY_STANDARD
            },
            // Frame type: data or remote
            if self.is_remote_frame() {
                PROTO_FIXED_FRAME_REMOTE
            } else {
                PROTO_FIXED_FRAME_DATA
            },
            // Frame ID
            id[0],
            id[1],
            id[2],
            id[3],
            // Data length
            self.dlc() as u8,
            // Data
            self.data().get(0).copied().unwrap_or(0),
            self.data().get(1).copied().unwrap_or(0),
            self.data().get(2).copied().unwrap_or(0),
            self.data().get(3).copied().unwrap_or(0),
            self.data().get(4).copied().unwrap_or(0),
            self.data().get(5).copied().unwrap_or(0),
            self.data().get(6).copied().unwrap_or(0),
            self.data().get(7).copied().unwrap_or(0),
            // Reserved
            0x00,
            // Checksum (will be filled in later)
            0x00,
        ];

        fill_checksum(&mut message);
        message
    }

    fn to_message_variable(&self) -> Vec<u8> {
        let mut message = Vec::with_capacity(MAX_VARIABLE_MESSAGE_SIZE);

        // Header
        message.push(PROTO_HEADER);

        // Type
        let mut message_type = PROTO_HEADER_TYPE_VARIABLE_FRAME_FLAG;
        if self.is_extended() {
            message_type |= PROTO_HEADER_TYPE_VARIABLE_FRAME_EXTENDED_FLAG;
        }
        if self.is_remote_frame() {
            message_type |= PROTO_HEADER_TYPE_VARIABLE_FRAME_REMOTE_FLAG;
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
        message.push(PROTO_VARIABLE_END);

        message
    }

    fn length_bound_in_bits(&self) -> u32 {
        // This is maximum possible length, actual length may be shorter.
        // See https://en.wikipedia.org/wiki/CAN_bus#Bit_stuffing for
        // details.

        let data_len = if self.is_data_frame() {
            self.dlc() as u32 * 8
        } else {
            0
        };

        let frame_without_data = if self.is_standard() {
            44 + (34 + data_len - 1) / 4
        } else {
            64 + (54 + data_len - 1) / 4
        };

        let interframe_spacing = 3;

        data_len + frame_without_data + interframe_spacing
    }

    fn add_byte_to_id(&mut self, bytes_left: usize, byte: u8) -> Result<()> {
        self.id = match self.id {
            Id::Standard(standard_id) => {
                let standard_id = standard_id.as_raw() | (byte as u16) << ((1 - bytes_left) * 8);
                Id::Standard(StandardId::new(standard_id).ok_or_else(|| {
                    Error::RecvUnexpected(format!("Invalid standard ID: 0x{:04x}", standard_id))
                })?)
            }

            Id::Extended(extended_id) => {
                let extended_id = extended_id.as_raw() | (byte as u32) << ((3 - bytes_left) * 8);
                Id::Extended(ExtendedId::new(extended_id).ok_or_else(|| {
                    Error::RecvUnexpected(format!("Invalid extended ID: 0x{:08x}", extended_id))
                })?)
            }
        };

        Ok(())
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
    use proptest::{arbitrary::any, collection::vec, proptest};
    use rustix::{
        fd::OwnedFd,
        pty::{grantpt, openpt, ptsname, unlockpt, OpenptFlags},
    };

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
        let message = frame.to_message_variable();
        assert!(message.len() < MAX_VARIABLE_MESSAGE_SIZE);
    }

    #[test]
    fn max_send_message_size() {
        // Largest message.
        let frame = Frame::new(ExtendedId::MAX, &[0, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        let message = frame.to_message_variable();
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
            let message = frame.to_message_variable();
            assert!(message.len() <= MAX_VARIABLE_MESSAGE_SIZE);
        }

        #[test]
        fn random_send_message_size_extended(
                id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid extended ID");
            let frame = Frame::new(id, &data).unwrap();
            let message = frame.to_message_variable();
            assert!(message.len() <= MAX_VARIABLE_MESSAGE_SIZE);
        }
    }

    #[test]
    fn standard_id_endianness() {
        let frame = Frame::new_remote(StandardId::MAX, 0).unwrap();
        let message = frame.to_message_variable();
        assert_eq!(message[2], 0xff);
        assert_eq!(message[3], 0x07);
    }

    #[test]
    fn extended_id_endianness() {
        let frame = Frame::new_remote(ExtendedId::MAX, 0).unwrap();
        let message = frame.to_message_variable();
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
    fn fixed_standard_data_frame() {
        let frame = Frame::new(
            StandardId::new(0x123).unwrap(),
            &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88],
        )
        .unwrap();
        let message = frame.to_message_fixed();
        assert_eq!(
            message,
            [
                0xaa, 0x55, // Header
                0x01, // This is normal data or remote frame
                0x01, // Frame type: standard or extended
                0x01, // Frame type: data or remote
                0x23, 0x01, 0x00, 0x00, // Frame ID
                0x08, // Data length
                0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, // Data
                0x00, // Reserved
                0x93, // Checksum
            ]
        );
    }

    #[test]
    fn fixed_extended_data_frame() {
        let frame = Frame::new(
            ExtendedId::new(0x12345678).unwrap(),
            &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
        )
        .unwrap();
        let message = frame.to_message_fixed();
        assert_eq!(
            message,
            [
                0xaa, 0x55, // Header
                0x01, // This is normal data or remote frame
                0x02, // Frame type: standard or extended
                0x01, // Frame type: data or remote
                0x78, 0x56, 0x34, 0x12, // Frame ID
                0x08, // Data length
                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, // Data
                0x00, // Reserved
                0x44, // Checksum
            ]
        );
    }

    #[test]
    fn variable_standard_data_frame_short() {
        let frame = Frame::new(StandardId::MAX, &[]).unwrap();
        let message = frame.to_message_variable();
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
    fn variable_standard_data_frame_long() {
        let frame = Frame::new(
            StandardId::MAX,
            &[0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef],
        )
        .unwrap();
        let message = frame.to_message_variable();
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
    fn variable_extended_data_frame_short() {
        let frame = Frame::new(ExtendedId::MAX, &[]).unwrap();
        let message = frame.to_message_variable();
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
    fn variable_extended_data_frame_long() {
        let frame = Frame::new(
            ExtendedId::MAX,
            &[0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef],
        )
        .unwrap();
        let message = frame.to_message_variable();
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
    fn variable_standard_remote_frame_short() {
        let frame = Frame::new_remote(StandardId::MAX, 0).unwrap();
        let message = frame.to_message_variable();
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
    fn variable_standard_remote_frame_long() {
        let frame = Frame::new_remote(StandardId::MAX, MAX_DATA_LENGTH).unwrap();
        let message = frame.to_message_variable();
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
    fn variable_extended_remote_frame_short() {
        let frame = Frame::new_remote(ExtendedId::MAX, 0).unwrap();
        let message = frame.to_message_variable();
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
    fn variable_extended_remote_frame_long() {
        let frame = Frame::new_remote(ExtendedId::MAX, MAX_DATA_LENGTH).unwrap();
        let message = frame.to_message_variable();
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
    fn variable_serde_frame_standard() {
        let frame = Frame::new(StandardId::MAX, &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();
        check_variable_serialize_deserialize_frame(&frame);

        let frame = Frame::new(StandardId::MAX, &[]).unwrap();
        check_variable_serialize_deserialize_frame(&frame);
    }

    #[test]
    fn variable_serde_frame_extended() {
        let frame = Frame::new(ExtendedId::MAX, &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();
        check_variable_serialize_deserialize_frame(&frame);

        let frame = Frame::new(ExtendedId::MAX, &[]).unwrap();
        check_variable_serialize_deserialize_frame(&frame);
    }

    #[test]
    fn variable_serde_frame_remote_standard() {
        let frame = Frame::new_remote(StandardId::MAX, MAX_DATA_LENGTH).unwrap();
        check_variable_serialize_deserialize_frame(&frame);

        let frame = Frame::new_remote(StandardId::MAX, 0).unwrap();
        check_variable_serialize_deserialize_frame(&frame);
    }

    #[test]
    fn variable_serde_frame_remote_extended() {
        let frame = Frame::new_remote(ExtendedId::MAX, MAX_DATA_LENGTH).unwrap();
        check_variable_serialize_deserialize_frame(&frame);

        let frame = Frame::new_remote(ExtendedId::MAX, 0).unwrap();
        check_variable_serialize_deserialize_frame(&frame);
    }

    #[test]
    fn fixed_serde_frame_standard() {
        let frame = Frame::new(StandardId::MAX, &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();
        check_fixed_serialize_deserialize_frame(&frame);

        let frame = Frame::new(StandardId::MAX, &[]).unwrap();
        check_fixed_serialize_deserialize_frame(&frame);
    }

    #[test]
    fn fixed_serde_frame_extended() {
        let frame = Frame::new(ExtendedId::MAX, &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();
        check_fixed_serialize_deserialize_frame(&frame);

        let frame = Frame::new(ExtendedId::MAX, &[]).unwrap();
        check_fixed_serialize_deserialize_frame(&frame);
    }

    #[test]
    fn fixed_serde_frame_remote_standard() {
        let frame = Frame::new_remote(StandardId::MAX, MAX_DATA_LENGTH).unwrap();
        check_fixed_serialize_deserialize_frame(&frame);

        let frame = Frame::new_remote(StandardId::MAX, 0).unwrap();
        check_fixed_serialize_deserialize_frame(&frame);
    }

    #[test]
    fn fixed_serde_frame_remote_extended() {
        let frame = Frame::new_remote(ExtendedId::MAX, MAX_DATA_LENGTH).unwrap();
        check_fixed_serialize_deserialize_frame(&frame);

        let frame = Frame::new_remote(ExtendedId::MAX, 0).unwrap();
        check_fixed_serialize_deserialize_frame(&frame);
    }

    proptest! {
        #[test]
        fn variable_random_serde_frame_standard(
                id in StandardId::ZERO.as_raw()..=StandardId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = StandardId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new(id, &data).unwrap();
            check_variable_serialize_deserialize_frame(&frame);
        }

        #[test]
        fn variable_random_serde_frame_extended(
                id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new(id, &data).unwrap();
            check_variable_serialize_deserialize_frame(&frame);
        }

        fn variable_random_serde_frame_remote_standard(
            id in StandardId::ZERO.as_raw()..=StandardId::MAX.as_raw(),
            dlc in 0..=MAX_DATA_LENGTH,
        ) {
            let id = StandardId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new_remote(id, dlc).unwrap();
            check_variable_serialize_deserialize_frame(&frame);
        }

        fn variable_random_serde_frame_remote_extended(
            id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
            dlc in 0..=MAX_DATA_LENGTH,
        ) {
            let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new_remote(id, dlc).unwrap();
            check_variable_serialize_deserialize_frame(&frame);
        }
    }

    proptest! {
        #[test]
        fn fixed_random_serde_frame_standard(
                id in StandardId::ZERO.as_raw()..=StandardId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = StandardId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new(id, &data).unwrap();
            check_fixed_serialize_deserialize_frame(&frame);
        }

        #[test]
        fn fixed_random_serde_frame_extended(
                id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new(id, &data).unwrap();
            check_fixed_serialize_deserialize_frame(&frame);
        }

        fn fixed_random_serde_frame_remote_standard(
            id in StandardId::ZERO.as_raw()..=StandardId::MAX.as_raw(),
            dlc in 0..=MAX_DATA_LENGTH,
        ) {
            let id = StandardId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new_remote(id, dlc).unwrap();
            check_fixed_serialize_deserialize_frame(&frame);
        }

        fn fixed_random_serde_frame_remote_extended(
            id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
            dlc in 0..=MAX_DATA_LENGTH,
        ) {
            let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new_remote(id, dlc).unwrap();
            check_fixed_serialize_deserialize_frame(&frame);
        }
    }

    // TODO: Test serialization and deserialization with injected errors.

    fn check_variable_serialize_deserialize_frame(frame: &Frame) {
        let message = frame.to_message_variable();
        let mut reader_fn = into_reader_fn(message);

        let received_frame = VariableFrameReceiveState::read_frame(&mut reader_fn).unwrap();

        assert_eq!(reader_fn().unwrap(), PROTO_VARIABLE_END);
        assert!(reader_fn().is_err());

        assert_eq!(frame, &received_frame);
    }

    fn check_fixed_serialize_deserialize_frame(frame: &Frame) {
        let message = frame.to_message_fixed();
        let mut reader_fn = into_reader_fn(message.to_vec());

        let received_frame = FixedFrameReceiveState::read_frame(&mut reader_fn).unwrap();

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

    proptest! {
        #[test]
        fn random_length_bound_frame_standard(
                id in StandardId::ZERO.as_raw()..=StandardId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = StandardId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new(id, &data).unwrap();
            // https://en.wikipedia.org/wiki/CAN_bus#Bit_stuffing
            assert!(frame.length_bound_in_bits() <= 132 + 3);
        }

        #[test]
        fn random_length_bound_frame_extended(
                id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new(id, &data).unwrap();
            // https://en.wikipedia.org/wiki/CAN_bus#Bit_stuffing
            assert!(frame.length_bound_in_bits() <= 157 + 3);
        }
    }

    #[test]
    fn stored_id_filter_disabled() {
        let filter = StoredIdFilter::Disabled;
        assert_eq!(
            filter.to_configuration_message().unwrap(),
            [
                0xaa, 0x55, // Header
                0x11, // Type
                0x00, // Number of IDs
                0x11, // Checksum
            ]
        );
    }

    #[test]
    fn stored_id_filter_block_list() {
        let filter = StoredIdFilter::BlockList(vec![ExtendedId::new(0x0ff1f2f3).unwrap().into()]);
        assert_eq!(
            filter.to_configuration_message().unwrap(),
            [
                0xaa, 0x55, // Header
                0x11, // Type
                0x01, // Number of IDs
                0xf3, 0xf2, 0xf1, 0x0f, // ID
                0xf7, // Checksum
            ]
        );

        let filter = StoredIdFilter::BlockList(vec![
            ExtendedId::new(0x0ff1f2f3).unwrap().into(),
            ExtendedId::new(0x0ee1e2e3).unwrap().into(),
        ]);
        assert_eq!(
            filter.to_configuration_message().unwrap(),
            [
                0xaa, 0x55, // Header
                0x11, // Type
                0x02, // Number of IDs
                0xf3, 0xf2, 0xf1, 0x0f, // ID
                0xe3, 0xe2, 0xe1, 0x0e, // ID
                0xac, // Checksum
            ]
        );
    }

    #[test]
    fn stored_id_filter_allow_list() {
        let filter = StoredIdFilter::AllowList(vec![
            ExtendedId::new(0x0ff1f2f3).unwrap().into(),
            ExtendedId::new(0x0ee1e2e3).unwrap().into(),
        ]);
        assert_eq!(
            filter.to_configuration_message().unwrap(),
            [
                0xaa, 0x55, // Header
                0x10, // Type
                0x02, // Number of IDs
                0xf3, 0xf2, 0xf1, 0x0f, // ID
                0xe3, 0xe2, 0xe1, 0x0e, // ID
                0xab, // Checksum
            ]
        );
    }

    #[test]
    fn stored_id_filter_disabled_is_empty_block_list() {
        let filter_disabled = StoredIdFilter::Disabled;
        let filter_empty_blocklist = StoredIdFilter::BlockList(vec![]);
        assert_eq!(
            filter_disabled.to_configuration_message().unwrap(),
            filter_empty_blocklist.to_configuration_message().unwrap()
        );
    }

    #[test]
    fn rust_serialport_cloned_timeout() {
        // Check that the timeout in the cloned serial port is independent of the original.
        let (_mpt, mut port) = open_pty();
        let mut port_clone = port.try_clone().expect("serialport::try_clone() failed");

        port.set_timeout(Duration::from_secs(1)).unwrap();
        port_clone.set_timeout(Duration::from_secs(2)).unwrap();

        assert_eq!(port.timeout(), Duration::from_secs(1));
        assert_eq!(port_clone.timeout(), Duration::from_secs(2));
    }

    #[test]
    fn rust_serialport_large_timeout() {
        let (_mpt, mut port) = open_pty();
        port.set_timeout(INFINITE_TIMEOUT).unwrap();
        port.write_all(&[0x00]).unwrap();
    }

    fn open_pty() -> (OwnedFd, Box<dyn SerialPort>) {
        let mpt = openpt(OpenptFlags::RDWR | OpenptFlags::NOCTTY).expect("posix_openpt() failed");
        grantpt(&mpt).expect("grantpt() failed");
        unlockpt(&mpt).expect("unlockpt() failed");

        let pt_path = ptsname(&mpt, vec![0u8; 1024]).expect("ptsname() failed");
        let pt_path =
            String::from_utf8(pt_path.into_bytes()).expect("ptsname() returned invalid UTF-8");

        (
            mpt,
            serialport::new(pt_path, 9600)
                .open()
                .expect("serialport::open() failed"),
        )
    }
}
