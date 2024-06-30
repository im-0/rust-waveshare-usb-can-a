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
    fmt::{self, Display},
    result,
    time::Duration,
};

use embedded_can::{ExtendedId, Frame as FrameTrait, Id};
use thiserror::Error;

mod proto;
#[cfg(test)]
mod tests;

#[cfg(feature = "sync")]
pub mod sync;
#[cfg(feature = "tokio")]
pub mod tokio;

const CONFIGURATION_DELAY: Duration = Duration::from_millis(100);
const BLINK_DELAY: Duration = Duration::from_millis(700);

pub const DEFAULT_SERIAL_BAUD_RATE: SerialBaudRate = SerialBaudRate::R2000000Bd;
pub const DEFAULT_FRAME_DELAY_MULTIPLIER: f64 = 1.05;

pub const MAX_DATA_LENGTH: usize = 8;

pub const BASE_ID_BITS: usize = 11;
pub const EXTENDED_ID_BITS: usize = 29;
pub const EXTENDED_ID_EXTRA_BITS: usize = EXTENDED_ID_BITS - BASE_ID_BITS;

type Result<T> = core::result::Result<T, CommonConfigurationError>;

#[derive(Error, Debug)]
pub enum CommonConfigurationError {
    #[error("Common configuration error: {0}")]
    Error(String),
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
                return Err(CommonConfigurationError::Error(
                    "Filter and mask must have the same type (standard or extended)".to_string(),
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
}

impl Display for SerialBaudRate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} Bd", u32::from(*self))
    }
}

impl TryFrom<u32> for SerialBaudRate {
    type Error = CommonConfigurationError;

    fn try_from(value: u32) -> result::Result<Self, Self::Error> {
        match value {
            9_600 => Ok(Self::R9600Bd),
            19_200 => Ok(Self::R19200Bd),
            38_400 => Ok(Self::R38400Bd),
            115_200 => Ok(Self::R115200Bd),
            1_228_800 => Ok(Self::R1228800Bd),
            2_000_000 => Ok(Self::R2000000Bd),
            _ => Err(CommonConfigurationError::Error(format!(
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
    type Error = CommonConfigurationError;

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
            _ => Err(CommonConfigurationError::Error(format!(
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
enum StoredIdFilterInner {
    Disabled,
    AllowList(Vec<Id>),
    BlockList(Vec<Id>),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StoredIdFilter(StoredIdFilterInner);

impl StoredIdFilter {
    pub const fn new_disabled() -> Self {
        Self(StoredIdFilterInner::Disabled)
    }

    pub fn new_allow(ids: Vec<Id>) -> Result<Self> {
        Self::check_ids_len(&ids)?;
        Ok(Self(StoredIdFilterInner::AllowList(ids)))
    }

    pub fn new_block(ids: Vec<Id>) -> Result<Self> {
        Self::check_ids_len(&ids)?;
        Ok(Self(StoredIdFilterInner::BlockList(ids)))
    }

    fn check_ids_len(ids: &[Id]) -> Result<()> {
        if ids.len() > u8::MAX as usize {
            Err(CommonConfigurationError::Error(format!(
                "Stored ID filter list is too long (max {} IDs)",
                u8::MAX
            )))
        } else {
            Ok(())
        }
    }

    pub const fn is_disabled(&self) -> bool {
        matches!(&self.0, StoredIdFilterInner::Disabled)
    }

    pub const fn is_allow(&self) -> bool {
        matches!(&self.0, StoredIdFilterInner::AllowList(_))
    }

    pub const fn is_block(&self) -> bool {
        matches!(&self.0, StoredIdFilterInner::BlockList(_))
    }

    pub fn to_ids(&self) -> &[Id] {
        match &self.0 {
            StoredIdFilterInner::Disabled => &[],
            StoredIdFilterInner::AllowList(ids) | StoredIdFilterInner::BlockList(ids) => ids,
        }
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
}

impl FrameTrait for Frame {
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
