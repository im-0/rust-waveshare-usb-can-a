use embedded_can::{ExtendedId, Frame as FrameTrait, Id, StandardId};

use crate::{
    CanBaudRate, Frame, SerialBaudRate, StoredIdFilter, Usb2CanConfiguration, MAX_DATA_LENGTH,
};

const MAX_FIXED_MESSAGE_SIZE: usize = 20;
const MAX_VARIABLE_MESSAGE_SIZE: usize = 15;
pub(crate) const MAX_MESSAGE_SIZE: usize =
    const_fn_max(MAX_FIXED_MESSAGE_SIZE, MAX_VARIABLE_MESSAGE_SIZE);
// Should be enough to sync as leftovers of a single frame are kept in the buffers.
pub(crate) const SYNC_ATTEMPTS: usize = MAX_MESSAGE_SIZE - 1;

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

pub(crate) trait NewRecvUnexpectedError {
    fn new_recv_unexpected_error(message: impl Into<String>) -> Self;
}

trait ToProtocolValue {
    fn to_protocol_value(&self) -> u8;
}

pub(crate) trait ToFixedMessage {
    type Output;

    fn to_fixed_message(&self) -> Self::Output;
}

pub(crate) trait ToVariableMessage {
    fn to_variable_message(&self) -> Vec<u8>;
}

impl ToFixedMessage for Usb2CanConfiguration {
    type Output = [u8; MAX_FIXED_MESSAGE_SIZE];

    fn to_fixed_message(&self) -> Self::Output {
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
            self.can_baud_rate.to_protocol_value(),
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

impl ToProtocolValue for SerialBaudRate {
    fn to_protocol_value(&self) -> u8 {
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

impl ToFixedMessage for SerialBaudRate {
    type Output = [u8; MAX_FIXED_MESSAGE_SIZE];

    fn to_fixed_message(&self) -> Self::Output {
        let mut message = [
            // Header
            PROTO_HEADER,
            PROTO_HEADER_TYPE_FIXED,
            // Set serial baud rate
            PROTO_FIXED_TYPE_CFG_FLAG | PROTO_FIXED_TYPE_CFG_SET_SERIAL_BAUD_RATE_FLAG,
            // Serial baud rate
            self.to_protocol_value(),
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
}

impl ToProtocolValue for CanBaudRate {
    fn to_protocol_value(&self) -> u8 {
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

impl ToProtocolValue for StoredIdFilter {
    fn to_protocol_value(&self) -> u8 {
        if self.is_disabled() || self.is_block() {
            PROTO_FIXED_TYPE_CFG_SET_STORED_ID_FILTER_FLAG
                | PROTO_FIXED_TYPE_CFG_SET_STORED_ID_FILTER_BLOCKLIST_FLAG
        } else {
            PROTO_FIXED_TYPE_CFG_SET_STORED_ID_FILTER_FLAG
        }
    }
}

impl ToFixedMessage for StoredIdFilter {
    type Output = Vec<u8>;

    fn to_fixed_message(&self) -> Self::Output {
        let mut message = vec![
            // Header
            PROTO_HEADER,
            PROTO_HEADER_TYPE_FIXED,
            // Message type
            self.to_protocol_value(),
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
        message
    }
}

pub(crate) enum FixedFrameReceiveState {
    HeaderOrHeaderFixed,
    HeaderFixed,
    HeaderFrame,
    StandardOrExtended,
    DataOrRemote {
        frame: PartialFrame,
    },
    Id {
        bytes_left: usize,
        frame: PartialFrame,
    },
    SkipUnusedId {
        bytes_left: usize,
        frame: PartialFrame,
    },
    DataLength(PartialFrame),
    Data {
        byte_n: usize,
        frame: PartialFrame,
    },
    SkipUnusedData {
        bytes_left: usize,
        frame: PartialFrame,
    },
    Reserved(PartialFrame),
    Checksum(PartialFrame),
    Finished {
        checksum: u8,
        frame: PartialFrame,
    },
}

impl FixedFrameReceiveState {
    pub(crate) fn read_frame<E>(read_byte: &mut impl FnMut() -> Result<u8, E>) -> Result<Frame, E>
    where
        E: NewRecvUnexpectedError,
    {
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
                | Self::SkipUnusedData { .. }
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
                    break Ok(frame.into());
                } else {
                    break Err(E::new_recv_unexpected_error(format!(
                        "Wrong checksum, message has 0x{:02x} (calculated 0x{:02x})",
                        checksum, our_checksum
                    )));
                }
            }
        }
    }

    fn advance<E>(self, byte: u8) -> Result<Self, E>
    where
        E: NewRecvUnexpectedError,
    {
        let next_state = match (self, byte) {
            (Self::HeaderOrHeaderFixed, PROTO_HEADER) => Self::HeaderFixed,
            (Self::HeaderOrHeaderFixed, PROTO_HEADER_TYPE_FIXED) => Self::HeaderFrame,
            (Self::HeaderOrHeaderFixed, byte) => {
                return Err(E::new_recv_unexpected_error(format!(
                    "Expected header, received 0x{:02x} (!= 0x{:02x})",
                    byte, PROTO_HEADER
                )))
            }

            (Self::HeaderFixed, PROTO_HEADER_TYPE_FIXED) => Self::HeaderFrame,
            (Self::HeaderFixed, byte) => {
                return Err(E::new_recv_unexpected_error(format!(
                    "Expected fixed header, received 0x{:02x} (!= 0x{:02x})",
                    byte, PROTO_HEADER_TYPE_FIXED
                )))
            }

            (Self::HeaderFrame, PROTO_FIXED_TYPE_FRAME_FLAG) => Self::StandardOrExtended,
            (Self::HeaderFrame, byte) => {
                return Err(E::new_recv_unexpected_error(format!(
                    "Expected frame header, received 0x{:02x} (!= 0x{:02x})",
                    byte, PROTO_FIXED_TYPE_FRAME_FLAG
                )))
            }

            (Self::StandardOrExtended, PROTO_FIXED_ANY_STANDARD) => Self::DataOrRemote { frame: PartialFrame::new_standard() },
            (Self::StandardOrExtended, PROTO_FIXED_ANY_EXTENDED) => Self::DataOrRemote { frame: PartialFrame::new_extended() },
            (Self::StandardOrExtended, byte) => {
                return Err(E::new_recv_unexpected_error(format!(
                    "Expected frame type (standard or extended), received 0x{:02x} (!= {{0x{:02x}, 0x{:02x}}})",
                    byte, PROTO_FIXED_ANY_STANDARD, PROTO_FIXED_ANY_EXTENDED
                )))
            }

            (Self::DataOrRemote { mut frame }, PROTO_FIXED_FRAME_DATA) => {
                frame.is_remote = false;
                match frame.id {
                    Id::Standard(_) => Self::Id { bytes_left: 1, frame },
                    Id::Extended(_) => Self::Id { bytes_left: 3, frame },
                }
            }
            (Self::DataOrRemote { mut frame }, PROTO_FIXED_FRAME_REMOTE) => {
                frame.is_remote = true;
                match frame.id {
                    Id::Standard(_) => Self::Id { bytes_left: 1, frame },
                    Id::Extended(_) => Self::Id { bytes_left: 3, frame },
                }
            }
            (Self::DataOrRemote { .. }, byte) => {
                return Err(E::new_recv_unexpected_error(format!(
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
                    match frame.id {
                        Id::Standard(_) => Self::SkipUnusedId { bytes_left: 2, frame },
                        Id::Extended(_) => Self::DataLength(frame),
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
                return Err(E::new_recv_unexpected_error(format!(
                    "Expected unused ID byte, received 0x{:02x} (!= 0x00)",
                    byte
                )))
            }

            (
                Self::DataLength(mut frame),
                byte,
            ) if byte as usize <= MAX_DATA_LENGTH => {
                if frame.is_remote {
                    frame.dlc = byte as usize;
                    Self::SkipUnusedData {
                        bytes_left: MAX_DATA_LENGTH,
                        frame,
                    }
                } else if byte == 0 {
                    Self::SkipUnusedData { bytes_left: MAX_DATA_LENGTH, frame }
                } else {
                    frame.data.resize(byte as usize, 0);
                    Self::Data {
                        byte_n: 0,
                        frame,
                    }
                }
            }
            (
                Self::DataLength { .. },
                byte,
            ) => {
                return Err(E::new_recv_unexpected_error(format!(
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
            ) => {
                if frame.is_remote {
                    unreachable!("Logic error: trying to receive data for remote frame")
                } else {
                    frame.data[byte_n] = byte;

                    byte_n += 1;
                    if byte_n == frame.data.len() {
                        if frame.data.len() == MAX_DATA_LENGTH {
                            Self::Reserved(frame)
                        } else {
                            Self::SkipUnusedData { bytes_left: MAX_DATA_LENGTH - frame.data.len(), frame }
                        }
                    } else {
                        Self::Data { byte_n, frame }
                    }
                }
            },

            (
                Self::SkipUnusedData {
                    mut bytes_left,
                    frame,
                },
                _, // Unused fields: remote frame or less than 8 bytes of data.
            ) => {
                bytes_left -= 1;
                if bytes_left == 0 {
                    Self::Reserved(frame)
                } else {
                    Self::SkipUnusedData { bytes_left, frame }
                }
            }

            (Self::Reserved(frame), 0) => Self::Checksum(frame),
            (Self::Reserved(_), byte) => {
                return Err(E::new_recv_unexpected_error(format!(
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

pub(crate) enum VariableFrameReceiveState {
    EndOrHeader,
    Header,
    Type,
    Id {
        bytes_left: usize,
        frame: PartialFrame,
    },
    Data {
        byte_n: usize,
        frame: PartialFrame,
    },
    SkipRemote {
        bytes_left: usize,
        frame: PartialFrame,
    },
    Finished(PartialFrame),
}

impl VariableFrameReceiveState {
    pub(crate) fn read_frame<E>(read_byte: &mut impl FnMut() -> Result<u8, E>) -> Result<Frame, E>
    where
        E: NewRecvUnexpectedError,
    {
        let mut state = Self::EndOrHeader;

        Ok(loop {
            state = state.advance(read_byte()?)?;
            if let Self::Finished(frame) = state {
                break frame.into();
            }
        })
    }

    fn advance<E>(self, byte: u8) -> Result<Self, E>
    where
        E: NewRecvUnexpectedError,
    {
        let next_state = match (self, byte) {
            (Self::EndOrHeader, PROTO_VARIABLE_END) => Self::Header,
            (Self::EndOrHeader, PROTO_HEADER) => Self::Type,
            (Self::EndOrHeader, byte) => {
                return Err(E::new_recv_unexpected_error(format!(
                    "Expected end or header, received 0x{:02x} (!= {{0x{:02x}, 0x{:02x}}})",
                    byte, PROTO_VARIABLE_END, PROTO_HEADER
                )))
            }

            (Self::Header, PROTO_HEADER) => Self::Type,
            (Self::Header, byte) => {
                return Err(E::new_recv_unexpected_error(format!(
                    "Expected header, received 0x{:02x} (!= 0x{:02x})",
                    byte, PROTO_HEADER
                )))
            }

            (Self::Type, byte) => {
                if byte & PROTO_HEADER_TYPE_VARIABLE_FRAME_MASK
                    != PROTO_HEADER_TYPE_VARIABLE_FRAME_FLAG
                {
                    return Err(E::new_recv_unexpected_error(format!(
                        "Expected type, received 0x{:02x} (0x{:02x} not set)",
                        byte, PROTO_HEADER_TYPE_VARIABLE_FRAME_FLAG
                    )));
                }

                let mut frame = if byte & PROTO_HEADER_TYPE_VARIABLE_FRAME_EXTENDED_FLAG == 0 {
                    PartialFrame::new_standard()
                } else {
                    PartialFrame::new_extended()
                };

                let data_len = (byte & PROTO_VARIABLE_FRAME_LENGTH_MASK) as usize;
                if data_len > MAX_DATA_LENGTH {
                    return Err(E::new_recv_unexpected_error(format!(
                        "Expected type, received 0x{:02x} (data length > {})",
                        byte, MAX_DATA_LENGTH
                    )));
                }

                if byte & PROTO_HEADER_TYPE_VARIABLE_FRAME_REMOTE_FLAG == 0 {
                    frame.is_remote = false;
                    frame.data.resize(data_len, 0);
                } else {
                    frame.is_remote = true;
                    frame.dlc = data_len;
                };

                Self::Id {
                    bytes_left: match frame.id {
                        Id::Standard(_) => 1,
                        Id::Extended(_) => 3,
                    },

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
                    if frame.is_remote && (frame.dlc != 0) {
                        Self::SkipRemote {
                            bytes_left: frame.dlc,
                            frame,
                        }
                    } else if frame.data.is_empty() {
                        Self::Finished(frame)
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
            ) => {
                if frame.is_remote {
                    unreachable!("Logic error: trying to receive data for remote frame")
                } else {
                    frame.data[byte_n] = byte;

                    byte_n += 1;
                    if byte_n == frame.data.len() {
                        Self::Finished(frame)
                    } else {
                        Self::Data { byte_n, frame }
                    }
                }
            }

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

impl<T> ToFixedMessage for T
where
    T: FrameTrait,
{
    type Output = [u8; MAX_FIXED_MESSAGE_SIZE];

    fn to_fixed_message(&self) -> Self::Output {
        let id = id_to_bytes(self.id());

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
}

impl<T> ToVariableMessage for T
where
    T: FrameTrait,
{
    fn to_variable_message(&self) -> Vec<u8> {
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
}

pub(crate) struct PartialFrame {
    id: Id,
    is_remote: bool,
    dlc: usize,
    data: Vec<u8>,
}

impl PartialFrame {
    fn new_standard() -> Self {
        Self {
            id: StandardId::ZERO.into(),
            is_remote: false,
            dlc: 0,
            data: Vec::with_capacity(MAX_DATA_LENGTH),
        }
    }

    fn new_extended() -> Self {
        Self {
            id: ExtendedId::ZERO.into(),
            is_remote: false,
            dlc: 0,
            data: Vec::with_capacity(MAX_DATA_LENGTH),
        }
    }

    fn add_byte_to_id<E>(&mut self, bytes_left: usize, byte: u8) -> Result<(), E>
    where
        E: NewRecvUnexpectedError,
    {
        self.id = match self.id {
            Id::Standard(standard_id) => {
                let standard_id = standard_id.as_raw() | (byte as u16) << ((1 - bytes_left) * 8);
                Id::Standard(StandardId::new(standard_id).ok_or_else(|| {
                    E::new_recv_unexpected_error(format!(
                        "Invalid standard ID: 0x{:04x}",
                        standard_id
                    ))
                })?)
            }

            Id::Extended(extended_id) => {
                let extended_id = extended_id.as_raw() | (byte as u32) << ((3 - bytes_left) * 8);
                Id::Extended(ExtendedId::new(extended_id).ok_or_else(|| {
                    E::new_recv_unexpected_error(format!(
                        "Invalid extended ID: 0x{:08x}",
                        extended_id
                    ))
                })?)
            }
        };

        Ok(())
    }
}

impl From<PartialFrame> for Frame {
    fn from(frame: PartialFrame) -> Self {
        if frame.is_remote {
            Self::new_remote(frame.id, frame.dlc)
                .expect("Logic error: invalid remote partial frame")
        } else {
            Self::new(frame.id, &frame.data).expect("Logic error: invalid data partial frame")
        }
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

fn fill_checksum(message: &mut [u8]) {
    message[message.len() - 1] = generate_checksum(message);
}

fn generate_checksum(message: &mut [u8]) -> u8 {
    message[2..message.len() - 1]
        .iter()
        .fold(0u8, |acc, &x| acc.wrapping_add(x))
}

#[cfg(test)]
mod tests {
    use proptest::{arbitrary::any, collection::vec, proptest};

    use super::*;

    #[derive(Debug)]
    struct RecvUnexpected;

    impl NewRecvUnexpectedError for RecvUnexpected {
        fn new_recv_unexpected_error(_message: impl Into<String>) -> Self {
            Self {}
        }
    }

    #[test]
    fn small_send_message_size() {
        // Smallest message.
        let frame = Frame::new_remote(StandardId::MAX, 0).unwrap();
        let message = frame.to_variable_message();
        assert!(message.len() < MAX_VARIABLE_MESSAGE_SIZE);
    }

    #[test]
    fn max_send_message_size() {
        // Largest message.
        let frame = Frame::new(ExtendedId::MAX, &[0, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        let message = frame.to_variable_message();
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
            let message = frame.to_variable_message();
            assert!(message.len() <= MAX_VARIABLE_MESSAGE_SIZE);
        }

        #[test]
        fn random_send_message_size_extended(
                id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid extended ID");
            let frame = Frame::new(id, &data).unwrap();
            let message = frame.to_variable_message();
            assert!(message.len() <= MAX_VARIABLE_MESSAGE_SIZE);
        }
    }

    #[test]
    fn standard_id_endianness() {
        let frame = Frame::new_remote(StandardId::MAX, 0).unwrap();
        let message = frame.to_variable_message();
        assert_eq!(message[2], 0xff);
        assert_eq!(message[3], 0x07);
    }

    #[test]
    fn extended_id_endianness() {
        let frame = Frame::new_remote(ExtendedId::MAX, 0).unwrap();
        let message = frame.to_variable_message();
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
        let message = frame.to_fixed_message();
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
        let message = frame.to_fixed_message();
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
        let message = frame.to_variable_message();
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
        let message = frame.to_variable_message();
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
        let message = frame.to_variable_message();
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
        let message = frame.to_variable_message();
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
        let message = frame.to_variable_message();
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
        let message = frame.to_variable_message();
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
        let message = frame.to_variable_message();
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
        let message = frame.to_variable_message();
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
        let message = frame.to_variable_message();
        let mut reader_fn = into_reader_fn(message);

        let received_frame = VariableFrameReceiveState::read_frame(&mut reader_fn).unwrap();

        assert_eq!(reader_fn().unwrap(), PROTO_VARIABLE_END);
        assert!(reader_fn().is_err());

        assert_eq!(frame, &received_frame);
    }

    fn check_fixed_serialize_deserialize_frame(frame: &Frame) {
        let message = frame.to_fixed_message();
        let mut reader_fn = into_reader_fn(message.to_vec());

        let received_frame = FixedFrameReceiveState::read_frame(&mut reader_fn).unwrap();

        assert!(reader_fn().is_err());

        assert_eq!(frame, &received_frame);
    }

    fn into_reader_fn(data: Vec<u8>) -> impl FnMut() -> Result<u8, RecvUnexpected> {
        let mut iter = data.into_iter();
        move || iter.next().ok_or(RecvUnexpected {})
    }

    #[test]
    fn stored_id_filter_disabled() {
        let filter = StoredIdFilter::new_disabled();
        assert_eq!(
            filter.to_fixed_message(),
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
        let filter =
            StoredIdFilter::new_block(vec![ExtendedId::new(0x0ff1f2f3).unwrap().into()]).unwrap();
        assert_eq!(
            filter.to_fixed_message(),
            [
                0xaa, 0x55, // Header
                0x11, // Type
                0x01, // Number of IDs
                0xf3, 0xf2, 0xf1, 0x0f, // ID
                0xf7, // Checksum
            ]
        );

        let filter = StoredIdFilter::new_block(vec![
            ExtendedId::new(0x0ff1f2f3).unwrap().into(),
            ExtendedId::new(0x0ee1e2e3).unwrap().into(),
        ])
        .unwrap();
        assert_eq!(
            filter.to_fixed_message(),
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
        let filter = StoredIdFilter::new_allow(vec![
            ExtendedId::new(0x0ff1f2f3).unwrap().into(),
            ExtendedId::new(0x0ee1e2e3).unwrap().into(),
        ])
        .unwrap();
        assert_eq!(
            filter.to_fixed_message(),
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
        let filter_disabled = StoredIdFilter::new_disabled();
        let filter_empty_blocklist = StoredIdFilter::new_block(vec![]).unwrap();
        assert_eq!(
            filter_disabled.to_fixed_message(),
            filter_empty_blocklist.to_fixed_message(),
        );
    }
}
