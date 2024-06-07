use std::{str::FromStr, time::Duration};

use anyhow::{anyhow, ensure, Context, Error, Result};
use clap::{Parser, Subcommand};
use embedded_can::{ExtendedId, Frame as _, Id, StandardId};
use waveshare_usb_can_a::{CanBaudRate, Frame, SerialBaudRate, EXTENDED_ID_EXTRA_BITS};

#[derive(Parser)]
#[command(about, version)]
pub(crate) struct Cli {
    /// Path to the serial device file for USB2CAN adapter.
    pub serial_path: String,

    /// Command.
    #[command(subcommand)]
    pub subcommand: SubCommand,
}

#[derive(Subcommand)]
pub(crate) enum SubCommand {
    /// Dump traffic on CAN bus.
    #[command()]
    Dump(DumpOptions),

    /// Inject frames into CAN bus.
    #[command()]
    Inject(InjectOptions),

    /// CAN bus throughput test.
    #[command()]
    Perf(PerfOptions),

    /// Configure serial baud rate. Serial baud rate persists when adapter is powered off
    /// and can be reset to the default 2000000 Bd by pressing the button during power on.
    #[command()]
    SetSerialBaudRate(SetSerialBaudRateOptions),

    /// Try to reset the adapter to factory defaults. This sets the serial baud rate to 2000000 Bd.
    #[command()]
    ResetToFactoryDefaults,

    /// Run self-test.
    #[command()]
    SelfTest(SelfTestOptions),
}

#[derive(Parser)]
pub(crate) struct DumpOptions {
    /// Serial receive timeout.
    #[arg(
        short = 't',
        long,
        value_name = "SECONDS",
        default_value = "86400",
        value_parser = parse_duration_s,
    )]
    pub receive_timeout: Duration,

    /// Receive only extended frames (CAN 2.0B). By default, both standard and extended frames
    /// (CAN 2.0A & CAN 2.0B) are received.
    #[arg(short = 'e', long)]
    pub receive_only_extended_frames: bool,

    /// Filter and mask for ID filtering. Examples: "7ff/7ff" (standard ID), "3ffff.7ff/3ffff.7ff" (extended ID).
    /// There is no filtering by default.
    #[arg(short = 'f', long, value_name = "FILTER/MASK")]
    pub filter_with_mask: Option<FilterWithMask>,

    /// Filter out already seen frames. By default, all frames are received and printed.
    /// The mask is applied to the frame before checking if it has been seen.
    ///
    /// Example masks:
    ///
    ///     "3ffff.7ff:ffffffffffffffff" - Test full frame for uniqueness.
    ///     "3ffff.7ff:ff" - Test ID and first data byte for uniqueness.
    ///     "3ffff.7ff:00ff" - Test ID and second data byte for uniqueness.
    ///     "3ffff.7ff:00" - Test only ID uniqueness.
    #[arg(
        short = 'u',
        long,
        value_name = "MASK",
        value_parser = parse_data_frame,
        verbatim_doc_comment,
    )]
    pub unique: Option<Frame>,

    /// Serial baud rate. Supported values: 9600, 19200, 38400, 115200, 1228800, 2000000.
    #[arg(
        short = 'b',
        long,
        value_name = "BAUD_RATE",
        default_value = "2000000",
        value_parser = parse_serial_baud_rate,
    )]
    pub serial_baud_rate: SerialBaudRate,

    /// CAN bus baud rate. Supported values: 5000, 10000, 20000, 50000, 100000, 125000, 200000,
    /// 250000, 400000, 500000, 800000, 1000000.
    #[arg(
        value_name = "BAUD_RATE",
        value_parser = parse_can_baud_rate,
    )]
    pub can_baud_rate: CanBaudRate,
}

#[derive(Parser)]
pub(crate) struct InjectOptions {
    /// Enable automatic frame retransmission.
    #[arg(short = 'r', long)]
    pub automatic_retransmission: bool,

    /// Inject packets in an infinte loop. Argument is the delay between each loop iteration.
    #[arg(
        short = 'l',
        long,
        value_name = "MILLISECONDS",
        value_parser = parse_duration_ms,
    )]
    pub loop_inject: Option<Duration>,

    /// Serial baud rate. Supported values: 9600, 19200, 38400, 115200, 1228800, 2000000.
    #[arg(
        short = 'b',
        long,
        value_name = "BAUD_RATE",
        default_value = "2000000",
        value_parser = parse_serial_baud_rate,
    )]
    pub serial_baud_rate: SerialBaudRate,

    /// Frame delay multiplier. The delay between frames is multiplied by this value.
    #[arg(
        short = 'd',
        long,
        value_name = "DELAY_MULTIPLIER",
        default_value = "1.05"
    )]
    pub frame_delay_multiplier: f64,

    /// CAN bus baud rate. Supported values: 5000, 10000, 20000, 50000, 100000, 125000, 200000,
    /// 250000, 400000, 500000, 800000, 1000000.
    #[arg(
        value_name = "BAUD_RATE",
        value_parser = parse_can_baud_rate,
    )]
    pub can_baud_rate: CanBaudRate,

    /// Frames to inject.
    #[arg(
        value_name = "FRAME",
        required = true,
        value_parser = parse_frame,
    )]
    pub frames: Vec<Frame>,
}

#[derive(Parser)]
pub(crate) struct PerfOptions {
    /// Serial receive timeout.
    #[arg(
        short = 't',
        long,
        value_name = "SECONDS",
        default_value = "86400",
        value_parser = parse_duration_s,
    )]
    pub receive_timeout: Duration,

    /// Serial baud rate. Supported values: 9600, 19200, 38400, 115200, 1228800, 2000000.
    #[arg(
        short = 'b',
        long,
        value_name = "BAUD_RATE",
        default_value = "2000000",
        value_parser = parse_serial_baud_rate,
    )]
    pub serial_baud_rate: SerialBaudRate,

    /// Frame delay multiplier. The delay between frames is multiplied by this value.
    #[arg(
        short = 'd',
        long,
        value_name = "DELAY_MULTIPLIER",
        default_value = "1.05"
    )]
    pub frame_delay_multiplier: f64,

    /// CAN bus baud rate. Supported values: 5000, 10000, 20000, 50000, 100000, 125000, 200000,
    /// 250000, 400000, 500000, 800000, 1000000.
    #[arg(
        value_name = "BAUD_RATE",
        value_parser = parse_can_baud_rate,
    )]
    pub can_baud_rate: CanBaudRate,

    /// Transmit or receive.
    #[command(subcommand)]
    pub transmit_or_receive: PerfSubCommand,
}

#[derive(Subcommand)]
pub(crate) enum PerfSubCommand {
    /// Transmit frames.
    #[command()]
    Transmit,

    /// Receive frames.
    #[command()]
    Receive,
}

#[derive(Parser)]
pub(crate) struct SetSerialBaudRateOptions {
    /// Old serial baud rate. Supported values: 9600, 19200, 38400, 115200, 1228800, 2000000.
    #[arg(
        short = 'b',
        long,
        value_name = "OLD_BAUD_RATE",
        default_value = "2000000",
        value_parser = parse_serial_baud_rate,
    )]
    pub old_serial_baud_rate: SerialBaudRate,

    /// Old serial baud rate. Supported values: 9600, 19200, 38400, 115200, 1228800, 2000000.
    #[arg(
        value_name = "NEW_BAUD_RATE",
        default_value = "2000000",
        value_parser = parse_serial_baud_rate,
    )]
    pub new_serial_baud_rate: SerialBaudRate,
}

#[derive(Parser)]
pub(crate) struct SelfTestOptions {
    /// Transmit test data frames onto the actual CAN bus in addition to the normal loopback test.
    /// Frames are always sent onto the CAN bus if second adapter is provided.
    #[arg(short = 's', long)]
    pub send_frames: bool,

    /// Serial receive timeout.
    #[arg(
        short = 't',
        long,
        value_name = "MILLISECONDS",
        default_value = "1000",
        value_parser = parse_duration_ms,
    )]
    pub receive_timeout: Duration,

    /// Serial baud rate for the first serial port. Supported values: 9600, 19200, 38400, 115200, 1228800, 2000000.
    #[arg(
        short = 'b',
        long,
        value_name = "BAUD_RATE",
        default_value = "2000000",
        value_parser = parse_serial_baud_rate,
    )]
    pub first_serial_baud_rate: SerialBaudRate,

    /// Serial baud rate for the second serial port. Supported values: 9600, 19200, 38400, 115200, 1228800, 2000000.
    #[arg(
        short = 'B',
        long,
        value_name = "BAUD_RATE",
        default_value = "2000000",
        value_parser = parse_serial_baud_rate,
    )]
    pub second_serial_baud_rate: SerialBaudRate,

    /// Path to the serial device file for the second USB2CAN adapter.
    /// If not provided, loopback mode of the only adapter is used for testing.
    pub second_serial_path: Option<String>,
}

fn parse_duration_s(duration: &str) -> Result<Duration> {
    Ok(Duration::from_secs_f64(
        duration
            .parse::<f64>()
            .with_context(|| format!("Unable to parse seconds \"{}\"", duration))?,
    ))
}

fn parse_duration_ms(duration: &str) -> Result<Duration> {
    Ok(Duration::from_millis(
        duration
            .parse::<u64>()
            .with_context(|| format!("Unable to parse milliseconds \"{}\"", duration))?,
    ))
}

fn parse_frame(str_frame: &str) -> Result<Frame> {
    let parts = str_frame.splitn(2, ':').collect::<Vec<_>>();
    ensure!(
        parts.len() == 2,
        "Invalid frame format, no type: \"{}\"",
        str_frame
    );

    match parts[0].to_lowercase().as_str() {
        "r" => parse_remote_frame(parts[1]),
        "d" => parse_data_frame(parts[1]),
        _ => Err(anyhow!("Invalid frame type: \"{}\"", str_frame)),
    }
}

fn parse_remote_frame(str_frame: &str) -> Result<Frame> {
    let parts = str_frame.splitn(3, ':').collect::<Vec<_>>();
    ensure!(
        parts.len() == 2,
        "Invalid frame format, no remote length: \"{}\"",
        str_frame
    );

    let id = parse_id(parts[0])?;

    let dlc = parts[1].parse::<usize>().with_context(|| {
        format!(
            "Unable to parse data length for remote frame: \"{}\"",
            str_frame
        )
    })?;
    Frame::new_remote(id, dlc)
        .with_context(|| format!("Unable to create remote frame for \"{}\"", str_frame))
}

fn parse_data_frame(str_frame: &str) -> Result<Frame> {
    let parts = str_frame.splitn(2, ':').collect::<Vec<_>>();
    ensure!(
        parts.len() == 2,
        "Invalid frame format, no data: \"{}\"",
        str_frame
    );

    let id = parse_id(parts[0])?;

    let str_data = ensure_no_prefix(parts[1])
        .ok_or_else(|| anyhow!("Data contains prefix: \"{}\"", str_frame))?;

    let data = if str_data.is_empty() {
        vec![]
    } else {
        let data = u64::from_str_radix(str_data, 16)
            .with_context(|| format!("Invalid hex: \"{}\"", str_frame))?;

        let data_len = str_data.len() / 2;
        ensure!(
            str_data.len() % 2 == 0,
            "Odd number of hex digits: \"{}\"",
            str_frame
        );

        data.to_be_bytes()[8 - data_len..].to_vec()
    };

    Frame::new(id, &data)
        .with_context(|| format!("Unable to create data frame for \"{}\"", str_frame))
}

fn parse_serial_baud_rate(str_rate: &str) -> Result<SerialBaudRate> {
    str_rate
        .parse::<u32>()
        .with_context(|| format!("Invalid serial rate string: \"{}\"", str_rate))
        .and_then(|value| SerialBaudRate::try_from(value).map_err(Error::from))
}

fn parse_can_baud_rate(str_rate: &str) -> Result<CanBaudRate> {
    str_rate
        .parse::<u32>()
        .with_context(|| format!("Invalid CAN baud rate string: \"{}\"", str_rate))
        .and_then(|value| CanBaudRate::try_from(value).map_err(Error::from))
}

#[derive(Clone)]
pub(crate) struct FilterWithMask {
    pub filter: Id,
    pub mask: Id,
}

impl FromStr for FilterWithMask {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self> {
        let parts = s.splitn(2, '/').collect::<Vec<_>>();
        ensure!(parts.len() == 2, "Invalid filter/mask format");

        let filter = parse_id(parts[0])?;
        let mask = parse_id(parts[1])?;

        Ok(Self { filter, mask })
    }
}

fn parse_id(str_id: &str) -> Result<Id> {
    let parts = str_id
        .splitn(2, '.')
        .map(ensure_no_prefix)
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| anyhow!("Invalid ID format"))?;
    let standard_id = u16::from_str_radix(parts[parts.len() - 1], 16)
        .with_context(|| format!("Unable to parse standard ID \"{}\"", str_id))?;

    Ok(if parts.len() == 1 {
        StandardId::new(standard_id)
            .ok_or_else(|| anyhow!("Invalid standard ID"))?
            .into()
    } else {
        let extended_id = u32::from_str_radix(parts[0], 16)
            .with_context(|| format!("Unable to parse extended ID \"{}\"", str_id))?;
        let combined_id = (u32::from(standard_id) << EXTENDED_ID_EXTRA_BITS) | extended_id;
        ExtendedId::new(combined_id)
            .ok_or_else(|| anyhow!("Invalid extended ID"))?
            .into()
    })
}

fn ensure_no_prefix(hex: &str) -> Option<&str> {
    if hex.is_empty() || "0123456789abcdefABCDEF".contains(&hex[0..1]) {
        Some(hex)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use proptest::{arbitrary::any, collection::vec, proptest};
    use waveshare_usb_can_a::MAX_DATA_LENGTH;

    use super::*;

    #[test]
    fn parse_data_frame_standard() {
        assert_eq!(
            parse_frame("d:7ff:0123456789abcdef").unwrap(),
            Frame::new(
                StandardId::MAX,
                &[0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef]
            )
            .unwrap()
        );

        assert_eq!(
            parse_frame("d:7ff:01234567").unwrap(),
            Frame::new(StandardId::MAX, &[0x01, 0x23, 0x45, 0x67]).unwrap()
        );

        assert_eq!(
            parse_frame("d:7ff:").unwrap(),
            Frame::new(StandardId::MAX, &[]).unwrap()
        );
    }

    #[test]
    fn parse_data_frame_extended() {
        assert_eq!(
            parse_frame("d:3ffff.7ff:0123456789abcdef").unwrap(),
            Frame::new(
                ExtendedId::MAX,
                &[0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef]
            )
            .unwrap()
        );

        assert_eq!(
            parse_frame("d:3ffff.7ff:01234567").unwrap(),
            Frame::new(ExtendedId::MAX, &[0x01, 0x23, 0x45, 0x67]).unwrap()
        );

        assert_eq!(
            parse_frame("d:3ffff.7ff:").unwrap(),
            Frame::new(ExtendedId::MAX, &[]).unwrap()
        );
    }

    #[test]
    fn parse_remote_frame_standard() {
        assert_eq!(
            parse_frame("r:7ff:8").unwrap(),
            Frame::new_remote(StandardId::MAX, 8).unwrap()
        );

        assert_eq!(
            parse_frame("r:7ff:0").unwrap(),
            Frame::new_remote(StandardId::MAX, 0).unwrap()
        );
    }

    #[test]
    fn parse_remote_frame_extended() {
        assert_eq!(
            parse_frame("r:3ffff.7ff:8").unwrap(),
            Frame::new_remote(ExtendedId::MAX, 8).unwrap()
        );

        assert_eq!(
            parse_frame("r:3ffff.7ff:0").unwrap(),
            Frame::new_remote(ExtendedId::MAX, 0).unwrap()
        );
    }

    proptest! {
        #[test]
        fn random_parse_data_frame_standard(
                id in StandardId::ZERO.as_raw()..=StandardId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = StandardId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new(id, &data).unwrap();
            check_fmt_parse_frame(&frame);
        }

        #[test]
        fn random_parse_data_frame_extended(
                id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
                data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
        ) {
            let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new(id, &data).unwrap();
            check_fmt_parse_frame(&frame);
        }

        fn random_parse_remote_frame_standard(
            id in StandardId::ZERO.as_raw()..=StandardId::MAX.as_raw(),
            dlc in 0..=MAX_DATA_LENGTH,
        ) {
            let id = StandardId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new_remote(id, dlc).unwrap();
            check_fmt_parse_frame(&frame);
        }

        fn random_parse_remote_frame_extended(
            id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
            dlc in 0..=MAX_DATA_LENGTH,
        ) {
            let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid standard ID");
            let frame = Frame::new_remote(id, dlc).unwrap();
            check_fmt_parse_frame(&frame);
        }
    }

    #[test]
    fn parse_id_standard() {
        assert_eq!(parse_id("000").unwrap(), StandardId::ZERO.into());
        assert_eq!(parse_id("7ff").unwrap(), StandardId::MAX.into());
    }

    #[test]
    fn parse_id_extended() {
        assert_eq!(parse_id("00000.000").unwrap(), ExtendedId::ZERO.into());
        assert_eq!(parse_id("3ffff.7ff").unwrap(), ExtendedId::MAX.into());
    }

    fn check_fmt_parse_frame(frame: &Frame) {
        let frame_str = format!("{}", frame);
        let frame_str = frame_str.split_ascii_whitespace().collect::<Vec<_>>();

        let parsed_frame = parse_frame(frame_str[0]).unwrap();

        assert_eq!(frame, &parsed_frame);
    }
}
