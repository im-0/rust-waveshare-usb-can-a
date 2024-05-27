use std::{str::FromStr, time::Duration};

use anyhow::{anyhow, bail, ensure, Context, Result};
use clap::{Parser, Subcommand};
use embedded_can::{ExtendedId, Frame as _, Id, StandardId};
use waveshare_usb_can_a::{CanBaudRate, Frame};

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

    /// Run self-test.
    #[command()]
    SelfTest(SelfTestOptions),
}

#[derive(Parser)]
pub(crate) struct DumpOptions {
    /// Serial receive timeout.
    #[arg(short = 't', long, value_name = "SECONDS", default_value = "86400", value_parser = parse_duration_s)]
    pub receive_timeout: Duration,

    /// Receive only extended frames (CAN 2.0B). By default, both standard and extended frames
    /// (CAN 2.0A & CAN 2.0B) are received.
    #[arg(short = 'e', long)]
    pub receive_only_extended_frames: bool,

    /// Filter and mask for ID filtering. Examples: "7ff/7ff" (standard ID), "7ff.3ffff/7ff.3ffff" (extended ID).
    /// There is no filtering by default.
    #[arg(short = 'f', long, value_name = "FILTER/MASK")]
    pub filter_with_mask: Option<FilterWithMask>,

    /// CAN bus baud rate.
    #[arg(value_name = "KBITS_PER_S")]
    pub can_baud_rate: CanBaudRate,
}

#[derive(Parser)]
pub(crate) struct InjectOptions {
    /// Enable automatic frame retransmission.
    #[arg(short = 'r', long)]
    pub automatic_retransmission: bool,

    /// CAN bus baud rate.
    #[arg(value_name = "KBITS_PER_S")]
    pub can_baud_rate: CanBaudRate,

    /// Frames to inject.
    #[arg(value_name = "FRAME", required = true, value_parser = parse_frame)]
    pub frames: Vec<Frame>,
}

#[derive(Parser)]
pub(crate) struct SelfTestOptions {
    /// Transmit test data frames onto the actual CAN bus in addition to the normal loopback test.
    #[arg(short = 's', long)]
    pub send_frames: bool,

    /// Serial receive timeout.
    #[arg(short = 't', long, value_name = "MILLISECONDS", default_value = "1000", value_parser = parse_duration_ms)]
    pub receive_timeout: Duration,
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
    let parts = str_frame.splitn(3, ':').collect::<Vec<_>>();
    ensure!(parts.len() == 3, "Invalid frame format: \"{}\"", str_frame);

    let remote = match parts[0].to_uppercase().as_str() {
        "R" => true,
        "D" => false,
        _ => bail!("Invalid frame type: \"{}\"", str_frame),
    };

    let id = parse_id(parts[1])?;

    if remote {
        let dlc = parts[2].parse::<usize>().with_context(|| {
            format!(
                "Unable to parse data length for remote frame: \"{}\"",
                str_frame
            )
        })?;
        Frame::new_remote(id, dlc)
            .with_context(|| format!("Unable to create remote frame for \"{}\"", str_frame))
    } else {
        let str_data = ensure_no_prefix(parts[2])
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
    let standard_id = u16::from_str_radix(parts[0], 16)
        .with_context(|| format!("Unable to parse standard ID \"{}\"", str_id))?;

    Ok(if parts.len() == 1 {
        StandardId::new(standard_id)
            .ok_or_else(|| anyhow!("Invalid standard ID"))?
            .into()
    } else {
        let extended_id = u32::from_str_radix(parts[1], 16)
            .with_context(|| format!("Unable to parse extended ID \"{}\"", str_id))?;
        let combined_id = (u32::from(standard_id) << 18) | extended_id;
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
            parse_frame("d:7ff.3ffff:0123456789abcdef").unwrap(),
            Frame::new(
                ExtendedId::MAX,
                &[0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef]
            )
            .unwrap()
        );

        assert_eq!(
            parse_frame("d:7ff.3ffff:01234567").unwrap(),
            Frame::new(ExtendedId::MAX, &[0x01, 0x23, 0x45, 0x67]).unwrap()
        );

        assert_eq!(
            parse_frame("d:7ff.3ffff:").unwrap(),
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
            parse_frame("r:7ff.3ffff:8").unwrap(),
            Frame::new_remote(ExtendedId::MAX, 8).unwrap()
        );

        assert_eq!(
            parse_frame("r:7ff.3ffff:0").unwrap(),
            Frame::new_remote(ExtendedId::MAX, 0).unwrap()
        );
    }

    #[test]
    fn parse_id_standard() {
        assert_eq!(parse_id("000").unwrap(), StandardId::ZERO.into());
        assert_eq!(parse_id("7ff").unwrap(), StandardId::MAX.into());
    }

    #[test]
    fn parse_id_extended() {
        assert_eq!(parse_id("000.00000").unwrap(), ExtendedId::ZERO.into());
        assert_eq!(parse_id("7ff.3ffff").unwrap(), ExtendedId::MAX.into());
    }
}
