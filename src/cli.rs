use std::{str::FromStr, time::Duration};

use anyhow::{anyhow, ensure, Context, Result};
use clap::{Parser, Subcommand};
use embedded_can::{ExtendedId, Id, StandardId};
use waveshare_usb_can_a::CanBaudRate;

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
    if hex.is_empty() {
        None
    } else if "0123456789abcdefABCDEF".contains(&hex[0..1]) {
        Some(hex)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
