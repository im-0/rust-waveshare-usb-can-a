use std::time::Duration;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};

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
    /// Run self-test.
    #[command()]
    SelfTest(SelfTestOptions),
}

#[derive(Parser)]
pub(crate) struct SelfTestOptions {
    /// Transmit test data frames onto the actual CAN bus in addition to the normal loopback test.
    #[arg(short = 's', long)]
    pub send_frames: bool,

    /// Serial receive timeout.
    #[arg(short = 't', long, value_name = "MILLISECONDS", default_value = "1000", value_parser = parse_duration)]
    pub receive_timeout: Duration,
}

fn parse_duration(duration: &str) -> Result<Duration> {
    Ok(Duration::from_millis(
        duration
            .parse::<u64>()
            .with_context(|| format!("Unable to parse milliseconds \"{}\"", duration))?,
    ))
}
