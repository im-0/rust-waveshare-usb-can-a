use std::time::Duration;

use anyhow::{Context, Result};
use clap::Parser;

#[derive(Parser)]
#[command(about, version)]
pub(crate) struct Cli {
    /// Transmit test data frames onto the actual CAN bus in addition to the normal loopback test.
    #[arg(short = 'S', long)]
    pub send_frames: bool,

    /// Serial receive timeout.
    #[arg(short = 't', long, value_name = "MILLISECONDS", default_value = "1000", value_parser = parse_duration)]
    pub receive_timeout: Duration,

    /// Path to the serial device file of the USB2CAN adapter.
    pub serial_path: String,
}

fn parse_duration(duration: &str) -> Result<Duration> {
    Ok(Duration::from_millis(
        duration
            .parse::<u64>()
            .with_context(|| format!("Unable to parse milliseconds \"{}\"", duration))?,
    ))
}
