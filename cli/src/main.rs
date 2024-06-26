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

use std::collections::HashSet;
use std::io::stderr;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{bail, ensure, Context, Result};
use clap::Parser;
use cli::PerfSubCommand;
use embedded_can::{blocking::Can, StandardId};
use embedded_can::{ExtendedId, Frame as _, Id};
use tracing::level_filters::LevelFilter;
use tracing::{error, info};
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;
use tracing_subscriber::EnvFilter;
use waveshare_usb_can_a::sync::{self, Usb2Can};
use waveshare_usb_can_a::{
    CanBaudRate, Frame, SerialBaudRate, StoredIdFilter, Usb2CanConfiguration,
    DEFAULT_SERIAL_BAUD_RATE, EXTENDED_ID_EXTRA_BITS,
};

mod cli;

const ADAPTER_PROBE_TIMEOUT: Duration = Duration::from_secs(1);
const REPORT_THROUGHPUT_EACH: Duration = Duration::from_secs(1);

fn main() -> Result<()> {
    // Configure logging.
    tracing_subscriber::registry()
        .with(tracing_subscriber::fmt::layer().with_writer(stderr))
        .with(
            EnvFilter::builder()
                .with_default_directive(LevelFilter::INFO.into())
                .from_env_lossy(),
        )
        .init();

    // Parse command line arguments.
    let args = cli::Cli::parse();

    match &args.subcommand {
        cli::SubCommand::Dump(options) => run_dump(&args, options),
        cli::SubCommand::Inject(options) => run_inject(&args, options),
        cli::SubCommand::Perf(options) => run_perf(&args, options),
        cli::SubCommand::SetSerialBaudRate(options) => run_set_serial_baud_rate(&args, options),
        cli::SubCommand::SetStoredIdFilter(options) => run_set_stored_id_filter(&args, options),
        cli::SubCommand::ResetToFactoryDefaults => run_reset_to_factory_defaults(&args),
    }
}

fn run_dump(args: &cli::Cli, options: &cli::DumpOptions) -> Result<()> {
    info!(
        "Dumping frames from CAN bus with baud rate {}...",
        options.can_baud_rate
    );

    // Open USB2CAN adapter.
    let usb2can_conf = Usb2CanConfiguration::new(options.can_baud_rate)
        .set_variable_encoding(options.variable_encoding)
        .set_receive_only_extended_frames(options.receive_only_extended_frames);

    let usb2can_conf = if let Some(filter_with_mask) = &options.filter_with_mask {
        usb2can_conf.set_filter(filter_with_mask.filter, filter_with_mask.mask)?
    } else {
        usb2can_conf
    };

    let mut usb2can = waveshare_usb_can_a::sync::new(&args.serial_path, &usb2can_conf)
        .set_serial_baud_rate(options.serial_baud_rate)
        .set_serial_receive_timeout(options.receive_timeout)
        .open()
        .context("Failed to open USB2CAN device")?;

    // Prepare mask.
    let mask = options.unique.as_ref().map(get_hashable_id_and_data);

    // Dump traffic on CAN bus.
    let mut already_seen = HashSet::new();
    loop {
        match (receive(&mut usb2can), mask) {
            (Ok(_), None) => {
                // receive() already prints the frame.
            }

            (Ok(frame), Some((id_mask, data_mask))) => {
                let (mut id, mut data) = get_hashable_id_and_data(&frame);
                id &= id_mask;
                data.iter_mut()
                    .zip(data_mask.iter())
                    .for_each(|(data, mask)| {
                        *data &= mask;
                    });

                if already_seen.insert((id, data)) {
                    // Additionally print the frame as INFO log entry if it is unique.
                    info!("-> {}", frame);
                }
            }

            (Err(error), _) => {
                error!("{}", error)
            }
        }
    }
}

fn get_hashable_id_and_data(frame: &Frame) -> (u32, [u8; 8]) {
    let id = match frame.id() {
        Id::Standard(id) => (id.as_raw() as u32) << EXTENDED_ID_EXTRA_BITS,
        Id::Extended(id) => id.as_raw(),
    };

    #[allow(clippy::get_first)]
    let data = [
        frame.data().get(0).copied().unwrap_or(0),
        frame.data().get(1).copied().unwrap_or(0),
        frame.data().get(2).copied().unwrap_or(0),
        frame.data().get(3).copied().unwrap_or(0),
        frame.data().get(4).copied().unwrap_or(0),
        frame.data().get(5).copied().unwrap_or(0),
        frame.data().get(6).copied().unwrap_or(0),
        frame.data().get(7).copied().unwrap_or(0),
    ];

    (id, data)
}

fn run_inject(args: &cli::Cli, options: &cli::InjectOptions) -> Result<()> {
    info!(
        "Injecting frames into CAN bus with baud rate {}...",
        options.can_baud_rate
    );

    // Open USB2CAN adapter.
    let usb2can_conf = Usb2CanConfiguration::new(options.can_baud_rate)
        .set_variable_encoding(options.variable_encoding)
        .set_automatic_retransmission(options.automatic_retransmission);

    let mut usb2can = waveshare_usb_can_a::sync::new(&args.serial_path, &usb2can_conf)
        .set_serial_baud_rate(options.serial_baud_rate)
        .set_frame_delay_multiplier(options.frame_delay_multiplier)?
        .open()
        .context("Failed to open USB2CAN device")?;

    // Inject frames into CAN bus.
    loop {
        for frame in &options.frames {
            transmit(&mut usb2can, frame)?;
        }

        if let Some(delay) = &options.loop_inject {
            if delay.as_millis() > 0 {
                std::thread::sleep(*delay);
            }
        } else {
            break;
        }
    }

    info!("Done!");
    Ok(())
}

fn run_perf(args: &cli::Cli, options: &cli::PerfOptions) -> Result<()> {
    info!(
        "CAN bus throughput test with baud rate {}...",
        options.can_baud_rate
    );

    // Open USB2CAN adapter.
    let usb2can_conf = Usb2CanConfiguration::new(options.can_baud_rate)
        .set_variable_encoding(options.variable_encoding)
        .set_automatic_retransmission(false);

    let mut usb2can = waveshare_usb_can_a::sync::new(&args.serial_path, &usb2can_conf)
        .set_serial_baud_rate(options.serial_baud_rate)
        .set_serial_receive_timeout(options.receive_timeout)
        .set_frame_delay_multiplier(options.frame_delay_multiplier)?
        .open()
        .context("Failed to open USB2CAN device")?;

    match options.transmit_or_receive {
        PerfSubCommand::Transmit => perf_transmit(&mut usb2can),
        PerfSubCommand::Receive => perf_receive(&mut usb2can),
    }
}

fn perf_transmit(usb2can: &mut Usb2Can) -> Result<()> {
    info!("Transmitting frames...");
    loop {
        for id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw() {
            let data = [
                id.to_le_bytes()[0],
                id.to_le_bytes()[1],
                id.to_le_bytes()[2],
                id.to_le_bytes()[3],
                id.to_le_bytes()[0],
                id.to_le_bytes()[1],
                id.to_le_bytes()[2],
                id.to_le_bytes()[3],
            ];
            let frame = Frame::new(
                ExtendedId::new(id).expect("Logic error: invalid test ID"),
                &data,
            )
            .expect("Logic error: bad test frame");
            usb2can.transmit(&frame)?;
        }
    }
}

fn perf_receive(usb2can: &mut Usb2Can) -> Result<()> {
    info!("Receiving frames...");
    let mut expected_id = ExtendedId::ZERO.as_raw();
    let mut frames_missing = 0usize;
    let mut frames_corrupted = 0usize;
    let mut frames_ok = 0usize;

    // Receive first frame to start measuring throughput.
    let mut first_frame = Some(usb2can.receive());

    let started = Instant::now();
    let mut prev_report = started;
    loop {
        let now = Instant::now();
        if now.duration_since(prev_report) >= REPORT_THROUGHPUT_EACH {
            let elapsed = now.duration_since(started).as_secs_f64();
            let (frames_per_second, bits_per_second) = if elapsed > f64::EPSILON {
                (
                    frames_ok as f64 / elapsed,
                    frames_ok as f64 * 8.0 * 8.0 / elapsed,
                )
            } else {
                (0.0, 0.0)
            };

            info!(
                "{:.03} frames/s, {:.03} bits/s ({} frames ok, {} missing, {} corrupted)",
                frames_per_second, bits_per_second, frames_ok, frames_missing, frames_corrupted
            );
            prev_report = now;
        }

        let frame = if let Some(frame) = first_frame.take() {
            frame
        } else {
            usb2can.receive()
        };

        let frame = match frame {
            Ok(frame) => frame,

            Err(error) => {
                error!("Unable to receive frame: {}", error);
                frames_corrupted += 1;
                continue;
            }
        };

        let frame_id = match frame.id() {
            Id::Standard(_) => {
                error!("Received unexpected standard frame: {}", frame);
                frames_corrupted += 1;
                continue;
            }

            Id::Extended(id) => id.as_raw(),
        };

        if frame.is_remote_frame() {
            error!("Received unexpected remote frame: {}", frame);
            frames_corrupted += 1;
            continue;
        }

        if (frame_id.to_le_bytes() != frame.data().get(0..4).unwrap_or(&[0; 4]))
            || (frame_id.to_le_bytes() != frame.data().get(4..8).unwrap_or(&[0; 4]))
        {
            error!("Received corrupted frame: {}", frame);
            frames_corrupted += 1;
            continue;
        }

        if frame_id != expected_id {
            if frame_id > expected_id {
                frames_missing += (frame_id - expected_id) as usize;
            } else {
                // Overflow happened.
                frames_missing +=
                    (ExtendedId::MAX.as_raw() - expected_id) as usize + frame_id as usize + 1;
            }

            expected_id = frame_id + 1;
            if expected_id > ExtendedId::MAX.as_raw() {
                expected_id = ExtendedId::ZERO.as_raw();
            }

            continue;
        }

        frames_ok += 1;

        expected_id += 1;
        if expected_id > ExtendedId::MAX.as_raw() {
            expected_id = ExtendedId::ZERO.as_raw();
        }
    }
}

fn run_set_serial_baud_rate(
    args: &cli::Cli,
    options: &cli::SetSerialBaudRateOptions,
) -> Result<()> {
    info!(
        "Opening adapter with serial baud rate {}...",
        options.old_serial_baud_rate
    );

    // Open USB2CAN adapter.
    let mut usb2can = waveshare_usb_can_a::sync::new(
        &args.serial_path,
        &Usb2CanConfiguration::new(CanBaudRate::R5kBd),
    )
    .set_serial_baud_rate(options.old_serial_baud_rate)
    .open()
    .context("Failed to open USB2CAN device")?;

    info!(
        "Changing serial baud rate to {}...",
        options.new_serial_baud_rate
    );
    usb2can
        .set_serial_baud_rate(options.new_serial_baud_rate)
        .context("Failed to set new serial baud rate")?;

    info!("Done!");
    Ok(())
}

fn run_set_stored_id_filter(
    args: &cli::Cli,
    options: &cli::SetStoredIdFilterOptions,
) -> Result<()> {
    info!("Opening adapter...");

    // Open USB2CAN adapter.
    let usb2can_conf = Usb2CanConfiguration::new(CanBaudRate::R5kBd)
        .set_loopback(true)
        .set_silent(true);
    let mut usb2can = waveshare_usb_can_a::sync::new(&args.serial_path, &usb2can_conf)
        .set_serial_baud_rate(options.serial_baud_rate)
        .open()
        .context("Failed to open USB2CAN device")?;

    let filter = match &options.filter_mode {
        cli::SetStoredIdFilterSubCommand::Disabled => {
            info!("Disabling filter...");
            StoredIdFilter::new_disabled()
        }

        cli::SetStoredIdFilterSubCommand::Blocklist(ids) => {
            info!("Configuring blocklist...");
            StoredIdFilter::new_block(ids.id.clone())
                .context("Failed to create new blocklist filter")?
        }

        cli::SetStoredIdFilterSubCommand::Allowlist(ids) => {
            info!("Configuring allowlist...");
            StoredIdFilter::new_allow(ids.id.clone())
                .context("Failed to create new allowlist filter")?
        }
    };
    usb2can
        .set_stored_id_filter(&filter)
        .context("Failed to set new stored ID filter")?;
    usb2can.delay()?;

    info!("Done!");
    Ok(())
}

fn run_reset_to_factory_defaults(args: &cli::Cli) -> Result<()> {
    let mut adapter_found = false;
    for serial_baud_rate in [
        SerialBaudRate::R9600Bd,
        SerialBaudRate::R19200Bd,
        SerialBaudRate::R38400Bd,
        SerialBaudRate::R115200Bd,
        SerialBaudRate::R1228800Bd,
        SerialBaudRate::R2000000Bd,
    ] {
        info!(
            "Trying to open adapter with serial baud rate {}...",
            serial_baud_rate
        );

        // Open USB2CAN adapter.
        let usb2can_conf = Usb2CanConfiguration::new(CanBaudRate::R5kBd)
            .set_loopback(true)
            .set_silent(true);

        let mut usb2can_a = waveshare_usb_can_a::sync::new(&args.serial_path, &usb2can_conf)
            .set_serial_receive_timeout(ADAPTER_PROBE_TIMEOUT)
            .set_serial_baud_rate(serial_baud_rate)
            .open()
            .context("Failed to open USB2CAN device")?;

        let mut usb2can_b = usb2can_a.clone();

        info!("Trying to disable stored ID filter...");
        usb2can_a
            .set_stored_id_filter(&StoredIdFilter::new_disabled())
            .context("Failed to disable stored ID filter")?;

        if check_echo(&mut usb2can_b, &mut usb2can_a).context("Echo test failed")? {
            info!(
                "Adapter responded with serial baud rate {}",
                serial_baud_rate
            );

            info!(
                "Changing serial baud rate to {}...",
                DEFAULT_SERIAL_BAUD_RATE
            );
            usb2can_a
                .set_serial_baud_rate(DEFAULT_SERIAL_BAUD_RATE)
                .context("To set serial baud rate to 2000000 Bd")?;

            adapter_found = true;
            break;
        }
    }

    ensure!(
        adapter_found,
        "No adapter found to reset to factory defaults"
    );

    info!("Done!");
    Ok(())
}

fn check_echo(usb2can_t: &mut Usb2Can, usb2can_r: &mut Usb2Can) -> Result<bool> {
    #![allow(clippy::print_stdout)]

    let frame = Frame::new_remote(StandardId::MAX, 0).expect("Logic error: bad test frame");

    transmit(usb2can_t, &frame)?;

    match usb2can_r.receive() {
        Err(sync::Error::SerialReadTimedOut) => Ok(false),
        Err(error) => Err(error).context("Failed to receive CAN echo frame"),

        Ok(received) => {
            println!("{:.03} -> {}", timestamp()?, received);

            if received == frame {
                Ok(true)
            } else {
                bail!("Received unexpected frame: {}", received)
            }
        }
    }
}

fn transmit(usb2can: &mut Usb2Can, frame: &Frame) -> Result<()> {
    #![allow(clippy::print_stdout)]

    usb2can
        .transmit(frame)
        .context("Failed to transmit CAN frame")?;
    println!("{:.03} <- {}", timestamp()?, frame);
    Ok(())
}

fn receive(usb2can: &mut Usb2Can) -> Result<Frame> {
    #![allow(clippy::print_stdout)]

    let frame = usb2can.receive().context("Failed to receive CAN frame")?;
    println!("{:.03} -> {}", timestamp()?, frame);
    Ok(frame)
}

fn timestamp() -> Result<f64> {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .context("Failed to get current time")
        .map(|timestamp| timestamp.as_secs_f64())
}
