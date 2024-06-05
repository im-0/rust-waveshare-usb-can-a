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
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{ensure, Context, Result};
use clap::Parser;
use embedded_can::{blocking::Can, StandardId};
use embedded_can::{ExtendedId, Frame as _, Id};
use tracing::level_filters::LevelFilter;
use tracing::{error, info};
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;
use tracing_subscriber::EnvFilter;
use waveshare_usb_can_a::{
    CanBaudRate, Frame, SerialBaudRate, Usb2Can, DEFAULT_SERIAL_BAUD_RATE, EXTENDED_ID_EXTRA_BITS,
};

mod cli;

const ADAPTER_PROBE_TIMEOUT: Duration = Duration::from_secs(1);

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
        cli::SubCommand::SetSerialBaudRate(options) => run_set_serial_baud_rate(&args, options),
        &cli::SubCommand::ResetToFactoryDefaults => run_reset_to_factory_defaults(&args),
        cli::SubCommand::SelfTest(options) => run_self_test(&args, options),
    }
}

fn run_dump(args: &cli::Cli, options: &cli::DumpOptions) -> Result<()> {
    // Open USB2CAN adapter.
    let usb2can = waveshare_usb_can_a::new(&args.serial_path, options.can_baud_rate)
        .serial_baud_rate(options.serial_baud_rate)
        .serial_receive_timeout(options.receive_timeout)
        .receive_only_extended_frames(options.receive_only_extended_frames);

    let usb2can = if let Some(filter_with_mask) = &options.filter_with_mask {
        usb2can.filter(filter_with_mask.filter, filter_with_mask.mask)?
    } else {
        usb2can
    };

    let mut usb2can = usb2can.open().context("Failed to open USB2CAN device")?;

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
                error!("{:?}", error)
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
    // Open USB2CAN adapter.
    let mut usb2can = waveshare_usb_can_a::new(&args.serial_path, options.can_baud_rate)
        .serial_baud_rate(options.serial_baud_rate)
        .automatic_retransmission(options.automatic_retransmission)
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

    Ok(())
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
    let mut usb2can = waveshare_usb_can_a::new(&args.serial_path, CanBaudRate::R5kBd)
        .serial_baud_rate(options.old_serial_baud_rate)
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
        let mut usb2can_a = waveshare_usb_can_a::new(&args.serial_path, CanBaudRate::R5kBd)
            .serial_receive_timeout(ADAPTER_PROBE_TIMEOUT)
            .serial_baud_rate(serial_baud_rate)
            .loopback(true)
            .silent(true)
            .open()
            .context("Failed to open USB2CAN device")?;

        let mut usb2can_b = usb2can_a
            .try_clone()
            .context("Failed to clone USB2CAN device for separate handle")?;

        let frame = Frame::new_remote(StandardId::MAX, 0).expect("Logic error: bad test frame");
        if check_echo(&mut usb2can_b, &mut usb2can_a, &frame, 1).is_ok() {
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

fn run_self_test(args: &cli::Cli, options: &cli::SelfTestOptions) -> Result<()> {
    // Run tests with different settings.
    info!(
        "Starting {}self-test for the Waveshare USB-CAN-A adapter...",
        if options.second_serial_path.is_none() {
            "loopback "
        } else {
            ""
        }
    );

    self_test_can_rates_frame_types_and_filtering(args, options)?;
    self_test_serial_rates(args, options)?;

    info!("All tests passed successfully.");

    Ok(())
}

fn self_test_can_rates_frame_types_and_filtering(
    args: &cli::Cli,
    options: &cli::SelfTestOptions,
) -> Result<()> {
    info!("Testing all supported CAN baud rates, frame types, and filtering...");

    for can_baud_rate in [
        CanBaudRate::R5kBd,
        CanBaudRate::R10kBd,
        CanBaudRate::R20kBd,
        CanBaudRate::R50kBd,
        CanBaudRate::R100kBd,
        CanBaudRate::R125kBd,
        CanBaudRate::R200kBd,
        CanBaudRate::R250kBd,
        CanBaudRate::R400kBd,
        CanBaudRate::R500kBd,
        CanBaudRate::R800kBd,
        CanBaudRate::R1000kBd,
    ] {
        for extended_frame in [false, true] {
            for filtering in [false, true] {
                info!(
                    "Running tests with CAN baud rate {}, {} frames, {}...",
                    can_baud_rate,
                    if extended_frame {
                        "extended"
                    } else {
                        "standard"
                    },
                    if filtering {
                        "with filtering"
                    } else {
                        "without filtering"
                    }
                );

                let (filter, mask) = if filtering {
                    let id: Id = if extended_frame {
                        ExtendedId::MAX.into()
                    } else {
                        StandardId::MAX.into()
                    };
                    (id, id)
                } else {
                    let id: Id = if extended_frame {
                        ExtendedId::ZERO.into()
                    } else {
                        StandardId::ZERO.into()
                    };
                    (id, id)
                };

                // Open USB2CAN adapter.
                let mut usb2can_a = waveshare_usb_can_a::new(&args.serial_path, can_baud_rate)
                    .filter(filter, mask)?
                    .serial_baud_rate(options.first_serial_baud_rate)
                    .serial_receive_timeout(options.receive_timeout)
                    .loopback(options.second_serial_path.is_none())
                    .silent(options.second_serial_path.is_none() && !options.send_frames)
                    .automatic_retransmission(false)
                    .open()
                    .context("Failed to open USB2CAN device")?;

                if let Some(ref second_serial_path) = options.second_serial_path {
                    let mut usb2can_b = waveshare_usb_can_a::new(second_serial_path, can_baud_rate)
                        .filter(filter, mask)?
                        .serial_baud_rate(options.second_serial_baud_rate)
                        .serial_receive_timeout(options.receive_timeout)
                        .automatic_retransmission(false)
                        .open()
                        .context("Failed to open second USB2CAN device")?;

                    self_test_one_way(&mut usb2can_b, &mut usb2can_a, filtering, extended_frame)?;

                    info!(
                        "Running tests with baud rate {}, {} frames, {}, reverse...",
                        can_baud_rate,
                        if extended_frame {
                            "extended"
                        } else {
                            "standard"
                        },
                        if filtering {
                            "with filtering"
                        } else {
                            "without filtering"
                        }
                    );
                    self_test_one_way(&mut usb2can_a, &mut usb2can_b, filtering, extended_frame)?;
                } else {
                    let mut usb2can_b = usb2can_a
                        .try_clone()
                        .context("Failed to clone USB2CAN device for separate handle")?;

                    self_test_one_way(&mut usb2can_b, &mut usb2can_a, filtering, extended_frame)?;
                };
            }
        }
    }

    Ok(())
}

fn self_test_serial_rates(args: &cli::Cli, options: &cli::SelfTestOptions) -> Result<()> {
    info!("Testing all supported serial baud rates...");

    // Open USB2CAN adapter.
    let mut usb2can_a = waveshare_usb_can_a::new(&args.serial_path, CanBaudRate::R5kBd)
        .serial_baud_rate(options.first_serial_baud_rate)
        .serial_receive_timeout(options.receive_timeout)
        .loopback(options.second_serial_path.is_none())
        .silent(options.second_serial_path.is_none() && !options.send_frames)
        .automatic_retransmission(false)
        .open()
        .context("Failed to open USB2CAN device")?;

    let mut usb2can_b = if let Some(ref second_serial_path) = options.second_serial_path {
        waveshare_usb_can_a::new(second_serial_path, CanBaudRate::R5kBd)
            .serial_baud_rate(options.second_serial_baud_rate)
            .serial_receive_timeout(options.receive_timeout)
            .automatic_retransmission(false)
            .open()
            .context("Failed to open second USB2CAN device")?
    } else {
        usb2can_a
            .try_clone()
            .context("Failed to clone USB2CAN device for separate handle")?
    };

    for serial_baud_rate in [
        SerialBaudRate::R9600Bd,
        SerialBaudRate::R19200Bd,
        SerialBaudRate::R38400Bd,
        SerialBaudRate::R115200Bd,
        SerialBaudRate::R1228800Bd,
        SerialBaudRate::R2000000Bd,
    ] {
        info!(
            "Running tests with serial baud rate {}...",
            serial_baud_rate
        );

        usb2can_a
            .set_serial_baud_rate(serial_baud_rate)
            .context("Failed to set serial baud rate of USB2CAN device to test value")?;
        if options.second_serial_path.is_some() {
            usb2can_b
                .set_serial_baud_rate(serial_baud_rate)
                .context("Failed to set serial baud rate of second USB2CAN device to test value")?;
        }

        self_test_one_way(&mut usb2can_b, &mut usb2can_a, false, true)?;
        if options.second_serial_path.is_some() {
            info!(
                "Running tests with serial baud rate {}, reverse...",
                serial_baud_rate
            );
            self_test_one_way(&mut usb2can_a, &mut usb2can_b, false, true)?;
        }
    }

    info!("Resetting to the original serial baud rate value...");
    usb2can_a
        .set_serial_baud_rate(options.first_serial_baud_rate)
        .context("Failed to set serial baud rate of USB2CAN device to orinal value")?;
    if options.second_serial_path.is_some() {
        usb2can_b
            .set_serial_baud_rate(options.second_serial_baud_rate)
            .context("Failed to set serial baud rate of second USB2CAN device to orinal value")?;
    }

    Ok(())
}

fn self_test_one_way(
    usb2can_t: &mut Usb2Can,
    usb2can_r: &mut Usb2Can,
    filtering: bool,
    extended_frame: bool,
) -> Result<()> {
    if filtering {
        // Send additional frame that will be ignored by the receiver because of enabled filter.
        let wrong_id: Id = if extended_frame {
            ExtendedId::ZERO.into()
        } else {
            StandardId::ZERO.into()
        };

        let wrong_frame =
            Frame::new(wrong_id, &[7, 6, 5, 4, 3, 2, 1, 0]).expect("Logic error: bad test frame");
        transmit(usb2can_t, &wrong_frame)?;
    }

    let id: Id = if extended_frame {
        ExtendedId::MAX.into()
    } else {
        StandardId::MAX.into()
    };

    // Data frame with maximum supported data length.
    let frame = Frame::new(id, &[0, 1, 2, 3, 4, 5, 6, 7]).expect("Logic error: bad test frame");
    check_echo(usb2can_t, usb2can_r, &frame, 1)?;

    // Data frame with minimum data length.
    let frame = Frame::new(id, &[]).expect("Logic error: bad test frame");
    check_echo(usb2can_t, usb2can_r, &frame, 1)?;

    // Remote frame with maximum supported data length.
    let frame = Frame::new_remote(id, 0).expect("Logic error: bad test frame");
    check_echo(usb2can_t, usb2can_r, &frame, 1)?;

    // Remote frame with minimum data length.
    let frame = Frame::new_remote(id, 8).expect("Logic error: bad test frame");
    check_echo(usb2can_t, usb2can_r, &frame, 1)?;

    // Send and receive multiple frames to test buffering.
    let frame = Frame::new(id, &[0, 1, 2, 3, 4, 5, 6, 7]).expect("Logic error: bad test frame");
    check_echo(usb2can_t, usb2can_r, &frame, 2)
}

fn check_echo(
    usb2can_t: &mut Usb2Can,
    usb2can_r: &mut Usb2Can,
    frame: &Frame,
    num: usize,
) -> Result<()> {
    for _ in 0..num {
        transmit(usb2can_t, frame)?;
    }

    for _ in 0..num {
        let received = receive(usb2can_r)?;
        assert_eq!(frame, &received);
    }

    Ok(())
}

fn transmit(usb2can: &mut Usb2Can, frame: &Frame) -> Result<()> {
    #![allow(clippy::print_stdout)]

    println!("{:.03} <- {}", timestamp()?, frame);
    usb2can
        .transmit(frame)
        .context("Failed to transmit CAN frame")
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
