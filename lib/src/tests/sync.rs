use anyhow::{Context, Result};
use embedded_can::blocking::Can;
use embedded_can::{ExtendedId, Frame as _, Id, StandardId};
use tracing::info;

use crate::sync::Usb2CanBuilder;
use crate::tests::{initialize_test, invert_frame};
use crate::{
    sync::Usb2Can, CanBaudRate, Frame, SerialBaudRate, StoredIdFilter, Usb2CanConfiguration,
};

#[test]
#[ignore]
fn stored_id_filter() -> Result<()> {
    let (serial_path, second_serial_path, send_frames) = initialize_test()?;

    info!("Testing stored ID filter...");

    // Open USB2CAN adapters.
    let usb2can_conf = Usb2CanConfiguration::new(CanBaudRate::R5kBd)
        .set_loopback(second_serial_path.is_none())
        .set_silent(second_serial_path.is_none() && !send_frames)
        .set_automatic_retransmission(false);

    let (mut usb2can_a, mut usb2can_b) =
        open_adapters(&serial_path, &second_serial_path, &usb2can_conf)?;

    // Blocklist filter.
    let filter = StoredIdFilter::new_block(vec![ExtendedId::ZERO.into()])
        .context("Failed to create blocklist filter")?;
    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| {
        usb2can
            .set_stored_id_filter(&filter)
            .context("Failed to set blocklist filter")
    })?;

    self_test(&mut usb2can_a, &mut usb2can_b, true, true)?;

    // Allowlist filter.
    let filter = StoredIdFilter::new_allow(vec![
        ExtendedId::MAX.into(),
        Id::Extended(
            ExtendedId::new(ExtendedId::MAX.as_raw() ^ 0x0f).expect("Logic error: bad inverted ID"),
        ),
    ])
    .context("Failed to create allowlist filter")?;
    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| {
        usb2can
            .set_stored_id_filter(&filter)
            .context("Failed to set allowlist filter")
    })?;

    self_test(&mut usb2can_a, &mut usb2can_b, true, true)?;

    // Disable filter.
    info!("Disabling stored ID filter...");
    let filter = StoredIdFilter::new_disabled();
    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| {
        usb2can
            .set_stored_id_filter(&filter)
            .context("Failed to set disabled filter")
    })?;

    self_test(&mut usb2can_a, &mut usb2can_b, false, true)?;

    Ok(())
}

#[test]
#[ignore]
fn can_rates_frame_types_and_filtering() -> Result<()> {
    let (serial_path, second_serial_path, send_frames) = initialize_test()?;

    info!("Testing all supported CAN baud rates, frame types, and filtering...");

    // Open USB2CAN adapters.
    let usb2can_conf = Usb2CanConfiguration::new(CanBaudRate::R5kBd)
        .set_loopback(second_serial_path.is_none())
        .set_silent(second_serial_path.is_none() && !send_frames)
        .set_automatic_retransmission(false);

    let (mut usb2can_a, mut usb2can_b) =
        open_adapters(&serial_path, &second_serial_path, &usb2can_conf)?;

    for variable_encoding in [false, true] {
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
                    let (filter, mask) = if filtering {
                        let id: Id = if extended_frame {
                            ExtendedId::new(0x0f0)
                                .expect("Logic error: bad test ID")
                                .into()
                        } else {
                            StandardId::new(0x0f0)
                                .expect("Logic error: bad test ID")
                                .into()
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

                    info!(
                        "Running tests with {} encoding, CAN baud rate {}, {} frames, {}...",
                        if variable_encoding {
                            "variable"
                        } else {
                            "fixed"
                        },
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

                    let usb2can_conf = usb2can_a
                        .configuration()?
                        .set_variable_encoding(variable_encoding)
                        .set_can_baud_rate(can_baud_rate)
                        .set_filter(filter, mask)?;
                    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| {
                        usb2can
                            .set_configuration(&usb2can_conf)
                            .context("Failed to set new adapter configuration")
                    })?;

                    self_test(&mut usb2can_a, &mut usb2can_b, filtering, extended_frame)?;
                }
            }
        }
    }

    Ok(())
}

#[test]
#[ignore]
fn serial_rates() -> Result<()> {
    let (serial_path, second_serial_path, send_frames) = initialize_test()?;

    info!("Testing all supported serial baud rates...");

    // Open USB2CAN adapters.
    let usb2can_conf = Usb2CanConfiguration::new(CanBaudRate::R5kBd)
        .set_loopback(second_serial_path.is_none())
        .set_silent(second_serial_path.is_none() && !send_frames)
        .set_automatic_retransmission(false);

    let (mut usb2can_a, mut usb2can_b) =
        open_adapters(&serial_path, &second_serial_path, &usb2can_conf)?;

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

        reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| {
            usb2can
                .set_serial_baud_rate(serial_baud_rate)
                .context("Failed to set serial baud rate of USB2CAN device to test value")
        })?;

        self_test(&mut usb2can_a, &mut usb2can_b, false, true)?;
    }

    info!("Resetting to the original serial baud rate value...");
    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| {
        usb2can
            .set_serial_baud_rate(SerialBaudRate::R2000000Bd)
            .context("Failed to set serial baud rate of USB2CAN device to original value")
    })?;

    Ok(())
}

fn open_adapters(
    serial_path: &str,
    second_serial_path: &Option<String>,
    usb2can_conf: &Usb2CanConfiguration,
) -> Result<(Usb2Can, Option<Usb2Can>)> {
    open_adapters_custom(serial_path, second_serial_path, usb2can_conf, |builder| {
        Ok(builder)
    })
}

fn open_adapters_custom(
    serial_path: &str,
    second_serial_path: &Option<String>,
    usb2can_conf: &Usb2CanConfiguration,
    builder_adjust_fn: impl Fn(Usb2CanBuilder) -> Result<Usb2CanBuilder>,
) -> Result<(Usb2Can, Option<Usb2Can>)> {
    // Open USB2CAN adapters.
    let usb2can_a = builder_adjust_fn(crate::sync::new(serial_path, usb2can_conf))?
        .open()
        .context("Failed to open USB2CAN device")?;

    let usb2can_b = if let Some(ref second_serial_path) = second_serial_path {
        Some(
            builder_adjust_fn(crate::sync::new(second_serial_path, usb2can_conf))
                .context("Second device")?
                .open()
                .context("Failed to open second USB2CAN device")?,
        )
    } else {
        None
    };

    Ok((usb2can_a, usb2can_b))
}

fn reconfigure_adapters(
    usb2can_a: &mut Usb2Can,
    usb2can_b: &mut Option<Usb2Can>,
    reconfigure_fn: impl Fn(&mut Usb2Can) -> Result<()>,
) -> Result<()> {
    reconfigure_fn(usb2can_a)?;
    if let Some(usb2can_b) = usb2can_b {
        reconfigure_fn(usb2can_b).context("Second device")?;
    }

    Ok(())
}

fn self_test(
    usb2can_a: &mut Usb2Can,
    usb2can_b: &mut Option<Usb2Can>,
    filtering: bool,
    extended_frame: bool,
) -> Result<()> {
    if let Some(usb2can_b) = usb2can_b {
        self_test_one_way(usb2can_a, usb2can_b, filtering, extended_frame)?;
        self_test_one_way(usb2can_b, usb2can_a, filtering, extended_frame)?;
    } else {
        self_test_one_way(usb2can_a, &mut usb2can_a.clone(), filtering, extended_frame)?;
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
    let inverted_frame = invert_frame(frame);

    for i in 0..num {
        let test_frame = if i % 2 == 0 { frame } else { &inverted_frame };

        transmit(usb2can_t, test_frame)?;
    }

    for i in 0..num {
        let received = receive(usb2can_r)?;

        if i % 2 == 0 {
            assert_eq!(frame, &received);
        } else {
            assert_eq!(inverted_frame, received);
        }
    }

    Ok(())
}

fn transmit(usb2can: &mut Usb2Can, frame: &Frame) -> Result<()> {
    usb2can
        .transmit(frame)
        .context("Failed to transmit CAN frame")
}

fn receive(usb2can: &mut Usb2Can) -> Result<Frame> {
    usb2can.receive().context("Failed to receive CAN frame")
}
