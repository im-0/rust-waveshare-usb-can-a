use std::future::Future;

use anyhow::{Context, Result};
use embedded_can::{ExtendedId, Frame as _, Id, StandardId};
use tokio::try_join;
use tracing::info;

use crate::tests::{
    initialize_test, invert_frame, SERIAL_BAUD_RATE_DEFAULT, SERIAL_BAUD_RATE_STABLE,
};
use crate::tokio::Usb2CanBuilder;
use crate::{
    tokio::Usb2Can, CanBaudRate, Frame, SerialBaudRate, StoredIdFilter, Usb2CanConfiguration,
};

#[tokio::test]
#[ignore]
async fn stored_id_filter() -> Result<()> {
    let (serial_path, second_serial_path, send_frames) = initialize_test()?;

    info!("Testing stored ID filter...");

    // Open USB2CAN adapters.
    let usb2can_conf = Usb2CanConfiguration::new(CanBaudRate::R5kBd)
        .set_loopback(second_serial_path.is_none())
        .set_silent(second_serial_path.is_none() && !send_frames)
        .set_automatic_retransmission(false);

    let (mut usb2can_a, mut usb2can_b) =
        open_adapters(&serial_path, &second_serial_path, &usb2can_conf).await?;

    info!("Configure to the stable serial baud rate value...");
    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| async {
        usb2can
            .set_serial_baud_rate(SERIAL_BAUD_RATE_STABLE)
            .await
            .context("Failed to set serial baud rate of USB2CAN device to stable value")
    })
    .await?;

    // Blocklist filter.
    let filter = StoredIdFilter::new_block(vec![ExtendedId::ZERO.into()])
        .context("Failed to create blocklist filter")?;
    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| async {
        usb2can
            .set_stored_id_filter(&filter)
            .await
            .context("Failed to set blocklist filter")
    })
    .await?;

    self_test(&mut usb2can_a, &mut usb2can_b, true, true).await?;

    // Allowlist filter.
    let filter = StoredIdFilter::new_allow(vec![
        ExtendedId::MAX.into(),
        Id::Extended(
            ExtendedId::new(ExtendedId::MAX.as_raw() ^ 0x0f).expect("Logic error: bad inverted ID"),
        ),
    ])
    .context("Failed to create allowlist filter")?;
    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| async {
        usb2can
            .set_stored_id_filter(&filter)
            .await
            .context("Failed to set allowlist filter")
    })
    .await?;

    self_test(&mut usb2can_a, &mut usb2can_b, true, true).await?;

    // Disable filter.
    info!("Disabling stored ID filter...");
    let filter = StoredIdFilter::new_disabled();
    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| async {
        usb2can
            .set_stored_id_filter(&filter)
            .await
            .context("Failed to set disabled filter")
    })
    .await?;

    self_test(&mut usb2can_a, &mut usb2can_b, false, true).await?;

    info!("Resetting to the original serial baud rate value...");
    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| async {
        usb2can
            .set_serial_baud_rate(SERIAL_BAUD_RATE_DEFAULT)
            .await
            .context("Failed to set serial baud rate of USB2CAN device to original value")
    })
    .await?;

    Ok(())
}

#[tokio::test]
#[ignore]
async fn can_rates_frame_types_and_filtering() -> Result<()> {
    let (serial_path, second_serial_path, send_frames) = initialize_test()?;

    info!("Testing all supported CAN baud rates, frame types, and filtering...");

    // Open USB2CAN adapters.
    let usb2can_conf = Usb2CanConfiguration::new(CanBaudRate::R5kBd)
        .set_loopback(second_serial_path.is_none())
        .set_silent(second_serial_path.is_none() && !send_frames)
        .set_automatic_retransmission(false);

    let (mut usb2can_a, mut usb2can_b) =
        open_adapters(&serial_path, &second_serial_path, &usb2can_conf).await?;

    info!("Configure to the stable serial baud rate value...");
    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| async {
        usb2can
            .set_serial_baud_rate(SERIAL_BAUD_RATE_STABLE)
            .await
            .context("Failed to set serial baud rate of USB2CAN device to stable value")
    })
    .await?;

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
                        .configuration()
                        .await?
                        .set_variable_encoding(variable_encoding)
                        .set_can_baud_rate(can_baud_rate)
                        .set_filter(filter, mask)?;
                    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| async {
                        usb2can
                            .set_configuration(&usb2can_conf)
                            .await
                            .context("Failed to set new adapter configuration")
                    })
                    .await?;

                    self_test(&mut usb2can_a, &mut usb2can_b, filtering, extended_frame).await?;
                }
            }
        }
    }

    info!("Resetting to the original serial baud rate value...");
    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| async {
        usb2can
            .set_serial_baud_rate(SERIAL_BAUD_RATE_DEFAULT)
            .await
            .context("Failed to set serial baud rate of USB2CAN device to original value")
    })
    .await?;

    Ok(())
}

#[tokio::test]
#[ignore]
async fn serial_rates() -> Result<()> {
    let (serial_path, second_serial_path, send_frames) = initialize_test()?;

    info!("Testing all supported serial baud rates...");

    // Open USB2CAN adapters.
    let usb2can_conf = Usb2CanConfiguration::new(CanBaudRate::R5kBd)
        .set_loopback(second_serial_path.is_none())
        .set_silent(second_serial_path.is_none() && !send_frames)
        .set_automatic_retransmission(false);

    let (mut usb2can_a, mut usb2can_b) =
        open_adapters(&serial_path, &second_serial_path, &usb2can_conf).await?;

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

        reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| async {
            usb2can
                .set_serial_baud_rate(serial_baud_rate)
                .await
                .context("Failed to set serial baud rate of USB2CAN device to test value")
        })
        .await?;

        self_test(&mut usb2can_a, &mut usb2can_b, false, true).await?;
    }

    info!("Resetting to the original serial baud rate value...");
    reconfigure_adapters(&mut usb2can_a, &mut usb2can_b, |usb2can| async {
        usb2can
            .set_serial_baud_rate(SERIAL_BAUD_RATE_DEFAULT)
            .await
            .context("Failed to set serial baud rate of USB2CAN device to original value")
    })
    .await?;

    Ok(())
}

async fn open_adapters(
    serial_path: &str,
    second_serial_path: &Option<String>,
    usb2can_conf: &Usb2CanConfiguration,
) -> Result<(Usb2Can, Option<Usb2Can>)> {
    open_adapters_custom(serial_path, second_serial_path, usb2can_conf, |builder| {
        Ok(builder)
    })
    .await
}

async fn open_adapters_custom(
    serial_path: &str,
    second_serial_path: &Option<String>,
    usb2can_conf: &Usb2CanConfiguration,
    builder_adjust_fn: impl Fn(Usb2CanBuilder) -> Result<Usb2CanBuilder>,
) -> Result<(Usb2Can, Option<Usb2Can>)> {
    // Open USB2CAN adapters.
    try_join!(
        open_adapter_custom_a(serial_path, usb2can_conf, &builder_adjust_fn),
        open_adapter_custom_b(second_serial_path, usb2can_conf, &builder_adjust_fn)
    )
}

async fn open_adapter_custom_a(
    serial_path: &str,
    usb2can_conf: &Usb2CanConfiguration,
    builder_adjust_fn: &impl Fn(Usb2CanBuilder) -> Result<Usb2CanBuilder>,
) -> Result<Usb2Can> {
    let usb2can_a = builder_adjust_fn(crate::tokio::new(serial_path, usb2can_conf))?
        .open()
        .await
        .context("Failed to open USB2CAN device")?;

    Ok(usb2can_a)
}

async fn open_adapter_custom_b(
    second_serial_path: &Option<String>,
    usb2can_conf: &Usb2CanConfiguration,
    builder_adjust_fn: &impl Fn(Usb2CanBuilder) -> Result<Usb2CanBuilder>,
) -> Result<Option<Usb2Can>> {
    let usb2can = if let Some(ref second_serial_path) = second_serial_path {
        Some(
            builder_adjust_fn(crate::tokio::new(second_serial_path, usb2can_conf))
                .context("Second device")?
                .open()
                .await
                .context("Failed to open second USB2CAN device")?,
        )
    } else {
        None
    };

    Ok(usb2can)
}

async fn reconfigure_adapters<'a, Fut>(
    usb2can_a: &'a mut Usb2Can,
    usb2can_b: &'a mut Option<Usb2Can>,
    reconfigure_fn: impl Fn(&'a mut Usb2Can) -> Fut,
) -> Result<()>
where
    Fut: Future<Output = Result<()>>,
{
    try_join!(reconfigure_fn(usb2can_a), async move {
        if let Some(usb2can_b) = usb2can_b {
            reconfigure_fn(usb2can_b).await.context("Second device")
        } else {
            Ok(())
        }
    },)?;

    Ok(())
}

async fn self_test(
    usb2can_a: &mut Usb2Can,
    usb2can_b: &mut Option<Usb2Can>,
    filtering: bool,
    extended_frame: bool,
) -> Result<()> {
    if let Some(usb2can_b) = usb2can_b {
        self_test_one_way(usb2can_a, usb2can_b, filtering, extended_frame).await?;
        self_test_one_way(usb2can_b, usb2can_a, filtering, extended_frame).await?;
    } else {
        self_test_one_way(usb2can_a, &mut usb2can_a.clone(), filtering, extended_frame).await?;
    }

    Ok(())
}

async fn self_test_one_way(
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
        transmit(usb2can_t, &wrong_frame).await?;
    }

    let id: Id = if extended_frame {
        ExtendedId::MAX.into()
    } else {
        StandardId::MAX.into()
    };

    // Data frame with maximum supported data length.
    let frame = Frame::new(id, &[0, 1, 2, 3, 4, 5, 6, 7]).expect("Logic error: bad test frame");
    check_echo(usb2can_t, usb2can_r, &frame, 1).await?;

    // Data frame with minimum data length.
    let frame = Frame::new(id, &[]).expect("Logic error: bad test frame");
    check_echo(usb2can_t, usb2can_r, &frame, 1).await?;

    // Remote frame with maximum supported data length.
    let frame = Frame::new_remote(id, 0).expect("Logic error: bad test frame");
    check_echo(usb2can_t, usb2can_r, &frame, 1).await?;

    // Remote frame with minimum data length.
    let frame = Frame::new_remote(id, 8).expect("Logic error: bad test frame");
    check_echo(usb2can_t, usb2can_r, &frame, 1).await?;

    // Send and receive multiple frames to test buffering.
    let frame = Frame::new(id, &[0, 1, 2, 3, 4, 5, 6, 7]).expect("Logic error: bad test frame");
    check_echo(usb2can_t, usb2can_r, &frame, 2).await
}

async fn check_echo(
    usb2can_t: &mut Usb2Can,
    usb2can_r: &mut Usb2Can,
    frame: &Frame,
    num: usize,
) -> Result<()> {
    let inverted_frame = invert_frame(frame);

    for i in 0..num {
        let test_frame = if i % 2 == 0 { frame } else { &inverted_frame };

        transmit(usb2can_t, test_frame).await?;
    }

    for i in 0..num {
        let received = receive(usb2can_r).await?;

        if i % 2 == 0 {
            assert_eq!(frame, &received);
        } else {
            assert_eq!(inverted_frame, received);
        }
    }

    Ok(())
}

async fn transmit(usb2can: &mut Usb2Can, frame: &Frame) -> Result<()> {
    usb2can
        .transmit(frame)
        .await
        .context("Failed to transmit CAN frame")
}

async fn receive(usb2can: &mut Usb2Can) -> Result<Frame> {
    usb2can
        .receive()
        .await
        .context("Failed to receive CAN frame")
}
