use std::{env, io::stderr, sync::Once};

use anyhow::{Context, Result};
use embedded_can::{ExtendedId, StandardId};
use proptest::{arbitrary::any, collection::vec, proptest};
use tracing::level_filters::LevelFilter;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};

use crate::*;

mod sync;
mod tokio;

const SERIAL_BAUD_RATE_DEFAULT: SerialBaudRate = SerialBaudRate::R2000000Bd;
const SERIAL_BAUD_RATE_STABLE: SerialBaudRate = SerialBaudRate::R1228800Bd;

fn initialize_test() -> Result<(String, Option<String>, bool)> {
    // Configure logging.
    static CONFIGURE_LOGGING_ONCE: Once = Once::new();
    CONFIGURE_LOGGING_ONCE.call_once(|| {
        tracing_subscriber::registry()
            .with(tracing_subscriber::fmt::layer().with_writer(stderr))
            .with(
                EnvFilter::builder()
                    .with_default_directive(LevelFilter::TRACE.into())
                    .from_env_lossy(),
            )
            .init()
    });

    // HW configuration.
    let serial_path = env::var("HW_TEST_SERIAL_PATH").context("Missing environment variable")?;
    let second_serial_path = env::var("HW_TEST_SECOND_SERIAL_PATH").ok();
    let send_frames = env::var("HW_TEST_SEND_FRAMES")
        .map(|s| s == "1")
        .unwrap_or(false);

    Ok((serial_path, second_serial_path, send_frames))
}

fn invert_frame(frame: &Frame) -> Frame {
    let data = frame.data().iter().map(|byte| !byte).collect::<Vec<_>>();

    let id = match frame.id() {
        Id::Standard(id) => {
            Id::Standard(StandardId::new(id.as_raw() ^ 0x0f).expect("Logic error: bad inverted ID"))
        }
        Id::Extended(id) => {
            Id::Extended(ExtendedId::new(id.as_raw() ^ 0x0f).expect("Logic error: bad inverted ID"))
        }
    };

    if frame.is_data_frame() {
        Frame::new(id, &data)
    } else {
        Frame::new_remote(id, frame.dlc())
    }
    .expect("Logic error: bad inverted frame")
}

#[test]
fn invalid_filter_conf() {
    let conf = Usb2CanConfiguration::new(CanBaudRate::R1000kBd);
    let conf = conf.set_filter(StandardId::ZERO.into(), ExtendedId::ZERO.into());
    assert!(matches!(conf, Err(CommonConfigurationError::Error(_))));

    let conf = Usb2CanConfiguration::new(CanBaudRate::R1000kBd);
    let conf = conf.set_filter(ExtendedId::ZERO.into(), StandardId::ZERO.into());
    assert!(matches!(conf, Err(CommonConfigurationError::Error(_))));
}

proptest! {
    #[test]
    fn random_length_bound_frame_standard(
            id in StandardId::ZERO.as_raw()..=StandardId::MAX.as_raw(),
            data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
    ) {
        let id = StandardId::new(id).expect("Logic error: proptest produced invalid standard ID");
        let frame = Frame::new(id, &data).unwrap();
        // https://en.wikipedia.org/wiki/CAN_bus#Bit_stuffing
        assert!(frame.length_bound_in_bits() <= 132 + 3);
    }

    #[test]
    fn random_length_bound_frame_extended(
            id in ExtendedId::ZERO.as_raw()..=ExtendedId::MAX.as_raw(),
            data in vec(any::<u8>(), 0..=MAX_DATA_LENGTH),
    ) {
        let id = ExtendedId::new(id).expect("Logic error: proptest produced invalid standard ID");
        let frame = Frame::new(id, &data).unwrap();
        // https://en.wikipedia.org/wiki/CAN_bus#Bit_stuffing
        assert!(frame.length_bound_in_bits() <= 157 + 3);
    }
}
