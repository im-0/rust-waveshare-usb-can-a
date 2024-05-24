use anyhow::{Context, Result};
use clap::Parser;
use embedded_can::{blocking::Can, StandardId};
use embedded_can::{ExtendedId, Frame as _, Id};
use tracing::info;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;
use tracing_subscriber::EnvFilter;
use waveshare_usb_can_a::{CanBaudRate, Frame, Usb2Can};

#[derive(Parser)]
#[command(about, version)]
pub struct Cli {
    /// Transmit test data frames onto the actual CAN bus in addition to the normal loopback test.
    #[arg(short = 'S', long)]
    pub send_frames: bool,

    /// Path to the serial device file of the USB2CAN adapter.
    pub serial_path: String,
}

fn main() -> Result<()> {
    // Configure logging.
    tracing_subscriber::registry()
        .with(tracing_subscriber::fmt::layer())
        .with(EnvFilter::from_default_env())
        .init();

    // Parse command line arguments.
    let args = Cli::parse();

    // Run tests with different settings.
    info!("Starting loopback self-test for the Waveshare USB-CAN-A adapter...");
    for can_baud_rate in [
        CanBaudRate::Kbitps5,
        CanBaudRate::Kbitps10,
        CanBaudRate::Kbitps20,
        CanBaudRate::Kbitps50,
        CanBaudRate::Kbitps100,
        CanBaudRate::Kbitps125,
        CanBaudRate::Kbitps200,
        CanBaudRate::Kbitps250,
        CanBaudRate::Kbitps400,
        CanBaudRate::Kbitps500,
        CanBaudRate::Kbitps800,
        CanBaudRate::Kbitps1000,
    ] {
        for extended_frame in [false, true] {
            info!(
                "Running tests with baud rate {} and {} frames...",
                can_baud_rate,
                if extended_frame {
                    "extended"
                } else {
                    "standard"
                }
            );

            // Open USB2CAN adapter.
            let mut usb2can = waveshare_usb_can_a::new(&args.serial_path, can_baud_rate)
                .loopback(true)
                .silent(!args.send_frames)
                .automatic_retransmission(false)
                .extended_frame(extended_frame)
                .open()
                .context("Failed to open USB2CAN device")?;

            let id: Id = if extended_frame {
                ExtendedId::MAX.into()
            } else {
                StandardId::MAX.into()
            };

            let frame =
                Frame::new(id, &[0, 1, 2, 3, 4, 5, 6, 7]).expect("Logic error: bad test frame");
            check_echo(&mut usb2can, &frame)?;

            let frame = Frame::new(id, &[]).expect("Logic error: bad test frame");
            check_echo(&mut usb2can, &frame)?;

            let frame = Frame::new_remote(id, 0).expect("Logic error: bad test frame");
            check_echo(&mut usb2can, &frame)?;

            let frame = Frame::new_remote(id, 8).expect("Logic error: bad test frame");
            check_echo(&mut usb2can, &frame)?;
        }
    }

    info!("All tests passed successfully.");

    Ok(())
}

fn check_echo(usb2can: &mut Usb2Can, frame: &Frame) -> Result<()> {
    info!("<- {:?}", frame);
    usb2can
        .transmit(frame)
        .context("Failed to transmit CAN frame")?;

    let received = usb2can.receive().context("Failed to receive CAN frame")?;
    info!("-> {:?}", received);

    assert_eq!(frame, &received);
    Ok(())
}
