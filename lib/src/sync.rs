use std::{
    borrow::Cow,
    io::{self, Read},
    result,
    sync::{Arc, Mutex, MutexGuard},
    thread::sleep,
    time::{Duration, Instant},
};

use embedded_can::blocking;
use serialport::{ClearBuffer, DataBits, SerialPort, StopBits};
use thiserror::Error;
use tracing::{debug, trace};

use crate::{
    proto::{
        new_receive_buffer, GenericReceiveBuffer, NewRecvUnexpectedError, ToFixedMessage,
        ToVariableMessage, MAX_MESSAGE_SIZE,
    },
    Frame, SerialBaudRate, StoredIdFilter, Usb2CanConfiguration, CONFIGURATION_DELAY,
    DEFAULT_FRAME_DELAY_MULTIPLIER, DEFAULT_SERIAL_BAUD_RATE,
};

pub const DEFAULT_SERIAL_RECEIVE_TIMEOUT: Duration = Duration::from_millis(1000);

// Not really infinite...
// TODO: Fix rust-serialport to allow blocking mode.
const INFINITE_TIMEOUT: Duration = Duration::from_secs(60 * 60 * 24 * 365 * 100);

#[derive(Error, Debug)]
pub enum Error {
    #[error("Configuration error: {0}")]
    SetConfiguration(String),

    #[error("Configuration reading error: {0}")]
    GetConfiguration(String),

    #[error("Locking error: {0}")]
    Locking(String),

    #[error("Serial port error: {}", .0.description)]
    Serial(#[from] serialport::Error),

    #[error("Serial read timed out")]
    SerialReadTimedOut,

    #[error("Serial IO error: {0}")]
    SerialIO(#[from] io::Error),

    #[error("Received unexpected data: {0}")]
    RecvUnexpected(String),
}

impl embedded_can::Error for Error {
    fn kind(&self) -> embedded_can::ErrorKind {
        // We don't have a way to distinguish between different kinds of errors.
        embedded_can::ErrorKind::Other
    }
}

impl NewRecvUnexpectedError for Error {
    fn new_recv_unexpected_error(message: impl Into<String>) -> Self {
        Self::RecvUnexpected(message.into())
    }
}

pub type Result<T> = result::Result<T, Error>;

#[derive(Debug, Clone, PartialEq)]
pub struct Usb2CanBuilder {
    path: String,
    serial_baud_rate: SerialBaudRate,
    serial_receive_timeout: Duration,
    frame_delay_multiplier: f64,
    adapter_configuration: Usb2CanConfiguration,
}

impl Usb2CanBuilder {
    pub fn path(&self) -> &str {
        &self.path
    }

    #[must_use]
    pub fn set_path<'a>(mut self, path: impl Into<std::borrow::Cow<'a, str>>) -> Self {
        self.path = path.into().to_string();
        self
    }

    pub const fn serial_baud_rate(&self) -> SerialBaudRate {
        self.serial_baud_rate
    }

    #[must_use]
    pub const fn set_serial_baud_rate(mut self, serial_baud_rate: SerialBaudRate) -> Self {
        self.serial_baud_rate = serial_baud_rate;
        self
    }

    pub const fn serial_receive_timeout(&self) -> Duration {
        self.serial_receive_timeout
    }

    #[must_use]
    pub const fn set_serial_receive_timeout(mut self, serial_receive_timeout: Duration) -> Self {
        self.serial_receive_timeout = serial_receive_timeout;
        self
    }

    pub const fn frame_delay_multiplier(&self) -> f64 {
        self.frame_delay_multiplier
    }

    pub fn set_frame_delay_multiplier(mut self, frame_delay_multiplier: f64) -> Result<Self> {
        if frame_delay_multiplier.is_finite() && frame_delay_multiplier.is_sign_positive() {
            self.frame_delay_multiplier = frame_delay_multiplier;
            Ok(self)
        } else {
            Err(Error::SetConfiguration(
                "Frame delay multiplier must be a positive finite number".into(),
            ))
        }
    }

    pub const fn adapter_configuration(&self) -> &Usb2CanConfiguration {
        &self.adapter_configuration
    }

    #[must_use]
    pub const fn set_adapter_configuration(
        mut self,
        adapter_configuration: Usb2CanConfiguration,
    ) -> Self {
        self.adapter_configuration = adapter_configuration;
        self
    }

    pub fn open_without_blink_delay(&self) -> Result<Usb2Can> {
        self.open_inner(false)
    }

    pub fn open(&self) -> Result<Usb2Can> {
        self.open_inner(true)
    }

    fn open_inner(&self, with_blink_delay: bool) -> Result<Usb2Can> {
        debug!("Opening USB2CAN with configuration {:?}", self);

        let serial = serialport::new(&self.path, u32::from(self.serial_baud_rate))
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::Two);
        let mut serial = serial.open()?;
        debug!("Serial port opened: {:?}", serial.name());

        let mut receiver = Receiver::new(
            serial.try_clone()?,
            self.adapter_configuration.variable_encoding(),
        );
        receiver.set_receive_timeout(self.serial_receive_timeout)?;

        // Set timeout for writing to "infinite" value to make every write blocking.
        serial.set_timeout(INFINITE_TIMEOUT).map_err(|error| {
            Error::SetConfiguration(format!("Failed to set write timeout to MAX: {}", error))
        })?;
        let blink_delay = self.serial_baud_rate.to_blink_delay();
        let mut transmitter = Transmitter::new(serial, blink_delay);
        transmitter.set_frame_delay_multiplier(self.frame_delay_multiplier)?;

        if with_blink_delay {
            debug!(
                "Initial blink delay enabled: {:.03}s",
                blink_delay.as_secs_f64()
            );
            receiver.add_delay(blink_delay);
            transmitter.add_delay(blink_delay);
        };

        let mut usb2can = Usb2Can {
            receiver: Arc::new(Mutex::new(receiver)),
            transmitter: Arc::new(Mutex::new(transmitter)),
            configuration: Arc::new(Mutex::new(self.adapter_configuration.clone())),
        };

        usb2can.set_configuration_inner(None)?;

        Ok(usb2can)
    }
}

pub fn new<'a>(
    path: impl Into<Cow<'a, str>>,
    adapter_configuration: &Usb2CanConfiguration,
) -> Usb2CanBuilder {
    Usb2CanBuilder {
        path: path.into().into_owned(),
        serial_baud_rate: DEFAULT_SERIAL_BAUD_RATE,
        serial_receive_timeout: DEFAULT_SERIAL_RECEIVE_TIMEOUT,
        frame_delay_multiplier: DEFAULT_FRAME_DELAY_MULTIPLIER,
        adapter_configuration: adapter_configuration.clone(),
    }
}

#[derive(Clone)]
pub struct Usb2Can {
    receiver: Arc<Mutex<Receiver>>,
    transmitter: Arc<Mutex<Transmitter>>,
    configuration: Arc<Mutex<Usb2CanConfiguration>>,
}

impl Usb2Can {
    pub fn name(&self) -> Result<String> {
        self.lock_transmitter()
            .and_then(|transmitter| transmitter.name())
    }

    pub fn serial_receive_timeout(&self) -> Result<Duration> {
        self.lock_receiver()
            .map(|receiver| receiver.receive_timeout())
    }

    pub fn set_serial_receive_timeout(&mut self, timeout: Duration) -> Result<()> {
        debug!(
            "Changing serial receive timeout to {:.03}s",
            timeout.as_secs_f64()
        );
        self.lock_receiver()
            .and_then(|mut receiver| receiver.set_receive_timeout(timeout))
    }

    pub fn configuration(&self) -> Result<Usb2CanConfiguration> {
        Ok(self.lock_configuration()?.clone())
    }

    pub fn set_configuration(&mut self, configuration: &Usb2CanConfiguration) -> Result<()> {
        self.set_configuration_inner(Some(configuration.clone()))
    }

    fn set_configuration_inner(
        &mut self,
        configuration: Option<Usb2CanConfiguration>,
    ) -> Result<()> {
        debug!("Changing adapter configuration to {:?}...", configuration);

        let mut receiver_guard = self.lock_receiver()?;
        let mut transmitter_guard = self.lock_transmitter()?;
        let mut configuration_guard = self.lock_configuration()?;

        let config_message = if let Some(configuration) = configuration {
            let config_message = configuration.to_fixed_message();
            *configuration_guard = configuration;
            config_message
        } else {
            configuration_guard.to_fixed_message()
        };

        receiver_guard.set_encoding(configuration_guard.variable_encoding());
        receiver_guard.clear()?;
        transmitter_guard.transmit_all(&config_message)?;

        // Adapter needs some time to process the configuration change.
        receiver_guard.add_delay(CONFIGURATION_DELAY);
        transmitter_guard.add_delay(CONFIGURATION_DELAY);

        debug!("Done changing adapter configuration!");
        Ok(())
    }

    pub fn serial_baud_rate(&self) -> Result<SerialBaudRate> {
        self.lock_transmitter()?.serial_baud_rate()
    }

    pub fn set_serial_baud_rate(&mut self, serial_baud_rate: SerialBaudRate) -> Result<()> {
        debug!("Changing serial baud rate to {}...", serial_baud_rate);

        let mut receiver_guard = self.lock_receiver()?;
        let mut transmitter_guard = self.lock_transmitter()?;

        let config_message = serial_baud_rate.to_fixed_message();
        transmitter_guard.transmit_all(&config_message)?;

        // Adapater does not respond while blinking.
        let blink_delay = serial_baud_rate.to_blink_delay();
        transmitter_guard.set_blink_delay(blink_delay);

        receiver_guard.add_delay(blink_delay);
        transmitter_guard.add_delay(blink_delay);

        // Set the new baud rate on underlying serial interface.
        transmitter_guard.set_serial_baud_rate(serial_baud_rate)?;
        receiver_guard.clear()?;

        debug!("Done changing serial baud rate!");
        Ok(())
    }

    pub fn frame_delay_multiplier(&self) -> Result<f64> {
        self.lock_transmitter()
            .map(|transmitter| transmitter.frame_delay_multiplier())
    }

    pub fn set_frame_delay_multiplier(&mut self, frame_delay_multiplier: f64) -> Result<()> {
        debug!(
            "Changing frame delay multiplier to {:.03}...",
            frame_delay_multiplier
        );
        self.lock_transmitter().and_then(|mut transmitter| {
            transmitter.set_frame_delay_multiplier(frame_delay_multiplier)
        })
    }

    pub fn set_stored_id_filter(&mut self, filter: &StoredIdFilter) -> Result<()> {
        debug!("Changing stored ID filter to {:?}...", filter);

        let mut receiver_guard = self.lock_receiver()?;
        let mut transmitter_guard = self.lock_transmitter()?;

        let config_message = filter.to_fixed_message();
        transmitter_guard.transmit_all(&config_message)?;

        // Adapter blinks after setting the filter settings.
        let blink_delay = transmitter_guard.blink_delay();
        transmitter_guard.add_delay(blink_delay);

        receiver_guard.clear()?;

        debug!("Done changing stored ID filter!");
        Ok(())
    }

    pub fn delay(&self) -> Result<()> {
        debug!("Sleeping to remove receiver and transmitter delays...");

        let receiver_guard = self.lock_receiver()?;
        let transmitter_guard = self.lock_transmitter()?;

        receiver_guard.delay();
        transmitter_guard.delay();

        Ok(())
    }

    fn lock_configuration(&self) -> Result<MutexGuard<Usb2CanConfiguration>> {
        self.configuration
            .lock()
            .map_err(|error| Error::Locking(format!("{} (configuration)", error)))
    }

    fn lock_transmitter(&self) -> Result<MutexGuard<Transmitter>> {
        self.transmitter
            .lock()
            .map_err(|error| Error::Locking(format!("{} (transmitter)", error)))
    }

    fn lock_receiver(&self) -> Result<MutexGuard<Receiver>> {
        self.receiver
            .lock()
            .map_err(|error| Error::Locking(format!("{} (receiver)", error)))
    }
}

impl blocking::Can for Usb2Can {
    type Frame = Frame;
    type Error = Error;

    fn transmit(&mut self, frame: &Frame) -> Result<()> {
        trace!("Transmitting frame: {:?}", frame);

        let mut transmitter_guard = self.lock_transmitter()?;
        let configuration_guard = self.lock_configuration()?;

        if configuration_guard.variable_encoding() {
            transmitter_guard.transmit_all(&frame.to_variable_message())?;
        } else {
            transmitter_guard.transmit_all(&frame.to_fixed_message())?;
        }

        let delay_multipyer = transmitter_guard.frame_delay_multiplier();
        if delay_multipyer > f64::EPSILON {
            let delay =
                configuration_guard.can_baud_rate.to_bit_length() * frame.length_bound_in_bits();
            let delay = delay.mul_f64(delay_multipyer);
            transmitter_guard.add_delay(delay);
        }

        Ok(())
    }

    fn receive(&mut self) -> Result<Self::Frame> {
        let mut receiver_guard = self.lock_receiver()?;
        let _configuration_guard = self.lock_configuration()?;

        receiver_guard.receive_frame()
    }
}

struct Transmitter {
    serial: Box<dyn SerialPort>,
    blink_delay: Duration,
    ready_at: Instant,
    frame_delay_multiplier: f64,
}

impl Transmitter {
    fn new(serial: Box<dyn SerialPort>, blink_delay: Duration) -> Self {
        Self {
            serial,
            blink_delay,
            ready_at: Instant::now(),
            frame_delay_multiplier: 0.0,
        }
    }

    fn name(&self) -> Result<String> {
        self.serial
            .name()
            .ok_or_else(|| Error::GetConfiguration("Failed to get serial port name".into()))
    }

    const fn blink_delay(&self) -> Duration {
        self.blink_delay
    }

    fn set_blink_delay(&mut self, blink_delay: Duration) {
        self.blink_delay = blink_delay;
    }

    fn serial_baud_rate(&self) -> Result<SerialBaudRate> {
        self.serial
            .baud_rate()
            .map_err(|error| {
                Error::GetConfiguration(format!("Failed to get serial baud rate: {}", error))
            })
            .and_then(|value| {
                SerialBaudRate::try_from(value).map_err(|error| {
                    Error::GetConfiguration(format!(
                        "Unsupported baud rate on underlying serial interface: {}",
                        error
                    ))
                })
            })
    }

    fn set_serial_baud_rate(&mut self, serial_baud_rate: SerialBaudRate) -> Result<()> {
        self.delay();
        self.serial
            .set_baud_rate(u32::from(serial_baud_rate))
            .map_err(|error| {
                Error::SetConfiguration(format!("Failed to set serial baud rate: {}", error))
            })
    }

    const fn frame_delay_multiplier(&self) -> f64 {
        self.frame_delay_multiplier
    }

    fn set_frame_delay_multiplier(&mut self, frame_delay_multiplier: f64) -> Result<()> {
        if frame_delay_multiplier.is_finite() && frame_delay_multiplier.is_sign_positive() {
            self.frame_delay_multiplier = frame_delay_multiplier;
            Ok(())
        } else {
            Err(Error::SetConfiguration(
                "Frame delay multiplier must be a positive finite number".into(),
            ))
        }
    }

    fn transmit_all(&mut self, message: &[u8]) -> Result<()> {
        self.delay();
        trace!("Writing into serial: {:?}", message);
        self.serial.write_all(message)?;
        self.serial.flush()?;
        Ok(())
    }

    fn delay(&self) {
        let now = Instant::now();
        if now < self.ready_at {
            let delay = self.ready_at - now;
            trace!(
                "Sleeping for {:.03}s before transmitting anything...",
                delay.as_secs_f64()
            );
            sleep(delay);
        }
    }

    fn add_delay(&mut self, delay: Duration) {
        let now = Instant::now();
        if now > self.ready_at {
            self.ready_at = now + delay;
        } else {
            self.ready_at += delay;
        }
    }
}

struct Receiver {
    serial: Box<dyn SerialPort>,
    buffer: Box<dyn GenericReceiveBuffer<Error = Error> + Send>,
    ready_at: Instant,
}

impl Receiver {
    fn new(serial: Box<dyn SerialPort>, variable: bool) -> Self {
        Self {
            serial,
            buffer: new_receive_buffer(variable),
            ready_at: Instant::now(),
        }
    }

    fn set_encoding(&mut self, variable: bool) {
        if self.buffer.is_variable() != variable {
            self.buffer = new_receive_buffer(variable);
        }
    }

    fn receive_timeout(&self) -> Duration {
        self.serial.timeout()
    }

    fn set_receive_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.serial.set_timeout(timeout).map_err(|error| {
            Error::SetConfiguration(format!("Failed to set receive timeout: {}", error))
        })
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        let mut buffer = [0u8; MAX_MESSAGE_SIZE];

        loop {
            if let Some(frame) = self.buffer.receive_frame()? {
                break Ok(frame);
            };

            match self.serial.read(&mut buffer) {
                Ok(bytes_read) => self.buffer.feed_bytes(&buffer[..bytes_read])?,

                Err(error) => {
                    if error.kind() == io::ErrorKind::TimedOut {
                        return Err(Error::SerialReadTimedOut);
                    } else {
                        return Err(Error::from(error));
                    }
                }
            };
        }
    }

    fn clear(&mut self) -> Result<()> {
        self.serial.clear(ClearBuffer::All)?;
        self.buffer.clear();
        self.buffer.resync();
        Ok(())
    }

    fn delay(&self) {
        let now = Instant::now();
        if now < self.ready_at {
            let delay = self.ready_at - now;
            debug!(
                "Sleeping for {:.03}s before trying to receive anything...",
                delay.as_secs_f64()
            );
            sleep(delay);
        }
    }

    fn add_delay(&mut self, delay: Duration) {
        let now = Instant::now();
        if now > self.ready_at {
            self.ready_at = now + delay;
        } else {
            self.ready_at += delay;
        }
    }
}

#[cfg(test)]
mod tests {
    use rustix::{
        fd::OwnedFd,
        pty::{grantpt, openpt, ptsname, unlockpt, OpenptFlags},
    };

    use super::*;

    #[test]
    fn rust_serialport_cloned_timeout() {
        // Check that the timeout in the cloned serial port is independent of the original.
        let (_mpt, mut port) = open_pty();
        let mut port_clone = port.try_clone().expect("serialport::try_clone() failed");

        port.set_timeout(Duration::from_secs(1)).unwrap();
        port_clone.set_timeout(Duration::from_secs(2)).unwrap();

        assert_eq!(port.timeout(), Duration::from_secs(1));
        assert_eq!(port_clone.timeout(), Duration::from_secs(2));
    }

    #[test]
    fn rust_serialport_large_timeout() {
        let (_mpt, mut port) = open_pty();
        port.set_timeout(INFINITE_TIMEOUT).unwrap();
        port.write_all(&[0x00]).unwrap();
    }

    fn open_pty() -> (OwnedFd, Box<dyn SerialPort>) {
        let mpt = openpt(OpenptFlags::RDWR | OpenptFlags::NOCTTY).expect("posix_openpt() failed");
        grantpt(&mpt).expect("grantpt() failed");
        unlockpt(&mpt).expect("unlockpt() failed");

        let pt_path = ptsname(&mpt, vec![0u8; 1024]).expect("ptsname() failed");
        let pt_path =
            String::from_utf8(pt_path.into_bytes()).expect("ptsname() returned invalid UTF-8");

        (
            mpt,
            serialport::new(pt_path, 9600)
                .open()
                .expect("serialport::open() failed"),
        )
    }
}
