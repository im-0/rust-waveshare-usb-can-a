use std::{
    borrow::Cow,
    result,
    sync::Arc,
    time::{Duration, Instant},
};

use thiserror::Error;
use tokio::{
    io::{self, AsyncReadExt, AsyncWriteExt},
    sync::{Mutex, MutexGuard},
    time::sleep,
};
use tokio_serial::{
    ClearBuffer, DataBits, SerialPort, SerialPortBuilderExt, SerialStream, StopBits,
};
use tracing::{debug, trace};

use crate::{
    proto::{
        new_receive_buffer, GenericReceiveBuffer, NewRecvUnexpectedError, ToFixedMessage,
        ToVariableMessage, MAX_MESSAGE_SIZE,
    },
    Frame, SerialBaudRate, StoredIdFilter, Usb2CanConfiguration, CONFIGURATION_DELAY,
    DEFAULT_FRAME_DELAY_MULTIPLIER, DEFAULT_SERIAL_BAUD_RATE,
};

#[derive(Error, Debug)]
pub enum Error {
    #[error("Configuration error: {0}")]
    SetConfiguration(String),

    #[error("Configuration reading error: {0}")]
    GetConfiguration(String),

    #[error("Locking error: {0}")]
    Locking(String),

    #[error("Serial port error: {}", .0.description)]
    Serial(#[from] tokio_serial::Error),

    #[error("Serial IO error: {0}")]
    SerialIO(#[from] io::Error),

    #[error("Received unexpected data: {0}")]
    RecvUnexpected(String),
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

    pub async fn open_without_blink_delay(&self) -> Result<Usb2Can> {
        self.open_inner(false).await
    }

    pub async fn open(&self) -> Result<Usb2Can> {
        self.open_inner(true).await
    }

    async fn open_inner(&self, with_blink_delay: bool) -> Result<Usb2Can> {
        debug!("Opening USB2CAN with configuration {:?}", self);

        let serial = tokio_serial::new(&self.path, u32::from(self.serial_baud_rate))
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::Two);
        let serial = serial.open_native_async()?;
        debug!("Serial port opened: {:?}", serial.name());
        let serial = Arc::new(Mutex::new(serial));

        let mut receiver = Receiver::new(
            serial.clone(),
            self.adapter_configuration.variable_encoding(),
        );

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

        usb2can.set_configuration_inner(None).await?;

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
    pub async fn name(&self) -> Result<String> {
        self.lock_transmitter().await.name().await
    }

    pub async fn configuration(&self) -> Result<Usb2CanConfiguration> {
        Ok(self.lock_configuration().await.clone())
    }

    pub async fn set_configuration(&mut self, configuration: &Usb2CanConfiguration) -> Result<()> {
        self.set_configuration_inner(Some(configuration.clone()))
            .await
    }

    async fn set_configuration_inner(
        &mut self,
        configuration: Option<Usb2CanConfiguration>,
    ) -> Result<()> {
        debug!("Changing adapter configuration to {:?}...", configuration);

        let mut receiver_guard = self.lock_receiver().await;
        let mut transmitter_guard = self.lock_transmitter().await;
        let mut configuration_guard = self.lock_configuration().await;

        let config_message = if let Some(configuration) = configuration {
            let config_message = configuration.to_fixed_message();
            *configuration_guard = configuration;
            config_message
        } else {
            configuration_guard.to_fixed_message()
        };

        receiver_guard.set_encoding(configuration_guard.variable_encoding());
        receiver_guard.clear().await?;
        transmitter_guard.transmit_all(&config_message).await?;

        // Adapter needs some time to process the configuration change.
        receiver_guard.add_delay(CONFIGURATION_DELAY);
        transmitter_guard.add_delay(CONFIGURATION_DELAY);

        debug!("Done changing adapter configuration!");
        Ok(())
    }

    pub async fn serial_baud_rate(&self) -> Result<SerialBaudRate> {
        self.lock_transmitter().await.serial_baud_rate().await
    }

    pub async fn set_serial_baud_rate(&mut self, serial_baud_rate: SerialBaudRate) -> Result<()> {
        debug!("Changing serial baud rate to {}...", serial_baud_rate);

        let mut receiver_guard = self.lock_receiver().await;
        let mut transmitter_guard = self.lock_transmitter().await;

        let config_message = serial_baud_rate.to_fixed_message();
        transmitter_guard.transmit_all(&config_message).await?;

        // Adapater does not respond while blinking.
        let blink_delay = serial_baud_rate.to_blink_delay();
        transmitter_guard.set_blink_delay(blink_delay);

        receiver_guard.add_delay(blink_delay);
        transmitter_guard.add_delay(blink_delay);

        // Set the new baud rate on underlying serial interface.
        transmitter_guard
            .set_serial_baud_rate(serial_baud_rate)
            .await?;
        receiver_guard.clear().await?;

        debug!("Done changing serial baud rate!");
        Ok(())
    }

    pub async fn frame_delay_multiplier(&self) -> f64 {
        self.lock_transmitter().await.frame_delay_multiplier()
    }

    pub async fn set_frame_delay_multiplier(&mut self, frame_delay_multiplier: f64) -> Result<()> {
        debug!(
            "Changing frame delay multiplier to {:.03}...",
            frame_delay_multiplier
        );
        self.lock_transmitter()
            .await
            .set_frame_delay_multiplier(frame_delay_multiplier)
    }

    pub async fn set_stored_id_filter(&mut self, filter: &StoredIdFilter) -> Result<()> {
        debug!("Changing stored ID filter to {:?}...", filter);

        let mut receiver_guard = self.lock_receiver().await;
        let mut transmitter_guard = self.lock_transmitter().await;

        let config_message = filter.to_fixed_message();
        transmitter_guard.transmit_all(&config_message).await?;

        // Adapter blinks after setting the filter settings.
        let blink_delay = transmitter_guard.blink_delay();
        transmitter_guard.add_delay(blink_delay);

        receiver_guard.clear().await?;

        debug!("Done changing stored ID filter!");
        Ok(())
    }

    pub async fn delay(&self) -> Result<()> {
        debug!("Sleeping to remove receiver and transmitter delays...");

        let receiver_guard = self.lock_receiver().await;
        let transmitter_guard = self.lock_transmitter().await;

        receiver_guard.delay().await;
        transmitter_guard.delay().await;

        Ok(())
    }

    pub async fn transmit(&mut self, frame: &Frame) -> Result<()> {
        trace!("Transmitting frame: {:?}", frame);

        let mut transmitter_guard = self.lock_transmitter().await;
        let configuration_guard = self.lock_configuration().await;

        if configuration_guard.variable_encoding() {
            transmitter_guard
                .transmit_all(&frame.to_variable_message())
                .await?;
        } else {
            transmitter_guard
                .transmit_all(&frame.to_fixed_message())
                .await?;
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

    pub async fn receive(&mut self) -> Result<Frame> {
        let mut receiver_guard = self.lock_receiver().await;
        let _configuration_guard = self.lock_configuration().await;

        receiver_guard.receive_frame().await
    }

    async fn lock_configuration(&self) -> MutexGuard<Usb2CanConfiguration> {
        self.configuration.lock().await
    }

    async fn lock_transmitter(&self) -> MutexGuard<Transmitter> {
        self.transmitter.lock().await
    }

    async fn lock_receiver(&self) -> MutexGuard<Receiver> {
        self.receiver.lock().await
    }
}

struct Transmitter {
    serial: Arc<Mutex<SerialStream>>,
    blink_delay: Duration,
    ready_at: Instant,
    frame_delay_multiplier: f64,
}

impl Transmitter {
    fn new(serial: Arc<Mutex<SerialStream>>, blink_delay: Duration) -> Self {
        Self {
            serial,
            blink_delay,
            ready_at: Instant::now(),
            frame_delay_multiplier: 0.0,
        }
    }

    async fn name(&self) -> Result<String> {
        self.serial
            .lock()
            .await
            .name()
            .ok_or_else(|| Error::GetConfiguration("Failed to get serial port name".into()))
    }

    const fn blink_delay(&self) -> Duration {
        self.blink_delay
    }

    const fn set_blink_delay(&mut self, blink_delay: Duration) {
        self.blink_delay = blink_delay;
    }

    async fn serial_baud_rate(&self) -> Result<SerialBaudRate> {
        self.serial
            .lock()
            .await
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

    async fn set_serial_baud_rate(&mut self, serial_baud_rate: SerialBaudRate) -> Result<()> {
        self.delay().await;
        self.serial
            .lock()
            .await
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

    async fn transmit_all(&mut self, message: &[u8]) -> Result<()> {
        self.delay().await;
        trace!("Writing into serial: {:?}", message);

        let mut serial = self.serial.lock().await;
        serial.write_all(message).await?;
        serial.flush().await?;

        Ok(())
    }

    async fn delay(&self) {
        let now = Instant::now();
        if now < self.ready_at {
            let delay = self.ready_at - now;
            trace!(
                "Sleeping for {:.03}s before transmitting anything...",
                delay.as_secs_f64()
            );
            sleep(delay).await;
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
    serial: Arc<Mutex<SerialStream>>,
    buffer: Box<dyn GenericReceiveBuffer<Error = Error> + Send>,
    ready_at: Instant,
}

impl Receiver {
    fn new(serial: Arc<Mutex<SerialStream>>, variable: bool) -> Self {
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

    async fn receive_frame(&mut self) -> Result<Frame> {
        let mut buffer = [0u8; MAX_MESSAGE_SIZE];

        loop {
            if let Some(frame) = self.buffer.receive_frame()? {
                break Ok(frame);
            };

            let bytes_read = self.serial.lock().await.read(&mut buffer).await?;
            self.buffer.feed_bytes(&buffer[..bytes_read])?;
        }
    }

    async fn clear(&mut self) -> Result<()> {
        self.serial.lock().await.clear(ClearBuffer::All)?;
        self.buffer.clear();
        self.buffer.resync();
        Ok(())
    }

    async fn delay(&self) {
        let now = Instant::now();
        if now < self.ready_at {
            let delay = self.ready_at - now;
            debug!(
                "Sleeping for {:.03}s before trying to receive anything...",
                delay.as_secs_f64()
            );
            sleep(delay).await;
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
