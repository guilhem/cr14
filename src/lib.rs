//! Independent, synchronous CR14 access over Linux I2C.
//!
//! Construct with an `i2cdev::linux::LinuxI2CDevice`, then call
//! [`Cr14::reinitialize`] before discovery or memory operations. Deadlines bound
//! controlled waits, not an ioctl already executing in the Linux kernel.
//! See `PROVENANCE.md` for the specifications used to implement the protocol.
#![cfg(target_os = "linux")]

mod clock;

use clock::Clock;
use i2cdev::core::{I2CDevice, I2CMessage, I2CTransfer};
use i2cdev::linux::LinuxI2CError;
use std::fmt;
use std::time::{Duration, Instant};

const PARAMETER: u8 = 0x00;
const FRAME: u8 = 0x01;
const SLOT_MARKER: u8 = 0x03;
const RF_ON: u8 = 0x10; // CR14 table 3: RF on, 500 us answer watchdog.
const POWER_ON: Duration = Duration::from_millis(20); // CR14 table 13, tPOR.
const TAG_POWER_ON: Duration = Duration::from_millis(5); // SRI table 9, tMIN CD.
const INTER_REQUEST: Duration = Duration::from_micros(200); // >= SRI t2, 132 us.
const READ_BACKOFF: Duration = Duration::from_micros(250);
// Conservative guards for an ambiguous submission: <= 6 request bytes,
// <= 8 answer bytes, hardware CRC/SOF/EOF and the 500 us watchdog.
// CR14 sections 6, 7 and tables 13/14; these are library budgets, not tW.
const RF_EXCHANGE: Duration = Duration::from_millis(4);
const SLOT_EXCHANGE: Duration = Duration::from_millis(32); // 16 exchanges.
const WRITE_TRANSMISSION: Duration = Duration::from_millis(2);
const SRI_PROGRAMMING: Duration = Duration::from_millis(7);

/// Eight UID bytes in RF order: least significant byte first.
/// Display reverses this order into the usual hexadecimal UID notation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct Uid(pub [u8; 8]);

impl fmt::Display for Uid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, byte) in self.0.iter().rev().enumerate() {
            if i != 0 {
                f.write_str(":")?;
            }
            write!(f, "{byte:02x}")?;
        }
        Ok(())
    }
}

/// Four data bytes in RF order. Numeric interpretation is the caller's choice.
pub type Block = [u8; 4];

/// Library lifecycle. A failed cleanup requires explicit verified reinitialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum State {
    Uninitialized,
    Ready,
    Active,
    NeedsReinitialization,
}

/// Typed operation errors, preserving the concrete Linux I2C error.
#[derive(Debug)]
pub enum Error {
    I2c(LinuxI2CError),
    Deadline {
        last_i2c_error: Option<LinuxI2CError>,
    },
    InvalidArgument(&'static str),
    NeedsReinitialization,
    UnexpectedDevice,
    PartialTransfer {
        expected: u32,
        actual: u32,
    },
    InvalidFrame {
        expected: usize,
        actual: u8,
    },
    InvalidSlot {
        slot: usize,
        value: u8,
    },
    SelectMismatch {
        expected: u8,
        actual: u8,
    },
    Crc,
    NoResponse,
    TagNotFound(Uid),
    RegisterMismatch {
        expected: u8,
        actual: u8,
    },
    /// Cleanup failure never hides the original operation failure.
    Cleanup {
        operation: Option<Box<Error>>,
        cleanup: Box<Error>,
    },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::I2c(e) => write!(f, "I2C: {e}"),
            Self::Deadline { last_i2c_error } => {
                f.write_str("operation deadline reached")?;
                if let Some(e) = last_i2c_error {
                    write!(f, " (last I2C error: {e})")?;
                }
                Ok(())
            }
            Self::InvalidArgument(e) => write!(f, "invalid argument: {e}"),
            Self::NeedsReinitialization => f.write_str("verified reinitialization required"),
            Self::UnexpectedDevice => f.write_str("device does not exhibit CR14 register behavior"),
            Self::PartialTransfer { expected, actual } => {
                write!(f, "I2C transferred {actual}/{expected} messages")
            }
            Self::InvalidFrame { expected, actual } => {
                write!(f, "expected {expected} RF bytes, received length {actual}")
            }
            Self::InvalidSlot { slot, value } => write!(f, "invalid slot {slot}: {value:#04x}"),
            Self::SelectMismatch { expected, actual } => {
                write!(f, "select returned {actual:#04x}, expected {expected:#04x}")
            }
            Self::Crc => f.write_str("RF CRC error"),
            Self::NoResponse => f.write_str("tag did not respond"),
            Self::TagNotFound(uid) => write!(f, "target tag {uid} not found"),
            Self::RegisterMismatch { expected, actual } => write!(
                f,
                "parameter readback {actual:#04x}, expected {expected:#04x}"
            ),
            Self::Cleanup { operation, cleanup } => {
                if let Some(e) = operation {
                    write!(f, "{e}; ")?;
                }
                write!(f, "RF cleanup failed: {cleanup}")
            }
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::I2c(e) => Some(e),
            Self::Deadline {
                last_i2c_error: Some(e),
            } => Some(e),
            Self::Cleanup {
                operation: Some(e), ..
            } => Some(e.as_ref()),
            Self::Cleanup { cleanup, .. } => Some(cleanup.as_ref()),
            _ => None,
        }
    }
}

/// Progress for sequential writes. Successful transmission is not proof of programming.
#[derive(Debug, Default)]
pub struct WriteProgress {
    /// Requests whose I2C write succeeded, in request order (duplicates preserved).
    pub written: Vec<u8>,
    /// Blocks for which a subsequent read succeeded, with the observed data.
    pub readback: Vec<(u8, Block)>,
    /// A request may have reached this block, but no reliable readback is available.
    pub possibly_changed: Option<u8>,
}

/// A write failure retains both progression and the original/cleanup errors.
#[derive(Debug)]
pub struct WriteError {
    pub progress: WriteProgress,
    pub error: Error,
}

impl fmt::Display for WriteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} ({} requests sent, {} blocks read back, uncertain {:?})",
            self.error,
            self.progress.written.len(),
            self.progress.readback.len(),
            self.progress.possibly_changed
        )
    }
}

impl std::error::Error for WriteError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.error)
    }
}

/// One exclusively owned CR14 transport. No worker threads or application events.
///
/// The transport must implement the existing i2cdev traits. Production uses
/// `LinuxI2CDevice`; its configured slave address is retained by the library.
pub struct Cr14<D>
where
    D: I2CDevice<Error = LinuxI2CError>,
    for<'a> D: I2CTransfer<'a, Error = LinuxI2CError>,
{
    device: D,
    state: State,
    clock: Clock,
    programming_time: Duration,
    quiet_until: Option<Instant>,
    programming_until: Option<Instant>,
}

impl<D> Cr14<D>
where
    D: I2CDevice<Error = LinuxI2CError>,
    for<'a> D: I2CTransfer<'a, Error = LinuxI2CError>,
{
    /// Takes ownership without bus traffic. Call `reinitialize(deadline)` next.
    /// The default programming budget is the SRI512/SRI4K maximum of 7 ms.
    pub fn new(device: D) -> Self {
        Self {
            device,
            state: State::Uninitialized,
            clock: Clock::new(),
            programming_time: SRI_PROGRAMMING,
            quiet_until: None,
            programming_until: None,
        }
    }

    /// Allows a longer model-specific programming budget; never below 7 ms.
    /// This changes a timing bound, not the supported tag command protocol.
    pub fn with_programming_time(device: D, programming_time: Duration) -> Result<Self, Error> {
        if programming_time < SRI_PROGRAMMING
            || Instant::now()
                .checked_add(programming_time)
                .and_then(|t| t.checked_add(WRITE_TRANSMISSION))
                .is_none()
        {
            return Err(Error::InvalidArgument(
                "programming time must be representable and at least 7 ms",
            ));
        }
        let mut reader = Self::new(device);
        reader.programming_time = programming_time;
        Ok(reader)
    }

    pub fn state(&self) -> State {
        self.state
    }

    /// Probe documented register behavior and confirm RF off. There is no silicon
    /// identification register: this is a behavior check, not unique identification.
    /// Only this operation can recover a failed cleanup.
    pub fn reinitialize(&mut self, deadline: Instant) -> Result<(), Error> {
        self.state = State::NeedsReinitialization;
        self.wait_safe();
        self.wait_for(POWER_ON, deadline)?;
        let result = (|| {
            self.check_deadline(deadline, None)?;
            self.read_register::<1>(PARAMETER)?;
            self.check_deadline(deadline, None)?;
            match self.read_register::<1>(0x7f) {
                Err(Error::I2c(e)) if is_nack(&e) => {}
                Err(e) => return Err(e),
                Ok(_) => return Err(Error::UnexpectedDevice),
            }
            self.check_deadline(deadline, None)?;
            self.off_verified()?;
            self.check_deadline(deadline, None)
        })();
        if result.is_ok() {
            self.state = State::Ready;
        }
        result
    }

    /// Always waits through possible RF transmission/programming before RF off.
    /// Cleanup deliberately has no cancellable deadline. An ioctl still depends
    /// on the Linux adapter timeout. Failure requires explicit reinitialization.
    pub fn shutdown(&mut self) -> Result<(), Error> {
        let was_ready = matches!(self.state, State::Ready | State::Active);
        self.state = State::NeedsReinitialization;
        self.wait_safe();
        self.off_verified()?;
        if was_ready {
            self.state = State::Ready;
        }
        Ok(())
    }

    /// Enumerate all resolved tags. Returns a deadline error rather than silently
    /// claiming a complete inventory when collision resolution times out.
    pub fn inventory(&mut self, deadline: Instant) -> Result<Vec<Uid>, Error> {
        self.session(deadline, |reader| {
            let mut found = Vec::new();
            reader.scan(deadline, |_, uid| {
                if !found.contains(&uid) {
                    found.push(uid);
                }
                false
            })?;
            Ok(found)
        })
    }

    pub fn read_block(&mut self, uid: Uid, address: u8, deadline: Instant) -> Result<Block, Error> {
        self.read_blocks(uid, &[address], deadline)
            .map(|blocks| blocks[0])
    }

    /// Reads arbitrary addresses in order, after selecting and verifying the UID.
    pub fn read_blocks(
        &mut self,
        uid: Uid,
        addresses: &[u8],
        deadline: Instant,
    ) -> Result<Vec<Block>, Error> {
        if addresses.is_empty() {
            return Err(Error::InvalidArgument("empty address list"));
        }
        self.session(deadline, |reader| {
            reader.select_uid(uid, deadline)?;
            let mut blocks = Vec::with_capacity(addresses.len());
            for &address in addresses {
                blocks.push(reader.read_selected(address, deadline)?);
            }
            reader.no_answer(&[0x0f], deadline)?;
            Ok(blocks)
        })
    }

    /// Write once and return observed readback; memory semantics may transform data.
    pub fn write_block(
        &mut self,
        uid: Uid,
        address: u8,
        data: Block,
        deadline: Instant,
    ) -> Result<Block, WriteError> {
        self.write_blocks(uid, &[(address, data)], deadline)
            .map(|p| p.readback[0].1)
    }

    /// Sequential, non-atomic writes. Every request is sent at most once, followed
    /// by its programming window and readback before proceeding to the next block.
    /// OTP/counter readback is observed data, not an unconditional equality promise.
    pub fn write_blocks(
        &mut self,
        uid: Uid,
        writes: &[(u8, Block)],
        deadline: Instant,
    ) -> Result<WriteProgress, WriteError> {
        let mut progress = WriteProgress::default();
        if writes.is_empty() {
            return Err(WriteError {
                progress,
                error: Error::InvalidArgument("empty write list"),
            });
        }
        let result = self.session(deadline, |reader| {
            reader.select_uid(uid, deadline)?;
            for &(address, data) in writes {
                reader.before_request(deadline)?;
                // Calculate before submission, then start the full guard after the
                // ioctl returns: failure can still have launched programming.
                let guard = reader.programming_time + WRITE_TRANSMISSION;
                reader
                    .clock
                    .now()
                    .checked_add(guard)
                    .ok_or(Error::InvalidArgument(
                        "programming time exceeds clock range",
                    ))?;
                progress.possibly_changed = Some(address);
                let mut payload = [0_u8; 7];
                payload[0] = 6;
                payload[1] = 0x09;
                payload[2] = address;
                payload[3..].copy_from_slice(&data);
                let submitted = reader.device.smbus_write_i2c_block_data(FRAME, &payload);
                reader.programming_until = reader.clock.now().checked_add(guard);
                submitted.map_err(Error::I2c)?;
                progress.written.push(address);
                // Cannot end this wait early: the tag is powered by the RF field.
                reader.wait_safe();
                reader.check_deadline(deadline, None)?;
                let observed = reader.read_selected(address, deadline)?;
                progress.readback.push((address, observed));
                progress.possibly_changed = None;
            }
            reader.no_answer(&[0x0f], deadline)
        });
        match result {
            Ok(()) => Ok(progress),
            Err(error) => Err(WriteError { progress, error }),
        }
    }

    fn session<T>(
        &mut self,
        deadline: Instant,
        operation: impl FnOnce(&mut Self) -> Result<T, Error>,
    ) -> Result<T, Error> {
        if self.state != State::Ready {
            return Err(Error::NeedsReinitialization);
        }
        self.before_request(deadline)?;
        // Even a failed RF-on write may have changed the register.
        self.state = State::Active;
        let result = (|| {
            self.device
                .smbus_write_byte_data(PARAMETER, RF_ON)
                .map_err(Error::I2c)?;
            self.check_deadline(deadline, None)?;
            self.check_parameter(RF_ON)?;
            // Conservatively allow reader tPOR as well as the tag's field warmup.
            self.wait_for(POWER_ON + TAG_POWER_ON, deadline)?;
            operation(self)
        })();
        let cleanup = self.shutdown();
        match (result, cleanup) {
            (result, Ok(())) => result,
            (result, Err(cleanup)) => Err(Error::Cleanup {
                operation: result.err().map(Box::new),
                cleanup: Box::new(cleanup),
            }),
        }
    }

    fn select_uid(&mut self, target: Uid, deadline: Instant) -> Result<(), Error> {
        if self.scan(deadline, |_, uid| uid == target)? {
            Ok(())
        } else {
            Err(Error::TagNotFound(target))
        }
    }

    // SRI command/state tables: Initiate, then PCALL16 + slot markers on CRC.
    // Keep slot IDs in a separate owned array: each selection replaces the frame.
    fn scan(
        &mut self,
        deadline: Instant,
        mut selected: impl FnMut(u8, Uid) -> bool,
    ) -> Result<bool, Error> {
        let initial = self.exchange(&[0x06, 0x00], 1, deadline);
        let mut slots = match initial {
            Ok(bytes) => vec![bytes[0]],
            Err(Error::NoResponse) => return Ok(false),
            Err(Error::Crc) => Vec::new(),
            Err(e) => return Err(e),
        };
        let mut another_round = slots.is_empty();
        loop {
            self.check_deadline(deadline, None)?;
            if another_round {
                self.before_request(deadline)?;
                let result = self.device.smbus_write_byte(SLOT_MARKER);
                self.quiet_until = Some(self.clock.now() + SLOT_EXCHANGE);
                result.map_err(Error::I2c)?;
                let response = self.read_frame::<19>(deadline)?;
                if response[0] != 18 {
                    return Err(Error::InvalidFrame {
                        expected: 18,
                        actual: response[0],
                    });
                }
                let mask = u16::from_le_bytes([response[1], response[2]]);
                slots.clear();
                another_round = false;
                for slot in 0..16 {
                    let value = response[3 + slot];
                    if mask & (1 << slot) != 0 {
                        slots.push(value);
                    } else {
                        match value {
                            0 => {}
                            0xff => another_round = true,
                            _ => return Err(Error::InvalidSlot { slot, value }),
                        }
                    }
                }
            } else {
                another_round = false;
            }
            for id in slots.clone() {
                match self.identify(id, deadline) {
                    Ok(uid) => {
                        if selected(id, uid) {
                            return Ok(true);
                        }
                        self.no_answer(&[0x0f], deadline)?;
                    }
                    Err(Error::Crc) => {
                        self.no_answer(&[0x0c], deadline)?;
                        another_round = true;
                    }
                    Err(Error::NoResponse) => {
                        // A Select answer or Get_UID can be lost after selection.
                        // Restore Inventory so this candidate cannot be skipped.
                        self.no_answer(&[0x0c], deadline)?;
                        another_round = true;
                    }
                    Err(e) => return Err(e),
                }
            }
            if !another_round {
                return Ok(false);
            }
        }
    }

    fn identify(&mut self, id: u8, deadline: Instant) -> Result<Uid, Error> {
        let reply = self.exchange(&[0x0e, id], 1, deadline)?;
        if reply[0] != id {
            return Err(Error::SelectMismatch {
                expected: id,
                actual: reply[0],
            });
        }
        let uid = self.exchange(&[0x0b], 8, deadline)?;
        let mut bytes = [0_u8; 8];
        bytes.copy_from_slice(&uid);
        Ok(Uid(bytes))
    }

    fn read_selected(&mut self, address: u8, deadline: Instant) -> Result<Block, Error> {
        let response = self.exchange(&[0x08, address], 4, deadline)?;
        let mut block = [0_u8; 4];
        block.copy_from_slice(&response);
        Ok(block)
    }

    fn exchange(
        &mut self,
        request: &[u8],
        expected: usize,
        deadline: Instant,
    ) -> Result<Vec<u8>, Error> {
        self.send(request, deadline)?;
        // Every implemented ordinary reply fits nine bytes including length.
        let mut response = [0_u8; 9];
        self.poll_read(FRAME, &mut response[..expected + 1], deadline)?;
        match response[0] {
            0 => Err(Error::NoResponse),
            0xff => Err(Error::Crc),
            actual if usize::from(actual) != expected => {
                Err(Error::InvalidFrame { expected, actual })
            }
            _ => Ok(response[1..=expected].to_vec()),
        }
    }

    fn no_answer(&mut self, request: &[u8], deadline: Instant) -> Result<(), Error> {
        self.send(request, deadline)?;
        let reply = self.read_frame::<1>(deadline)?;
        match reply[0] {
            0 => Ok(()),
            0xff => Err(Error::Crc),
            actual => Err(Error::InvalidFrame {
                expected: 0,
                actual,
            }),
        }
    }

    fn send(&mut self, request: &[u8], deadline: Instant) -> Result<(), Error> {
        self.before_request(deadline)?;
        let mut payload = [0_u8; 7];
        payload[0] = request.len() as u8;
        payload[1..=request.len()].copy_from_slice(request);
        let result = self
            .device
            .smbus_write_i2c_block_data(FRAME, &payload[..request.len() + 1]);
        self.quiet_until = Some(self.clock.now() + RF_EXCHANGE);
        result.map_err(Error::I2c)
    }

    fn read_frame<const N: usize>(&mut self, deadline: Instant) -> Result<[u8; N], Error> {
        let mut bytes = [0_u8; N];
        self.poll_read(FRAME, &mut bytes, deadline)?;
        Ok(bytes)
    }

    fn poll_read(
        &mut self,
        register: u8,
        bytes: &mut [u8],
        deadline: Instant,
    ) -> Result<(), Error> {
        let mut last_error = None;
        loop {
            if self.clock.now() >= deadline {
                return Err(Error::Deadline {
                    last_i2c_error: last_error,
                });
            }
            match self.read_into(register, bytes) {
                Ok(()) => {
                    self.quiet_until = Some(self.clock.now() + INTER_REQUEST);
                    self.check_deadline(deadline, last_error.take())?;
                    return Ok(());
                }
                Err(Error::I2c(e)) if is_busy(&e) => last_error = Some(e),
                Err(e) => return Err(e),
            }
            let remaining = deadline.saturating_duration_since(self.clock.now());
            self.clock.sleep(READ_BACKOFF.min(remaining));
        }
    }

    fn read_register<const N: usize>(&mut self, register: u8) -> Result<[u8; N], Error> {
        let mut bytes = [0_u8; N];
        self.read_into(register, &mut bytes)?;
        Ok(bytes)
    }

    fn read_into(&mut self, register: u8, bytes: &mut [u8]) -> Result<(), Error> {
        let pointer = [register];
        // One ioctl: register pointer + repeated START + read + STOP.
        let mut messages = [
            <D as I2CTransfer<'_>>::Message::write(&pointer),
            <D as I2CTransfer<'_>>::Message::read(bytes),
        ];
        let actual = self.device.transfer(&mut messages).map_err(Error::I2c)?;
        if actual != 2 {
            return Err(Error::PartialTransfer {
                expected: 2,
                actual,
            });
        }
        Ok(())
    }

    fn check_parameter(&mut self, expected: u8) -> Result<(), Error> {
        let actual = self.read_register::<1>(PARAMETER)?[0];
        if actual != expected {
            return Err(Error::RegisterMismatch { expected, actual });
        }
        Ok(())
    }

    fn off_verified(&mut self) -> Result<(), Error> {
        self.device
            .smbus_write_byte_data(PARAMETER, 0)
            .map_err(Error::I2c)?;
        self.check_parameter(0)?;
        // Allow field-powered tags to reset before another session. This is a
        // conservative library interval, subject to antenna hardware qualification.
        self.quiet_until = Some(self.clock.now() + POWER_ON);
        Ok(())
    }

    fn check_deadline(
        &self,
        deadline: Instant,
        last_i2c_error: Option<LinuxI2CError>,
    ) -> Result<(), Error> {
        if self.clock.now() >= deadline {
            Err(Error::Deadline { last_i2c_error })
        } else {
            Ok(())
        }
    }

    fn before_request(&mut self, deadline: Instant) -> Result<(), Error> {
        if let Some(until) = self.quiet_until {
            self.wait_until(until, deadline)?;
        }
        self.check_deadline(deadline, None)
    }

    fn wait_for(&self, duration: Duration, deadline: Instant) -> Result<(), Error> {
        self.wait_until(self.clock.now() + duration, deadline)
    }

    fn wait_until(&self, until: Instant, deadline: Instant) -> Result<(), Error> {
        let now = self.clock.now();
        self.clock
            .sleep(until.min(deadline).saturating_duration_since(now));
        self.check_deadline(deadline, None)
    }

    fn wait_safe(&mut self) {
        for until in [self.programming_until.take(), self.quiet_until.take()]
            .into_iter()
            .flatten()
        {
            self.clock
                .sleep(until.saturating_duration_since(self.clock.now()));
        }
    }
}

fn errno(error: &LinuxI2CError) -> Option<i32> {
    match error {
        LinuxI2CError::Errno(code) => Some(*code),
        LinuxI2CError::Io(error) => error.raw_os_error(),
    }
}

fn is_nack(error: &LinuxI2CError) -> bool {
    matches!(errno(error), Some(libc::EREMOTEIO | libc::ENXIO))
}

fn is_busy(error: &LinuxI2CError) -> bool {
    is_nack(error) || errno(error) == Some(libc::ETIMEDOUT)
}

#[cfg(test)]
mod tests;
