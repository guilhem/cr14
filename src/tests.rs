use super::*;
use std::collections::VecDeque;
use std::{cell::RefCell, rc::Rc};

// This seam implements i2cdev's own traits. Every script asserts the exact
// helper, bytes, read length and message order; an unexpected access panics.
enum Message<'a> {
    Read(&'a mut [u8]),
    Write(&'a [u8]),
}
impl<'a> I2CMessage<'a> for Message<'a> {
    fn read(data: &'a mut [u8]) -> Self {
        Self::Read(data)
    }
    fn write(data: &'a [u8]) -> Self {
        Self::Write(data)
    }
}

#[derive(Debug)]
enum Access {
    Write(Vec<u8>),
    I2cBlock(u8, Vec<u8>),
    Read(u8, usize),
}
#[derive(Debug)]
struct Step {
    access: Access,
    reply: Vec<u8>,
    error: Option<i32>,
    messages: u32,
    delay: Duration,
}
fn write(bytes: &[u8]) -> Step {
    step(Access::Write(bytes.to_vec()), &[])
}
fn frame(bytes: &[u8]) -> Step {
    step(Access::I2cBlock(1, bytes.to_vec()), &[])
}
fn read(reg: u8, len: usize, bytes: &[u8]) -> Step {
    step(Access::Read(reg, len), bytes)
}
fn step(access: Access, reply: &[u8]) -> Step {
    Step {
        access,
        reply: reply.to_vec(),
        error: None,
        messages: 2,
        delay: Duration::ZERO,
    }
}
impl Step {
    fn fails(mut self, code: i32) -> Self {
        self.error = Some(code);
        self
    }
    fn delay(mut self, delay: Duration) -> Self {
        self.delay = delay;
        self
    }
}
struct Script {
    steps: VecDeque<Step>,
    seen: Vec<(Instant, Access)>,
    clock: Clock,
}
#[derive(Clone)]
struct Bus(Rc<RefCell<Script>>);
impl Bus {
    fn access(&self, access: Access) -> Result<Step, LinuxI2CError> {
        let mut script = self.0.borrow_mut();
        let expected = script.steps.pop_front().expect("unscripted I2C operation");
        let matches = match (&access, &expected.access) {
            (Access::Write(a), Access::Write(b)) => a == b,
            (Access::I2cBlock(ar, a), Access::I2cBlock(br, b)) => ar == br && a == b,
            (Access::Read(ar, a), Access::Read(br, b)) => ar == br && a == b,
            _ => false,
        };
        assert!(matches, "actual {access:?}, expected {:?}", expected.access);
        let now = script.clock.now();
        script.seen.push((now, access));
        script.clock.sleep(expected.delay);
        match expected.error {
            Some(code) => Err(LinuxI2CError::Errno(code)),
            None => Ok(expected),
        }
    }
    fn done(&self) {
        assert!(self.0.borrow().steps.is_empty(), "unconsumed script");
    }
    fn extend(&self, steps: impl IntoIterator<Item = Step>) {
        self.0.borrow_mut().steps.extend(steps);
    }
}
impl I2CDevice for Bus {
    type Error = LinuxI2CError;
    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.access(Access::Write(bytes.to_vec())).map(|_| ())
    }
    fn read(&mut self, _: &mut [u8]) -> Result<(), Self::Error> {
        panic!("unaddressed read")
    }
    fn smbus_write_quick(&mut self, _: bool) -> Result<(), Self::Error> {
        panic!("quick")
    }
    fn smbus_read_block_data(&mut self, _: u8) -> Result<Vec<u8>, Self::Error> {
        panic!("SMBus count read")
    }
    fn smbus_read_i2c_block_data(&mut self, _: u8, _: u8) -> Result<Vec<u8>, Self::Error> {
        panic!("use I2CTransfer for checked length")
    }
    fn smbus_write_block_data(&mut self, _: u8, _: &[u8]) -> Result<(), Self::Error> {
        panic!("SMBus count write")
    }
    fn smbus_write_i2c_block_data(
        &mut self,
        register: u8,
        values: &[u8],
    ) -> Result<(), Self::Error> {
        self.access(Access::I2cBlock(register, values.to_vec()))
            .map(|_| ())
    }
    fn smbus_process_block(&mut self, _: u8, _: &[u8]) -> Result<Vec<u8>, Self::Error> {
        panic!("process block")
    }
}
impl<'a> I2CTransfer<'a> for Bus {
    type Error = LinuxI2CError;
    type Message = Message<'a>;
    fn transfer(&mut self, messages: &mut [Self::Message]) -> Result<u32, Self::Error> {
        match messages {
            [Message::Write(pointer), Message::Read(data)] => {
                assert_eq!(pointer.len(), 1, "one pointer byte before repeated START");
                let result = self.access(Access::Read(pointer[0], data.len()))?;
                assert!(result.reply.len() <= data.len());
                data.fill(0);
                data[..result.reply.len()].copy_from_slice(&result.reply);
                Ok(result.messages)
            }
            _ => panic!("expected pointer-write / repeated-START / read / STOP"),
        }
    }
}

const UID: Uid = Uid([1, 2, 3, 4, 5, 6, 2, 0xd0]);
fn initialization() -> Vec<Step> {
    vec![
        read(0, 1, &[0]),
        read(0x7f, 1, &[]).fails(libc::ENXIO),
        write(&[0, 0]),
        read(0, 1, &[0]),
    ]
}
fn on() -> Vec<Step> {
    vec![write(&[0, 0x10]), read(0, 1, &[0x10])]
}
fn off() -> Vec<Step> {
    vec![write(&[0, 0]), read(0, 1, &[0])]
}
fn exchange(request: &[u8], reply: &[u8]) -> Vec<Step> {
    let mut payload = vec![request.len() as u8];
    payload.extend_from_slice(request);
    let mut response = vec![reply.len() as u8];
    response.extend_from_slice(reply);
    vec![frame(&payload), read(1, response.len(), &response)]
}
fn finish() -> Vec<Step> {
    exchange(&[0x0f], &[])
}
fn identify(id: u8, uid: Uid) -> Vec<Step> {
    let mut steps = exchange(&[0x0e, id], &[id]);
    steps.extend(exchange(&[0x0b], &uid.0));
    steps
}
fn target(uid: Uid) -> Vec<Step> {
    let mut steps = on();
    steps.extend(exchange(&[6, 0], &[0x42]));
    steps.extend(identify(0x42, uid));
    steps
}
fn slots(mask: u16, ids: [u8; 16]) -> Vec<Step> {
    let mut response = vec![18];
    response.extend_from_slice(&mask.to_le_bytes());
    response.extend_from_slice(&ids);
    vec![write(&[3]), read(1, 19, &response)]
}
fn reader(steps: Vec<Step>) -> (Cr14<Bus>, Bus) {
    let clock = Clock::default();
    let bus = Bus(Rc::new(RefCell::new(Script {
        steps: initialization().into_iter().chain(steps).collect(),
        seen: Vec::new(),
        clock: clock.clone(),
    })));
    let mut reader = Cr14::new(bus.clone());
    reader.clock = clock;
    reader.reinitialize(deadline(&reader)).unwrap();
    (reader, bus)
}
fn deadline(reader: &Cr14<Bus>) -> Instant {
    reader.clock.now() + Duration::from_secs(1)
}
fn has_errno(error: &Error, code: i32) -> bool {
    matches!(error, Error::I2c(e) | Error::Deadline {last_i2c_error: Some(e)} if errno(e)==Some(code))
}

#[test]
fn probe_and_empty_inventory_use_exact_stop_separated_transactions() {
    let mut script = on();
    script.extend([frame(&[2, 6, 0]), read(1, 2, &[0])]);
    script.extend(off());
    let (mut reader, bus) = reader(script);
    assert!(reader.inventory(deadline(&reader)).unwrap().is_empty());
    assert_eq!(reader.state(), State::Ready);
    bus.done();
    let script = bus.0.borrow();
    assert!(script.seen[4].0.duration_since(script.seen[3].0) >= POWER_ON);
    assert!(script.seen[6].0.duration_since(script.seen[4].0) >= POWER_ON + TAG_POWER_ON);
}

#[test]
fn single_inventory_completes_tag_and_preserves_uid_rf_order() {
    let mut script = target(UID);
    script.extend(finish());
    script.extend(off());
    let (mut reader, bus) = reader(script);
    assert_eq!(reader.inventory(deadline(&reader)).unwrap(), vec![UID]);
    assert_eq!(UID.to_string(), "d0:02:06:05:04:03:02:01");
    bus.done();
}

#[test]
fn sixteen_slots_resolve_collision_across_rounds_and_keep_ids_after_frame_overwrite() {
    let second = Uid([8; 8]);
    let third = Uid([9; 8]);
    let mut script = on();
    script.extend([frame(&[2, 6, 0]), read(1, 2, &[0xff])]);
    let mut ids = [0; 16];
    ids[0] = 0;
    ids[15] = 0xff;
    ids[1] = 0xff;
    // Valid IDs 00 and FF must not be confused with clear-mask error sentinels.
    script.extend(slots(0x8001, ids));
    script.extend(identify(0, UID));
    script.extend(finish());
    script.extend(identify(0xff, second));
    script.extend(finish());
    let mut ids = [0; 16];
    ids[8] = 0x58;
    script.extend(slots(1 << 8, ids));
    script.extend(identify(0x58, third));
    script.extend(finish());
    script.extend(off());
    let (mut reader, bus) = reader(script);
    assert_eq!(
        reader.inventory(deadline(&reader)).unwrap(),
        vec![UID, second, third]
    );
    bus.done();
}

#[test]
fn colliding_select_or_uid_restores_inventory_before_new_round() {
    for during_uid in [false, true] {
        let mut script = on();
        script.extend(exchange(&[6, 0], &[0x42]));
        if during_uid {
            script.extend(exchange(&[0x0e, 0x42], &[0x42]));
        }
        script.push(frame(if during_uid {
            &[1, 0x0b]
        } else {
            &[2, 0x0e, 0x42]
        }));
        script.push(read(1, if during_uid { 9 } else { 2 }, &[0xff]));
        script.extend(exchange(&[0x0c], &[]));
        let mut ids = [0; 16];
        ids[2] = 0x22;
        script.extend(slots(4, ids));
        script.extend(identify(0x22, UID));
        script.extend(finish());
        script.extend(off());
        let (mut reader, bus) = reader(script);
        assert_eq!(reader.inventory(deadline(&reader)).unwrap(), vec![UID]);
        bus.done();
    }
}

#[test]
fn multiple_reads_select_and_verify_uid_then_keep_address_order() {
    let mut script = target(UID);
    script.extend(exchange(&[8, 7], &[1, 2, 3, 4]));
    script.extend(exchange(&[8, 255], &[5, 6, 7, 8]));
    script.extend(finish());
    script.extend(off());
    let (mut reader, bus) = reader(script);
    assert_eq!(
        reader
            .read_blocks(UID, &[7, 255], deadline(&reader))
            .unwrap(),
        vec![[1, 2, 3, 4], [5, 6, 7, 8]]
    );
    bus.done();
    let times = &bus.0.borrow().seen;
    for pair in times.windows(2) {
        if let (Access::Read(1, _), Access::I2cBlock(1, _)) = (&pair[0].1, &pair[1].1) {
            assert!(pair[1].0.duration_since(pair[0].0) >= INTER_REQUEST);
        }
    }
}

#[test]
fn wrong_uid_never_reaches_memory_read_or_write() {
    for writing in [false, true] {
        let mut script = target(Uid([0; 8]));
        script.extend(finish());
        script.extend(off());
        let (mut reader, bus) = reader(script);
        let error = if writing {
            reader
                .write_block(UID, 7, [0; 4], deadline(&reader))
                .unwrap_err()
                .error
        } else {
            reader.read_block(UID, 7, deadline(&reader)).unwrap_err()
        };
        assert!(matches!(error,Error::TagNotFound(uid) if uid==UID));
        bus.done();
    }
}

#[test]
fn crc_invalid_and_absent_memory_answers_are_typed_and_cleaned_up() {
    for (length, kind) in [(0xff, 0), (3, 1), (0, 2)] {
        let mut script = target(UID);
        script.extend([frame(&[2, 8, 7]), read(1, 5, &[length])]);
        script.extend(off());
        let (mut reader, bus) = reader(script);
        let error = reader.read_block(UID, 7, deadline(&reader)).unwrap_err();
        match kind {
            0 => assert!(matches!(error, Error::Crc)),
            1 => assert!(matches!(
                error,
                Error::InvalidFrame {
                    expected: 4,
                    actual: 3
                }
            )),
            _ => assert!(matches!(error, Error::NoResponse)),
        }
        assert_eq!(reader.state(), State::Ready);
        bus.done();
    }
}

#[test]
fn invalid_slot_table_and_select_echo_are_rejected() {
    let mut script = on();
    script.extend([frame(&[2, 6, 0]), read(1, 2, &[0xff])]);
    let mut ids = [0; 16];
    ids[9] = 0x12;
    script.extend(slots(0, ids));
    script.extend(off());
    let (mut r, bus) = reader(script);
    assert!(matches!(
        r.inventory(deadline(&r)),
        Err(Error::InvalidSlot {
            slot: 9,
            value: 0x12
        })
    ));
    bus.done();
    let mut script = on();
    script.extend(exchange(&[6, 0], &[0x42]));
    script.extend([frame(&[2, 0x0e, 0x42]), read(1, 2, &[1, 0x43])]);
    script.extend(off());
    let (mut r, bus) = reader(script);
    assert!(matches!(
        r.inventory(deadline(&r)),
        Err(Error::SelectMismatch {
            expected: 0x42,
            actual: 0x43
        })
    ));
    bus.done();
}

#[test]
fn busy_reads_retry_only_reads_then_succeed() {
    for code in [libc::EREMOTEIO, libc::ETIMEDOUT, libc::ENXIO, libc::EIO] {
        let mut script = on();
        script.extend([
            frame(&[2, 6, 0]),
            read(1, 2, &[]).fails(code),
            read(1, 2, &[0]),
        ]);
        script.extend(off());
        let (mut r, bus) = reader(script);
        assert!(r.inventory(deadline(&r)).unwrap().is_empty());
        bus.done();
    }
}

#[test]
fn ack_expiration_preserves_last_linux_errno_even_if_ioctl_crosses_deadline() {
    for last_errno in [libc::ETIMEDOUT, libc::EIO] {
        let mut script = on();
        script.extend([
            frame(&[2, 6, 0]),
            read(1, 2, &[]).fails(libc::EREMOTEIO),
            read(1, 2, &[])
                .fails(last_errno)
                .delay(Duration::from_secs(2)),
        ]);
        script.extend(off());
        let (mut r, bus) = reader(script);
        let error = r.inventory(deadline(&r)).unwrap_err();
        assert!(has_errno(&error, last_errno));
        assert!(matches!(error, Error::Deadline { .. }));
        bus.done();
    }
}

#[test]
fn nonbusy_read_errors_and_partial_transfers_do_not_retry() {
    for partial in [false, true] {
        let mut script = on();
        script.push(frame(&[2, 6, 0]));
        let mut failed = read(1, 2, &[]);
        if partial {
            failed.messages = 1;
        } else {
            failed.error = Some(libc::EBADF);
        }
        script.push(failed);
        script.extend(off());
        let (mut r, bus) = reader(script);
        let error = r.inventory(deadline(&r)).unwrap_err();
        if partial {
            assert!(matches!(
                error,
                Error::PartialTransfer {
                    expected: 2,
                    actual: 1
                }
            ));
        } else {
            assert!(has_errno(&error, libc::EBADF));
        }
        bus.done();
    }
}

#[test]
fn expired_deadline_and_invalid_arguments_produce_no_bus_traffic() {
    let (mut r, bus) = reader(vec![]);
    assert!(matches!(
        r.inventory(r.clock.now()),
        Err(Error::Deadline { .. })
    ));
    assert!(matches!(
        r.read_blocks(UID, &[], deadline(&r)),
        Err(Error::InvalidArgument(_))
    ));
    assert!(matches!(
        r.write_blocks(UID, &[], deadline(&r)).unwrap_err().error,
        Error::InvalidArgument(_)
    ));
    assert_eq!(r.state(), State::Ready);
    bus.done();
}

#[test]
fn deadline_during_warmup_still_verifies_rf_off() {
    let mut script = on();
    script.extend(off());
    let (mut r, bus) = reader(script);
    let deadline = r.clock.now() + POWER_ON + Duration::from_millis(1);
    assert!(matches!(r.inventory(deadline), Err(Error::Deadline { .. })));
    bus.done();
}

#[test]
fn sequential_writes_return_observed_data_after_full_programming_windows() {
    let mut script = target(UID);
    script.push(frame(&[6, 9, 7, 1, 2, 3, 4]));
    script.extend(exchange(&[8, 7], &[1, 2, 3, 4]));
    script.push(frame(&[6, 9, 8, 5, 6, 7, 8]));
    script.extend(exchange(&[8, 8], &[0, 6, 7, 8]));
    script.extend(finish());
    script.extend(off());
    let (mut r, bus) = reader(script);
    let p = r
        .write_blocks(UID, &[(7, [1, 2, 3, 4]), (8, [5, 6, 7, 8])], deadline(&r))
        .unwrap();
    assert_eq!(p.written, vec![7, 8]);
    assert_eq!(p.readback, vec![(7, [1, 2, 3, 4]), (8, [0, 6, 7, 8])]);
    assert!(p.possibly_changed.is_none());
    bus.done();
    let times = &bus.0.borrow().seen;
    for pair in times.windows(2) {
        if matches!(&pair[0].1,Access::I2cBlock(1,payload) if payload[1]==9) {
            assert!(pair[1].0.duration_since(pair[0].0) >= SRI_PROGRAMMING + WRITE_TRANSMISSION);
            assert!(
                matches!(&pair[1].1,Access::I2cBlock(1,payload) if payload[1]==8),
                "Write_block has no answer read"
            );
        }
    }
}

#[test]
fn ambiguous_second_write_is_not_replayed_and_reports_partial_progress() {
    let mut script = target(UID);
    script.push(frame(&[6, 9, 7, 1, 2, 3, 4]));
    script.extend(exchange(&[8, 7], &[1, 2, 3, 4]));
    script.push(frame(&[6, 9, 8, 5, 6, 7, 8]).fails(libc::ETIMEDOUT));
    script.extend(off());
    let (mut r, bus) = reader(script);
    let failure = r
        .write_blocks(
            UID,
            &[(7, [1, 2, 3, 4]), (8, [5, 6, 7, 8]), (9, [0; 4])],
            deadline(&r),
        )
        .unwrap_err();
    assert_eq!(failure.progress.written, vec![7]);
    assert_eq!(failure.progress.readback, vec![(7, [1, 2, 3, 4])]);
    assert_eq!(failure.progress.possibly_changed, Some(8));
    assert!(has_errno(&failure.error, libc::ETIMEDOUT));
    bus.done();
    let times = &bus.0.borrow().seen;
    let write_index = times
        .iter()
        .rposition(|(_, access)| matches!(access,Access::I2cBlock(1,p) if p[1]==9))
        .unwrap();
    assert!(
        times[write_index + 1]
            .0
            .duration_since(times[write_index].0)
            >= SRI_PROGRAMMING + WRITE_TRANSMISSION
    );
}

#[test]
fn write_deadline_expiration_keeps_rf_power_until_programming_is_safe() {
    let mut script = target(UID);
    script.push(frame(&[6, 9, 7, 0, 0, 0, 0]).delay(Duration::from_secs(2)));
    script.extend(off());
    let (mut r, bus) = reader(script);
    let deadline = deadline(&r);
    let failure = r.write_block(UID, 7, [0; 4], deadline).unwrap_err();
    assert_eq!(failure.progress.written, vec![7]);
    assert_eq!(failure.progress.possibly_changed, Some(7));
    assert!(matches!(failure.error, Error::Deadline { .. }));
    bus.done();
    let times = &bus.0.borrow().seen;
    let write_index = times
        .iter()
        .position(|(_, access)| matches!(access,Access::I2cBlock(1,p) if p[1]==9))
        .unwrap();
    assert!(times[write_index + 1].0 > deadline);
    assert!(
        times[write_index + 1]
            .0
            .duration_since(times[write_index].0)
            >= Duration::from_secs(2) + SRI_PROGRAMMING + WRITE_TRANSMISSION
    );
}

#[test]
fn failed_write_readback_preserves_sent_and_uncertain_block() {
    let mut script = target(UID);
    script.push(frame(&[6, 9, 7, 0, 0, 0, 0]));
    script.extend([frame(&[2, 8, 7]), read(1, 5, &[0xff])]);
    script.extend(off());
    let (mut r, bus) = reader(script);
    let failure = r.write_block(UID, 7, [0; 4], deadline(&r)).unwrap_err();
    assert_eq!(failure.progress.written, vec![7]);
    assert!(failure.progress.readback.is_empty());
    assert_eq!(failure.progress.possibly_changed, Some(7));
    assert!(matches!(failure.error, Error::Crc));
    bus.done();
}

#[test]
fn cleanup_failure_retains_primary_errno_and_blocks_reuse_until_reinitialize() {
    let mut script = on();
    script.extend([
        frame(&[2, 6, 0]).fails(libc::EIO),
        write(&[0, 0]).fails(libc::EREMOTEIO),
    ]);
    let (mut r, bus) = reader(script);
    let error = r.inventory(deadline(&r)).unwrap_err();
    assert!(
        matches!(error,Error::Cleanup{operation:Some(ref op),ref cleanup} if has_errno(op,libc::EIO) && has_errno(cleanup,libc::EREMOTEIO))
    );
    assert_eq!(r.state(), State::NeedsReinitialization);
    assert!(matches!(
        r.inventory(deadline(&r)),
        Err(Error::NeedsReinitialization)
    ));
    bus.extend(off());
    r.shutdown().unwrap();
    assert_eq!(r.state(), State::NeedsReinitialization);
    bus.extend(initialization());
    r.reinitialize(deadline(&r)).unwrap();
    assert_eq!(r.state(), State::Ready);
    bus.done();
}

#[test]
fn parameter_readback_failure_is_not_a_successful_cleanup() {
    let mut script = on();
    script.extend([
        frame(&[2, 6, 0]),
        read(1, 2, &[0]),
        write(&[0, 0]),
        read(0, 1, &[0x10]),
    ]);
    let (mut r, bus) = reader(script);
    let error = r.inventory(deadline(&r)).unwrap_err();
    assert!(
        matches!(error,Error::Cleanup{operation:None,cleanup} if matches!(*cleanup,Error::RegisterMismatch{expected:0,actual:0x10}))
    );
    assert_eq!(r.state(), State::NeedsReinitialization);
    bus.done();
}

#[test]
fn model_specific_programming_time_must_not_shorten_sri_safe_window() {
    let (r, bus) = reader(vec![]);
    assert!(Cr14::with_programming_time(bus.clone(), Duration::from_millis(6)).is_err());
    let longer = Cr14::with_programming_time(bus.clone(), Duration::from_millis(10)).unwrap();
    assert_eq!(longer.programming_time, Duration::from_millis(10));
    bus.done();
    drop(r);
}

#[test]
fn unknown_device_and_probe_deadline_cannot_be_used() {
    let clock = Clock::default();
    let bus = Bus(Rc::new(RefCell::new(Script {
        steps: vec![read(0, 1, &[0]), read(0x7f, 1, &[0])].into(),
        seen: vec![],
        clock: clock.clone(),
    })));
    let mut r = Cr14::new(bus.clone());
    r.clock = clock;
    assert!(matches!(
        r.inventory(deadline(&r)),
        Err(Error::NeedsReinitialization)
    ));
    assert!(matches!(
        r.reinitialize(deadline(&r)),
        Err(Error::UnexpectedDevice)
    ));
    assert_eq!(r.state(), State::NeedsReinitialization);
    bus.done();
    assert!(matches!(
        r.reinitialize(r.clock.now()),
        Err(Error::Deadline { .. })
    ));
    bus.done();
}
