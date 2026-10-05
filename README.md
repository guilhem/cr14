# cr14

Synchronous Linux I2C access to a CR14 reader and ST SRI512/SRI4K tags. This independent MIT implementation uses `i2cdev = 0.6.2` and takes an already opened device. It provides discovery with 16-slot anticollision, UID-verified selection, four-byte block reads, and sequential writes with readback. It does not require `/dev/rfid0`.

Import by the tested immutable Git revision:

```toml
[dependencies]
cr14 = { git = "https://github.com/guilhem/cr14", rev = "f134e8980620a9e4aff4c00ae1090fbed3d18063" }
i2cdev = "=0.6.2"
```

```rust,no_run
use cr14::Cr14;
use i2cdev::linux::LinuxI2CDevice;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut reader = Cr14::new(LinuxI2CDevice::new("/dev/i2c-1", 0x50)?);
    let deadline = || Instant::now() + Duration::from_secs(3);
    let operation = (|| {
        reader.reinitialize(deadline())?;
        for uid in reader.inventory(deadline())? {
            println!("{uid}");
            println!("{:02x?}", reader.read_block(uid, 7, deadline())?);
        }
        Ok::<_, cr14::Error>(())
    })();
    let shutdown = reader.shutdown();
    operation?;
    shutdown?;
    Ok(())
}
```

The address is seven-bit `0x50` when E0/E1/E2 are low; configure another address in `LinuxI2CDevice` if the board straps differ. Use one owner of the reader. The kernel CR14 driver and another process must not operate it concurrently. Adapter permissions and any service stop/driver unbind are managed by the caller.

`new` has no bus effects. `reinitialize(Instant)` probes documented register behavior and confirms RF off; the chip has no identification register. All public discovery/memory operations take `&mut self` and their own absolute deadline, turn RF on, perform work, then verify RF off. `shutdown()` returns its own cleanup result. A failed cleanup puts the reader in `NeedsReinitialization`; successful shutdown alone does not clear this condition. A verified `reinitialize` is required before further tag access. There is no implicit `Drop` I/O; normal operation paths clean up explicitly, and applications must arrange shutdown on panic/abnormal exit. Process death, loss of power and tag removal cannot be repaired by software guarantees.

`Uid.0` contains eight bytes in RF order, least significant byte first. Display prints most significant byte first with colons. A `Block` contains four bytes in RF order; the caller chooses numeric interpretation. `read_blocks` and `write_blocks` accept arbitrary addresses in order, including duplicate addresses. Every memory operation first inventories candidates, sends Select, checks its Chip_ID echo, and reads all eight UID bytes to verify the target. No memory command is sent to a mismatched UID.

Writes are non-atomic and each Write_block request is submitted once. A failed ioctl may have launched programming: the request is never replayed. `WriteError` retains `WriteProgress`, with `written` (I2C submissions accepted), `readback` (address and observed data), and `possibly_changed` (the currently ambiguous block). Earlier observed blocks remain in the report even if a later write fails. Readback is observed data, not a promise of equality: OTP/counter operations can transform the supplied bytes, and locked or unsupported addresses can leave data unchanged. Callers must check the observation against the intended model semantics. Even successful I2C submission gives no tag acknowledgement; Write_block has no RF response. `write_block` returns its observed four bytes and an error carries the same progress information as `write_blocks`.

SRI512 blocks 0–4 are resettable OTP, 5–6 are decrement-only counters, 7–15 are lockable EEPROM, and 255 contains system/lock bits. SRI4K uses EEPROM 7–127. The library exposes these address bytes without adding application policy. Do not write a tag's system, lock, counter or OTP areas unless its model-specific behavior is intended. Other ISO14443-B protocols, ST25TB compatibility, generic RF passthrough, NDEF, events, background polling, cancellation and D-Bus are outside this crate's supported API. ST25TB has not been qualified on CR14 by this implementation.

The transport uses the existing `I2CDevice` and `I2CTransfer` traits with `LinuxI2CError`, so scripted buses need no separate adapter API. RF requests use SMBus **I2C block** writes: `[register=01, RF-length, RF-bytes...]`, without an additional SMBus Count byte. The write's STOP starts RF transmission. Answer reads are separate ioctls containing a register-pointer write, repeated START, fixed-length read and STOP; two transferred messages are required. CR14 adds/checks the RF CRC. Its length byte distinguishes no answer (`00`), CRC failure (`FF`) and a decoded frame. Slot reads comprise 19 bytes: length 18, two status bytes in little-endian slot order and sixteen Chip_ID/error bytes. Valid-mask bits allow Chip_ID values `00` and `FF`.

ACK polling retries **answer reads only** for Linux `EREMOTEIO`, `ENXIO`, `ETIMEDOUT` or `EIO`. The bcm2835 adapter on the qualification Pi returns `EIO` during an RF-busy answer read and succeeds once the exchange finishes. These can also mean a missing/disconnected device or an adapter failure, so they are a bounded polling classification, not proof of chip activity. Other errors return immediately. Deadline errors retain the last Linux error from that polling sequence. Mutating requests, including selection and RF register writes, are never retried by the library. An adapter/kernel may internally retry transactions; configure/qualify that behavior separately.

`Instant` deadlines bound waits and new submissions controlled by Rust. They cannot interrupt an executing Linux ioctl; the adapter timeout controls that call. Required safety waits and explicit cleanup can run beyond the deadline. The library allows 20 ms reader power-on, another 5 ms tag warmup, 200 µs between a completed RF answer and a new request, and a 20 ms RF-off interval between sessions. Ambiguous ordinary and slot submissions retain conservative 4 ms and 32 ms RF guards. Programming holds RF for **2 ms transmission budget plus at least 7 ms programming after the write ioctl returns**, even if the call failed or the deadline expired. The SRI specs list maximum programming times 3/5/7 ms for OTP/EEPROM/counters. `with_programming_time(device, duration)` allows a longer confirmed model-specific bound, never a shorter one. An alternate bound does not establish support for a different protocol/model. Budget derivations and evidence are in [PROVENANCE.md](PROVENANCE.md).

Build and run the example on Linux (native ARM64 is supported):

```sh
cargo run --example detect -- /dev/i2c-1
cargo run --example detect -- /dev/i2c-1 --read d0:02:00:00:00:00:00:01 7
```

The default detects once and performs no tag writes. The following **opt-in** exercise requires an expendable tag whose identity, model and unlocked EEPROM block have been confirmed independently. No safe block has been chosen for any physical tag in this repository. Substitute its actual UID and address; the example rejects addresses outside the selected model's EEPROM range:

```sh
cargo run --example detect -- /dev/i2c-1 \
  --write-reversible d0:02:00:00:00:00:00:01 7 --model sri512 --allow-write
```

The example saves/prints the original block, toggles one bit, checks the observed value, submits one explicit restoration of the original bytes, and checks restoration. Restoration is a new request, including after an ambiguous test write; neither value is automatically retried. An interruption, lost tag or restoration error can leave changed data. Preserve the printed backup. For a locked block, a differing test readback causes a reported failure even if restoration reads the original data.

Qualification checklist (keep simulated and physical results separate):

- Scripted tests verify exact framing, STOP-separated calls, slot status/ID handling, collision rounds, UID checks, CRC/invalid/no answers, errno polling, message counts, controlled deadlines, write progression, programming guards, and cleanup/reinitialization failures.
- Local validation: `cargo fmt --check`, `cargo clippy --all-targets -- -D warnings`, `cargo test`, `cargo build --examples`, `cargo package`. CI also checks library/examples for `arm-unknown-linux-gnueabihf` (ARMv6 hard-float) and `aarch64-unknown-linux-gnu`. Compile checks do not establish device behavior.
- On hardware, record board/antenna, chip and tag variants, adapter capabilities (SMBus I2C block writes and I2C_RDWR), bus speed, errno behavior and adapter timeout/retry settings. Observe request STOP and RF intervals with a logic analyser/oscilloscope.
- Check empty field, one tag, multiple tags, same-slot/Chip_ID collisions, tag removal, read-only UID/memory agreement with the existing reader, and verified RF off on both success and timeout.
- Only after selecting an expendable unlocked EEPROM block: backup, opt-in write/readback/restore, then removal/power-cycle verification. Qualify late/failed ioctls, cleanup failure and adapter recovery with fault injection. No physical write test is included in the simulated evidence.

This repository includes no NabOS integration. See [PROVENANCE.md](PROVENANCE.md) for independent implementation provenance and exact specifications; see [LICENSE](LICENSE) for the MIT terms.

Recorded software and hardware observations are in [VALIDATION.md](VALIDATION.md).
The small consumer in `integration/dual-reader` imports this checkout and a fixed
Git revision of `st25r391x`. Run `cargo run --locked --manifest-path
integration/dual-reader/Cargo.toml`; it opens no bus. The first delivery uses
`codex/i2c-userspace`, whose root commit has no parent. Future Rust pull requests
use this new history as their base; the older GPL branch remains separate.
