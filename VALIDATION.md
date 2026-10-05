# Validation evidence

Recorded on 2026-10-05. Keep compilation, scripted protocol checks and physical
observations separate.

## Local software

- Rust 2021 crate; `cargo fmt --all --check`, strict Clippy, 21 scripted tests,
  runnable example build and verified Cargo package passed.
- All targets checked with official Rust 1.99.0 for
  `arm-unknown-linux-gnueabihf` with `target-cpu=arm1176jzf-s`, and
  `aarch64-unknown-linux-gnu`. These checks compile tests and examples but do
  not execute ARM tests. A separate ARM64 example binary was linked and run.
- The small consumer in `integration/dual-reader` imports both concrete
  `LinuxI2CDevice` constructors. It opens no bus. Native execution and both ARM
  compilation checks passed using the two local implementations.
- Independent read-only review and post-fix confirmation returned `ship` for
  CR14 `f134e898` and ST25 `a6086572`, with no remaining findings. The reviewer
  reran all 49 tests, formatting, strict Clippy and example builds.

## Observed CR14 hardware

Host `nabaztag`, Raspberry Pi Zero 2 W, Linux `6.18.50+rpt-rpi-v8`, aarch64,
`bcm2835 (i2c@7e804000)` adapter, `/dev/i2c-1`, address `0x50`.
The existing `nab-hardware` service was stopped and the kernel CR14 driver
unbound only during each test. The driver was rebound and the service's active
state verified after every attempt, including the initial failure.

The first inventory failed with Linux `EIO`. Isolated transactions established:
valid parameter reads, expected `EREMOTEIO` on invalid register `7f`, accepted
RF activation and INITIATE writes, `EIO` on the immediate answer read, then a
successful zero-length answer after 4 ms. RF-off register readback succeeded.
After the answer-polling correction, the actual example initialized, inventoried
an empty field, verified shutdown and exited successfully.

No tag was present in that observation. Presence/removal, UID and memory reads,
multiple physical tags, EEPROM write/readback/restoration, programming under
faults, and analog field/timing measurements remain pending an identified test
tag and equipment. No tag memory was written by these tests.
