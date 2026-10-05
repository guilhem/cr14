//! Default: detect once. Reading and a reversible EEPROM exercise are explicit.
use cr14::{Cr14, Uid};
use i2cdev::linux::LinuxI2CDevice;
use std::error::Error;
use std::time::{Duration, Instant};

fn deadline() -> Instant {
    Instant::now() + Duration::from_secs(3)
}
fn uid(value: &str) -> Result<Uid, Box<dyn Error>> {
    let compact: String = value.chars().filter(|c| *c != ':').collect();
    if compact.len() != 16 || !compact.is_ascii() {
        return Err("UID must contain 16 hexadecimal digits, most significant byte first".into());
    }
    let mut bytes = [0; 8];
    for (index, byte) in bytes.iter_mut().rev().enumerate() {
        *byte = u8::from_str_radix(&compact[index * 2..index * 2 + 2], 16)?;
    }
    Ok(Uid(bytes))
}
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|arg| arg == "--help") {
        println!("detect [BUS] [--read UID BLOCK] [--write-reversible UID BLOCK --model sri512|sri4k --allow-write]\nBUS defaults to /dev/i2c-1; CR14 address is 0x50. UID uses displayed hex order.\nWrites require an independently confirmed expendable, unlocked EEPROM block.\nOriginal data is saved and restored once, even after an ambiguous test write.\nNo automatic retries; restoration can fail. Preserve the printed backup.");
        return Ok(());
    }
    let mut index = 0;
    let bus = if args.first().is_some_and(|arg| !arg.starts_with("--")) {
        index = 1;
        args[0].as_str()
    } else {
        "/dev/i2c-1"
    };
    let mut read_request = None;
    let mut write_request = None;
    let mut model = None;
    let mut allow_write = false;
    while index < args.len() {
        match args[index].as_str() {
            "--read" | "--write-reversible" => {
                let target = uid(args.get(index + 1).ok_or("missing UID")?)?;
                let block = args
                    .get(index + 2)
                    .ok_or("missing decimal block address")?
                    .parse::<u8>()?;
                if args[index] == "--read" {
                    read_request = Some((target, block));
                } else {
                    write_request = Some((target, block));
                }
                index += 3;
            }
            "--model" => {
                model = Some(args.get(index + 1).ok_or("missing tag model")?.as_str());
                index += 2;
            }
            "--allow-write" => {
                allow_write = true;
                index += 1;
            }
            _ => return Err("unknown argument; use --help".into()),
        }
    }
    // Validate before opening the bus. Never exercise OTP, counters or system bits.
    if let Some((_, block)) = write_request {
        let maximum = match model {
            Some("sri512") => 15,
            Some("sri4k") => 127,
            _ => return Err("write needs --model sri512 or sri4k".into()),
        };
        if !allow_write || !(7..=maximum).contains(&block) {
            return Err("write needs --allow-write and an expendable unlocked EEPROM address in the selected model's range".into());
        }
    } else if allow_write || model.is_some() {
        return Err("write options require --write-reversible".into());
    }
    let device = LinuxI2CDevice::new(bus, 0x50)?;
    let mut reader = Cr14::new(device);
    // All results, including a failed initialization, receive explicit cleanup.
    let operation = (|| -> Result<(), Box<dyn Error>> {
        reader.reinitialize(deadline())?;
        for found in reader.inventory(deadline())? {
            println!("UID {found}");
        }
        if let Some((target, address)) = read_request {
            let data = reader.read_block(target, address, deadline())?;
            println!("{target} block {address}: {data:02x?}");
        }
        if let Some((target, address)) = write_request {
            let original = reader.read_block(target, address, deadline())?;
            println!("BACKUP {target} block {address}: {original:02x?}");
            let mut changed = original;
            changed[0] ^= 1;
            let trial = reader.write_block(target, address, changed, deadline());
            match &trial {
                Ok(observed) => println!("Test readback: {observed:02x?}"),
                Err(error) => eprintln!("Test write: {error}"),
            }
            // A new explicit request restores the saved value. It is never a
            // retry of the test data. If cleanup failed, require reinitialization.
            if reader.state() == cr14::State::NeedsReinitialization {
                reader.reinitialize(deadline())?;
            }
            let restored = reader.write_block(target, address, original, deadline())?;
            if restored != original {
                return Err(
                    "restoration readback differs; retain BACKUP and inspect the tag".into(),
                );
            }
            println!("Original EEPROM data restored and read back.");
            match trial {
                Ok(observed) if observed == changed => {}
                Ok(_) => return Err("test readback differed; original data was restored".into()),
                Err(error) => return Err(error.into()),
            }
        }
        Ok(())
    })();
    let cleanup = reader.shutdown();
    if let Err(error) = &cleanup {
        eprintln!("Explicit shutdown failed: {error}");
    }
    operation?;
    cleanup?;
    Ok(())
}
