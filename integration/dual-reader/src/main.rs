use cr14::Cr14;
use i2cdev::linux::LinuxI2CDevice;
use st25r391x::St25r391x;

fn main() {
    // Check the concrete Linux constructors together without opening a bus.
    let _cr14: fn(LinuxI2CDevice) -> Cr14<LinuxI2CDevice> = Cr14::new;
    let _st25: fn(LinuxI2CDevice) -> St25r391x<LinuxI2CDevice> = St25r391x::new;
    println!("Both MIT reader crates import with the same Linux I2C device type.");
}
