extern crate picoborgrev;
extern crate linux_embedded_hal;

use std::path::Path;
use std::thread::sleep;
use std::time::Duration;
use linux_embedded_hal::I2cdev;

use picoborgrev::PicoBorgRev;

fn main() {
    let device = I2cdev::new(Path::new("/dev/i2c-1")).expect("Unable to create i2c device");
    let mut borg = PicoBorgRev::new(device).expect("Unable to create PicoBorgRev");
    for i in 0..10 {
        borg.set_led(true).unwrap();
        sleep(Duration::new(1, 0));
        borg.set_led(false).unwrap();
        sleep(Duration::new(1, 0));
    }
}