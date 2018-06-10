extern crate linux_embedded_hal;
extern crate picoborgrev;

use linux_embedded_hal::I2cdev;
use std::path::Path;
use std::thread::sleep;
use std::time::Duration;

use picoborgrev::PicoBorgRev;

fn main() {
    let device = I2cdev::new(Path::new("/dev/i2c-1")).expect("Unable to create i2c device");
    let mut borg = PicoBorgRev::new(device).expect("Unable to create PicoBorgRev");
    for _ in 0..10 {
        borg.set_led(true).unwrap();
        sleep(Duration::new(1, 0));
        borg.set_led(false).unwrap();
        sleep(Duration::new(1, 0));
    }
}
