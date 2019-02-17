#![allow(dead_code)]

extern crate robot_traits;

use self::robot_traits::{Led, Robot};
use hal::blocking::i2c::{Write, WriteRead};

use PicoBorgRev;

impl<T: Write + WriteRead> Robot for PicoBorgRev<T> {
    fn forward(&mut self, speed: f32) {
        let _ = self.set_motors(speed);
    }

    fn turn(&mut self, speed: f32) {
        let _ = self.set_motor_1(speed);
        let _ = self.set_motor_2(-speed);
    }

    fn stop(&mut self) {
        let _ = self.set_motors(0.0);
    }
}

impl<T: Write + WriteRead> Led for PicoBorgRev<T> {
    fn led_on(&mut self) {
        let _ = self.set_led(true);
    }

    fn led_off(&mut self) {
        let _ = self.set_led(false);
    }
}
