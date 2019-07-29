#![allow(dead_code)]

//! This module is designed to communicate with the PicoBorg Reverse via Rust
//! and the [embedded-hal](https://crates.io/crates/embedded-hal) traits.
//!
//! See the PiBorg website at www.piborg.org/picoborgreverse for more details
//! on the PicoBorgRev board.
//!
//! **Important:** This is still a work in progress and the API should not be considered stable until the
//! `1.0` release.
//!
//! # Usage
//!
//! The first step is to add `picoborgrev` to your `cargo.toml` file:
//!
//! ```ignore
//! [dependencies]
//! picoborgrev = "0.1"
//! ```
//!
//! Then in your module you then need to import the crate:
//!
//! ```ignore
//! extern crate picoborgrev;
//!
//! use picoborgrev::PicoBorgRev;
//! ```
//!
//! To create a new `PicoBorgRev` controller you will need to supply an `embedded-hal` implementation
//! such as `linux-embedded-hal`:
//!
//! ```ignore
//! extern crate linux_embedded_hal;
//!
//! use linux_embedded_hal::I2cdev;
//! use std::path::Path;
//!
//! let device = I2cdev::new(Path::new("/dev/i2c-1")).expect("Unable to create i2c device");
//! ```
//!
//! Finally create a new `PicoBorgRev` supplying the `I2C` implementation:
//!
//! ```ignore
//! let mut borg = PicoBorgRev::init(device).expect("Unable to create PicoBorgRev");
//! borg.set_led(true).unwrap();
//! ```

extern crate embedded_hal as hal;

use hal::blocking::i2c::{Write, WriteRead};
use std::thread::sleep;
use std::time::{Duration, Instant};

#[cfg(feature = "traits")]
pub mod traits;

const I2C_ADDRESS: u8 = 0x44;

// Constant values
const PWM_MAX: f32 = 255.0;
const I2C_MAX_LEN: u8 = 4;

const I2C_ID_PICOBORG_REV: u8 = 0x15;

enum Command {
    SetLed = 1,         // Set the Led status
    GetLed = 2,         // Get the Led status
    SetAFwd = 3,        // Set motor 2 Pwm rate in a forwards direction
    SetARev = 4,        // Set motor 2 Pwm rate in a reverse direction
    GetA = 5,           // Get motor 2 direction and Pwm rate
    SetBFwd = 6,        // Set motor 1 Pwm rate in a forwards direction
    SetBRev = 7,        // Set motor 1 Pwm rate in a reverse direction
    GetB = 8,           // Get motor 1 direction and Pwm rate
    AllOff = 9,         // Switch everything off
    ResetEpo = 10, // Resets the Epo flag, use after Epo has been tripped and switch is now clear
    GetEpo = 11,   // Get the Epo latched flag
    SetEpoIgnore = 12, // Set the Epo ignored flag, allows the system to run without an Epo
    GetEpoIgnore = 13, // Get the Epo ignored flag
    GetDriveFault = 14, // Get the drive fault flag, indicates faults such as short-circuits and under voltage
    SetAllFwd = 15,     // Set all motors Pwm rate in a forwards direction
    SetAllRev = 16,     // Set all motors Pwm rate in a reverse direction
    SetFailsafe = 17, // Set the failsafe flag, turns the motors off if communication is interrupted
    GetFailsafe = 18, // Get the failsafe flag
    SetEncMode = 19,  // Set the board into encoder or speed mode
    GetEncMode = 20,  // Get the boards current mode, encoder or speed
    MoveAFwd = 21,    // Move motor 2 forward by n encoder ticks
    MoveARev = 22,    // Move motor 2 reverse by n encoder ticks
    MoveBFwd = 23,    // Move motor 1 forward by n encoder ticks
    MoveBRev = 24,    // Move motor 1 reverse by n encoder ticks
    MoveAllFwd = 25,  // Move all motors forward by n encoder ticks
    MoveAllRev = 26,  // Move all motors reverse by n encoder ticks
    GetEncMoving = 27, // Get the status of encoders moving
    SetEncSpeed = 28, // Set the maximum Pwm rate in encoder mode
    GetEncSpeed = 29, // Get the maximum Pwm rate in encoder mode
    GetId = 0x99,     // Get the board identifier
    SetI2CAdd = 0xaa, // Set a new I2C address
}

const FORWARD: u8 = 1; // I2C value representing forward
const REVERSE: u8 = 2; // I2C value representing reverse

const VALUE_ON: u8 = 1; // I2C value representing on
const VALUE_OFF: u8 = 0; // I2C value representing off

/// The PicoBorgRev provides an easy way to interact with a PicoBorg Reverse
/// from rust. The communication is done via the `embedded-hal` i2c traits and
/// the PicoBorg Reverse is located at the i2c address `0x44` by default.
pub struct PicoBorgRev<T: Write + WriteRead> {
    device: T,
}

impl<T: Write + WriteRead> PicoBorgRev<T> {
    /// Attempt to create a new `PicoBorgRev`.
    ///
    /// This will fail if either the i2c device has an error or the PicoBorg
    /// Reverse is not found at the default address (`0x44`).
    pub fn init(device: T) -> Result<PicoBorgRev<T>, <T as WriteRead>::Error> {
        let mut picoborg = PicoBorgRev { device };

        // Check for PicoBorg Reverse
        let id = picoborg.read_u8(Command::GetId)?;

        if id != I2C_ID_PICOBORG_REV {
            // TODO: Return an error
            println!(
                "Found a device at {}, but it is not a PicoBorg Reverse (Id {} instead of {})",
                I2C_ADDRESS, id, I2C_ID_PICOBORG_REV
            );
        } else {
            println!("Found PicoBorg Reverse at {}", I2C_ADDRESS);
        }

        Ok(picoborg)
    }

    fn write_bool(&mut self, command: Command, value: bool) -> Result<(), <T as Write>::Error> {
        self.device.write(
            I2C_ADDRESS,
            &[command as u8, if value { VALUE_ON } else { VALUE_OFF }],
        )
    }

    fn write_u8(&mut self, command: Command, value: u8) -> Result<(), <T as Write>::Error> {
        self.device.write(I2C_ADDRESS, &[command as u8, value])
    }

    fn write_u16(&mut self, command: Command, value: u16) -> Result<(), <T as Write>::Error> {
        let mut data = vec![command as u8];
        data.extend(&u16_to_vu8(value));
        self.device.write(I2C_ADDRESS, &data)
    }

    fn read_bool(&mut self, command: Command) -> Result<bool, <T as WriteRead>::Error> {
        let mut recv: [u8; 2] = [0; 2];
        self.device
            .write_read(I2C_ADDRESS, &[command as u8], &mut recv)?;
        Ok(recv[1] == VALUE_ON)
    }
    fn read_u8(&mut self, command: Command) -> Result<u8, <T as WriteRead>::Error> {
        let mut recv: [u8; 2] = [0; 2];
        self.device
            .write_read(I2C_ADDRESS, &[command as u8], &mut recv)?;
        Ok(recv[1])
    }

    fn read_u8_u8(&mut self, command: Command) -> Result<(u8, u8), <T as WriteRead>::Error> {
        let mut recv: [u8; 3] = [0; 3];
        self.device
            .write_read(I2C_ADDRESS, &[command as u8], &mut recv)?;
        Ok((recv[1], recv[2]))
    }

    /// Sets the drive level for motor 1, from +1 to -1.
    pub fn set_motor_1(&mut self, power: f32) -> Result<(), <T as Write>::Error> {
        if power < 0.0 {
            self.write_u8(Command::SetARev, (PWM_MAX * -power) as u8)
        } else {
            self.write_u8(Command::SetAFwd, (PWM_MAX * power) as u8)
        }
    }

    /// Sets the drive level for motor 2, from +1 to -1
    pub fn set_motor_2(&mut self, power: f32) -> Result<(), <T as Write>::Error> {
        if power < 0.0 {
            self.write_u8(Command::SetBRev, (PWM_MAX * -power) as u8)
        } else {
            self.write_u8(Command::SetBFwd, (PWM_MAX * power) as u8)
        }
    }

    /// Sets the drive level for all motors, from +1 to -1.
    pub fn set_motors(&mut self, power: f32) -> Result<(), <T as Write>::Error> {
        if power < 0.0 {
            self.write_u8(Command::SetAllRev, (PWM_MAX * -power) as u8)
        } else {
            self.write_u8(Command::SetAllFwd, (PWM_MAX * power) as u8)
        }
    }

    /// Sets all motors to stopped, useful when ending a program
    pub fn motors_off(&mut self) -> Result<(), <T as Write>::Error> {
        self.write_u8(Command::AllOff, 0)
    }

    /// Gets the drive level for motor 1, from +1 to -1.
    pub fn get_motor_1(&mut self) -> Result<f32, <T as WriteRead>::Error> {
        let (direction, power) = self.read_u8_u8(Command::GetA)?;
        let power = f32::from(power) / PWM_MAX;
        match direction {
            FORWARD => Ok(power),
            REVERSE => Ok(-power),
            _ => Ok(0.0),
        }
    }

    /// Gets the drive level for motor 1, from +1 to -1.
    pub fn get_motor_2(&mut self) -> Result<f32, <T as WriteRead>::Error> {
        let (direction, power) = self.read_u8_u8(Command::GetB)?;
        let power = f32::from(power) / PWM_MAX;
        match direction {
            FORWARD => Ok(power),
            REVERSE => Ok(-power),
            _ => Ok(0.0),
        }
    }

    /// Sets the current state of the Led, False for off, True for on
    pub fn set_led(&mut self, state: bool) -> Result<(), <T as Write>::Error> {
        self.write_bool(Command::SetLed, state)
    }

    /// Reads the current state of the Led, False for off, True for on
    pub fn get_led(&mut self) -> Result<bool, <T as WriteRead>::Error> {
        self.read_bool(Command::GetLed)
    }

    /// Resets the Epo latch state, use to allow movement again after the Epo
    /// has been tripped.
    pub fn reset_epo(&mut self) -> Result<(), <T as Write>::Error> {
        self.write_u8(Command::ResetEpo, 0)
    }

    /// Reads the system Epo latch state.
    /// If `false` the Epo has not been tripped, and movement is allowed.
    /// If `true` the Epo has been tripped, movement is disabled if the Epo
    /// is not ignored (see `set_epo_ignore`).
    ///
    /// Movement can be re-enabled by calling `reset_epo`.
    pub fn get_epo(&mut self) -> Result<bool, <T as WriteRead>::Error> {
        self.read_bool(Command::GetEpo)
    }

    /// Sets the system to ignore or use the Epo latch, set to `false` if you
    /// have an Epo switch, `true` if you do not.
    pub fn set_epo_ignore(&mut self, state: bool) -> Result<(), <T as Write>::Error> {
        self.write_bool(Command::SetEpoIgnore, state)
    }

    /// Reads the system Epo ignore state, `false` for using the Epo latch,
    /// `true` for ignoring the Epo latch.
    pub fn get_epo_ignore(&mut self) -> Result<bool, <T as WriteRead>::Error> {
        self.read_bool(Command::GetEpoIgnore)
    }

    /// Sets the system to enable or disable the communications failsafe.
    /// The failsafe will turn the motors off unless it is commanded at least
    /// once every 1/4 of a second.
    ///
    /// Set to `true` to enable this failsafe or set to `false` to disable.
    ///
    /// The failsafe is disabled at power on.
    pub fn set_comms_failsafe(&mut self, state: bool) -> Result<(), <T as Write>::Error> {
        self.write_bool(Command::SetFailsafe, state)
    }

    /// Read the current system state of the communications failsafe, `true`
    /// for enabled, `false` for disabled. The failsafe will turn the motors
    /// off unless it is commanded at least once every 1/4 of a second.
    pub fn get_comms_failsafe(&mut self) -> Result<bool, <T as WriteRead>::Error> {
        self.read_bool(Command::GetFailsafe)
    }

    /// Reads the system drive fault state, False for no problems, True for a
    /// fault has been detected. Faults may indicate power problems, such as
    /// under-voltage (not enough power), and may be cleared by setting a
    /// lower drive power.
    ///
    /// If a fault is persistent, it repeatably occurs when trying to control
    /// the board, this may indicate a wiring problem such as:
    /// - The supply is not powerful enough for the motors
    ///   - The board has a bare minimum requirement of 6V to operate correctly
    ///   - A recommended minimum supply of 7.2V should be sufficient for
    ///     smaller motors
    /// - The + and - connections for either motor are connected to each other
    /// - Either + or - is connected to ground (Gnd, also known as 0V or earth)
    /// - Either + or - is connected to the power supply (V+, directly to the
    ///   battery or power pack)
    /// - One of the motors may be damaged
    ///
    /// Faults will self-clear, they do not need to be reset, however some
    /// faults require both motors to be moving at less than 100% to clear.
    ///
    /// The easiest way to check is to put both motors at a low power setting
    /// which is high enough for them to rotate easily, such as 30%. Note that
    /// the fault state may be true at power up, this is normal and should
    /// clear when both motors have been driven.
    ///
    /// If there are no faults but you cannot make your motors move check
    /// `get_epo` to see if the safety switch has been tripped.
    ///
    /// For more details check the website at www.piborg.org/picoborgrev and
    /// double check the wiring instructions.
    pub fn get_drive_fault(&mut self) -> Result<bool, <T as WriteRead>::Error> {
        self.read_bool(Command::GetDriveFault)
    }

    /// Sets the system to enable or disable the encoder based move mode. In
    /// encoder move mode (enabled) the `encoder_move_motor*` commands are
    /// available to move fixed distances. In non-encoder move mode
    /// (disabled) the `set_motor*` commands should be used to set drive
    /// levels.
    ///
    /// The encoder move mode requires that the encoder feedback is attached
    /// to an encoder signal, see the website at www.piborg.org/picoborgrev
    /// for wiring instructions.
    ///
    /// The encoder based move mode is disabled at power on.
    pub fn set_encoder_move_mode(&mut self, state: bool) -> Result<(), <T as Write>::Error> {
        self.write_bool(Command::SetEncMode, state)
    }

    /// Read the current system state of the encoder based move mode, `true`
    /// for enabled (encoder moves), `false` for disabled (power level moves)
    pub fn get_encoder_move_mode(&mut self) -> Result<bool, <T as WriteRead>::Error> {
        self.read_bool(Command::GetEncMode)
    }

    /// Moves motor 1 until it has seen a number of encoder counts, up to 32767
    /// Use negative values to move in reverse
    pub fn encoder_move_motor_1(&mut self, counts: i16) -> Result<(), <T as Write>::Error> {
        if counts < 0 {
            self.write_u16(Command::MoveARev, -counts as u16)
        } else {
            self.write_u16(Command::MoveAFwd, counts as u16)
        }
    }

    /// Moves motor 2 until it has seen a number of encoder counts.
    /// Use negative values to move in reverse.
    pub fn encoder_move_motor_2(&mut self, counts: i16) -> Result<(), <T as Write>::Error> {
        if counts < 0 {
            self.write_u16(Command::MoveBRev, -counts as u16)
        } else {
            self.write_u16(Command::MoveBFwd, counts as u16)
        }
    }

    /// Moves all motors until it has seen a number of encoder counts.
    /// Use negative values to move in reverse.
    pub fn encoder_move_motors(&mut self, counts: i16) -> Result<(), <T as Write>::Error> {
        if counts < 0 {
            self.write_u16(Command::MoveAllRev, -counts as u16)
        } else {
            self.write_u16(Command::MoveAllFwd, counts as u16)
        }
    }

    /// Reads the current state of the encoder motion, `false` for all motors
    /// have finished, `true` for any motor is still moving.
    pub fn is_encoder_moving(&mut self) -> Result<bool, <T as WriteRead>::Error> {
        self.read_bool(Command::GetEncMoving)
    }

    /// Waits until all motors have finished performing encoder based moves
    /// If the motors stop moving the function will return `true` or if the
    /// `timeout` occurs it will return `false`.
    pub fn wait_while_encoder_moving(
        &mut self,
        timeout: Duration,
    ) -> Result<bool, <T as WriteRead>::Error> {
        let now = Instant::now();
        while self.is_encoder_moving()? && now.elapsed() < timeout {
            sleep(Duration::from_millis(100));
        }

        Ok(now.elapsed() < timeout)
    }

    /// Sets the drive limit for encoder based moves, from 0 to 1.
    pub fn set_encoder_speed(&mut self, power: f32) -> Result<(), <T as Write>::Error> {
        self.write_u8(Command::SetEncSpeed, (PWM_MAX * power) as u8)
    }

    /// Gets the drive limit for encoder based moves, from 0 to 1.
    pub fn get_encoder_speed(&mut self) -> Result<f32, <T as WriteRead>::Error> {
        let data = self.read_u8(Command::GetEncMode)?;
        Ok(f32::from(data) / PWM_MAX)
    }
}

impl<T: Write + WriteRead> Drop for PicoBorgRev<T> {
    fn drop(&mut self) {
        let _ = self.motors_off();
        let _ = self.set_led(false);
    }
}

fn u16_to_vu8(input: u16) -> [u8; 2] {
    let one: u8 = ((input >> 8) & 0xff) as u8;
    let two: u8 = (input & 0xff) as u8;
    [one, two]
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
