[![Build Status](https://travis-ci.org/MJohnson459/picoborgrev.svg?branch=master)](https://travis-ci.org/MJohnson459/picoborgrev)

# PicoBorgRev
This module is designed to communicate with the PicoBorg Reverse via Rust
and the [embedded-hal](https://crates.io/crates/embedded-hal) traits.

See the PiBorg website at www.piborg.org/picoborgreverse for more details
on the PicoBorgRev board.

# Usage

The first step is to add `picoborgrev` to your `cargo.toml` file:
```toml
[dependencies]
picoborgrev = "0.1"
```

Then in your module you then need to import the crate:
```rust
extern crate picoborgrev;

use picoborgrev::PicoBorgRev;
```

To create a new `PicoBorgRev` controller you will need to supply an `embedded-hal` implementation
such as `linux-embedded-hal`:
```rust
extern crate linux_embedded_hal;

use linux_embedded_hal::I2cdev;
use std::path::Path;

let device = I2cdev::new(Path::new("/dev/i2c-1")).expect("Unable to create i2c device");
```

Finally create a new `PicoBorgRev` supplying the `I2C` implementation:
```rust
let mut borg = PicoBorgRev::new(device).expect("Unable to create PicoBorgRev");
borg.set_led(true).unwrap();
```

# Examples
- [Blink LED](examples/toggle_led.rs)