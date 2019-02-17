[![Build Status]][travis] [![Latest Version]][crates.io] [![Latest Docs]][docs]

[Build Status]: https://travis-ci.org/marvin-rs/picoborgrev.svg?branch=master
[travis]: https://travis-ci.org/marvin-rs/picoborgrev
[Latest Version]: https://img.shields.io/crates/v/picoborgrev.svg
[crates.io]: https://crates.io/crates/picoborgrev
[Latest Docs]: https://docs.rs/picoborgrev/badge.svg
[docs]: https://docs.rs/picoborgrev

# picoborgrev

This module is designed to communicate with the PicoBorg Reverse via Rust
and the [embedded-hal](https://crates.io/crates/embedded-hal) traits.

See the PiBorg website at www.piborg.org/picoborgreverse for more details
on the PicoBorgRev board.

**Important:** This is still a work in progress and the API should not be considered stable until the
`1.0` release.

## TODO

- Make enum names more descriptive
- Add custom error type

## Usage

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
let mut borg = PicoBorgRev::init(device).expect("Unable to create PicoBorgRev");
borg.set_led(true).unwrap();
```

## Examples

- [Blink LED](examples/toggle_led.rs)

## License

Licensed under either of

- Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.
