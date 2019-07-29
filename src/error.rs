#[macro_use] extern crate quick_error;

quick_error! {
    #[derive(Debug)]
    pub enum PicoError {
        WriteRead::Error {}
    }
}
