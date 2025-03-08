use esp_hal::gpio::{GpioPin, Output};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::peripherals::I2C0;
use esp_hal::Blocking;

pub fn init(i2c0: I2C0, sda: GpioPin<21>, scl: GpioPin<22>) -> I2c<'static, Blocking> {
    I2c::new(i2c0, I2cConfig::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl)
}
