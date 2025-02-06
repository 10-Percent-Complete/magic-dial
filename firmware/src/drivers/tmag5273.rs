use embedded_hal::i2c::{I2c, Error};

pub const DEFAULT_I2C_ADDR: u8 = 0x22;

#[derive(Copy, Clone, Debug)]
pub struct Tmag5273<I2C> {
    i2c: I2C,
}

impl<I2C: I2c> Tmag5273<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    pub fn read_device_id(&mut self) -> Result<u8, I2C::Error> {
        let mut device_id = [0];
        self.i2c.write_read(DEFAULT_I2C_ADDR, &[0x0D], &mut device_id)?;
        Ok(device_id[0])
    }
}
