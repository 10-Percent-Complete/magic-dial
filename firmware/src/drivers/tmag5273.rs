use embedded_hal::i2c::I2c;

pub const DEFAULT_I2C_ADDR: u8 = 0x22;


#[allow(dead_code)]
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum Register {
    DeviceConfig1 = 0x0,
    DeviceConfig2 = 0x1,
    SensorConfig1 = 0x2,
    SensorConfig2 = 0x3,
    XThrConfig = 0x4,
    YThrConfig = 0x5,
    ZThrConfig = 0x6,
    TConfig = 0x7,
    IntConfig1 = 0x8,
    MagGainConfig = 0x9,
    MagOffsetConfig1 = 0xA,
    MagOffsetConfig2 = 0xB,
    I2cAddress = 0xC,
    DeviceId = 0xD,
    ManufacturerIdLsb = 0xE,
    ManufacturerIdMsb = 0xF,
    TMsbResult = 0x10,
    TLsbResult = 0x11,
    XMsbResult = 0x12,
    XLsbResult = 0x13,
    YMsbResult = 0x14,
    YLsbResult = 0x15,
    ZMsbResult = 0x16,
    ZLsbResult = 0x17,
    ConvStatus = 0x18,
    AngleResultMsb = 0x19,
    AngleResultLsb = 0x1A,
    MagnitudeResult = 0x1B,
    DeviceStatus = 0x1C
}


#[derive(Copy, Clone, Debug)]
pub struct Tmag5273<I2C> {
    i2c: I2C,
    address: u8
}

impl<I2C: I2c> Tmag5273<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    fn write_register_8(&mut self, reg: Register, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[reg as u8, value])
    }

    fn read_register_8(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut temp = [0];
        self.i2c.write_read(self.address, &[reg as u8], &mut temp)?;
        Ok(temp[0])
    }

    pub fn config_temp(&mut self, threshold: u8, enable: bool) -> Result<(), I2C::Error> {
        let val = (threshold << 1) | (enable as u8);
        Ok(self.write_register_8(Register::TConfig, val)?) 
    }

    pub fn read_temp(&mut self) -> Result <u16, I2C::Error> {
        let msb = self.read_register_8(Register::TMsbResult)?;
        let lsb = self.read_register_8(Register::TLsbResult)?;

        Ok(u16::from_be_bytes([msb, lsb]))
    }

    pub fn read_x(&mut self) -> Result <u16, I2C::Error> {
        let msb = self.read_register_8(Register::XMsbResult)?;
        let lsb = self.read_register_8(Register::XLsbResult)?;

        Ok(u16::from_be_bytes([msb, lsb]))
    }

    pub fn read_y(&mut self) -> Result <u16, I2C::Error> {
        let msb = self.read_register_8(Register::YMsbResult)?;
        let lsb = self.read_register_8(Register::YLsbResult)?;

        Ok(u16::from_be_bytes([msb, lsb]))
    }

    pub fn read_z(&mut self) -> Result <u16, I2C::Error> {
        let msb = self.read_register_8(Register::ZMsbResult)?;
        let lsb = self.read_register_8(Register::ZLsbResult)?;

        Ok(u16::from_be_bytes([msb, lsb]))
    }
}
