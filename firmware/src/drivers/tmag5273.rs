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
    DeviceStatus = 0x1C,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum MagneticConfig {
    DisableAll = 0x0,
    EnableX = 0x1,
    EnableY = 0x2,
    EnableXY = 0x3,
    EnableZ = 0x4,
    EnableZX = 0x5,
    EnableYZ = 0x6,
    EnableXYZ = 0x7,
    EnableXYX = 0x8,
    EnableYXY = 0x9,
    EnableYZY = 0xA,
    EnableXZX = 0xB,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum SleepConfig {
    Sleep1ms = 0x0,
    Sleep5ms = 0x1,
    Sleep10ms = 0x2,
    Sleep15ms = 0x3,
    Sleep20ms = 0x4,
    Sleep30ms = 0x5,
    Sleep50ms = 0x6,
    Sleep100ms = 0x7,
    Sleep500ms = 0x8,
    Sleep1000ms = 0x9,
    Sleep2000ms = 0xA,
    Sleep5000ms = 0xB,
    Sleep20000ms = 0xC,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AngleConfig {
    Disable = 0x0,
    EnableXY = 0x1,
    EnableYZ = 0x2,
    EnableXZ = 0x3,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum ConversionRate {
    Average1x = 0x0,
    Average2x = 0x1,
    Average4x = 0x2,
    Average8x = 0x3,
    Average16x = 0x4,
    Average32x = 0x5,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum I2cReadMode {
    I2cRead3 = 0x0,
    I2cRead2 = 0x1,
    I2cRead1 = 0x2,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum LowMode {
    LowPower = 0x0,
    LowNoise = 0x1,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum I2cGlitchFilter {
    On = 0x0,
    Off = 0x1,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum TriggerMode {
    Default = 0x0,
    Interrupt = 0x1,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum OpMode {
    Standby = 0x0,
    Sleep = 0x1,
    Continuous = 0x2,
    WakeAndSleep = 0x3,
}

#[derive(Copy, Clone, Debug)]
pub struct Tmag5273<I2C> {
    i2c: I2C,
    address: u8,
}
#[allow(dead_code)]
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

    pub fn config_magnetic_channel(&mut self, config: MagneticConfig) -> Result<(), I2C::Error> {
        /* Read contents of register and ensure to not overwrite */
        let mut val = self.read_register_8(Register::SensorConfig1)?;
        val = (val & !(0xF0)) | ((config as u8) << 4);

        self.write_register_8(Register::SensorConfig1, val)
    }

    pub fn config_temp(&mut self, threshold: u8, enable: bool) -> Result<(), I2C::Error> {
        let val = (threshold << 1) | (enable as u8);

        self.write_register_8(Register::TConfig, val)
    }

    pub fn config_operating_mode(&mut self, op_mode: OpMode) -> Result<(), I2C::Error> {
        /* Read contents of register and ensure to not overwrite */
        let mut val = self.read_register_8(Register::DeviceConfig2)?;
        val = (val & !(0x3)) | (op_mode as u8);

        self.write_register_8(Register::DeviceConfig2, val)
    }

    pub fn config_angle(&mut self, config: AngleConfig) -> Result<(), I2C::Error> {
        /* Read contents of register and ensure to not overwrite */
        let mut val = self.read_register_8(Register::SensorConfig2)?;
        val = (val & !(0xC)) | ((config as u8) << 2);

        self.write_register_8(Register::SensorConfig2, val)
    }

    pub fn config_conversion(&mut self, conversion: ConversionRate) -> Result<(), I2C::Error> {
        /* Read contents of register and ensure to not overwrite */
        let mut val = self.read_register_8(Register::DeviceConfig1)?;
        val = (val & !(0x1C)) | ((conversion as u8) << 2);

        self.write_register_8(Register::DeviceConfig1, val)
    }

    pub fn read_manufacturer_id(&mut self) -> Result<u16, I2C::Error> {
        let msb = self.read_register_8(Register::ManufacturerIdMsb)?;
        let lsb = self.read_register_8(Register::ManufacturerIdLsb)?;

        Ok(u16::from_be_bytes([msb, lsb]))
    }

    pub fn read_device_status(&mut self) -> Result<u8, I2C::Error> {
        self.read_register_8(Register::DeviceStatus)
    }

    pub fn read_temp(&mut self) -> Result<u16, I2C::Error> {
        let msb = self.read_register_8(Register::TMsbResult)?;
        let lsb = self.read_register_8(Register::TLsbResult)?;

        Ok(u16::from_be_bytes([msb, lsb]))
    }

    pub fn read_x(&mut self) -> Result<u16, I2C::Error> {
        let msb = self.read_register_8(Register::XMsbResult)?;
        let lsb = self.read_register_8(Register::XLsbResult)?;

        Ok(u16::from_be_bytes([msb, lsb]))
    }

    pub fn read_y(&mut self) -> Result<u16, I2C::Error> {
        let msb = self.read_register_8(Register::YMsbResult)?;
        let lsb = self.read_register_8(Register::YLsbResult)?;

        Ok(u16::from_be_bytes([msb, lsb]))
    }

    pub fn read_z(&mut self) -> Result<u16, I2C::Error> {
        let msb = self.read_register_8(Register::ZMsbResult)?;
        let lsb = self.read_register_8(Register::ZLsbResult)?;

        Ok(u16::from_be_bytes([msb, lsb]))
    }

    pub fn read_angle(&mut self) -> Result<f32, I2C::Error> {
        /* Read and combine angle registers into single 16b variable */
        let msb = self.read_register_8(Register::AngleResultMsb)?;
        let lsb = self.read_register_8(Register::AngleResultLsb)?;
        let raw = u16::from_be_bytes([msb, lsb]);

        /* Convert raw bytes into a float w/ bottom 4 bits being fracitonal */
        Ok((raw as f32) / 16.0_f32)
    }

    pub fn read_magnitude(&mut self) -> Result<u8, I2C::Error> {
        self.read_register_8(Register::MagnitudeResult)
    }
}
