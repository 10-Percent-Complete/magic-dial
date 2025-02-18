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
pub enum CrcEn {
    Off = 0x0,
    On = 0x1,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum TempCoeff {
    _0 = 0x0,
    _0_12 = 0x1,
    _0_2 = 0x3,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum ConversionRate {
    _1x = 0x0,
    _2x = 0x1,
    _4x = 0x2,
    _8x = 0x3,
    _16x = 0x4,
    _32x = 0x5,
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
pub enum HystersisThreshold {
    TwoComplement = 0x0,
    SevenLsb = 0x1,
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

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum ThresholdXCount {
    One = 0x0,
    Four = 0x1,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum ThresholdTriggerDirection {
    Above = 0x0,
    Below = 0x1,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum MagnitudeGainChannel {
    One = 0x0,
    Two = 0x1,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AngleEnable {
    Disable = 0x0,
    EnableXY = 0x1,
    EnableYZ = 0x2,
    EnableXZ = 0x3,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum XYRange {
    Default = 0x0,
    Double = 0x1,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum ZRange {
    Default = 0x0,
    Double = 0x1,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum InterruptMode {
    Off = 0x0,
    IntPin = 0x1,
    IntPinI2cBusy = 0x2,
    SclPin = 0x3,
    SclPinI2cBusy = 0x4,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum InterruptState {
    Latch = 0x0,
    Pulse = 0x1,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum InterruptThreshold {
    Off = 0x0,
    On = 0x1,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum InterruptResult {
    NotOnConversion = 0x0,
    OnConversion = 0x1,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum InterruptMask {
    IntPinEnable = 0x0,
    IntPinDisable = 0x1,
}

#[derive(Copy, Clone, Debug)]
pub struct Tmag5273<I2C> {
    i2c: I2C,
    address: u8,
}
#[allow(dead_code)]
impl<I2C: I2c> Tmag5273<I2C> {
    /* Construtor  */
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /* Helper functions */
    fn write_register_8(&mut self, reg: Register, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[reg as u8, value])
    }

    fn read_register_8(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut temp = [0];
        self.i2c.write_read(self.address, &[reg as u8], &mut temp)?;

        Ok(temp[0])
    }

    /* Configuration functions */
    pub fn config_device_1(
        &mut self,
        crc: CrcEn,
        temp_coeff: TempCoeff,
        rate: ConversionRate,
        i2c_mode: I2cReadMode,
    ) -> Result<(), I2C::Error> {
        let val = (crc as u8) << 7 | (temp_coeff as u8) << 5 | (rate as u8) << 2 | (i2c_mode as u8);

        self.write_register_8(Register::DeviceConfig1, val)
    }

    pub fn config_device_2(
        &mut self,
        threshold: HystersisThreshold,
        low_mode: LowMode,
        glitch: I2cGlitchFilter,
        trigger: TriggerMode,
        op_mode: OpMode,
    ) -> Result<(), I2C::Error> {
        let val = (threshold as u8) << 5
            | (low_mode as u8) << 4
            | (glitch as u8) << 3
            | (trigger as u8) << 2
            | op_mode as u8;

        self.write_register_8(Register::DeviceConfig2, val)
    }

    pub fn config_sensor_1(
        &mut self,
        config: MagneticConfig,
        sleep: SleepConfig,
    ) -> Result<(), I2C::Error> {
        let val = (config as u8) << 4 | (sleep as u8);

        self.write_register_8(Register::SensorConfig1, val)
    }

    pub fn config_sensor_2(
        &mut self,
        count: ThresholdXCount,
        direction: ThresholdTriggerDirection,
        gain_channel: MagnitudeGainChannel,
        angle_en: AngleEnable,
        x_y_range: XYRange,
        z_range: ZRange,
    ) -> Result<(), I2C::Error> {
        let val = (count as u8) << 6
            | (direction as u8) << 5
            | (gain_channel as u8) << 4
            | (angle_en as u8) << 2
            | (x_y_range as u8) << 1
            | z_range as u8;

        self.write_register_8(Register::SensorConfig2, val)
    }

    pub fn config_x_threshold(&mut self, threshold: u8) -> Result<(), I2C::Error> {
        self.write_register_8(Register::XThrConfig, threshold)
    }

    pub fn config_y_threshold(&mut self, threshold: u8) -> Result<(), I2C::Error> {
        self.write_register_8(Register::YThrConfig, threshold)
    }

    pub fn config_z_threshold(&mut self, threshold: u8) -> Result<(), I2C::Error> {
        self.write_register_8(Register::ZThrConfig, threshold)
    }

    pub fn config_temp(&mut self, threshold: u8, enable: bool) -> Result<(), I2C::Error> {
        let val = (threshold << 1) | (enable as u8);

        self.write_register_8(Register::TConfig, val)
    }

    pub fn config_interrupt(
        &mut self,
        conversion: InterruptResult,
        threshold: InterruptThreshold,
        state: InterruptState,
        mode: InterruptMode,
        mask: InterruptMask,
    ) -> Result<(), I2C::Error> {
        let val = (conversion as u8) << 7
            | (threshold as u8) << 6
            | (state as u8) << 5
            | (mode as u8) << 4
            | (mask as u8);
        self.write_register_8(Register::IntConfig1, val)
    }

    pub fn config_mag_offset_1(&mut self, offset: u8) -> Result<(), I2C::Error> {
        self.write_register_8(Register::MagOffsetConfig1, offset)
    }

    pub fn config_mag_offset_2(&mut self, offset: u8) -> Result<(), I2C::Error> {
        self.write_register_8(Register::MagOffsetConfig1, offset)
    }

    pub fn config_i2c_address(&mut self, address: u8, update_en: bool) -> Result<(), I2C::Error> {
        let val = (address << 1) | (update_en as u8);

        self.write_register_8(Register::I2cAddress, val)
    }

    /* Status functions */
    pub fn read_device_id(&mut self) -> Result<u8, I2C::Error> {
        self.read_register_8(Register::DeviceId)
    }

    pub fn read_manufacturer_id(&mut self) -> Result<u16, I2C::Error> {
        let msb = self.read_register_8(Register::ManufacturerIdMsb)?;
        let lsb = self.read_register_8(Register::ManufacturerIdLsb)?;

        Ok(u16::from_be_bytes([msb, lsb]))
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

    pub fn read_conversion_status(&mut self) -> Result<u8, I2C::Error> {
        self.read_register_8(Register::ConvStatus)
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

    pub fn read_device_status(&mut self) -> Result<u8, I2C::Error> {
        self.read_register_8(Register::DeviceStatus)
    }
}
