#![no_std]
#![no_main]

use drivers::tmag5273::{
    AngleEnable, ConversionRate, CrcEn, HystersisThreshold, I2cGlitchFilter, I2cReadMode, LowMode,
    MagneticConfig, MagnitudeGainChannel, OpMode, SleepConfig, TempCoeff,
    ThresholdTriggerDirection, ThresholdXCount, Tmag5273, TriggerMode, XYRange, ZRange,
    DEFAULT_I2C_ADDR,
};

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig},
    delay::Delay,
    gpio::Input,
    i2c::master::{Config as I2cConfig, I2c},
    main,
    mcpwm::{
        operator::{PwmPinConfig, PwmUpdateMethod},
        McPwm, PeripheralClockConfig,
    },
    time::RateExtU32,
    Config,
};
use log::info;

mod drivers;

#[main]
fn main() -> ! {
    /* Initialize esp32 hardware */
    let peripherals = esp_hal::init(Config::default());

    /* Initialize log functionality */
    esp_println::logger::init_logger_from_env();

    /* Create delay object to use for code delays */
    let _delay = Delay::new();

    /* Apply labels for all used pins */
    /* I2C magnetic encoder pins */
    let pin_i2c_sda = peripherals.GPIO21;
    let pin_i2c_scl = peripherals.GPIO22;

    /* ADC current sense pins */
    let pin_u_current = peripherals.GPIO35;
    let pin_v_current = peripherals.GPIO36;
    let pin_w_current = peripherals.GPIO39;
    let pin_supply_current = peripherals.GPIO32;

    /* Motor driver pwm high pins */
    let pin_uh = peripherals.GPIO16;
    let pin_vh = peripherals.GPIO18;
    let pin_wh = peripherals.GPIO19;

    /* Motor driver pwm low pins */
    let pin_ul = peripherals.GPIO17;
    let pin_vl = peripherals.GPIO23;
    let pin_wl = peripherals.GPIO33;

    /* Buttons */
    let gpio_13 = peripherals.GPIO13;
    let gpio_14 = peripherals.GPIO14;

    /* Initialize i2c0 controller in EPS32 */
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(pin_i2c_sda)
        .with_scl(pin_i2c_scl);

    /* Create adc config to use for all ADC channels */
    let mut adc_current_config = AdcConfig::new();

    /* Enable all current ADC pins */
    let mut _adc_u_current_pin =
        adc_current_config.enable_pin(pin_u_current, esp_hal::analog::adc::Attenuation::_11dB);
    let mut _adc_v_current_pin =
        adc_current_config.enable_pin(pin_v_current, esp_hal::analog::adc::Attenuation::_11dB);
    let mut _adc_w_current_pin =
        adc_current_config.enable_pin(pin_w_current, esp_hal::analog::adc::Attenuation::_11dB);
    let mut _adc_supply_current_pin =
        adc_current_config.enable_pin(pin_supply_current, esp_hal::analog::adc::Attenuation::_11dB);

    /* Initialize ADC1 module */
    let mut _adc_current = Adc::new(peripherals.ADC1, adc_current_config);

    /* Initialize buttons */
    let btn_13 = Input::new(gpio_13, esp_hal::gpio::Pull::Up);
    let btn_14 = Input::new(gpio_14, esp_hal::gpio::Pull::Up);

    /* Initialize Motor Control PWM0 module */
    let pwm_clock_cfg = PeripheralClockConfig::with_frequency(32_u32.MHz()).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, pwm_clock_cfg);

    /* Configure main pwm frequency */
    let timer_clock_cfg = pwm_clock_cfg
        .timer_clock_with_frequency(
            99,
            esp_hal::mcpwm::timer::PwmWorkingMode::Increase,
            25_u32.kHz(), // Set PWM frequency to 15kHz
        )
        .unwrap();

    /* Link main pwm timer with each phase's operator object */
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);
    mcpwm.operator2.set_timer(&mcpwm.timer0);

    /* Extract out high and low pins of each phase's operator object */
    let (mut pwm_uh, mut pwm_ul) = mcpwm.operator0.with_pins(
        pin_uh,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pin_ul,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );
    let (mut pwm_vh, mut pwm_vl) = mcpwm.operator1.with_pins(
        pin_vh,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pin_vl,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );
    let (mut pwm_wh, mut pwm_wl) = mcpwm.operator2.with_pins(
        pin_wh,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pin_wl,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );

    /* Begin main pwm timer */
    mcpwm.timer0.start(timer_clock_cfg);

    /* Ensure all PWM timestamp changes are loaded immediately */
    pwm_uh.set_update_method(PwmUpdateMethod::empty());
    pwm_vh.set_update_method(PwmUpdateMethod::empty());
    pwm_wh.set_update_method(PwmUpdateMethod::empty());
    pwm_uh.set_update_method(PwmUpdateMethod::empty());
    pwm_vh.set_update_method(PwmUpdateMethod::empty());
    pwm_wh.set_update_method(PwmUpdateMethod::empty());

    /* Initialize hall effect sensor object */
    let mut magnetic_sensor = Tmag5273::new(i2c, DEFAULT_I2C_ADDR);

    /* Attempt to initialize the following magnetic sensor settings:
     * Average conversion rate: 32x
     * Low noise mode
     * Continuous sample mode
     * Enable XY channels
     * Enable angle conversion channels XY
     **/
    magnetic_sensor
        .config_device_1(
            CrcEn::Off,
            TempCoeff::_0,
            ConversionRate::_32x,
            I2cReadMode::I2cRead3,
        )
        .unwrap();

    magnetic_sensor
        .config_device_2(
            HystersisThreshold::TwoComplement,
            LowMode::LowNoise,
            I2cGlitchFilter::On,
            TriggerMode::Default,
            OpMode::Continuous,
        )
        .unwrap();

    magnetic_sensor
        .config_sensor_1(MagneticConfig::EnableXYX, SleepConfig::Sleep1ms)
        .unwrap();

    magnetic_sensor
        .config_sensor_2(
            ThresholdXCount::One,
            ThresholdTriggerDirection::Below,
            MagnitudeGainChannel::One,
            AngleEnable::EnableXY,
            XYRange::Default,
            ZRange::Default,
        )
        .unwrap();

    /* Size of unique commutation states in degrees */
    const _STEP_SIZE: f32 = 360.0_f32 / 42.0_f32;

    /* Initial motor control settings */
    let mut speed: i16 = 500;
    let mut commutation_state: i8 = 0;

    loop {
        if btn_13.is_low() && speed < 1000 {
            speed += 1;
        } else if btn_14.is_low() && speed > -1000 {
            speed -= 1;
        }

        /* Clockwise commutation */
        if speed >= 0 {
            match commutation_state as u8 {
                0 => {
                    /* W High -> U Low */
                    pwm_uh.set_timestamp(0);
                    pwm_vh.set_timestamp(0);
                    pwm_ul.set_timestamp(0);
                    pwm_vl.set_timestamp(0);
                    pwm_wl.set_timestamp(0);

                    pwm_ul.set_timestamp(speed as u16);
                    pwm_wh.set_timestamp(speed as u16);
                    commutation_state = 1;
                }
                1 => {
                    /* V High -> U Low */
                    pwm_uh.set_timestamp(0);
                    pwm_wh.set_timestamp(0);
                    pwm_vl.set_timestamp(0);
                    pwm_wl.set_timestamp(0);

                    pwm_ul.set_timestamp(speed as u16);
                    pwm_vh.set_timestamp(speed as u16);
                    commutation_state = 2;
                }
                2 => {
                    /* V High -> W Low */
                    pwm_uh.set_timestamp(0);
                    pwm_wh.set_timestamp(0);
                    pwm_ul.set_timestamp(0);
                    pwm_vl.set_timestamp(0);

                    pwm_wl.set_timestamp(speed as u16);
                    pwm_vh.set_timestamp(speed as u16);
                    commutation_state = 3;
                }
                3 => {
                    /* U High -> W Low */
                    pwm_vh.set_timestamp(0);
                    pwm_wh.set_timestamp(0);
                    pwm_ul.set_timestamp(0);
                    pwm_vl.set_timestamp(0);

                    pwm_wl.set_timestamp(speed as u16);
                    pwm_uh.set_timestamp(speed as u16);
                    commutation_state = 4;
                }
                4 => {
                    /* U High -> V Low */
                    pwm_vh.set_timestamp(0);
                    pwm_wh.set_timestamp(0);
                    pwm_ul.set_timestamp(0);
                    pwm_wl.set_timestamp(0);

                    pwm_vl.set_timestamp(speed as u16);
                    pwm_uh.set_timestamp(speed as u16);
                    commutation_state = 5;
                }
                5 => {
                    /* W High -> V Low */
                    pwm_uh.set_timestamp(0);
                    pwm_vh.set_timestamp(0);
                    pwm_ul.set_timestamp(0);
                    pwm_wl.set_timestamp(0);

                    pwm_vl.set_timestamp(speed as u16);
                    pwm_wh.set_timestamp(speed as u16);
                    commutation_state = 0;
                }
                _ => {
                    /* Default */
                    pwm_uh.set_timestamp(0);
                    pwm_vh.set_timestamp(0);
                    pwm_wh.set_timestamp(0);
                    pwm_ul.set_timestamp(0);
                    pwm_vl.set_timestamp(0);
                    pwm_wl.set_timestamp(0);
                }
            }
        }
        /* Counter Clockwise commutation */
        else {
            match commutation_state.abs() {
                0 => {
                    /* U High -> W Low */
                    pwm_vh.set_timestamp(0);
                    pwm_wh.set_timestamp(0);
                    pwm_ul.set_timestamp(0);
                    pwm_vl.set_timestamp(0);
                    
                    pwm_wl.set_timestamp(speed as u16);
                    pwm_uh.set_timestamp(speed as u16);
                    commutation_state = 5;
                }
                1 => {
                    /* U High -> V Low */
                    pwm_vh.set_timestamp(0);
                    pwm_wh.set_timestamp(0);
                    pwm_ul.set_timestamp(0);
                    pwm_wl.set_timestamp(0);
                    
                    pwm_vl.set_timestamp(speed as u16);
                    pwm_uh.set_timestamp(speed as u16);
                    commutation_state = 0;
                }
                2 => {
                    /* W High -> V Low */
                    pwm_uh.set_timestamp(0);
                    pwm_vh.set_timestamp(0);
                    pwm_ul.set_timestamp(0);
                    pwm_wl.set_timestamp(0);
                    
                    pwm_vl.set_timestamp(speed as u16);
                    pwm_wh.set_timestamp(speed as u16);
                    commutation_state = 1;
                }
                3 => {
                    /* W High -> U Low */
                    pwm_uh.set_timestamp(0);
                    pwm_vh.set_timestamp(0);
                    pwm_ul.set_timestamp(0);
                    pwm_vl.set_timestamp(0);
                    pwm_wl.set_timestamp(0);

                    pwm_ul.set_timestamp(speed as u16);
                    pwm_wh.set_timestamp(speed as u16);
                    commutation_state = 2;
                }
                4 => {
                    /* V High -> U Low */
                    pwm_uh.set_timestamp(0);
                    pwm_wh.set_timestamp(0);
                    pwm_vl.set_timestamp(0);
                    pwm_wl.set_timestamp(0);

                    pwm_ul.set_timestamp(speed as u16);
                    pwm_vh.set_timestamp(speed as u16);
                    commutation_state = 3;
                }
                5 => {
                    /* V High -> W Low */
                    pwm_uh.set_timestamp(0);
                    pwm_wh.set_timestamp(0);
                    pwm_ul.set_timestamp(0);
                    pwm_vl.set_timestamp(0);

                    pwm_wl.set_timestamp(speed as u16);
                    pwm_vh.set_timestamp(speed as u16);
                    commutation_state = 4;
                }
                _ => {
                    /* Default */
                    pwm_uh.set_timestamp(0);
                    pwm_vh.set_timestamp(0);
                    pwm_wh.set_timestamp(0);
                    pwm_ul.set_timestamp(0);
                    pwm_vl.set_timestamp(0);
                    pwm_wl.set_timestamp(0);
                }
            }
        }

        /* Sample magnetic encoder positions and load specific commutation state */
        let angle = magnetic_sensor.read_angle().unwrap();
        // let commutation_state = ((angle / STEP_SIZE) as i32) % 6;

        /* Print data serially */
        info!(
            "State: {}, Angle: {}, Speed: {}",
            commutation_state as u8, angle, speed
        );
        // delay.delay_millis(5);
    }
}
