#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig},
    delay::Delay,
    gpio::{Level, Output},
    i2c::master::{Config as I2cConfig, I2c},
    main,
    // mcpwm::{operator::PwmPinConfig, McPwm, PeripheralClockConfig},
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
    let delay = Delay::new();

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
    let _pin_vh = peripherals.GPIO18;
    let _pin_wh = peripherals.GPIO19;

    /* Motor driver pwm low pins */
    let _pin_ul = peripherals.GPIO17;
    let pin_vl = peripherals.GPIO23;
    let pin_wl = peripherals.GPIO33;

    /* Initialize i2c0 controller in EPS32 */
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(pin_i2c_sda)
        .with_scl(pin_i2c_scl);

    /* Create adc config to use for all ADC channels */
    let mut adc_current_config = AdcConfig::new();

    /* Enable all current ADC pins */
    let mut adc_u_current_pin =
        adc_current_config.enable_pin(pin_u_current, esp_hal::analog::adc::Attenuation::_11dB);
    let mut adc_v_current_pin =
        adc_current_config.enable_pin(pin_v_current, esp_hal::analog::adc::Attenuation::_11dB);
    let mut adc_w_current_pin =
        adc_current_config.enable_pin(pin_w_current, esp_hal::analog::adc::Attenuation::_11dB);
    let mut adc_supply_current_pin =
        adc_current_config.enable_pin(pin_supply_current, esp_hal::analog::adc::Attenuation::_11dB);

    /* Initialize ADC1 module */
    let mut adc_current = Adc::new(peripherals.ADC1, adc_current_config);

    /* Initial the following commutation state [u_h, v_l, w_l] to check initial current readings */
    let mut _uh = Output::new(pin_uh, Level::High);
    let mut _vl = Output::new(pin_vl, Level::High);
    let mut _wl = Output::new(pin_wl, Level::High);

    // let pwm_clock_cfg = PeripheralClockConfig::with_frequency(32_u32.MHz()).unwrap();
    // let mut mcpwm = McPwm::new(peripherals.MCPWM0, pwm_clock_cfg);

    // let timer_clock_cfg = pwm_clock_cfg
    //     .timer_clock_with_frequency(
    //         99,
    //         esp_hal::mcpwm::timer::PwmWorkingMode::Increase,
    //         20_u32.kHz(),
    //     )
    //     .unwrap();

    // mcpwm.operator0.set_timer(&mcpwm.timer0);
    // mcpwm.operator1.set_timer(&mcpwm.timer1);
    // mcpwm.operator2.set_timer(&mcpwm.timer2);

    // let mut pwm_uh = mcpwm
    //     .operator0
    //     .with_pin_a(pin_uh, PwmPinConfig::UP_ACTIVE_HIGH);

    // let mut pwm_vh = mcpwm
    //     .operator1
    //     .with_pin_a(pin_vh, PwmPinConfig::UP_ACTIVE_HIGH);

    // let mut pwm_wh = mcpwm
    //     .operator2
    //     .with_pin_a(pin_wh, PwmPinConfig::UP_ACTIVE_HIGH);

    // let mut pwm_ul = mcpwm
    //     .operator0
    //     .with_pin_b(pin_ul, PwmPinConfig::UP_ACTIVE_HIGH);

    // let mut pwm_vl = mcpwm
    //     .operator1
    //     .with_pin_b(pin_vl, PwmPinConfig::UP_ACTIVE_HIGH);

    // let mut pwm_wl = mcpwm
    //     .operator2
    //     .with_pin_b(pin_wl, PwmPinConfig::UP_ACTIVE_HIGH);

    // mcpwm.timer0.start(timer_clock_cfg);
    // mcpwm.timer1.start(timer_clock_cfg);
    // mcpwm.timer2.start(timer_clock_cfg);

    /* Initialize hall effect sensor object */
    let mut magnetic_sensor = drivers::Tmag5273::new(i2c, drivers::tmag5273::DEFAULT_I2C_ADDR);

    /* Attempt to initialize the following magnetic sensor settings:
     * Average conversion rate: 32x
     * Continuous sample mode
     * Enable XY channels
     * Enable angle conversion channels XY
     **/
    match magnetic_sensor.config_conversion(drivers::tmag5273::ConversionRate::Average32x) {
        Ok(_) => (),
        Err(e) => info!("Conversion Configuration Error: {:?}", e),
    }
    match magnetic_sensor.config_operating_mode(drivers::tmag5273::OpMode::Continuous) {
        Ok(_) => (),
        Err(e) => info!("Operation Mode Configuration Error: {:?}", e),
    }
    match magnetic_sensor.config_magnetic_channel(drivers::tmag5273::MagneticConfig::EnableXYX) {
        Ok(_) => (),
        Err(e) => info!("Magnetic Channel Configuration Error: {:?}", e),
    }
    match magnetic_sensor.config_angle(drivers::tmag5273::AngleConfig::EnableXY) {
        Ok(_) => (),
        Err(e) => info!("Angle Channel Configuration Error: {:?}", e),
    }

    loop {
        /* Sample all ADC current readings */
        let u_current_value: u16 =
            nb::block!(adc_current.read_oneshot(&mut adc_u_current_pin)).unwrap();
        let v_current_value: u16 =
            nb::block!(adc_current.read_oneshot(&mut adc_v_current_pin)).unwrap();
        let w_current_value: u16 =
            nb::block!(adc_current.read_oneshot(&mut adc_w_current_pin)).unwrap();
        let supply_current_value: u16 =
            nb::block!(adc_current.read_oneshot(&mut adc_supply_current_pin)).unwrap();

        /* Determine if magnetic sensor I2C communication is working based on successful
         * manufacturer ID. Sample and print out readings as a result */
        if magnetic_sensor.read_manufacturer_id() == Ok(0x5449_u16) {
            let x = magnetic_sensor.read_x().unwrap();
            let y = magnetic_sensor.read_y().unwrap();
            let angle = magnetic_sensor.read_angle().unwrap();

            /* Print data serially */
            info!(
                "X: {}, Y: {}, Angle: {}, Current | U: {}, V: {}, W: {}, Supply: {}",
                x,
                y,
                angle,
                u_current_value,
                v_current_value,
                w_current_value,
                supply_current_value
            );
        }

        delay.delay_millis(1000);
    }
}
