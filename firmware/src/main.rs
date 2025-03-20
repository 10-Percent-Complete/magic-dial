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
    delay::Delay,
    gpio::{Input, Output},
    main,
    mcpwm::timer::Timer,
    rtc_cntl::Rtc,
    time::ExtU64,
    timer::{timg::TimerGroup, PeriodicTimer},
    uart::{self, Config as UartConfig, Uart},
    Config,
};

use control::Control;
use motor::Motor;
use nb::block;
use pwm::{PwmPhases, PERIOD};

mod adc;
mod control;
mod drivers;
mod i2c;
mod motor;
mod pwm;

#[main]
fn main() -> ! {
    /* Initialize esp32 hardware */
    let peripherals = esp_hal::init(Config::default());

    /* Initialize log functionality */
    esp_println::logger::init_logger_from_env();

    /* Create delay object to use for code delays */
    let delay: Delay = Delay::new();

    /* Apply labels for all used pins */
    /* Buttons */
    let gpio_13 = peripherals.GPIO13;
    let gpio_14 = peripherals.GPIO14;

    /* UART Pins */
    let (tx_pin, rx_pin) = (peripherals.GPIO1, peripherals.GPIO3);

    let mut serial = Uart::new(peripherals.UART0, UartConfig::default())
        .unwrap()
        .with_rx(rx_pin)
        .with_tx(tx_pin);

    /* Initialize i2c controller */
    let i2c = i2c::init(
        peripherals.I2C0,
        peripherals.GPIO21, /* SDA */
        peripherals.GPIO22, /* SCL */
    );

    /* Initialize adc module */
    let adc = adc::init(
        peripherals.ADC1,
        peripherals.GPIO35, /* U Current */
        peripherals.GPIO36, /* V Current */
        peripherals.GPIO39, /* W Current */
        peripherals.GPIO32, /* Motor Current */
    );

    /* Initialize pwm module */
    let mut pwm_phases = PwmPhases::new(
        peripherals.MCPWM0,
        peripherals.GPIO16, /* U H */
        peripherals.GPIO17, /* U L */
        peripherals.GPIO18, /* V H */
        peripherals.GPIO23, /* V L */
        peripherals.GPIO19, /* W H */
        peripherals.GPIO33, /* W L */
    );

    /* Initialize motor object */
    let motor = Motor::new(4, 0.0, 0.0, 0.0, 3.3, 4.0, 4.0, 2.0);

    /* Initialize buttons */
    let btn_13 = Input::new(gpio_13, esp_hal::gpio::Pull::Up);
    let btn_14 = Input::new(gpio_14, esp_hal::gpio::Pull::Up);

    /* Initialize hall effect sensor object */
    let mut magnetic_sensor = Tmag5273::new(i2c, DEFAULT_I2C_ADDR);

    // /* Attempt to initialize the following magnetic sensor settings:
    //  * Average conversion rate: 32x
    //  * Low noise mode
    //  * Continuous sample mode
    //  * Enable XY channels
    //  * Enable angle conversion channels XY
    //  **/
    // magnetic_sensor
    //     .config_device_1(
    //         CrcEn::Off,
    //         TempCoeff::_0,
    //         ConversionRate::_32x,
    //         I2cReadMode::I2cRead3,
    //     )
    //     .unwrap();

    // magnetic_sensor
    //     .config_device_2(
    //         HystersisThreshold::TwoComplement,
    //         LowMode::LowNoise,
    //         I2cGlitchFilter::On,
    //         TriggerMode::Default,
    //         OpMode::Continuous,
    //     )
    //     .unwrap();

    // magnetic_sensor
    //     .config_sensor_1(MagneticConfig::EnableXYX, SleepConfig::Sleep1ms)
    //     .unwrap();

    // magnetic_sensor
    //     .config_sensor_2(
    //         ThresholdXCount::One,
    //         ThresholdTriggerDirection::Below,
    //         MagnitudeGainChannel::One,
    //         AngleEnable::EnableXY,
    //         XYRange::Default,
    //         ZRange::Default,
    //     )
    //     .unwrap();

    /* Initialize motor controller */
    let mut controller = Control::new(
        pwm_phases,
        motor,
        magnetic_sensor,
        control::CommutationMode::Trapezoid120,
        control::ControllerType::velocity_open_loop,
    );

    let mut target_velocity: f32 = PERIOD as f32 / 2.0;
    // let step_per_rev = 4 * 6;
    // let step_time = (60000.0 / (step_per_rev as f32)) as u32;

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut periodic = PeriodicTimer::new(timg0.timer0);
    periodic.start(1.millis()).unwrap();

    // let mut byte = [0u8; 1];

    loop {
        match periodic.wait() {
            Ok(()) => {
                controller.exec(target_velocity);
            }
            Err(nb::Error::WouldBlock) => {
                if btn_13.is_low() && target_velocity < (PERIOD as f32) {
                    target_velocity += 1.0;
                } else if btn_14.is_low() && target_velocity > 0.0 {
                    target_velocity -= 1.0;
                }
            }
        }

        /* Print data serially */
        // info!(
        //     "State: {}, Angle: {}, Speed: {}",
        //     commutation_state as u8, angle, speed
        // );
        // serial.read_buffered_bytes(&mut byte).unwrap();
        // serial.write_bytes(&byte).unwrap();
    }
}
