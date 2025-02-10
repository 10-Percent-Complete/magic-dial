#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    main, Config,
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

    /* Initialize i2c0 controller in EPS32 */
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

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
        /* Determine if magnetic sensor I2C communication is working based on successful
         * manufacturer ID. Sample and print out readings as a result */
        if magnetic_sensor.read_manufacturer_id() == Ok(0x5449_u16) {
            let x = magnetic_sensor.read_x().unwrap();
            let y = magnetic_sensor.read_y().unwrap();
            let angle = magnetic_sensor.read_angle().unwrap();

            info!("X: {}, Y: {}, Angle: {}", x, y, angle);
        }

        delay.delay_millis(50);
    }
}
