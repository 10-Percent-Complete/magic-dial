#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    Config,
    main,
    i2c::master::{I2c, Config as I2cConfig}
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

    /* Initialize i2c controller in EPS32 */
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default()
        )
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

   /* Initialize encoder driver object */
    let mut encoder = drivers::tmag5273::Tmag5273::new(i2c);

    /* I2C address for hall encoder IC */
    // let encoder_addr = 0x22;

    /* Variables to store raw encoder register readings */
    // let mut x_msb = [0u8; 1];
    // let mut x_lsb = [0u8; 1];
    // let mut y_msb = [0u8; 1];
    // let mut y_lsb = [0u8; 1];
    // let mut z_msb = [0u8; 1];
    // let mut z_lsb = [0u8; 1];
    //

    /* @todo Tmag encoder settings for fast conversion
     *
     * Device config 1 to 1
     * Sensor config 1 to 0x79
     * T config to 1
     * Int config to 0xA4
     * Device config 2 to 0x22
     * Wait for INT signal to assert low for conversion completion. At this point read T,X,Y,Z with
     * a single read command 
     */

    /* Enable x,y,z measurement registers */
    // i2c.write(encoder_addr, &[0x02, 0x79]).ok();

    /* Enable the largest gain for each axis */
    // i2c.write(encoder_addr, &[0x03, 0x03]).ok();


    loop {
        if let Ok(id_value) =  encoder.read_device_id() {
            info!("ID: 0x{:X}", id_value);
        }
        // i2c.write_read(encoder_addr, &[0x0D], &mut id_value).ok();


        /* Read x position register */
        // i2c.write_read(encoder_addr, &[0x12], &mut x_msb).ok();
        // i2c.write_read(encoder_addr, &[0x13], &mut x_lsb).ok();

        /* Read y position register */
        // i2c.write_read(encoder_addr, &[0x14], &mut y_msb).ok();
        // i2c.write_read(encoder_addr, &[0x15], &mut y_lsb).ok();

        /* Read z position register */
        // i2c.write_read(encoder_addr, &[0x16], &mut z_msb).ok();
        // i2c.write_read(encoder_addr, &[0x17], &mut z_lsb).ok();

        /* Combine both readings into single 16 bit value */
        // let x_value = (x_msb[0] as u16) << 8 | (x_lsb[0] as u16);
        // let y_value = (y_msb[0] as u16) << 8 | (y_lsb[0] as u16);
        // let z_value = (z_msb[0] as u16) << 8 | (z_lsb[0] as u16);

        /* Report readings */
        // info!("ID: 0x{:X}, X: 0x{:X}, Y: 0x{:X}, Z: 0x{:X}", id_value[0], x_value, y_value, z_value);
        delay.delay_millis(100);
    }
}
