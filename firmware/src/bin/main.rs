#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    prelude::*,
    spi::{
        master::{Spi, Config},
        SpiMode}
};
use log::info;
use ws2812_spi::Ws2812;
use smart_leds::{SmartLedsWrite, RGB8};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_println::logger::init_logger_from_env();
    let delay = Delay::new();

    let mosi = peripherals.GPIO2;
    let rgb_spi = Spi::new_with_config(
        peripherals.SPI2,
        Config {
            frequency: 3u32.MHz(),
            mode: SpiMode::Mode0,
            ..Config::default()
        },
    ).with_mosi(mosi);

    let mut rgb_led = Ws2812::new(rgb_spi);
    let on_led: [RGB8; 1] = [RGB8{r:0x10, g:0x10, b:0x10}];
    let off_led: [RGB8; 1] = [RGB8{r:0, g:0, b:0}];

    loop {
        info!("On");
        rgb_led.write(on_led.iter().cloned()).unwrap();
        delay.delay(500.millis());
        info!("Off");
        rgb_led.write(off_led.iter().cloned()).unwrap();
        delay.delay(500.millis());
    }
}
