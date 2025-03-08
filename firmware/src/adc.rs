use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::gpio::GpioPin;
use esp_hal::peripherals::ADC1;

pub fn init(
    adc1: ADC1,
    u: GpioPin<35>,
    v: GpioPin<36>,
    w: GpioPin<39>,
    supply: GpioPin<32>,
) -> Adc<'static, ADC1> {
    let mut adc_config = AdcConfig::new();
    adc_config.enable_pin(u, Attenuation::_11dB);
    adc_config.enable_pin(v, Attenuation::_11dB);
    adc_config.enable_pin(w, Attenuation::_11dB);
    adc_config.enable_pin(supply, Attenuation::_11dB);
    Adc::new(adc1, adc_config)
}
