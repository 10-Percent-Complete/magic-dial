use embedded_hal::pwm::SetDutyCycle;
use esp_hal::gpio::GpioPin;
use esp_hal::mcpwm::operator::{DeadTimeCfg, LinkedPins, Operator, PwmPin};
use esp_hal::mcpwm::{
    operator::{PwmPinConfig, PwmUpdateMethod},
    timer::PwmWorkingMode,
    McPwm, PeripheralClockConfig,
};
use esp_hal::peripherals::MCPWM0;
use esp_hal::time::RateExtU32;

pub struct PwmPhases<'a> {
    u: LinkedPins<'a, MCPWM0, 0>,
    v: LinkedPins<'a, MCPWM0, 1>,
    w: LinkedPins<'a, MCPWM0, 2>,
}

impl PwmPhases<'_> {
    pub fn new(
        mcpwm0: MCPWM0,
        uh: GpioPin<16>,
        ul: GpioPin<17>,
        vh: GpioPin<18>,
        vl: GpioPin<23>,
        wh: GpioPin<19>,
        wl: GpioPin<33>,
    ) -> Self {
        let pwm_clock_cfg = PeripheralClockConfig::with_frequency(160_u32.MHz()).unwrap();
        let mut mcpwm = McPwm::new(mcpwm0, pwm_clock_cfg);

        let mut op0 = mcpwm.operator0;
        let mut op1 = mcpwm.operator1;
        let mut op2 = mcpwm.operator2;

        op0.set_timer(&mcpwm.timer0);
        op1.set_timer(&mcpwm.timer0);
        op2.set_timer(&mcpwm.timer0);

        let pwm_u = op0.with_linked_pins(
            uh,
            PwmPinConfig::UP_ACTIVE_HIGH,
            ul,
            PwmPinConfig::EMPTY,
            DeadTimeCfg::new_ahc()
        );
        let pwm_v = op1.with_linked_pins(
            vh,
            PwmPinConfig::UP_ACTIVE_HIGH,
            vl,
            PwmPinConfig::EMPTY,
            DeadTimeCfg::new_ahc()
        );
        let pwm_w = op2.with_linked_pins(
            wh,
            PwmPinConfig::UP_ACTIVE_HIGH,
            wl,
            PwmPinConfig::EMPTY,
            DeadTimeCfg::new_ahc()
        );

        mcpwm.timer0.start(
            pwm_clock_cfg
                .timer_clock_with_frequency(
                    7999,
                    PwmWorkingMode::Increase,
                    20_u32.kHz(),
                )
                .unwrap(),
        );

        Self {
            u: pwm_u,
            v: pwm_v,
            w: pwm_w,
        }
    }

    pub fn set_phase(&mut self, u_dc: u16, v_dc: u16, w_dc: u16) {

        self.u.set_timestamp_a(u_dc);
        self.v.set_timestamp_a(v_dc);
        self.w.set_timestamp_a(w_dc);
    }
}
