use core::f32;
use core::f32::consts::FRAC_PI_3;

use embedded_hal::pwm::SetDutyCycle;
use esp_hal::i2c::master::I2c;
use esp_hal::mcpwm::operator::DeadTimeCfg;
use esp_hal::time::{now, ExtU64};
use esp_hal::{sha, Blocking};
use libm::{fmodf, sinf};
use log::info;

use f32::consts::{FRAC_PI_6, PI};

use crate::motor::Motor;
use crate::pwm::{PwmPhases, PERIOD};
use crate::Tmag5273;

pub enum ControllerType {
    velocity_open_loop = 0x0,
    angle_open_loop = 0x1,
    velocity = 0x2,
    angle = 0x3,
    torque = 0x4,
}

pub struct Control<'a> {
    phases: PwmPhases<'a>,
    motor: Motor,
    position_sensor: Tmag5273<I2c<'static, Blocking>>,
    controller_type: ControllerType,
}

impl<'a> Control<'a> {
    pub fn new(
        phases: PwmPhases<'a>,
        motor: Motor,
        position_sensor: Tmag5273<I2c<'static, Blocking>>,
        controller_type: ControllerType,
    ) -> Self {
        Self {
            phases,
            motor,
            position_sensor,
            controller_type,
        }
    }
    pub fn exec(&mut self, target: f32) {
        const SQRT_3: f32 = 1.732050807568877293527446341505872367_f32;

        /* Get position */
        let angle = self.position_sensor.read_angle().unwrap();
        let rad_angle = angle * (PI / 180.0);
        let sector = (((6.0 * (rad_angle / (2.0 * PI))) as i8) % 6) + 1;

        let vector_a = SQRT_3 * sinf(sector as f32 * FRAC_PI_3 - target);
        let vector_b: f32 = SQRT_3 * sinf(target - (FRAC_PI_3 * (sector as f32 - 1.0)));
        let period_0: f32 = 1.0 - vector_a - vector_b;

        let mut phase_u: f32 = 0.0;
        let mut phase_v: f32 = 0.0;
        let mut phase_w: f32 = 0.0;

        match sector {
            1 => {
                phase_u = vector_a + vector_b + (period_0 / 2.0);
                phase_v = vector_b + (period_0 / 2.0);
                phase_w = period_0 / 2.0;
            }
            2 => {
                phase_u = vector_a + (period_0 / 2.0);
                phase_v = vector_a + vector_b + (period_0 / 2.0);
                phase_w = period_0 / 2.0;
            }
            3 => {
                phase_u = period_0 + 2.0;
                phase_v = vector_a + vector_b + (period_0 / 2.0);
                phase_w = vector_b + (period_0 / 2.0);
            }
            4 => {
                phase_u = period_0 / 2.0;
                phase_v = vector_a + (period_0 / 2.0);
                phase_w = vector_a + vector_b + (period_0 / 2.0);
            }
            5 => {
                phase_u = vector_b + (period_0 / 2.0);
                phase_v = period_0 / 2.0;
                phase_w = vector_a + vector_b + (period_0 / 2.0);
            }
            6 => {
                phase_u = vector_a + vector_b + (period_0 / 2.0);
                phase_v = period_0 / 2.0;
                phase_w = vector_b + (period_0 / 2.0);
            }
            /* Disable all pins */
            _ => {
                phase_u = 0.0;
                phase_v = 0.0;
                phase_w = 0.0;
            }
        }

        // self.phases.set_phase((phase_u) as u16, (phase_v) as u16, (phase_w) as u16);

        self.phases.u.set_timestamp_a((phase_u * PERIOD as f32) as u16);
        self.phases.v.set_timestamp_a((phase_v * PERIOD as f32) as u16);
        self.phases.w.set_timestamp_a((phase_w * PERIOD as f32) as u16);

        info!("Sector:{} U:{} V:{} W:{}", sector, phase_u, phase_v, phase_w);
    }
}
