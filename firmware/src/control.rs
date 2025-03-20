use core::f32;

use embedded_hal::pwm::SetDutyCycle;
use esp_hal::i2c::master::I2c;
use esp_hal::mcpwm::operator::DeadTimeCfg;
use esp_hal::time::{now, ExtU64};
use esp_hal::{sha, Blocking};
use libm::fmodf;
use log::info;

use f32::consts::{FRAC_PI_6, PI};

use crate::motor::Motor;
use crate::pwm::{PwmPhases, PERIOD};
use crate::Tmag5273;

const TRAP_120_MAP: [[i16; 3]; 6] = [
    [-1, 0, 1],
    [-1, 1, 0],
    [0, 1, -1],
    [1, 0, -1],
    [1, -1, 0],
    [0, -1, 1],
];

const TRAP_150_MAP: [[i16; 3]; 12] = [
    [-1, 0, 1],
    [-1, 1, 1],
    [-1, 1, 0],
    [-1, 1, -1],
    [0, 1, -1],
    [1, 1, -1],
    [1, 0, -1],
    [1, -1, -1],
    [1, -1, 0],
    [1, -1, 1],
    [0, -1, 1],
    [-1, -1, 1],
];

pub enum CommutationMode {
    Trapezoid120 = 0x0,
    Trapezoid150 = 0x1,
    SinePwm = 0x2,
    SpaceVectorPWM = 0x3,
}

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
    commutation_mode: CommutationMode,
    controller_type: ControllerType,
}

impl<'a> Control<'a> {
    pub fn new(
        phases: PwmPhases<'a>,
        motor: Motor,
        position_sensor: Tmag5273<I2c<'static, Blocking>>,
        commutation_mode: CommutationMode,
        controller_type: ControllerType,
    ) -> Self {
        Self {
            phases,
            motor,
            position_sensor,
            commutation_mode,
            controller_type,
        }
    }
    pub fn exec(&mut self, target: f32) {
        /* Get position */
        // let angle = self.position.read_angle().unwrap();
        // let mut rad_angle = angle * (PI / 180.0);

        let mut Uq = 0.0;
        let mut Ud: f32 = 0.0;

        let mut Uu: f32 = 0.0;
        let mut Uv: f32 = 0.0;
        let mut Uw: f32 = 0.0;

        // let sector = ((6.0 * ((rad_angle + _PI_6) / _2PI)) as i8) % 6;

        /* Determine controller method */
        match self.controller_type {
            ControllerType::velocity_open_loop => {
                /* Determine how to get to target velocity */
                // let time_delta = now().ticks() - self.motor.open_loop_timestamp;

                // let mut shaft_angle = self.motor.shaft_angle;

                // shaft_angle += target * (time_delta as f32);
                // shaft_angle = fmodf(shaft_angle, 2.0 * PI);
                // if shaft_angle.is_sign_negative() {
                //     shaft_angle += 2.0 * PI;
                // }

                // self.motor.shaft_angle = shaft_angle;
                // self.motor.shaft_velocity = target;
                // Uq = self.motor.voltage_limit;
                // Ud = 0.0;
            }
            ControllerType::angle_open_loop => { /* Determine how to get to target angle */ }
            ControllerType::torque => { /* Determine how to get to target torque */ }
            ControllerType::velocity => { /* Determine how to get to target velocity */ }
            ControllerType::angle => { /* Determine how to get to target angle */ }
        }

        match self.commutation_mode {
            CommutationMode::Trapezoid120 => {
                /* Perform 120 degree trapezoidal commutation */
                // let mut shaft_angle = self.motor.shaft_angle * self.motor.pole_pairs as f32;

                // let mut shaft_angle = fmodf(self.motor.shaft_angle + FRAC_PI_6, 2.0 * PI);
                // if shaft_angle.is_sign_negative() {
                //     shaft_angle += 2.0 * PI;
                // }

                // self.motor.shaft_angle = shaft_angle;

                // let shaft_angle = self.position_sensor.read_angle().unwrap() * (PI / 180.0);
                // let sector = ((6.0 * (shaft_angle / (2.0 * PI))) as u8) % 6;
                let sector = 0;

                // let center = self.motor.voltage_limit / 2.0;

                // Uu = Uq * TRAP_120_MAP[sector as usize][0] as f32 + center;
                // Uv = Uq * TRAP_120_MAP[sector as usize][1] as f32 + center;
                // Uw = Uq * TRAP_120_MAP[sector as usize][2] as f32 + center;

                // Uu = (Uu.clamp(0.0, self.motor.voltage_limit) / self.motor.power_supply_voltage)
                //     .clamp(0.0, 1.0)
                //     * 4999.0;
                // Uv = (Uv.clamp(0.0, self.motor.voltage_limit) / self.motor.power_supply_voltage)
                //     .clamp(0.0, 1.0)
                //     * 4999.0;
                // Uw = (Uw.clamp(0.0, self.motor.voltage_limit) / self.motor.power_supply_voltage)
                //     .clamp(0.0, 1.0)
                //     * 4999.0;

                match sector {
                    /* WH -> UL */
                    0 => {
                        self.phases.u.set_deadtime_cfg(DeadTimeCfg::new_ahc());
                        self.phases.v.set_deadtime_cfg(DeadTimeCfg::new_bypass());
                        self.phases.w.set_deadtime_cfg(DeadTimeCfg::new_ahc());

                        self.phases.u.set_timestamp_a(0);

                        self.phases.v.set_timestamp_a(0);
                        self.phases.v.set_timestamp_b(0);

                        self.phases.w.set_timestamp_a(target as u16);
                    }
                    /* VH -> UL */
                    1 => {
                        self.phases.u.set_deadtime_cfg(DeadTimeCfg::new_ahc());
                        self.phases.v.set_deadtime_cfg(DeadTimeCfg::new_ahc());
                        self.phases.w.set_deadtime_cfg(DeadTimeCfg::new_bypass());

                        self.phases.u.set_timestamp_a(0);

                        self.phases.v.set_timestamp_a(target as u16);

                        self.phases.w.set_timestamp_a(0);
                        self.phases.w.set_timestamp_b(0);
                    }
                    /* VH -> WL */
                    2 => {
                        self.phases.u.set_deadtime_cfg(DeadTimeCfg::new_bypass());
                        self.phases.v.set_deadtime_cfg(DeadTimeCfg::new_ahc());
                        self.phases.w.set_deadtime_cfg(DeadTimeCfg::new_ahc());

                        self.phases.u.set_timestamp_a(0);
                        self.phases.u.set_timestamp_b(0);

                        self.phases.v.set_timestamp_a(target as u16);

                        self.phases.w.set_timestamp_a(0);
                    }
                    /* UH -> WL */
                    3 => {
                        self.phases.u.set_deadtime_cfg(DeadTimeCfg::new_ahc());
                        self.phases.v.set_deadtime_cfg(DeadTimeCfg::new_bypass());
                        self.phases.w.set_deadtime_cfg(DeadTimeCfg::new_ahc());

                        self.phases.u.set_timestamp_a(target as u16);

                        self.phases.v.set_timestamp_a(0);
                        self.phases.v.set_timestamp_b(0);

                        self.phases.w.set_timestamp_a(0);
                    }
                    /* UH -> VL */
                    4 => {
                        self.phases.u.set_deadtime_cfg(DeadTimeCfg::new_ahc());
                        self.phases.v.set_deadtime_cfg(DeadTimeCfg::new_ahc());
                        self.phases.w.set_deadtime_cfg(DeadTimeCfg::new_bypass());

                        self.phases.u.set_timestamp_a(target as u16);

                        self.phases.v.set_timestamp_a(0);

                        self.phases.w.set_timestamp_a(0);
                        self.phases.w.set_timestamp_b(0);
                    }
                    /* WH -> VL */
                    5 => {
                        self.phases.u.set_deadtime_cfg(DeadTimeCfg::new_bypass());
                        self.phases.v.set_deadtime_cfg(DeadTimeCfg::new_ahc());
                        self.phases.w.set_deadtime_cfg(DeadTimeCfg::new_ahc());

                        self.phases.u.set_timestamp_a(0);
                        self.phases.u.set_timestamp_b(0);

                        self.phases.v.set_timestamp_a(0);

                        self.phases.w.set_timestamp_a(target as u16);
                    }
                    /* Disable all pins */
                    _ => {
                        self.phases.u.set_deadtime_cfg(DeadTimeCfg::new_bypass());
                        self.phases.v.set_deadtime_cfg(DeadTimeCfg::new_bypass());
                        self.phases.w.set_deadtime_cfg(DeadTimeCfg::new_bypass());

                        self.phases.u.set_timestamp_a(0);
                        self.phases.u.set_timestamp_b(0);

                        self.phases.v.set_timestamp_a(0);
                        self.phases.v.set_timestamp_b(0);

                        self.phases.w.set_timestamp_a(0);
                        self.phases.w.set_timestamp_b(0);
                    }
                }

                // self.phases.set_phase((Uu) as u16, (Uv) as u16, (Uw) as u16);

                // info!("{}", sector);
            }
            CommutationMode::Trapezoid150 => { /* Perform 150 degree trapezoidal commutation */ }
            CommutationMode::SinePwm => { /* Perform sine commutation */ }
            CommutationMode::SpaceVectorPWM => { /* Perform space vector commutation */ }
        }

        // self.motor.open_loop_timestamp = now().ticks();
    }
}
