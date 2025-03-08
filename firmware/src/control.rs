use core::f32;

use esp_hal::i2c::master::I2c;
use esp_hal::Blocking;
use libm::fmodf;

use f32::consts::{FRAC_PI_6, PI};

use crate::motor::Motor;
use crate::pwm::PwmPhases;
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
    [-1, 1,-1],
    [0, 1, -1],
    [1, 1, -1],
    [1, 0, -1],
    [1, -1,-1],
    [1, -1, 0],
    [1, -1, 1],
    [0, -1, 1],
    [-1,-1, 1],
];

pub enum CommutationMode {
  Trapezoid120 = 0x0,
  Trapezoid150 = 0x1,
  SinePwm = 0x2,
  SpaceVectorPWM = 0x3
}

pub enum ControllerType {
  velocity_open_loop = 0x0,
  angle_open_loop = 0x1,
  velocity = 0x2,
  angle = 0x3,
  torque = 0x4
}

pub struct Control<'a> {
  phases: PwmPhases<'a>,
  motor: Motor,
  position_sensor: Tmag5273<I2c<'static, Blocking>>,
  commutation_mode: CommutationMode,
  controller_type: ControllerType,
}

impl <'a>Control<'a> {
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
      controller_type
    }
  }
  pub fn exec(&mut self, sector: u8) {
      /* Get position */
      // let angle = self.position.read_angle().unwrap();
      let angle = 0.0;
      let mut rad_angle = angle * (PI / 180.0);
      rad_angle = fmodf(rad_angle, 2.0 * PI);
      if rad_angle.is_sign_negative() {
        rad_angle += 2.0 * PI;
      }
      // let sector = ((6.0 * ((rad_angle + _PI_6) / _2PI)) as i8) % 6;

      /* Determine controller method */
      match self.controller_type {
          ControllerType::velocity_open_loop => {
            /* Determine how to get to target velocity */
          },
          ControllerType::angle_open_loop => {
            /* Determine how to get to target angle */
          },
          ControllerType::torque => {
            /* Determine how to get to target torque */
          }
          ControllerType::velocity => {
            /* Determine how to get to target velocity */
          }
          ControllerType::angle => {
            /* Determine how to get to target angle */
          }
      }
  
      match self.commutation_mode {
        CommutationMode::Trapezoid120 => {
          /* Perform 120 degree trapezoidal commutation */
          self.phases.set_phase(
            (TRAP_120_MAP[sector as usize][0] * 4000) as u16,
            (TRAP_120_MAP[sector as usize][1] * 4000) as u16,
            (TRAP_120_MAP[sector as usize][2] * 4000) as u16,
          );
        }
        CommutationMode::Trapezoid150 => {
          /* Perform 150 degree trapezoidal commutation */
          self.phases.set_phase(
            (TRAP_150_MAP[sector as usize][0] * 4000) as u16,
            (TRAP_150_MAP[sector as usize][1] * 4000) as u16,
            (TRAP_150_MAP[sector as usize][2] * 4000) as u16,
          );
        }
        CommutationMode::SinePwm => {
          /* Perform sine commutation */
        },
        CommutationMode::SpaceVectorPWM => {
          /* Perform space vector commutation */
        },
      }
    }
}
