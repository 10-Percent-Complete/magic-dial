pub struct Motor {
    pole_pairs: u8,
    phase_resistance: f32,
    kv_rating: f32,
    phase_inductance: f32,

    voltage_limit: f32,
    current_limit: f32,
    velocity_limit: f32,
    
    status: Status,

    enabled: bool,
    target: f32,

    shaft_angle: f32,
    shaft_velocity: f32,

    shaft_angle_target: f32,
    shaft_velocity_target: f32,

    d_voltage: f32,
    q_voltage: f32,
    d_current: f32,
    q_current: f32,
}

pub enum Status {
  uninitialized = 0x0,
  uncalibrated = 0x1,
  ready = 0x2,
  error = 0x3
}

impl Motor {
    pub fn new(
        pole_pairs: u8,
        phase_resistance: f32,
        kv_rating: f32,
        phase_inductance: f32,
        voltage_limit: f32,
        current_limit: f32,
        velocity_limit: f32
    ) -> Self {


        Self {
            pole_pairs,
            phase_resistance,
            kv_rating,
            phase_inductance,
            voltage_limit,
            current_limit,
            velocity_limit,
            enabled: false,
            status: Status::uninitialized,
            target: 0.0,
            shaft_angle: 0.0,
            shaft_velocity: 0.0,
            shaft_angle_target: 0.0,
            shaft_velocity_target: 0.0,
            d_voltage: 0.0,
            q_voltage: 0.0,
            d_current: 0.0,
            q_current: 0.0,
        }
    }
}
