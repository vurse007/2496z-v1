use crate::util::Constants;
use crate::util::Timer;

pub struct PID {
    pub general: Constants,
    pub refined: Constants,

    pub refined_range: f64,
    pub settle_range: f64,

    pub dt: u64, //loop time in milliseconds

    pub current: f64,
    pub target: f64,

    pub error: f64,
    pub prev_error: f64,
    pub integral: f64,
    pub derivative: f64,

    pub slew: f64,
    pub prev_speed: f64,
    pub speed: f64,

    pub integral_threshold: f64,
    pub max_integral: f64,
    pub deadband: f64,
    pub settle_time: u64, //milliseconds

    pub settle_timer: Timer,
}

impl PID {
    pub fn new(general: Constants, refined: Constants, rr: f64, sr: f64, sv: f64, it: f64, mi: f64, db: f64, st: u64) -> Self {
        Self {
            general,
            refined,
            refined_range: rr,
            settle_range: sr,
            dt: 10,
            current: 0.0,
            target: 0.0,
            error: 0.0,
            prev_error: 0.0,
            integral: 0.0,
            derivative: 0.0,
            slew: sv,
            prev_speed: 0.0,
            speed: 0.0,
            integral_threshold: it,
            max_integral: mi,
            deadband: db,
            settle_time: st,
            settle_timer: Timer::new(st),
        }
    }

    pub fn calculate(&mut self, target: f64, current: f64, scale: f64) -> f64 {
        self.target = target;
        self.current = current;

        self.error = self.target - self.current;

        if self.error.abs() < self.deadband {
            self.error = 0.0;
        }

        let dt = self.dt as f64;

        if self.error.abs() < self.integral_threshold {
            self.integral += ((self.error + self.prev_error) / 2.0) * dt;
            self.integral = self.integral.clamp(-self.max_integral, self.max_integral);
        } //no else -> freeze (not reset) integral when out of range

        self.derivative = (self.error - self.prev_error) / dt;

        //pick a set of constants
        let c = if self.error.abs() < self.refined_range {
            &self.refined
        } else {
            &self.general
        };

        //max speed is 12V
        let raw_speed = (scale * (c.kp * self.error + c.ki * self.integral + c.kd * self.derivative)).clamp(-12.0, 12.0);

        //slew
        let delta = raw_speed - self.prev_speed;
        if delta > self.slew {
            self.speed = self.prev_speed + self.slew;
        } else if delta < -self.slew {
            self.speed = self.prev_speed - self.slew;
        } else {
            self.speed = raw_speed;
        }

        self.prev_error = self.error;
        self.prev_speed = self.speed;

        self.speed //return speed
    }

    pub fn reset(&mut self) {
        self.error       = 0.0;
        self.prev_error  = 0.0;
        self.integral    = 0.0;
        self.derivative  = 0.0;
        self.prev_speed  = 0.0;
        self.speed       = 0.0;
        self.settle_timer.stop();
        self.settle_timer.reset();
    }
}
