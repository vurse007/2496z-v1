use crate::util::Constants;
use crate::util::Timer;

// Plain per-loop PID (same as the lynx C++ one) -> no dt anywhere, so what you tune is what you get:
//   kp: volts per unit of error
//   ki: volts per (unit of error summed every loop)
//   kd: volts per (unit the error changed since the last loop)
//   slew: max volts the output can rise per loop (e.g. 3.0 -> 0 to 12V in 4 loops)
//   max_integral: cap on the summed error (in error units, not volts)
// Needs the loop to run at a steady rate (the 10ms sleep) -> retune if you change pid.dt.
pub struct PID {
    pub general: Constants,
    pub refined: Constants,

    pub refined_range: f64,
    pub settle_range: f64,

    pub dt: u64, //nominal loop time in milliseconds (fallback on the first loop)

    pub current: f64,
    pub target: f64,

    pub error: f64,
    pub prev_error: f64,
    pub integral: f64,
    pub derivative: f64,

    pub slew: f64, //volts per second, not volts per loop
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
            slew: sv, //now volts per second
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
        let raw_error = self.error; //saved before the deadband zeroes it

        //deadband: tiny error counts as zero for p and i
        if self.error.abs() < self.deadband {
            self.error = 0.0;
        }

        //integral: summed every loop, clamped, frozen (not reset) outside the threshold
        if self.error.abs() < self.integral_threshold {
            self.integral += (self.error + self.prev_error) / 2.0;
            self.integral = self.integral.clamp(-self.max_integral, self.max_integral);
        }

        //derivative uses the raw error so it tapers smoothly into the deadband
        self.derivative = raw_error - self.prev_error;

        //pick a set of constants
        let c = if self.error.abs() < self.refined_range {
            &self.refined
        } else {
            &self.general
        };

        //scale is a speed limit, 0.0-1.0 like a percent of max speed (12V) -> 0.5 = never more than 6V, gains stay the same
        let max_speed = 12.0 * scale.clamp(0.0, 1.0);
        let raw_speed = c.kp * self.error + c.ki * self.integral + c.kd * self.derivative;

        //slew: output can change by at most `slew` volts per loop (both directions, like the lynx one)
        let delta = raw_speed - self.prev_speed;
        self.speed = if delta > self.slew {
            self.prev_speed + self.slew
        } else if delta < -self.slew {
            self.prev_speed - self.slew
        } else {
            raw_speed
        };
        self.speed = self.speed.clamp(-max_speed, max_speed);

        self.prev_error = raw_error; //raw, so the next derivative is right
        self.prev_speed = self.speed;

        self.speed
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
