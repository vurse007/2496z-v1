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

    pub d_filter: f64, //0.0 = raw derivative, closer to 1.0 = smoother (but laggier) -> stops sensor noise from making the motors buzz
    pub first_run: bool, //true right after reset() -> skips the derivative on the first loop so the start doesn't get a kick
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
            d_filter: 0.6,
            first_run: true,
        }
    }

    pub fn calculate(&mut self, target: f64, current: f64, scale: f64) -> f64 {
        self.target = target;
        self.current = current;

        self.error = self.target - self.current;

        //deadband only zeroes the output (further down), it doesn't overwrite the error
        //-> overwriting it made the error jump to 0 at the edge, which the derivative saw as a big kick -> buzzing at the target
        let in_deadband = self.error.abs() < self.deadband;

        if self.first_run {
            self.prev_error = self.error; //no fake jump from 0 to the full error on the first loop
            self.first_run = false;
        }

        let dt = self.dt as f64;

        if self.error.abs() < self.integral_threshold {
            self.integral += ((self.error + self.prev_error) / 2.0) * dt;
            self.integral = self.integral.clamp(-self.max_integral, self.max_integral);
        } //no else -> freeze (not reset) integral when out of range

        //low-pass filtered derivative -> the sensors don't update exactly in sync with the 10ms loop,
        //so the raw difference jumps between 0 and double, and with a big kd that turns into vibration
        let raw_derivative = (self.error - self.prev_error) / dt;
        self.derivative = self.d_filter * self.derivative + (1.0 - self.d_filter) * raw_derivative;

        //pick a set of constants
        let c = if self.error.abs() < self.refined_range {
            &self.refined
        } else {
            &self.general
        };

        //scale is a speed limit, 0.0-1.0 like a percent of max speed (12V) -> 0.5 = never more than 6V, gains stay the same
        let max_speed = 12.0 * scale.clamp(0.0, 1.0);
        let raw_speed = if in_deadband {
            0.0
        } else {
            (c.kp * self.error + c.ki * self.integral + c.kd * self.derivative).clamp(-max_speed, max_speed)
        };

        //slew -> only limits speeding up (smooth start, less wheel slip)
        //slowing down / reversing is never limited, otherwise the pid couldn't brake in time and would overshoot
        let limit = if self.prev_speed == 0.0 {
            self.slew //starting from a stop -> ramp up from 0
        } else if raw_speed.signum() == self.prev_speed.signum() {
            self.prev_speed.abs() + self.slew //same direction -> can only get faster by slew per loop
        } else {
            12.0 //flipping direction = braking -> never limited
        };
        self.speed = raw_speed.clamp(-limit, limit);

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
        self.first_run   = true;
        self.settle_timer.stop();
        self.settle_timer.reset();
    }
}
