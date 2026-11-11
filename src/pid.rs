use std::time::Instant;

use crate::util::Constants;
use crate::util::Timer;

// Units, so the gains mean the same thing everywhere and match textbooks:
//   kp: volts per unit of error          (e.g. 0.6 V/deg -> full 12V at 20 deg error)
//   ki: volts per (unit * second)        (integral is now error * seconds, not error * ms)
//   kd: volts per (unit / second)        (derivative is now deg/s, not deg/ms)
//   slew: volts per second               (e.g. 300 V/s -> 0 to 12V in 40ms, same as 3.0 V/loop before)
//   max_integral: unit * seconds         (e.g. 100 deg*s)
// Full-power error for a loop is just 12 / kp, so you can tune by
// "how close before it starts slowing down" instead of by raw kp.
pub struct PID {
    pub general: Constants,
    pub refined: Constants,

    pub refined_range: f64,
    pub settle_range: f64,

    pub dt: u64, //nominal loop time in milliseconds (fallback on the first loop)

    pub current: f64,
    pub prev_current: f64,
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
    pub last_time: Instant,

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
            prev_current: 0.0,
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
            last_time: Instant::now(),
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

        //real dt in seconds, measured, not assumed 10ms -> sleep + work is usually 12-15ms,
        //so the old assumed dt made integral/derivative off by 20-50%. clamped so a pause
        //between loops can't make one giant integral/derivative step.
        let dt_s = if self.first_run {
            self.dt as f64 / 1000.0
        } else {
            self.last_time.elapsed().as_secs_f64().clamp(0.005, 0.05)
        };
        self.last_time = Instant::now();

        if self.first_run {
            self.prev_error = self.error; //no fake jump from 0 to the full error on the first loop
            self.prev_current = self.current;
            self.first_run = false;
        }

        if self.error.abs() < self.integral_threshold {
            self.integral += ((self.error + self.prev_error) / 2.0) * dt_s;
            self.integral = self.integral.clamp(-self.max_integral, self.max_integral);
        } //no else -> freeze (not reset) integral when out of range

        //derivative on measurement, not on error: same value when the target is
        //constant (the usual case), but no kick when the target changes.
        //low-pass filtered -> the sensors don't update exactly in sync with the 10ms loop,
        //so the raw difference jumps between 0 and double, and with a big kd that turns into vibration
        let raw_derivative = -(self.current - self.prev_current) / dt_s;
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
        //slew is now V/s, so the per-loop step is slew * dt_s
        let step = self.slew * dt_s;
        let limit = if self.prev_speed == 0.0 {
            step //starting from a stop -> ramp up from 0
        } else if raw_speed.signum() == self.prev_speed.signum() {
            self.prev_speed.abs() + step //same direction -> can only get faster by slew per loop
        } else {
            12.0 //flipping direction = braking -> never limited
        };
        self.speed = raw_speed.clamp(-limit, limit);

        self.prev_error = self.error;
        self.prev_current = self.current;
        self.prev_speed = self.speed;

        self.speed //return speed
    }

    pub fn reset(&mut self) {
        self.error       = 0.0;
        self.prev_error  = 0.0;
        self.prev_current = 0.0;
        self.integral    = 0.0;
        self.derivative  = 0.0;
        self.prev_speed  = 0.0;
        self.speed       = 0.0;
        self.first_run   = true;
        self.last_time   = Instant::now();
        self.settle_timer.stop();
        self.settle_timer.reset();
    }
}
