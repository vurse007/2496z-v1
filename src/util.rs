use std::time::Instant;

pub struct Constants {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

impl Constants {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self { kp, ki, kd }
    }
}

pub struct Timer {
    pub start_time: Instant,
    pub target_time: u64, //milliseconds
    pub running: bool,
}

impl Timer {
    pub fn new(target_time: u64) -> Self {
        Self {
            start_time: Instant::now(),
            target_time,
            running: false,
        }
    }

    pub fn start(&mut self, length: Option<u64>) { //timer.start(Some(1000)); or timer.start(None) to keep the current target
        self.start_time = Instant::now();
        if let Some(length) = length {
            self.target_time = length;
        }
        self.running = true;
    }

    pub fn elapsed(&self) -> u64 { //milliseconds
        self.start_time.elapsed().as_millis() as u64 //btw a line without a semicolon automatically returns the value
    }

    pub fn reset(&mut self) {
        self.start_time = Instant::now();
    }

    pub fn stop(&mut self) {
        self.running = false;
    }

    pub fn is_running(&self) -> bool {
        self.running
    }

    pub fn is_done(&self) -> bool {
        self.running && self.elapsed() >= self.target_time
    }
}

//shortest signed angle from current to target, in [-180, 180)
pub fn get_abs_rot_err(target: f64, current: f64) -> f64 {
    (target - current + 180.0).rem_euclid(360.0) - 180.0
}

//exponential drive curve -> less sensitive near the center of the stick, still reaches full power at the edge
//input / output are -1.0 to 1.0. t is the curve strength: 0 = steepest curve, bigger t = closer to linear (the graph's slider, 0-10)
//same as y = (e^(-t/10) + e^((|x|-127)/10) * (1 - e^(-t/10))) * x with x in -127..127, just scaled to -1..1
pub fn drive_curve(input: f64, t: f64) -> f64 {
    let x = input.clamp(-1.0, 1.0) * 127.0;
    let a = (-t / 10.0).exp();
    let y = (a + ((x.abs() - 127.0) / 10.0).exp() * (1.0 - a)) * x;
    y / 127.0
}
