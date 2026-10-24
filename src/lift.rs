use std::time::Duration;

use vexide::prelude::RotationSensor;
use vexide::smart::motor::BrakeMode;
use vexide::time::sleep;

use crate::global::MotorGroup;
use crate::pid::PID;
use crate::util::{Constants, Timer};

//constant volts always added to the pid output to fight gravity -> raise it until the lift holds still at mid height with kp = 0
const LIFT_KG: f64 = 0.0;

pub struct Lift {
    pub motors: MotorGroup,
    pub rotation: RotationSensor,
    pub pid: PID,
    pub target: f64, //degrees on the rotation sensor
}

impl Lift {
    pub fn new(mut motors: MotorGroup, mut rotation: RotationSensor) -> Self {
        motors.set_brake_mode(BrakeMode::Coast); //hold so the lift stays in place when no button is pressed

        //0 = wherever the lift is when the program starts -> start every match with the lift all the way down
        let _ = rotation.reset_position();
        let _ = rotation.set_data_interval(RotationSensor::MIN_DATA_INTERVAL); //5ms updates -> fresher readings than the 10ms loop

        //all pid values below are placeholders -> tune them
        //PID::new(general, refined, refined_range, settle_range, slew, integral_threshold, max_integral, deadband, settle_time_ms)
        let pid = PID::new(Constants::new(0.2, 0.0, 0.0), Constants::new(0.2, 0.0, 0.0), 0.0, 2.0, 12.0, 0.0, 0.0, 0.0, 200);

        Self { motors, rotation, pid, target: 0.0 }
    }

    //lift angle in degrees, None if the rotation sensor is unplugged
    //position() keeps counting past 360 -> use it instead of angle() so the error never wraps
    pub fn get_pos(&self) -> Option<f64> {
        self.rotation.position().ok().map(|p| p.as_degrees())
    }

    //changes where update() drives the lift to -> doesn't move anything by itself
    pub fn set_target(&mut self, target: f64) {
        if target != self.target {
            self.pid.reset(); //new target -> clear old integral / derivative
        }
        self.target = target;
    }

    //one pid step toward self.target -> call this every loop (driver) to hold or move the lift without blocking
    pub fn update(&mut self, scale: f64) {
        let Some(current) = self.get_pos() else {
            self.motors.brake(); //sensor unplugged -> stop instead of driving blind
            return;
        };

        let speed = self.pid.calculate(self.target, current, scale);
        self.motors.set_voltage((speed + LIFT_KG).clamp(-12.0, 12.0));
    }

    //moves the lift to target and waits until it settles (or the timeout runs out) -> for auton
    //the motors keep holding the target afterwards (no brake) so the lift doesn't fall
    pub async fn move_to(&mut self, target: f64, timeout: u64, scale: f64) {
        self.pid.reset();
        self.target = target;

        let mut safety_timer = Timer::new(timeout);
        safety_timer.start(None);

        loop {
            if self.get_pos().is_none() {
                break; //rotation sensor unplugged -> bail out
            }

            self.update(scale);

            //settling conditions
            if self.pid.error.abs() < self.pid.settle_range {
                if !self.pid.settle_timer.is_running() {
                    self.pid.settle_timer.start(None);
                }
            } else if self.pid.settle_timer.is_running() {
                self.pid.settle_timer.stop();
            }

            if self.pid.settle_timer.is_running() && self.pid.settle_timer.elapsed() > self.pid.settle_time {
                break;
            }

            if safety_timer.is_done() {
                break;
            }

            sleep(Duration::from_millis(self.pid.dt)).await;
        }
    }
}
