use std::cell::RefCell;
use std::rc::Rc;

use vexide::prelude::*;
use vexide::smart::motor::BrakeMode;

//an imu that several parts of the code can hold at once (drivetrain now, odometry later)
//Rc = shared ownership (like std::shared_ptr), RefCell = lets whoever holds it borrow it mutably when needed
//rule: never hold a .borrow() / .borrow_mut() across an .await -> another task could try to borrow it and panic
pub type SharedImu = Rc<RefCell<InertialSensor>>;

pub struct MotorGroup { //new struct for groups of motors
    motors: Vec<Motor>,
    brake_mode: BrakeMode, //what brake() does -> vexide motors don't store a brake mode, so we store it ourselves
}

impl MotorGroup{ //all the functions related to motor groups
    pub fn new(motors: Vec<Motor>) -> Self { //constructor -> making new motor groups
        Self { motors, brake_mode: BrakeMode::Coast }
    }

    //only remembers the mode -> doesn't touch the motors until brake() is called
    pub fn set_brake_mode(&mut self, mode: BrakeMode) { //&mut means we are allowed to modify the variable/object
        self.brake_mode = mode; //self tells the code that we are going to make these changes to whatever object this function is called on
    }

    pub fn set_voltage(&mut self, volts:f64){
        for m in &mut self.motors {
            let _ = m.set_voltage(volts); //underscore means i dont care what this value is -> set_voltage returns a Result (Ok or Err) but we don't need it so we're going to tell rust to disregard it
        }
    }

    //stops the motors using the stored brake mode
    pub fn brake(&mut self) {
        for m in &mut self.motors{
            let _ = m.brake(self.brake_mode);
        }
    }

    pub fn tare(&mut self){
        for m in &mut self.motors {
            let _ = m.set_position(vexide::math::Angle::from_degrees(0.0));
        }
    }

    pub fn get_avg_pos(&self) -> f64 {
        if self.motors.is_empty() {
            return 0.0;
        }
        let mut sum = 0.0;
        let mut count = 0;
        for m in &self.motors{
            match m.position() { //when you call position on the motor it returns a box -> which can either be Ok or Err
                Ok(p) => { //writing cases for whether the value is Ok or Err and working accordingly
                    sum += p.as_degrees();
                    count += 1;
                }
                Err(_) => continue
            }
        }
        if count ==0 {
            return 0.0;
        }
        sum / count as f64
    }

    pub fn get_avg_temp(&self) -> f64 {
        if self.motors.is_empty() {
            return 0.0;
        }
        let mut sum = 0.0;
        let mut count = 0;
        for m in &self.motors{
            match m.temperature() { //when you call position on the motor it returns a box -> which can either be Ok or Err
                Ok(p) => { //writing cases for whether the value is Ok or Err and working accordingly
                    sum += p;
                    count += 1;
                }
                Err(_) => continue
            }
        }
        if count ==0 {
            return 0.0;
        }
        sum / count as f64
    }
}