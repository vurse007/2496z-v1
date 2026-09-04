use vexide::smart::motor::BrakeMode; //importing the modules that we need
use vexide::math::Angle;
use vexide::prelude::*;

pub struct motorGroup { //new struct for groups of motors
    motors: Vec<Motor>
}

impl motorGroup{ //all the functions related to motor groups
    pub fn new(motors: Vec<Motor>) -> Self { //constructor -> making new motor groups
        Self {motors}
    }

    pub fn setBrakeMode(&mut self, mode: BrakeMode ){ //&mut means we are allowed to modify the variable/object
        for m in &mut self.motors { //self tells the code that we are going to make these changes to whatever object's motors this function is called on
            let _ = m.brake(mode); //underscore means i dont care what this value is, don't store it -> setting the brake mode returns a value (usually t/f for successful or not) but we don't need it so we're going to tell rust to disregard it
        }
    }

    pub fn setVoltage(&mut self, volts:f64){
        for m in &mut self.motors {
            let _ = m.set_voltage(volts);
        }
    }

    pub fn tare(&mut self){
        for m in &mut self.motors {
            let _ = m.set_position(Angle::from_degrees(0.0));
        }
    }

    pub fn avgPosition(&self) -> f64 {
        if self.motors.is_empty() {return 0.0;}
        let mut sum = 0.0;
        let mut count = 0;
        for m in self.motors{
            if !Ok(m) {continue}
            sum += m.position().unwrap().as_degrees();
            count++;
        }
        sum / count as f64
    }
}