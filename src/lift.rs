use vexide::prelude::RotationSensor;
use vexide::smart::motor::BrakeMode;

use crate::global::MotorGroup;

pub struct Lift {
    pub motors: MotorGroup,
    pub rotation: RotationSensor,
}

impl Lift {
    pub fn new(mut motors: MotorGroup, rotation: RotationSensor) -> Self {
        motors.set_brake_mode(BrakeMode::Coast); //hold so the lift stays in place when no button is pressed
        Self { motors, rotation }
    }
}
