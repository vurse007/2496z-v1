use vexide::prelude::RotationSensor;

use crate::global::MotorGroup;

pub struct Lift {
    pub motors: MotorGroup,
    pub rotation: RotationSensor,
}

impl Lift {
    pub fn new(motors: MotorGroup, rotation: RotationSensor) -> Self {
        Self { motors, rotation }
    }
}
