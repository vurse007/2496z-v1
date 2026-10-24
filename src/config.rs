use vexide::peripherals::DynamicPeripherals;
use vexide::prelude::*;
use vexide::adi::AdiPort;
use vexide::smart::SmartPort;

use crate::global::MotorGroup;

use Direction::{Forward, Reverse};

//every device on the robot -> one line each, change ports / gearsets / directions right here
pub struct Devices {
    pub left: MotorGroup,
    pub right: MotorGroup,
    pub imu: InertialSensor, //not calibrated yet -> see global::calibrate_imu
    pub lift_motors: MotorGroup,
    pub lift_rotation: RotationSensor,
    pub intake_motors: MotorGroup,
    pub claw: Motor,
    pub flipper: AdiDigitalOut, //piston
    pub controller: Controller,
}

impl Devices {
    pub fn new(p: &mut DynamicPeripherals) -> Self {
        Self {
            left: motors(p, &[(1, Reverse), (2, Reverse)], Gearset::Blue),
            right: motors(p, &[(11, Forward), (12, Forward)], Gearset::Blue),
            imu: InertialSensor::new(port(p, 18)),
            lift_motors: motors(p, &[(13, Forward), (3, Reverse)], Gearset::Green), //placeholder gearset
            lift_rotation: RotationSensor::new(port(p, 4), Direction::Forward),
            intake_motors: motors(p, &[(14, Reverse), (5, Reverse)], Gearset::Green), 
            claw: Motor::new(port(p, 15), Gearset::Blue, Reverse),
            flipper: AdiDigitalOut::new(adi_port(p, 1)), //adi port A
            controller: p.take_primary_controller().unwrap(),
        }
    }
}

//crashes on startup if two devices share a port
fn port(p: &mut DynamicPeripherals, num: u8) -> SmartPort {
    p.take_smart_port(num).expect("smart port is already in use")
}

//three-wire ports: 1 = A, 2 = B ... 8 = H
fn adi_port(p: &mut DynamicPeripherals, num: u8) -> AdiPort {
    p.take_adi_port(num).expect("adi port is already in use")
}

//each motor is (port, direction) -> Reverse flips that one motor
fn motors(p: &mut DynamicPeripherals, motors: &[(u8, Direction)], gearset: Gearset) -> MotorGroup {
    MotorGroup::new(motors.iter().map(|&(n, dir)| Motor::new(port(p, n), gearset, dir)).collect())
}
