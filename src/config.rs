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
    pub lift_rotation: RotationSensor, //lift angle for the lift pid (port 8) -> direction must make "lift up" count positive
    pub forward_pod: RotationSensor, //forward/backward tracking wheel (odom pod, 2in wheel) -> configured only, no odometry yet
    pub claw: Motor,
    pub toggle_motors: MotorGroup, //toggle 1 + toggle 2 -> spin together on one button (see main::driver)
    pub flipper: AdiDigitalOut, //piston
    pub park: AdiDigitalOut, //piston
    pub controller: Controller,
}

impl Devices {
    pub fn new(p: &mut DynamicPeripherals) -> Self {
        Self {
            left: motors(p, &[(4, Reverse), (10, Reverse), (21, Reverse)], Gearset::Blue),
            right: motors(p, &[(3, Forward), (14, Forward), (20, Forward)], Gearset::Blue),
            imu: InertialSensor::new(port(p, 7)),
            lift_motors: motors(p, &[(13, Forward), (1, Forward)], Gearset::Green), //placeholder gearset
            lift_rotation: RotationSensor::new(port(p, 8), Direction::Reverse), //flip to Reverse if raising the lift reads negative
            forward_pod: RotationSensor::new(port(p, 5), Direction::Forward), //PLACEHOLDER port (5 = unused) + direction -> hook up at practice
            claw: Motor::new(port(p, 11), Gearset::Blue, Forward),
            toggle_motors: motors(p, &[(6, Forward), (16, Reverse)], Gearset::Green), //PLACEHOLDER ports / directions / gearset -> hook up at practice
            flipper: AdiDigitalOut::new(adi_port(p, 1)), //adi port A
            park: AdiDigitalOut::new(adi_port(p, 8)), //adi port H
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
