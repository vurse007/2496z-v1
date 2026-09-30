use vexide::prelude::*;

use crate::global::{MotorGroup, SharedImu};
use crate::pid::PID;

pub struct Drivetrain {
    pub left: MotorGroup,
    pub right: MotorGroup,
    pub imu: SharedImu,
    pub heading_pid: PID,
    pub turn_pid: PID,
    pub drive_pid: PID,
    pub target_heading: f64, //used by the pid functions
}

impl Drivetrain {
    pub fn new(left: MotorGroup, right: MotorGroup, imu: SharedImu, heading_pid: PID, turn_pid: PID, drive_pid: PID) -> Self {
        Self { left, right, imu, heading_pid, turn_pid, drive_pid, target_heading: 0.0 }
    }

    //inputs are -1.0 to 1.0 (like joystick values)
    pub fn tank(&mut self, left: f64, right: f64) {
        self.left.set_voltage(left.clamp(-1.0, 1.0) * 12.0);
        self.right.set_voltage(right.clamp(-1.0, 1.0) * 12.0);
    }

    pub fn split_arcade(&mut self, forward: f64, turn: f64) {
        self.left.set_voltage((forward + turn).clamp(-1.0, 1.0) * 12.0);
        self.right.set_voltage((forward - turn).clamp(-1.0, 1.0) * 12.0);
    }

    //only remembers the mode -> it's applied when stop() is called
    pub fn set_brake_mode(&mut self, mode: vexide::smart::motor::BrakeMode) {
        self.left.set_brake_mode(mode);
        self.right.set_brake_mode(mode);
    }

    //inputs are volts, -12.0 to 12.0
    pub fn move_raw(&mut self, left: f64, right: f64) {
        self.left.set_voltage(left);
        self.right.set_voltage(right);
    }

    pub fn tare(&mut self) {
        self.right.tare();
        self.left.tare();
    }

    pub fn get_left_pos(&self) -> f64 {
        self.left.get_avg_pos()
    }

    pub fn get_right_pos(&self) -> f64 {
        self.right.get_avg_pos()
    }

    pub fn get_avg_pos(&self) -> f64 {
        (self.get_left_pos() + self.get_right_pos()) / 2.0
    }

    //heading in degrees [0, 360), None if the imu is unplugged or calibrating
    //everything should read heading through here -> when odometry exists, this can read from it instead
    pub fn get_heading(&self) -> Option<f64> {
        self.imu.borrow().heading().ok().map(|h| h.as_degrees())
    }

    //stops using the brake mode set with set_brake_mode
    pub fn stop(&mut self) {
        self.left.brake();
        self.right.brake();
    }
}
