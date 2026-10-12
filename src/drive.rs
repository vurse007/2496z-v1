use crate::global::{MotorGroup, SharedImu};
use crate::pid::PID;

pub struct Drivetrain {
    pub left: MotorGroup,
    pub right: MotorGroup,
    pub imu: SharedImu,
    pub heading_pid: PID,
    pub turn_pid: PID,
    pub drive_pid: PID,
    pub target_heading: f64,
}

impl Drivetrain {
    pub fn new(
        left: MotorGroup,
        right: MotorGroup,
        imu: SharedImu,
        heading_pid: PID,
        turn_pid: PID,
        drive_pid: PID,
    ) -> Self {
        Self {
            left,
            right,
            imu,
            heading_pid,
            turn_pid,
            drive_pid,
            target_heading: 0.0,
        }
    }

    // Inputs are -1.0 to 1.0, like joystick values.
    pub fn tank(&mut self, left: f64, right: f64) {
        self.left.set_voltage(left.clamp(-1.0, 1.0) * 12.0);
        self.right.set_voltage(right.clamp(-1.0, 1.0) * 12.0);
    }

    pub fn split_arcade(&mut self, forward: f64, turn: f64) {
        let left = forward.clamp(-1.0, 1.0) + turn.clamp(-1.0, 1.0);
        let right = forward.clamp(-1.0, 1.0) - turn.clamp(-1.0, 1.0);

        // Preserve the ratio between the two sides when either exceeds 1.0.
        let max_output = left.abs().max(right.abs()).max(1.0);

        self.left.set_voltage((left / max_output) * 12.0);
        self.right.set_voltage((right / max_output) * 12.0);
    }

    pub fn set_brake_mode(&mut self, mode: vexide::smart::motor::BrakeMode) {
        self.left.set_brake_mode(mode);
        self.right.set_brake_mode(mode);
    }

    // Inputs are volts, -12.0 to 12.0.
    pub fn move_raw(&mut self, left: f64, right: f64) {
        self.left.set_voltage(left.clamp(-12.0, 12.0));
        self.right.set_voltage(right.clamp(-12.0, 12.0));
    }

    pub fn tare(&mut self) {
        self.left.tare();
        self.right.tare();
    }

    pub fn get_left_pos(&self) -> Option<f64> {
        self.left.get_avg_pos()
    }

    pub fn get_right_pos(&self) -> Option<f64> {
        self.right.get_avg_pos()
    }

    pub fn get_avg_pos(&self) -> Option<f64> {
        let left = self.get_left_pos()?;
        let right = self.get_right_pos()?;

        Some((left + right) / 2.0)
    }

    // Unchanged, as requested.
    pub fn get_heading(&self) -> Option<f64> {
        self.imu.borrow().heading().ok().map(|h| h.as_degrees())
    }

    pub fn stop(&mut self) {
        self.left.brake();
        self.right.brake();
    }
}