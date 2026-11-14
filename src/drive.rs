use std::f64::consts::PI;

use vexide::prelude::RotationSensor;

use crate::global::{MotorGroup, SharedImu};
use crate::pid::PID;
use crate::util::{drive_curve, Constants};

const DRIVE_CURVE_T: f64 = 2.8; //curve strength -> change to taste (see util::drive_curve)
const FORWARD_POD_WHEEL_DIAMETER_IN: f64 = 2.0; //2in tracking wheel -> pod distance reads in inches

pub struct Drivetrain {
    pub left: MotorGroup,
    pub right: MotorGroup,
    pub imu: SharedImu,
    pub forward_pod: Option<RotationSensor>, //forward/backward tracking wheel (odom pod)
    pub use_odom: bool, //true -> drive PID reads the pod, false -> motor encoders (default)
    pub heading_pid: PID,
    pub turn_pid: PID,
    pub drive_pid: PID,
    pub target_heading: f64,
}

impl Drivetrain {
    //no pod -> flag stays off, everything behaves exactly like before
    pub fn new(left: MotorGroup, right: MotorGroup, imu: SharedImu) -> Self {
        Self::with_optional_pod(left, right, imu, None)
    }

    //passing the pod in flips use_odom on automatically -> drive PID reads the pod instead of the motor encoders.
    //call set_use_odom(false) after this if you want the pod wired up but the motors still driving the PID.
    pub fn with_forward_pod(left: MotorGroup, right: MotorGroup, imu: SharedImu, pod: RotationSensor) -> Self {
        Self::with_optional_pod(left, right, imu, Some(pod))
    }

    fn with_optional_pod(left: MotorGroup, right: MotorGroup, imu: SharedImu, pod: Option<RotationSensor>) -> Self {
        let use_odom = pod.is_some();
        //all pid values below are placeholders -> tune them
        //PID::new(general, refined, refined_range, settle_range, slew, integral_threshold, max_integral, deadband, settle_time_ms)
        //gains are now in textbook units (see pid.rs): kp in V/unit, kd in V/(unit/s) -> old kd / 1000, slew in V/s -> old slew per 10ms loop * 100. kp unchanged, so behaviour is the same as before.
        let heading_pid = PID::new(Constants::new(0.3, 0.0, 0.0), Constants::new(0.0, 0.0, 0.0), 0.0, 0.0, 12.0, 0.0, 0.0, 0.0, 0);
        //slew 300 / 200 V/s -> full speed in ~40ms / ~60ms instead of instantly (smoother starts, less wheel slip)
        //settle times: drive was 1000ms = the same as every auton timeout -> it never settled, just buzzed at the target until the timeout
        let turn_pid = PID::new(Constants::new(0.6, 0.0, 3.73), Constants::new(0.6, 0.0, 3.73), 3.0, 1.0, 3.0, 5.0, 100.0, 0.0, 150);
        let drive_pid = PID::new(Constants::new(0.3, 0.0, 3.21), Constants::new(0.05, 0.0, 0.205), 25.0, 5.0, 2.0, 7.0, 100.0, 3.0, 200);

        Self {
            left,
            right,
            imu,
            forward_pod: pod,
            use_odom,
            heading_pid,
            turn_pid,
            drive_pid,
            target_heading: 0.0,
        }
    }

    //manual override for the flag -> flip between pod and motor encoders without rebuilding the drivetrain
    pub fn set_use_odom(&mut self, use_odom: bool) {
        self.use_odom = use_odom;
    }

    // Inputs are -1.0 to 1.0, like joystick values.
    pub fn tank(&mut self, left: f64, right: f64) {
        self.left.set_voltage(left.clamp(-1.0, 1.0) * 12.0);
        self.right.set_voltage(right.clamp(-1.0, 1.0) * 12.0);
    }

    pub fn split_arcade(&mut self, forward: f64, turn: f64) {
        let forward = drive_curve(forward, DRIVE_CURVE_T);
        let turn = drive_curve(turn, DRIVE_CURVE_T);

        let left = forward + turn;
        let right = forward - turn;

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

    //forward distance from the pod in inches (2in wheel) -> None if no pod is wired up or the sensor can't be read
    pub fn get_forward_pod_distance(&self) -> Option<f64> {
        let pod = self.forward_pod.as_ref()?;
        let deg = pod.position().ok()?.as_degrees();
        Some(deg / 360.0 * PI * FORWARD_POD_WHEEL_DIAMETER_IN)
    }

    //the distance straight() feeds the drive PID:
    //pod inches when use_odom is on and a pod is wired up, motor-encoder degrees otherwise.
    //-> the two sources are in different units, so straight() distances are inches with the flag on
    //   and motor degrees with it off, and the drive PID gains are tuned for degrees -> retune for inches before trusting it.
    //pod unreadable -> falls back to the motors instead of bailing mid-auton.
    pub fn get_drive_distance(&self) -> Option<f64> {
        if self.use_odom {
            if let Some(d) = self.get_forward_pod_distance() {
                return Some(d);
            }
        }
        self.get_avg_pos()
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