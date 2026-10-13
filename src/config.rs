use std::cell::RefCell;
use std::rc::Rc;

use vexide::peripherals::DynamicPeripherals;
use vexide::prelude::*;

use crate::drive::Drivetrain;
use crate::global::{MotorGroup, SharedImu};
use crate::pid::PID;
use crate::util::Constants;

//port numbers -> change these to match the robot
const LEFT_PORTS: [u8; 2] = [1, 2];
const RIGHT_PORTS: [u8; 2] = [4, 5];
const IMU_PORT: u8 = 7;
const DRIVE_GEARSET: Gearset = Gearset::Blue;

fn motor_group(peripherals: &mut DynamicPeripherals, ports: &[u8], direction: Direction) -> MotorGroup {
    let motors = ports
        .iter()
        .map(|&port| {
            let port = peripherals
                .take_smart_port(port)
                .expect("smart port is already in use"); //crashes on startup if two devices share a port
            Motor::new(port, DRIVE_GEARSET, direction)
        })
        .collect();
    MotorGroup::new(motors)
}

//creates and calibrates the imu -> call once, then .clone() the result for anything that needs it (Rc clones share the same sensor)
pub async fn imu(peripherals: &mut DynamicPeripherals) -> SharedImu {
    let mut imu = InertialSensor::new(peripherals.take_smart_port(IMU_PORT).expect("smart port is already in use"));

    //has to calibrate (~2-3s, robot must be still) before heading() returns anything
    //done before sharing it, so no borrow is held across the .await
    if let Err(err) = imu.calibrate().await {
        println!("IMU failed to calibrate: {err:?}");
    }

    Rc::new(RefCell::new(imu))
}

pub fn drivetrain(peripherals: &mut DynamicPeripherals, imu: SharedImu) -> Drivetrain {
    let left = motor_group(peripherals, &LEFT_PORTS, Direction::Reverse);
    let right = motor_group(peripherals, &RIGHT_PORTS, Direction::Forward);

    //all pid values below are placeholders -> tune them
    //PID::new(general, refined, refined_range, settle_range, slew, integral_threshold, max_integral, deadband, settle_time_ms)
    let heading_pid = PID::new(Constants::new(0.0, 0.0, 0.0), Constants::new(0.0, 0.0, 0.0), 0.0, 0.0, 12.0, 0.0, 0.0, 0.0, 0);
    let turn_pid = PID::new(Constants::new(0.3, 0.0, 1.0), Constants::new(0.3, 0.0, 1.0), 5.0, 1.0, 12.0, 5.0, 100.0, 0.1, 100);
    let drive_pid = PID::new(Constants::new(0.0, 0.0, 0.0), Constants::new(0.0, 0.0, 0.0), 0.0, 0.0, 12.0, 0.0, 0.0, 0.0, 0);

    Drivetrain::new(left, right, imu, heading_pid, turn_pid, drive_pid)
}
