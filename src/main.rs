use std::time::Duration;

//Ethan was here

use vexide::peripherals::DynamicPeripherals;
use vexide::prelude::*;

mod config;
mod drive;
mod global;
mod lift;
mod motion;
mod pid;
mod util;

use drive::Drivetrain;
use global::MotorGroup;
use lift::Lift;

struct Robot {
    drive: Drivetrain,
    controller: Controller,
    lift: Lift,
    intake: MotorGroup,
    claw: Motor,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
        println!("Driver!");

        let mut intake_toggle = false;
        
        let mut intake_on = false;

        loop {
            //tank drive: left stick y -> left side, right stick y -> right side
            let state = self.controller.state().unwrap_or_default(); //controller disconnected -> all zeros -> robot stops
            self.drive.split_arcade(state.left_stick.y(), state.right_stick.x());

            if state.button_l1.is_pressed() { self.lift.motors.set_voltage(12.0); }
            else if state.button_l2.is_pressed() || state.button_r1.is_pressed() { self.lift.motors.set_voltage(-12.0); }
            else {
                self.lift.motors.brake();
            }

            //intake: y toggles it on / off
            if state.button_y.is_now_pressed() { intake_on = !intake_on; }
            self.intake.set_voltage(if intake_on { 12.0 } else { 0.0 });

            //claw: r1 opens, r2 closes
            if state.button_r1.is_pressed() { 
                let _ = self.claw.set_voltage(12.0);
            }
            else if state.button_r2.is_pressed() { 
                let _ = self.claw.set_voltage(-7.0); 
                let _ = self.intake.set_voltage(-12.0);
            }
            else {
                let _ = self.claw.set_voltage(0.0);
            }
            
            sleep(Duration::from_millis(10)).await; //always sleep in loops so other tasks get a turn
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut peripherals = DynamicPeripherals::new(peripherals);

    let d = config::Devices::new(&mut peripherals);

    let imu = global::calibrate_imu(d.imu).await;
    let drive = Drivetrain::new(d.left, d.right, imu.clone()); //odometry will get its own imu.clone() later

    let lift = Lift::new(d.lift_motors, d.lift_rotation);

    let robot = Robot { drive, lift, intake: d.intake_motors, claw: d.claw, controller: d.controller };

    robot.compete().await;
}
