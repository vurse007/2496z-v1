use std::time::Duration;

//Ethan was here

use futures_util::join;
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
    flipper: AdiDigitalOut,
}

//shortcuts so auton doesn't have to pass the controller every time (it's used to show the pid error)
impl Robot {
    async fn straight(&mut self, distance: f64, timeout: u64, scale: f64) {
        self.drive.straight(&mut self.controller, distance, timeout, scale).await;
    }

    async fn turn_abs(&mut self, target_heading: f64, timeout: u64, scale: f64) {
        self.drive.turn_abs(&mut self.controller, target_heading, timeout, scale).await;
    }

    async fn turn_rel(&mut self, target_heading: f64, timeout: u64, scale: f64) {
        self.drive.turn_rel(&mut self.controller, target_heading, timeout, scale).await;
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");

        join!(
            self.drive.straight(&mut self.controller, -1120.0, 1000, 0.6),
            self.lift.move_to(200.0, 1000, 1.0),
        );
        let _ = self.flipper.set_high();
        sleep(Duration::from_millis(500)).await;
        self.straight(230.0, 1000, 1.0).await;
        join!(
            self.drive.turn_abs(&mut self.controller, -56.0, 1000, 1.0),
            self.lift.move_to(0.0, 1000, 1.0),
        );
        let _ = self.claw.set_voltage(12.0);
        self.straight(-300.0, 1000, 0.5).await;
        self.turn_abs(-75.0, 1000, 1.0).await;
        self.straight(-300.0, 1000, 0.3).await;
        


        
        //self.turn_abs(90.0, 1000, 1.0).await;
        //self.straight(1000.0, 1000, 1.0).await;
        //self.turn_rel(90.0, 1000, 1.0).await;
        // 
        //drive starts right away, lift starts 0.5s in -> join! waits until both are done
        //calls self.drive directly (not the self.straight shortcut) so drive and lift can be borrowed at the same time
        // join!(
        //     self.drive.straight(&mut self.controller, 2000.0, 1000, 0.1),
        //     self.lift.move_to(160.0, 1000, 1.0),
        // );
    }

    async fn driver(&mut self) {
        println!("Driver!");
        
        let mut intake_on = false;

        loop {
            //tank drive: left stick y -> left side, right stick y -> right side
            let state = self.controller.state().unwrap_or_default(); //controller disconnected -> all zeros -> robot stops
            self.drive.split_arcade(state.left_stick.y(), state.right_stick.x());

            if state.button_l1.is_pressed() { self.lift.motors.set_voltage(12.0); }
            else if state.button_l2.is_pressed() || state.button_r1.is_pressed() { self.lift.motors.set_voltage(-8.0); }
            else {
                self.lift.motors.brake();
            }

            //intake: y toggles it on / off
            if state.button_y.is_now_pressed() { intake_on = !intake_on; }
            self.intake.set_voltage(if intake_on { 12.0 } else { 0.0 });

            //claw: r1 opens, r2 closes
            if state.button_r1.is_pressed() { 
                let _ = self.claw.set_voltage(12.0);
                let _ = self.lift.motors.set_voltage(-12.0);
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

    let robot = Robot { drive, lift, intake: d.intake_motors, claw: d.claw, flipper: d.flipper, controller: d.controller };

    robot.compete().await;
}
