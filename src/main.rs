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
    claw: Motor,
    toggles: MotorGroup,
    flipper: AdiDigitalOut,
}

//shortcuts so auton doesn't have to pass the controller every time (it's used to show the pid error)
impl Robot {
    async fn straight(&mut self, distance: f64, timeout: u64, scale: f64) {
        self.drive
            .straight(&mut self.controller, distance, timeout, scale)
            .await;
    }

    async fn turn_abs(&mut self, target_heading: f64, timeout: u64, scale: f64) {
        self.drive
            .turn_abs(&mut self.controller, target_heading, timeout, scale)
            .await;
    }

    async fn turn_rel(&mut self, target_heading: f64, timeout: u64, scale: f64) {
        self.drive
            .turn_rel(&mut self.controller, target_heading, timeout, scale)
            .await;
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
        // self.intake.set_voltage(12.0);
        // self.straight(-600.0, 1000, 1.0).await;
        // self.turn_abs(-27.0, 1000, 1.0).await;
        // self.straight(800.0, 1000, 0.7).await;
        // self.straight(-200.0, 1000, 1.0).await;
        // self.straight(300.0, 1000, 1.0).await;
        // self.straight(-600.0, 1000, 1.0).await;
        
        let _ = self.claw.set_voltage(12.0);
        sleep(Duration::from_millis(400)).await;
        join!(
            //to goal
            self.drive.straight(&mut self.controller, -1050.0, 1000, 0.6),
            self.lift.move_to(180.0, 1000, 1.0),
        );
        self.lift.move_to(70.0, 200, 0.3).await;
        sleep(Duration::from_millis(200)).await;
        let _ = self.claw.set_voltage(-12.0);
        sleep(Duration::from_millis(200)).await;
        self.straight(280.0, 1000, 1.0).await;
        join!(
            self.drive.turn_abs(&mut self.controller, -45.0, 1000, 1.0),
            self.lift.move_to(-52.0, 800, 1.0),
        );
        let _ = self.claw.set_voltage(12.0);
        self.straight(-305.0, 1000, 0.7).await;
        self.turn_abs(-75.0, 1000, 1.0).await;
        self.straight(-265.0, 1000, 0.3).await;
        join!(self.drive.straight(&mut self.controller, 950.0, 1000, 1.0),);
        let _ = self.claw.set_voltage(6.0);
        self.turn_abs(158.0, 1000, 1.0).await;
        join!(
            self.drive.straight(&mut self.controller, -950.0, 1000, 0.4),
            self.lift.move_to(190.0, 800, 1.0),
        );
        let claw = &mut self.claw; //borrow claw separately so the async block doesn't grab all of self
        join!(
            self.drive.straight(&mut self.controller, -100.0, 800, 0.4),
            self.lift.move_to(0.0, 800, 1.0),
            async {
                sleep(Duration::from_millis(100)).await; //claw opens 0.5s in
                let _ = claw.set_voltage(-12.0);
            },
        );
        join!(
            self.lift.move_to(-50.0, 800, 1.0),
            self.drive.straight(&mut self.controller, 550.0, 550, 1.0),
        );
        self.turn_abs(63.0, 500, 1.0).await;
        self.straight(1000.0, 700, 1.0).await;
        self.straight(-400.0, 700, 1.0).await;
        self.straight(500.0, 700, 1.0).await;
        self.straight(-1000.0, 700, 1.0).await;

        //testing below
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

        loop {
            //tank drive: left stick y -> left side, right stick y -> right side
            let state = self.controller.state().unwrap_or_default(); //controller disconnected -> all zeros -> robot stops
            self.drive
                .split_arcade(state.left_stick.y(), state.right_stick.x());

            if state.button_l1.is_pressed() {
                self.lift.motors.set_voltage(12.0);
            } else if state.button_l2.is_pressed() || state.button_r1.is_pressed() {
                self.lift.motors.set_voltage(-8.0);
            } else {
                self.lift.motors.brake();
            }

            //flipper: d-pad right toggles it on / off
            if state.button_right.is_now_pressed() {
                let _ = self.flipper.toggle();
            }

            //claw: r1 opens, r2 closes
            if state.button_r1.is_pressed() {
                let _ = self.claw.set_voltage(12.0);
                let _ = self.lift.motors.set_voltage(-12.0);
            } else if state.button_r2.is_pressed() {
                let _ = self.claw.set_voltage(-7.0);
            } else {
                let _ = self.claw.set_voltage(0.0);
            }

            //toggles: hold Y -> left + right toggle spin together, same direction
            if state.button_y.is_pressed() {
                self.toggles.set_voltage(12.0);
            } else {
                self.toggles.set_voltage(0.0);
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

    let lift = Lift::new(d.lift_motors, None); //no rotation sensor right now -> auton lift moves are skipped

    let robot = Robot {
        drive,
        lift,
        claw: d.claw,
        toggles: d.toggle_motors,
        flipper: d.flipper,
        controller: d.controller,
    };

    robot.compete().await;
}
