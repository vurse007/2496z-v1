use std::time::Duration;

//Ethan was here

use vexide::peripherals::DynamicPeripherals;
use vexide::prelude::*;

mod config;
mod drive;
mod global;
mod motion;
mod pid;
mod util;

use drive::Drivetrain;

struct Robot {
    drive: Drivetrain,
    controller: Controller,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
        println!("Driver!");

        loop {
            //tank drive: left stick y -> left side, right stick y -> right side
            let state = self.controller.state().unwrap_or_default(); //controller disconnected -> all zeros -> robot stops
            self.drive.tank(state.left_stick.y(), state.right_stick.y());

            sleep(Duration::from_millis(10)).await; //always sleep in loops so other tasks get a turn
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut peripherals = DynamicPeripherals::new(peripherals);

    let imu = config::imu(&mut peripherals).await;
    let drive = config::drivetrain(&mut peripherals, imu.clone()); //odometry will get its own imu.clone() later
    let controller = peripherals.take_primary_controller().unwrap();

    let robot = Robot { drive, controller };

    robot.compete().await;
}
