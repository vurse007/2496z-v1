use std::time::Duration;

use vexide::time::sleep;

use crate::drive::Drivetrain;
use crate::util::{self, Timer};

impl Drivetrain {
    pub async fn turn_abs(&mut self, target_heading: f64, timeout: u64, scale: f64) {
        self.turn_pid.reset();

        let mut safety_timer = Timer::new(timeout);
        safety_timer.start(None);

        loop {
            let Some(current_heading) = self.get_heading() else {
                break; //imu unplugged or still calibrating -> bail out instead of spinning forever
            };

            let heading_error = util::get_abs_rot_err(target_heading, current_heading);

            let turn_speed = self.turn_pid.calculate(heading_error, 0.0, scale);

            self.move_raw(turn_speed, -turn_speed); //pid output is already in volts

            //settling conditions
            if heading_error.abs() < self.turn_pid.settle_range {
                if !self.turn_pid.settle_timer.is_running() {
                    self.turn_pid.settle_timer.start(None);
                }
            } else if self.turn_pid.settle_timer.is_running() {
                self.turn_pid.settle_timer.stop();
            }

            if self.turn_pid.settle_timer.is_running() && self.turn_pid.settle_timer.elapsed() > self.turn_pid.settle_time {
                break;
            }

            if safety_timer.is_done() {
                break;
            }

            sleep(Duration::from_millis(self.turn_pid.dt)).await;
        }
        self.target_heading = target_heading;
        self.stop();
    }

    pub async fn turn_rel(&mut self, target_heading: f64, timeout: u64, scale: f64) {
        let current_heading = self.get_heading().unwrap_or(self.target_heading);
        let target_heading = (target_heading + current_heading).rem_euclid(360.0);
        self.turn_abs(target_heading, timeout, scale).await;
    }

    pub async fn straight(&mut self, distance: f64, timeout: u64, scale: f64) {
        self.drive_pid.reset();

        let Some(start_distance) = self.get_avg_pos() else {
            return;
        };

        let mut safety_timer = Timer::new(timeout);
        safety_timer.start(None);

        loop {
            let Some(current_distance) = self.get_avg_pos() else { //vv next time check the name of the function under the robot object -> you called it get_distance() but it's actually called get_avg_pos()
                break;
            };

            //let distance_error = distance - (current_distance - start_distance);
            let drive_speed = self.drive_pid.calculate(distance, current_distance - start_distance, scale); //for this one we can do something that makes a little more sense:
            self.move_raw(drive_speed, drive_speed); //the second paramater of calculate is current position, so let's replace that with (current_distance - start_distance), & then we can make the first param just the distance

            //making unnecessary variables isn't bad but it can be confusing and technically code runs ever so slightly faster if you don't make unnecessary things -> js a good practice to have when you deal with bigger sized variables/things
            
            if (distance - (current_distance - start_distance)).abs() < self.drive_pid.settle_range { 
                if !self.drive_pid.settle_timer.is_running() {
                    self.drive_pid.settle_timer.start(None);
                }
            } else if self.drive_pid.settle_timer.is_running() {
                self.drive_pid.settle_timer.stop();
            }

            if self.drive_pid.settle_timer.is_running() && self.drive_pid.settle_timer.elapsed() > self.drive_pid.settle_time {
                break;
            }

            if safety_timer.is_done() {
                break;
            }

            sleep(Duration::from_millis(self.drive_pid.dt)).await;
        }
        self.stop();         
    }
}
