use std::time::Duration;

use vexide::controller::ControllerError;
use vexide::prelude::Controller;
use vexide::time::sleep;

use crate::drive::Drivetrain;
use crate::util::{self, Timer};

//shows the live pid error on the controller's top line -> helps with tuning
//try_set_text skips the write if the screen is still busy, so it never slows the pid loop down
fn show_error(controller: &mut Controller, label: &str, error: f64) {
    let _ = controller.try_set_text(error_text(label, error), 1, 1);
}

//same as show_error but keeps retrying until the write goes through -> used for the final (settled) error
//don't swap this for controller.set_text(): in vexide 0.8 set_text and try_set_text put the text one column apart
async fn show_final_error(controller: &mut Controller, label: &str, error: f64) {
    while let Err(ControllerError::WriteBusy) = controller.try_set_text(error_text(label, error), 1, 1) {
        sleep(Duration::from_millis(10)).await;
    }
}

//padded to the full screen width so leftover characters from longer numbers get wiped
fn error_text(label: &str, error: f64) -> String {
    format!("{:<19}", format!("{label} err: {error:.2}"))
}

impl Drivetrain {
    pub async fn turn_abs(&mut self, controller: &mut Controller, target_heading: f64, timeout: u64, scale: f64) {
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
            show_error(controller, "turn", heading_error);

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

        let final_error = self.get_heading().map_or(self.turn_pid.error, |h| util::get_abs_rot_err(target_heading, h));
        show_final_error(controller, "turn", final_error).await;
    }

    pub async fn turn_rel(&mut self, controller: &mut Controller, target_heading: f64, timeout: u64, scale: f64) {
        let current_heading = self.get_heading().unwrap_or(self.target_heading);
        let target_heading = (target_heading + current_heading).rem_euclid(360.0);
        self.turn_abs(controller, target_heading, timeout, scale).await;
    }

    pub async fn straight(&mut self, controller: &mut Controller, distance: f64, timeout: u64, scale: f64) {
        self.drive_pid.reset();
        self.heading_pid.reset();

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
            show_error(controller, "drive", self.drive_pid.error);

            //heading correction: hold the heading from the last turn (self.target_heading), not whatever it drifted to -> errors don't stack up over the auton
            //imu unplugged -> no correction, just drive straight
            let heading_error = self.get_heading().map_or(0.0, |h| util::get_abs_rot_err(self.target_heading, h));
            let heading_correction = self.heading_pid.calculate(heading_error, 0.0, 1.0);

            //leave room for the correction -> at full speed one side would get clamped at 12V and the correction would be lost
            let max_drive = 12.0 - heading_correction.abs();
            let drive_speed = drive_speed.clamp(-max_drive, max_drive);

            self.move_raw(drive_speed + heading_correction, drive_speed - heading_correction); //the second paramater of calculate is current position, so let's replace that with (current_distance - start_distance), & then we can make the first param just the distance

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

        let final_error = self.get_avg_pos().map_or(self.drive_pid.error, |p| distance - (p - start_distance));
        show_final_error(controller, "drive", final_error).await;
    }
}
