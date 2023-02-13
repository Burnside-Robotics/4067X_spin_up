use crate::{utils::Dampener, DriverControlHandler};
use uom::si::angle::radian;
use uom::si::f64::{Angle, Length, Ratio};
use uom::si::length::inch;
use vex_rs_lib::tank_drive::TankDrive;
use vex_rs_lib::{controller::Controller, motor::Motor};
use vex_rs_lib::{gains, ratio};
use vex_rt::prelude::*;

// The drive ratio is the gearing ratio from the motors to the wheels.
// Our drive ratio is 36:60, making the wheels turn at 0.6 times the speed of the motors
// Since our drive train motors are 600rpm, this makes the wheels spin at 360rpm

pub struct DriveSystem {
    pub drive_train: TankDrive<3>,

    reversed_drive_state: bool,

    left_dampener: Dampener<Ratio>,

    right_dampener: Dampener<Ratio>,
}

impl DriveSystem {
    pub fn new(
        port_l1: SmartPort,
        port_l2: SmartPort,
        port_l3: SmartPort,
        port_r1: SmartPort,
        port_r2: SmartPort,
        port_r3: SmartPort,
    ) -> Self {
        let mut left_motors = [
            Motor::new(port_l1, Gearset::SixToOne, true),
            Motor::new(port_l2, Gearset::SixToOne, true),
            Motor::new(port_l3, Gearset::SixToOne, true),
        ];
        let mut right_motors = [
            Motor::new(port_r1, Gearset::SixToOne, false),
            Motor::new(port_r2, Gearset::SixToOne, false),
            Motor::new(port_r3, Gearset::SixToOne, false),
        ];

        for motor in left_motors.iter_mut() {
            motor.set_brake_mode(BrakeMode::Brake);
        }

        for motor in right_motors.iter_mut() {
            motor.set_brake_mode(BrakeMode::Brake);
        }

        Self {
            drive_train: TankDrive::new(
                left_motors,
                right_motors,
                ratio!(0.6),
                Length::new::<inch>(3.25),
                Length::new::<inch>(12.5),
                Length::new::<inch>(12.0),
                gains!(0.05, 1.5e-6, 0.0),
                gains!(0.11, 1.95e-5, 0.0),
                Angle::new::<radian>(0.15),
            ),

            reversed_drive_state: false,
            left_dampener: Dampener::new(ratio!(0.4)),
            right_dampener: Dampener::new(ratio!(0.4)),
        }
    }
}

impl DriverControlHandler for DriveSystem {
    fn driver_control_cycle(&mut self, controller: &Controller) {
        // Our drive train can be reversed by pressing controller rear paddle levers
        // This is to make it easy to drive when intaking, and when aiming to shoot by flipping the drive train to suit whichever current side of the robot is used for reference

        // Use outside paddle levers to switch drive direction
        if controller.down().is_pressed() {
            self.reversed_drive_state = true;
        } else if controller.b().is_pressed() {
            self.reversed_drive_state = false;
        }

        let mut left_input: Ratio = controller.left_stick().get_y();
        let mut right_input: Ratio = controller.right_stick().get_y();

        // Reverse Driver inputs if neccesary
        if self.reversed_drive_state {
            (left_input, right_input) = (-right_input, -left_input);
        }

        self.drive_train.drive_tank(
            self.left_dampener.cycle(left_input),
            self.right_dampener.cycle(right_input),
        );
    }
}