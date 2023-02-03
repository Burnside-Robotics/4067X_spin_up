use core::marker::PhantomData;

use uom::{si::f64::Ratio, ConstZero};
use vex_rs_lib::{controller::Controller, motor::Motor};
use vex_rt::prelude::*;

use crate::DriverControlHandler;

// To declare UOM values as consts, we cant use the `new` function as that is not a const function, we can declare using struct notation, with the value in the base units, which is this case is `ratio`
const INTAKE_SPEED: Ratio = Ratio {
    dimension: PhantomData,
    units: PhantomData,
    // 1.0 means full, or 100%
    value: 1.0,
};

pub struct IntakeSystem {
    intake_motor: Motor,
}

impl IntakeSystem {
    pub fn new(intake_motor_port: SmartPort) -> Self {
        Self {
            intake_motor: Motor::new(intake_motor_port, Gearset::EighteenToOne, false),
        }
    }
}

impl DriverControlHandler for IntakeSystem {
    fn driver_control_cycle(&mut self, controller: &Controller) {
        if controller.l1().is_pressed() {
            self.intake_motor.move_ratio(INTAKE_SPEED);
        } else if controller.l2().is_pressed() {
            self.intake_motor.move_ratio(-INTAKE_SPEED);
        } else {
            self.intake_motor.move_ratio(Ratio::ZERO);
        }
    }
}
