use uom::si::angular_velocity::revolution_per_minute;
use uom::si::electric_potential::volt;
use uom::si::f64::ElectricPotential;
use uom::ConstZero;
use vex_rs_lib::pid::VelocityController;
use vex_rt::prelude::*;

use crate::DriverControlHandler;
use uom::lib::time::Duration;
use vex_rs_lib::gains;
use vex_rs_lib::pid::PositionController;

const LOW_SPEED: f64 = 350.0;

const INDEXER_DURATION: Duration = Duration::from_millis(500);

pub struct ShooterSystem {
    flywheel_motor: Motor,
    indexer_solenoid: AdiDigitalOutput,

    indexer_timer: Loop,

    low_speed_controller: VelocityController,
}

impl ShooterSystem {
    pub fn new(flywheel_motor_port: SmartPort, indexer_solenoid_port: AdiPort) -> Self {
        Self {
            flywheel_motor: Motor::new(flywheel_motor_port, Gearset::SixToOne, false),
            indexer_solenoid: indexer_solenoid_port.try_into().unwrap(),

            indexer_timer: Loop::new(Duration::ZERO),

            low_speed_controller: VelocityController::new(LOW_SPEED, gains!(0.2, 0.0, 0.0), 0.1),
        }
    }
}

impl DriverControlHandler for ShooterSystem {
    fn driver_control_cycle(&mut self, controller: &Controller) -> Result<(), ControllerError> {
        // Get actual velocity of flywheel motor in revolutions per minute
        let current_flywheel_velocity = self
            .flywheel_motor
            .get_actual_velocity()?
            .get::<revolution_per_minute>();

        // Cycle the low speed PID controller with the current known velocity
        let low_speed_target = self.low_speed_controller.cycle(current_flywheel_velocity);

        println!("{low_speed_target}");

        // Determine the target voltage based on user input and PID targets
        let target_flywheel_voltage = if controller.r1.is_pressed()? {
            ElectricPotential::new::<volt>(low_speed_target)
        } else {
            ElectricPotential::ZERO
        };

        self.flywheel_motor.move_voltage(target_flywheel_voltage);

        if controller.r2.is_pressed()? {
            // When r2() is pressed, check whether the indexer_timer has completed a cycle, if it has, we push the piston, else we pull it back
            if self.indexer_timer.select().poll().is_ok() {
                self.indexer_solenoid.write(true);
            } else {
                self.indexer_solenoid.write(false);
            }
        } else {
            // When r2() isnt pressed, pull the piston back, and reset the indexer_timer
            self.indexer_solenoid.write(false);
            self.indexer_timer = Loop::new(INDEXER_DURATION);
        }

        Ok(())
    }
}
