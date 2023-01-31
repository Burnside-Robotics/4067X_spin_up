use core::time::Duration;

use uom::si::{
    angular_velocity::revolution_per_minute, electric_potential::volt, f64::ElectricPotential,
};
use vex_rs_lib::{gains, motor::Motor, pid::PidController};
use vex_rt::{
    prelude::AdiDigitalOutput,
    rtos::{Context, Loop, Mutex, Selectable},
    select,
};

pub fn score_discs(
    shoot_time: Duration,
    motor: &Motor,
    piston: &Mutex<AdiDigitalOutput>,
    speed_target: f64,
    ctx: Context,
) {
    motor.tare_position();

    let mut pause = Loop::new(Duration::from_millis(50));

    let mut flywheel_speed_controller = PidController::new(
        speed_target,
        gains!(1.0, 2e-3, 1.0),
        Duration::from_millis(50),
        0.0,
    );

    let mut indexer_timer = Loop::new(Duration::from_millis(300));

    let mut at_speed = false;

    loop {
        let flywheel_target = flywheel_speed_controller
            .cycle(motor.get_actual_velocity().get::<revolution_per_minute>());

        if flywheel_target >= 0.0 {
            motor.move_voltage(ElectricPotential::new::<volt>(flywheel_target));
        }

        if motor.get_actual_velocity().get::<revolution_per_minute>() >= speed_target {
            at_speed = true;
        }

        if at_speed {
            if indexer_timer.select().poll().is_ok() {
                piston.lock().write(true);
            } else {
                piston.lock().write(false);
            }
        }

        select! {
            _ = ctx.done() => break,
            _ = pause.select() => continue
        }
    }
}
