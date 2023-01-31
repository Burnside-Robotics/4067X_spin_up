#![no_std]
#![no_main]

mod auton;

use auton::score_discs;
use core::fmt::Debug;
use core::ops::AddAssign;
use core::ops::Sub;
use core::ops::SubAssign;
use core::time::Duration;
use uom::si::angle::radian;
use uom::si::angular_velocity::revolution_per_minute;
use uom::si::electric_potential::volt;
use uom::si::electric_potential::ElectricPotential;
use uom::si::f64::Angle;
use uom::si::f64::Length;
use uom::si::f64::Ratio;
use uom::si::length::inch;
use uom::si::ratio::percent;
use uom::si::ratio::ratio;
use uom::ConstZero;
use vex_rs_lib::controller::Controller;
use vex_rs_lib::gains;
use vex_rs_lib::motor::Motor;
use vex_rs_lib::pid::PidController;
use vex_rs_lib::ratio;
use vex_rs_lib::tank_drive::TankDrive;
use vex_rt::adi::AdiDigitalOutput;
use vex_rt::entry;
use vex_rt::motor::Gearset;
use vex_rt::peripherals::Peripherals;
use vex_rt::prelude::println;
use vex_rt::robot::Robot;
use vex_rt::rtos::Context;
use vex_rt::rtos::Loop;
use vex_rt::rtos::Mutex;
use vex_rt::rtos::Selectable;
use vex_rt::select;

struct Dampener<T> {
    current_value: T,

    acceleraion_limit: T,
}

impl<T: AddAssign + SubAssign + PartialOrd + Copy + Sub<Output = T>> Dampener<T> {
    fn cycle(&mut self, target: T) -> T {
        if target > self.current_value {
            if target - self.current_value < self.acceleraion_limit {
                self.current_value = target;
            } else {
                self.current_value += self.acceleraion_limit;
            }
        } else if target < self.current_value {
            if self.current_value - target < self.acceleraion_limit {
                self.current_value = target;
            } else {
                self.current_value -= self.acceleraion_limit;
            }
        }
        return self.current_value;
    }
}

struct Debouncer {
    value: bool,
}

impl Debouncer {
    pub fn new() -> Self {
        Self { value: false }
    }

    pub fn test(&mut self, value: bool) -> bool {
        let previous_value = self.value;
        self.value = value;
        !previous_value && value
    }
}

struct Bot {
    drive_train: TankDrive<3>,
    intake: Motor,
    flywheel: Motor,
    controller: Controller,

    indexer: Mutex<AdiDigitalOutput>,
    expansion: Mutex<AdiDigitalOutput>,
}

impl Robot for Bot {
    fn new(p: Peripherals) -> Self {
        println!("initialised");

        Bot {
            controller: p.master_controller.into(),
            intake: Motor::new(p.port09, Gearset::EighteenToOne, false),
            flywheel: Motor::new(p.port10, Gearset::SixToOne, false),

            indexer: Mutex::new(p.port_a.try_into().unwrap()),
            expansion: Mutex::new(p.port_b.try_into().unwrap()),
            drive_train: TankDrive::new(
                [
                    Motor::new(p.port13, Gearset::SixToOne, true),
                    Motor::new(p.port12, Gearset::SixToOne, true),
                    Motor::new(p.port11, Gearset::SixToOne, true),
                ],
                [
                    Motor::new(p.port17, Gearset::SixToOne, false),
                    Motor::new(p.port18, Gearset::SixToOne, false),
                    Motor::new(p.port19, Gearset::SixToOne, false),
                ],
                ratio!(0.6),
                Length::new::<inch>(3.25),
                Length::new::<inch>(12.5),
                Length::new::<inch>(12.0),
                gains!(0.05, 0.0, 0.0),
                gains!(0.05, 0.0, 0.0),
                Angle::new::<radian>(0.1),
            ),
        }
    }

    fn autonomous(&'static self, ctx: Context) {
        // score_discs(
        //     Duration::from_secs(500),
        //     &self.flywheel,
        //     &self.indexer,
        //     505.0,
        //     ctx,
        // );
        self.drive_train
            .drive_distance(Length::new::<inch>(-20.0), ctx);
    }

    fn opcontrol(&'static self, ctx: Context) {
        let mut pause = Loop::new(Duration::from_millis(50));

        let mut indexer_timer = Loop::new(Duration::from_millis(300));

        let mut backward_drive = false;

        let flywheel_target = 400.0;

        let mut flywheel_speed_controller = PidController::new(
            flywheel_target,
            gains!(0.9, 0.6e-3, 1.0),
            Duration::from_millis(50),
            0.0,
        );

        let mut left_dampener = Dampener {
            current_value: Ratio::ZERO,

            acceleraion_limit: Ratio::new::<ratio>(0.6),
        };

        let mut right_dampener = Dampener {
            current_value: Ratio::ZERO,

            acceleraion_limit: Ratio::new::<ratio>(0.6),
        };

        // We will run a loop to check controls on the controller and
        // perform appropriate actions.
        loop {
            if self.controller.down().is_pressed() {
                backward_drive = true;
            } else if self.controller.b().is_pressed() {
                backward_drive = false;
            }

            if self.controller.l1().is_pressed() {
                self.intake.move_ratio(Ratio::new::<percent>(100.0));
            } else if self.controller.l2().is_pressed() {
                self.intake.move_ratio(Ratio::new::<percent>(-100.0));
            } else {
                self.intake.move_ratio(Ratio::ZERO);
            }

            let flywheel_target = flywheel_speed_controller.cycle(
                self.flywheel
                    .get_actual_velocity()
                    .get::<revolution_per_minute>(),
            );
            if self.controller.r1().is_pressed() {
                self.flywheel
                    .move_voltage(ElectricPotential::new::<volt>(127.0));
            } else {
                self.flywheel
                    .move_voltage(ElectricPotential::new::<volt>(0.0));
            }

            if self.controller.r2().is_pressed() {
                if indexer_timer.select().poll().is_ok() {
                    self.indexer.lock().write(true);
                } else {
                    self.indexer.lock().write(false);
                }
            } else {
                self.indexer.lock().write(false);
                // Reset to retracted
                indexer_timer = Loop::new(Duration::from_millis(500));
            }

            if self.controller.down().is_pressed()
                && self.controller.right().is_pressed()
                && self.controller.y().is_pressed()
                && self.controller.b().is_pressed()
            {
                self.expansion.lock().write(true);
            }

            if !backward_drive {
                self.drive_train.drive_tank(
                    left_dampener.cycle(self.controller.left_stick().get_y()),
                    right_dampener.cycle(self.controller.right_stick().get_y()),
                );
            } else {
                self.drive_train.drive_tank(
                    left_dampener.cycle(-self.controller.right_stick().get_y()),
                    right_dampener.cycle(-self.controller.left_stick().get_y()),
                );
            }

            select! {
                _ = ctx.done() => break,
                _ = pause.select() => continue
            }
        }
    }
}

entry!(Bot);
