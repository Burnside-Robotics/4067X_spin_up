#![no_std]
#![no_main]
#![allow(unused_must_use)]

// mod auton;
mod utils;

use core::fmt::Debug;
use core::time::Duration;
use drive_system::DriveSystem;
use intake_system::IntakeSystem;
use shooter_system::ShooterSystem;
use uom::si::angle::degree;
use uom::si::f64::{Angle, Length};
use uom::si::length::{foot, inch};
use vex_rs_lib::controller::Controller;
use vex_rt::prelude::*;

use crate::expansion_system::ExpansionSystem;

pub trait DriverControlHandler {
    fn driver_control_initialize(&mut self) {}
    fn driver_control_cycle(&mut self, controller: &Controller);
}

mod drive_system;
mod expansion_system;
mod intake_system;
mod shooter_system;

pub struct Bot {
    drive_system: Mutex<DriveSystem>,
    intake_system: Mutex<IntakeSystem>,
    shooter_system: Mutex<ShooterSystem>,
    expansion_system: Mutex<ExpansionSystem>,

    controller: Controller,
}

impl Robot for Bot {
    fn new(p: Peripherals) -> Self {
        println!("initialised");

        Bot {
            drive_system: Mutex::new(DriveSystem::new(
                p.port13, p.port12, p.port11, p.port17, p.port18, p.port19,
            )),
            intake_system: Mutex::new(IntakeSystem::new(p.port09)),
            shooter_system: Mutex::new(ShooterSystem::new(p.port10, p.port_a)),

            expansion_system: Mutex::new(ExpansionSystem::new(p.port_b)),

            controller: p.master_controller.into(),
        }
    }

    fn autonomous(&self, ctx: Context) {
        // self.drive_system
        //     .lock()
        //     .drive_train
        //     .drive_distance(Length::new::<foot>(2.0), &ctx);
        // self.drive_system
        //     .lock()
        //     .drive_train
        //     .drive_distance(Length::new::<inch>(20.0), &ctx);
        self.drive_system
            .lock()
            .drive_train
            .rotate_angle(Angle::new::<degree>(-90.0), &ctx);
    }

    fn opcontrol(&'static self, ctx: Context) {
        let mut pause = Loop::new(Duration::from_millis(50));

        loop {
            self.drive_system
                .lock()
                .driver_control_cycle(&self.controller);
            self.intake_system
                .lock()
                .driver_control_cycle(&self.controller);
            self.shooter_system
                .lock()
                .driver_control_cycle(&self.controller);
            self.expansion_system
                .lock()
                .driver_control_cycle(&self.controller);

            select! {
                _ = ctx.done() => break,
                _ = pause.select() => continue
            }
        }
    }
}

entry!(Bot);
