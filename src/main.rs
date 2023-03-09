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
use uom::si::f64::Angle;
use uom::si::length::foot;
use uom::si::length::Length;
use vex_rs_lib::controller::Controller;
use vex_rt::prelude::*;
use vex_rt::vision::VisionSensor;
use vex_rt::vision::VisionSignature;

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
    // intake_system: Mutex<IntakeSystem>,
    // shooter_system: Mutex<ShooterSystem>,
    // expansion_system: Mutex<ExpansionSystem>,

    // camera: Mutex<VisionSensor>,
    encoder: AdiEncoder,
    controller: Controller,
}

impl Robot for Bot {
    fn new(p: Peripherals) -> Self {
        println!("initialised");

        Bot {
            drive_system: Mutex::new(DriveSystem::new(
                p.port13, p.port12, p.port11, p.port17, p.port18, p.port19,
            )),
            // intake_system: Mutex::new(IntakeSystem::new(p.port09)),
            // shooter_system: Mutex::new(ShooterSystem::new(p.port10, p.port_a)),

            // expansion_system: Mutex::new(ExpansionSystem::new(p.port_b)),

            // camera: Mutex::new(p.port15.into_vision()),
            encoder: p.port_a.into_adi_encoder(p.port_b).unwrap(),

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

        let signature: VisionSignature = VisionSignature {
            id: 1,
            u_min: 0,
            u_max: 10,
            u_mean: 5,
            v_min: 0,
            v_max: 10,
            v_mean: 5,
            range: 50.0,
            signature_type: 0,
        };
        // self.camera.lock().add_signature(signature);

        loop {
            // let objects = self.camera.lock().get_objects().unwrap();

            // println!("{objects:?}");

            self.drive_system
                .lock()
                .driver_control_cycle(&self.controller);
            // self.intake_system
            //     .lock()
            //     .driver_control_cycle(&self.controller);
            // self.shooter_system
            //     .lock()
            //     .driver_control_cycle(&self.controller);
            // self.expansion_system
            //     .lock()
            //     .driver_control_cycle(&self.controller);

            println!("{}", self.encoder.get().unwrap());

            select! {
                _ = ctx.done() => break,
                _ = pause.select() => continue
            }
        }
    }
}

entry!(Bot);
