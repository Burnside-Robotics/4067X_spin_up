use vex_rt::prelude::*;

use crate::DriverControlHandler;

pub struct ExpansionSystem {
    solenoid: AdiDigitalOutput,
}

impl ExpansionSystem {
    pub fn new(solenoid: AdiPort) -> Self {
        Self {
            solenoid: solenoid.try_into().unwrap(),
        }
    }
}

impl DriverControlHandler for ExpansionSystem {
    fn driver_control_cycle(&mut self, controller: &Controller) -> Result<(), ControllerError> {
        // Trigger expansion if all back paddles are pressed at the same time
        if controller.down.is_pressed()?
            && controller.right.is_pressed()?
            && controller.y.is_pressed()?
            && controller.b.is_pressed()?
        {
            self.solenoid.write(true);
        }

        Ok(())
    }
}
