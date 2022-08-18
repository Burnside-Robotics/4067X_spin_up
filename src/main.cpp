#include "main.h"
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/chassis/controller/chassisControllerIntegrated.hpp"
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/chassis/controller/chassisControllerBuilder.hpp"
#include "okapi/impl/device/controller.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

using namespace okapi;
using namespace std;

Controller master;

shared_ptr<ChassisController> chassis = 
	ChassisControllerBuilder()
		.withMotors(
			{ -1, 11 },
			{ 10, -19 }
		).withDimensions(AbstractMotor::gearset::green, {{ 4_in, 11.5_in}, imev5GreenTPR})
		.withGains(     
			{0.001, 0, 0.0001}, // Distance controller gains
			{0.001, 0, 0.0001},     // Turn controller gains
			{0.001, 0, 0.0001}     // Angle controller gains (helps drive straight)
		).withOdometry()
		.buildOdometry();

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "4067X Spin Up Program");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		chassis->getModel()->tank(
			master.getAnalog(ControllerAnalog::leftY), 
			master.getAnalog(ControllerAnalog::rightY), 
			0.05
		);

		pros::delay(20);
	}
}
