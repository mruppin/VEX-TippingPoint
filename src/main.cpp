#include "main.h"
#include "motors.h"
#include "sensors.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_front_wheel (LEFT_FRONT_WHEELS_PORT, true); // This reverses the motor
pros::Motor left_back_wheel (LEFT_BACK_WHEELS_PORT);
pros::Motor right_front_wheel (RIGHT_FRONT_WHEELS_PORT);
pros::Motor right_back_wheel (RIGHT_BACK_WHEELS_PORT, true); // This reverses the motor
pros::Motor sweeper_motor (SWEEPER_PORT, true); // This reverses the motor
pros::Motor elevator_motor (ELEVATOR_PORT);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::ADIUltrasonic ultrasonic (ULTRA_PING_PORT, ULTRA_ECHO_PORT);
}

/**
 * Runs while the robSWEEPER_PORTot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

  right_front_wheel.move_relative(1000, MOTOR_MAX_SPEED);
  left_front_wheel.move_relative(1000, MOTOR_MAX_SPEED);
  right_back_wheel.move_relative(1000, MOTOR_MAX_SPEED);
  left_back_wheel.move_relative(1000, MOTOR_MAX_SPEED);

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::c::adi_pin_mode(2, INPUT);
	pros::ADIDigitalIn button (DIGITAL_SENSOR_PORT);

	while (true) {
	//	pros::lcd::print(2, "LimitSwitch ->%d<-", pros::c::adi_digital_read(2));
    // pros::lcd::print(3, "button ->%d<-", button.get_value());

		left_front_wheel.move(master.get_analog(ANALOG_LEFT_Y));
		left_back_wheel.move(master.get_analog(ANALOG_LEFT_Y));
		right_front_wheel.move(master.get_analog(ANALOG_RIGHT_Y));
		right_back_wheel.move(master.get_analog(ANALOG_RIGHT_Y));
		if (master.get_digital(DIGITAL_A)){
			elevator_motor.move(127);
			sweeper_motor.move(127);
		} else{
			elevator_motor.move(0);
			sweeper_motor.move(0);
		}
		    pros::delay(2);

	}
	
}
