#include "main.h"
#include "motors.h"
#include "sensors.h"
#include "auto.h"
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_front_wheel(LEFT_FRONT_WHEELS_PORT); // This reverses the motor
pros::Motor left_back_wheel(LEFT_BACK_WHEELS_PORT);
pros::Motor right_front_wheel(RIGHT_FRONT_WHEELS_PORT, true);
pros::Motor right_back_wheel(RIGHT_BACK_WHEELS_PORT, true); // This reverses the motor
pros::Motor elevator_motor(ELEVATOR_PORT);
pros::Motor left_lift_motor(LEFT_LIFT_PORT);
pros::Motor right_lift_motor(RIGHT_LIFT_PORT, true);
// Motor initial code
pros::ADIDigitalIn up_switch(UP_SWITCH_PORT);
pros::ADIDigitalIn down_switch(DOWN_SWITCH_PORT);
int direction = 0; // The value 0 will tell the program what direction we want the motors to spin, and which switch we want deactivating the motors.

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	// pros::ADIUltrasonic ultrasonic(ULTRA_PING_PORT, ULTRA_ECHO_PORT);
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
void moveMM(int mm, int speed)
{
	delay(2);
	left_front_wheel.tare_position();
	left_front_wheel.move_relative(mm * 3.502, speed);
	left_back_wheel.move_relative(mm * 3.502, speed);
	right_front_wheel.move_relative(mm * 3.502, speed);
	right_back_wheel.move_relative(mm * 3.502, speed);
	while (!((left_front_wheel.get_position() < mm * 3.502 + 10) && (left_front_wheel.get_position() > mm * 3.502 - 10)))
	{
		// Continue running this loop as long as the motor is not within +-5 units of its goal
		pros::delay(2);
	}
	left_front_wheel.tare_position();
}
void autoTurn(double pos, int speed)
{
	delay(2);
	left_front_wheel.tare_position();
	left_front_wheel.move_relative(pos, speed);
	left_back_wheel.move_relative(pos, speed);
	right_front_wheel.move_relative(-pos, speed);
	right_back_wheel.move_relative(-pos, speed);
	while (!((left_front_wheel.get_position() < pos + 5) && (left_front_wheel.get_position() > pos - 5)))
	{
		// Continue running this loop as long as the motor is not within +-5 units of its goal
		pros::delay(2);
	}
	left_front_wheel.tare_position();
}
void goalLiftUp()
{
	while (true)
	{
		if (up_switch.get_value() == 1)
		{
			left_lift_motor.move_velocity(-90);
			right_lift_motor.move_velocity(-90);
		}
		else if (down_switch.get_value() == 1)
		{
			left_lift_motor.move_velocity(0);
			right_lift_motor.move_velocity(0);
			return;
		}
	}
}
void goalLiftDown()
{
	while (true)
	{
		if (down_switch.get_value() == 1)
		{
			left_lift_motor.move_velocity(90);
			right_lift_motor.move_velocity(90);
		}
		else if (up_switch.get_value() == 1)
		{
			left_lift_motor.move_velocity(0);
			right_lift_motor.move_velocity(0);
			return;
		}
	}
}
void turnLeft()
{
	left_front_wheel.move(-127);
	left_back_wheel.move(127);
	right_front_wheel.move(127);
	right_back_wheel.move(-127);
}
void turnRight()
{
	left_front_wheel.move(127);
	left_back_wheel.move(-127);
	right_front_wheel.move(-127);
	right_back_wheel.move(127);
}
void tankDrive()
{
	if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_X))
	{
		if (speedSetting == 2)
		{
			speedSetting = 1;
		}
		else
		{
			speedSetting = 2;
		}
	}
	left_front_wheel.move(master.get_analog(ANALOG_LEFT_Y) / speedSetting);
	left_back_wheel.move(master.get_analog(ANALOG_LEFT_Y) / speedSetting);
	right_front_wheel.move(master.get_analog(ANALOG_RIGHT_Y) / speedSetting);
	right_back_wheel.move(master.get_analog(ANALOG_RIGHT_Y) / speedSetting);
}
void elevatorLift()
{
	if (master.get_digital(E_CONTROLLER_DIGITAL_R1))
	{
		elevator_motor.move(127);
	}
	else if (master.get_digital(E_CONTROLLER_DIGITAL_L1))
	{
		elevator_motor.move(-127);
	}
	else
	{
		elevator_motor.move(0);
	}
}
void goalLift()
{
	if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A))
	{
		if (direction == 0)
		{
			direction++;
		}
		else
		{
			direction--;
		}
	}
	if (direction == 0 && up_switch.get_value() == 0)
	{
		left_lift_motor.move_velocity(90);
		right_lift_motor.move_velocity(90);
	}
	if (direction == 1 && down_switch.get_value() == 0)
	{
		left_lift_motor.move_velocity(-90);
		right_lift_motor.move_velocity(-90);
	}
	if (direction == 0 && up_switch.get_value() == 1)
	{
		left_lift_motor.move_velocity(0);
		right_lift_motor.move_velocity(0);
	}
	if (direction == 1 && down_switch.get_value() == 1)
	{
		left_lift_motor.move_velocity(0);
		right_lift_motor.move_velocity(0);
	}
}

void dispenseRing()
{
	delay(2);
	elevator_motor.tare_position();
	elevator_motor.move_relative(8000, 300);
	while (!((elevator_motor.get_position() < 8000 + 5) && (elevator_motor.get_position() > 8000 - 5)))
	{
		// Continue running this loop as long as the motor is not within +-5 units of its goal
		pros::delay(2);
	}
	elevator_motor.tare_position();
}
void stop()
{}
void autonomous()
{

	// Autonomous code
	if (autonomousIsRight == 0)
	{
		moveMM(-750, 225);
		// Aim lift mechanism toward goal
		// 1180 is 90 degrees
		autoTurn(1120, 120);
		moveMM(-2200, 300);
		// autoTurn(200,120);
		moveMM(-35,100);
		goalLiftUp();
		moveMM(-500, 150);
		// autoTurn(20, 50);
		goalLiftDown();
		dispenseRing();
		// Place goal down;
		// win point movement
		moveMM(500, 250);
		goalLiftUp();
		moveMM(200, 150);
	}
	else
	{
		goalLiftUp();
		moveMM(-360, 75);
		goalLiftDown();
		dispenseRing();
		autoTurn(-1110, 50);
		moveMM(-250, 75);
		goalLiftUp();
		moveMM(300, 75);
	}
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
void opcontrol()
{
	pros::c::adi_pin_mode(2, INPUT);
	pros::ADIDigitalIn button(DIGITAL_SENSOR_PORT);

	while (true)
	{
		// pros::lcd::print(2, "LimitSwitch ->%d<-", pros::c::adi_digital_read(2));
		// pros::lcd::print(3, "button ->%d<-", button.get_value());
		tankDrive();
		elevatorLift();
		goalLift();
		stop();
	}
}
