#include "main.h"
#include "motors.h"
#include "sensors.h"
#include "auto.h"
#include <iostream>
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_front_wheel(LEFT_FRONT_WHEELS_PORT); // This reverses the motor
pros::Motor left_back_wheel(LEFT_BACK_WHEELS_PORT);
pros::Motor right_front_wheel(RIGHT_FRONT_WHEELS_PORT, true);
pros::Motor right_back_wheel(RIGHT_BACK_WHEELS_PORT, true); // This reverses the motor
pros::Motor elevator_motor(ELEVATOR_PORT);
pros::Motor left_lift_motor(LEFT_LIFT_PORT);
pros::Motor right_lift_motor(RIGHT_LIFT_PORT, true);
pros::Motor arm_motor(ARM_PORT);
// Motor initial code
pros::ADIDigitalIn up_switch(UP_SWITCH_PORT);
pros::ADIDigitalIn down_switch(DOWN_SWITCH_PORT);
int direction = 0; // The value 0 will tell the program what direction we want the motors to spin, and which switch we want deactivating the motors.
int ranBefore = 1;
int direction2 = 1; // The value 0 will tell the program what direction we want the motors to spin, and which switch we want deactivating the motors.
int ranBefore2 = 1;

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
void goalLiftOut()
{
	delay(2);
	int pos = -850;
	left_lift_motor.tare_position();
	left_lift_motor.move_relative(pos, 100);
	right_lift_motor.move_relative(pos, 100);
	while (!((left_lift_motor.get_position() < pos + 10) && (left_lift_motor.get_position() > pos - 10)))
	{
		// Continue running this loop as long as the motor is not within +-5 units of its goal
		pros::delay(2);
	}
}
void goalLiftIn()
{
	delay(2);
	int pos = 850;
	left_lift_motor.tare_position();
	left_lift_motor.move_relative(pos, 100);
	right_lift_motor.move_relative(pos, 100);
	while (!((left_lift_motor.get_position() < pos + 10) && (left_lift_motor.get_position() > pos - 10)))
	{
		// Continue running this loop as long as the motor is not within +-5 units of its goal
		pros::delay(2);
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
void speedControl(){
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
}
void tankDrive()
{
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
		ranBefore = 0;
		if (direction == 0)
		{
			direction++;
		}
		else
		{
			direction--;
		}
	}
	if (direction == 0 && ranBefore == 0)
	{
		left_lift_motor.move_absolute(0, 100/speedSetting);
		right_lift_motor.move_absolute(0, 100/speedSetting);
		ranBefore=1;
	}
	if (direction == 1 && ranBefore == 0)
	{
		left_lift_motor.move_absolute(-700, 100/speedSetting);
		right_lift_motor.move_absolute(-700, 100/speedSetting);
		ranBefore=1;
	}
}

void armControl(){
	if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B))
	{
		ranBefore2 = 0;
		if (direction2 == 0)
		{
			direction2++;
		}
		else
		{
			direction2--;
		}
	}
	if (direction2 == 0 && ranBefore2 == 0)
	{
		arm_motor.move_absolute(450, 100/speedSetting);
		ranBefore2=1;
	}
	if (direction2 == 1 && ranBefore2 == 0)
	{
		arm_motor.move_absolute(50, 100/speedSetting);
		ranBefore2=1;
	}
}

void stop()
{
	if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y))
	{
		left_lift_motor.move_velocity(0);
		right_lift_motor.move_velocity(0);
		left_front_wheel.move_velocity(0);
		right_front_wheel.move_velocity(0);
		left_back_wheel.move_velocity(0);
		right_back_wheel.move_velocity(0);
		elevator_motor.move_velocity(0);
		while(master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y) != true){
			delay(2);
		}
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

void autonomous()
{
	left_lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// Autonomous code
	// if (autonomousIsRight == 0)
	// {
	// 	moveMM(-750, 225);
	// 	// Aim lift mechanism toward goal
	// 	// 1180 is 90 degrees
	// 	autoTurn(1120, 120);
	// 	moveMM(-2200, 300);
	// 	// autoTurn(200,120);
	// 	moveMM(-35, 100);
	// 	goalLiftOut();
	// 	moveMM(-500, 150);
	// 	// autoTurn(20, 50);
	// 	goalLiftIn();
	// 	dispenseRing();
	// 	// Place goal down;
	// 	// win point movement
	// 	moveMM(500, 250);
	// 	goalLiftOut();
	// 	moveMM(200, 150);
	// }
	// else
	//{
		// goalLiftOut();
		// moveMM(-360, 75);
		// goalLiftIn();
		// dispenseRing();
		// autoTurn(-1110, 50);
		// moveMM(-250, 75);
		// goalLiftOut();
		// moveMM(300, 75);
	//}


		//Right side code here
			// goalLiftOut();
			// moveMM(360, 75);
			// goalLiftIn();
			// dispenseRing();
			// autoTurn(-1110, 50);
			// moveMM(250, 75);
			// goalLiftOut();
			// moveMM(-300, 75);
			// goalLiftIn();
		//First Autonnomous goal
		moveMM(800, 300);
		goalLiftOut();
		moveMM(600, 80);
		goalLiftIn();
		moveMM(-1150, 300);
		goalLiftOut();
		moveMM(-100, 300);
		goalLiftIn();
		// Second autonomous
			goalLiftOut();
			moveMM(360, 75);
			goalLiftIn();
			dispenseRing();
			autoTurn(-1110, 50);
			moveMM(250, 75);
			goalLiftOut();
			moveMM(-300, 75);
			goalLiftIn();
		//Left side code here}
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
}
void opcontrol()
{
	arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	arm_motor.move_relative(30, 100);
	left_lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	pros::c::adi_pin_mode(2, INPUT);
	pros::ADIDigitalIn button(DIGITAL_SENSOR_PORT);
	while (true)
	{
		pros::lcd::print(1, "Speed Setting: %d percent \n ", 100/(int)speedSetting);
		delay(1);
		// pros::lcd::print(2, "LimitSwitch ->%d<-", pros::c::adi_digital_read(2));
		speedControl();
		tankDrive();
		// goalRamp();
		elevatorLift();
		goalLift();
		stop();
		armControl();
	}
}
