#include "main.h"
#include "lemlib/api.hpp"
#include "autoSelect/selection.h"
#include "definitions.hpp"

/*Asset Definitions*/
ASSET(example_txt);
/*End of Asset Definitions*/

/*Variable Definitions*/

	/*Flywheel Variables*/
	bool lastKnownButtonR2State;
	bool lastKnownButtonBState;
	int flywheelState = 0; /*0 = off, 1 = Spin Forward, 2 = Spin Backward*/

/*End of Variable Definitions*/




/*Device Initilization*/

	/*Drivetrain Initilizations*/

	pros::Motor lD1(LD1, SPEEDBOX, true);
	pros::Motor lD2(LD2, SPEEDBOX, true);
	pros::Motor lD3(LD3, SPEEDBOX, true);
	pros::Motor rD1(RD1, SPEEDBOX, false);
	pros::Motor rD2(RD2, SPEEDBOX, false);
	pros::Motor rD3(RD3, SPEEDBOX, false);

	pros::MotorGroup lDrive({lD1, lD2, lD3});
	pros::MotorGroup rDrive({rD1, rD2, rD3});

	pros::Imu imu(IMU_PORT);

	pros::Rotation odomRot(ODOM_ROT, false);

	lemlib::TrackingWheel odomWheel(&odomRot, 2.75, 0, 1);

	/*End of Drivetrain Initializations*/


	/*Non-DT Initializations*/

	pros::Motor armMotor(ARM_PORT, TORQUEBOX, false);
	pros::Motor flywheelMotor(FW_PORT, SPEEDBOX, true);

	pros::ADIDigitalOut wingPnuem(WING_ADIDO);

	/*End of Non-DT Initializations*/

	/*Controller Initialization*/

	pros::Controller master(pros::E_CONTROLLER_MASTER);

	/*End of Controller Initilization*/


/*End of Device Initilization*/


/*LemLib Chassis Initializations*/

	/*LemLib Drivetrain Initilization*/
	lemlib::Drivetrain drivetrain
	{
		&lDrive, /*Pointer to the left drive channel*/
		&rDrive, /*Pointer to the right drive channel*/
		10.5, /*Track Width*/
		3.25, /*Wheel Diameter*/
		450, /*Wheel RPM*/
		8 /*Chase Power*/
	};
	/*End of LemLib Drivetrain Initilization*/


	/*LemLib Odometry Initilization*/
	lemlib::OdomSensors odomSensors
	{
		&odomWheel, /*Center Wheel*/
		nullptr, /*No Tracking Wheel*/
		nullptr, /*No Tracking Wheel*/
		nullptr, /*No Tracking Wheel*/
		&imu /*Inertial Sensor*/
	};
	/*End of LemLib Odometery Sensors Initilization*/


	/*Lateral (Forwards/Backwards) PID Initilization*/
	lemlib::ControllerSettings lateralController
	{
		16,  //16, // kP
		72, //72, // kD
		1, // smallErrorRange
		100, // smallErrorTimeout
		3, // largeErrorRange
		500, // largeErrorTimeout
		10 // Slew Rate
	};
	/*End of Lateral (Forwards/Backwards) PID Initilization*/


	/*Angular (Turning) PID Initilization*/
	lemlib::ControllerSettings angularController
	{
		10, // kP
		60, // kD
		1, // smallErrorRange
		100, // smallErrorTimeout
		3, // largeErrorRange
		500, // largeErrorTimeout
		10 // Slew Rate
	};
	/*End of Angular (Turning) PID Initilization*/


	/*LemLib Chassis Initilization*/
	lemlib::Chassis drive(drivetrain, lateralController, angularController, odomSensors);
	/*End of LemLib Chassis Initilization*/


/*End of LemLib Chassis Initializations*/



/*Function Definitions*/

void drve()
{
	drive.tank(master.get_analog(LeftY), master.get_analog(RightY), 7);
}




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() 
{
	//sylib::initialize();
	//std::cout<<"Sylib Initialized"<<std::endl;
	selector::init();
	std::cout<<"Selector Initialized"<<std::endl;
	drive.calibrate(true);
	std::cout<<"Drive Calibrated"<<std::endl;
	wingPnuem.set_value(0);
	armMotor.set_brake_mode(HOLD);
	flywheelMotor.set_brake_mode(COAST);	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
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
void autonomous() 
{
	lDrive.set_brake_modes(HOLD);
	rDrive.set_brake_modes(HOLD);
	drive.setPose(0,0,0);
	drive.follow(example_txt, 15, 10000);
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
	lDrive.set_brake_modes(COAST);
	rDrive.set_brake_modes(COAST);
	while(true)
	{

		/*Drive Code*/
		drive.tank(master.get_analog(LeftY), master.get_analog(RightY), 7);


		std::cout<<master.get_analog(LeftY)<<std::endl;

		/*Wing Code*/
		if (master.get_digital(R1))
		{
			wingPnuem.set_value(1);
		}
		else 
		{
			wingPnuem.set_value(0);
			pros::delay(10);
		}

		/*Flywheel Control Code*/
		if (master.get_digital(R2) != lastKnownButtonR2State)
		{
			lastKnownButtonR2State = master.get_digital(R2);
			if (master.get_digital(R2) && flywheelState == 0 || flywheelState == 2)
			{
				flywheelState = 1;
			}
			else if (master.get_digital(R2) && flywheelState == 1)
			{
				flywheelState = 0;
			}
		}

		if (master.get_digital(B) != lastKnownButtonBState)
		{
			lastKnownButtonBState = master.get_digital(B);
			if (master.get_digital(B) && flywheelState == 0 || flywheelState == 1)
			{
				flywheelState = 2;
			}
			else if (master.get_digital(B) && flywheelState == 2)
			{
				flywheelState = 0;
			}
		}

		switch (flywheelState)
		{
			case 0:
				flywheelMotor.move(0);
				break;
			case 1:
				flywheelMotor.move(127);
				break;
			case 2:
				flywheelMotor.move(-127);
				break;
		}

		/*Arm Control*/
		if (master.get_digital(L1))
		{
			armMotor.move(127);
		}
		else if (master.get_digital(L2))
		{
			armMotor.move(-127);
		}
		else 
		{
			armMotor.move(0);
			pros::delay(10);
		}
		pros::delay(10);

	}
}
