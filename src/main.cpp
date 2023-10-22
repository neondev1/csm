#include "main.h"
#include "head.h"

using namespace pros;

#define MOTOR_LF		1
#define MOTOR_RF		2
#define MOTOR_LR		3
#define MOTOR_RR		4
#define MOTOR_INTAKE	5
#define MOTOR_CATA1		6
#define MOTOR_CATA2		7

#define SENSOR_ROT		8

#define ADI_INTAKE		72

#define CATA_STOP	34750

Controller master(E_CONTROLLER_MASTER);
Drivetrain drive(MOTOR_LF, MOTOR_RF, MOTOR_LR, MOTOR_RR);
Motor
	cata_1(MOTOR_CATA1, E_MOTOR_GEAR_RED, 1),
	cata_2(MOTOR_CATA2, E_MOTOR_GEAR_RED, 0),
	intake(MOTOR_INTAKE, E_MOTOR_GEAR_BLUE, 0);
Rotation c_rot(SENSOR_ROT);
ADIDigitalOut _intake(ADI_INTAKE);

int btn_l2 = 0, btn_r2 = 0, btn_a = 0;
int int_m = 0, int_p = 0, cata = 0, cata_stop = 0, dir = 1;

void initialize(void) {
	c_rot.set_data_rate(50);
}

void disabled(void) {

}

void competition_initialize(void) {

}

void autonomous(void) {

}

void opcontrol(void) {
	cata_1 = 30;
	cata_2 = 30;
	for (;;delay(10)) {
		int l_stick = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int r_stick = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		int l_motor = dir * (l_stick + r_stick * dir);
		int r_motor = dir * (l_stick - r_stick * dir);
		drive.lf = l_motor;
		drive.rf = r_motor;
		drive.lr = l_motor;
		drive.rr = r_motor;
		if (!btn_l2 && master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			btn_l2 = 1;
			int_m = 127 * !int_m;
			intake = int_m;
		}
		else if (btn_l2 && !master.get_digital(E_CONTROLLER_DIGITAL_L2))
			btn_l2 = 0;
		if (!btn_r2 && master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			btn_r2 = 1;
			if (cata = !cata) {
				cata_stop = 0;
				cata_1 = 127;
				cata_2 = 127;
			}
			else {
				cata_stop = 1;
				cata_1 = 100;
				cata_2 = 100;
			}
		}
		else if (btn_r2 && !master.get_digital(E_CONTROLLER_DIGITAL_R2))
			btn_r2 = 0;
		if (cata_stop
		&& (c_rot.get_angle() == PROS_ERR || (c_rot.get_angle() > 30000 && c_rot.get_angle() < CATA_STOP))) {
			cata_1 = 30;
			cata_2 = 30;
			cata_stop = 0;
		}
		if (master.get_digital(E_CONTROLLER_DIGITAL_UP))
			dir = 1;
		else if (master.get_digital(E_CONTROLLER_DIGITAL_DOWN))
			dir = -1;
		if (!btn_a && master.get_digital(E_CONTROLLER_DIGITAL_A)) {
			btn_a = 1;
			int_p = !int_p;
			_intake.set_value(int_p);
		}
		else if (btn_a && !master.get_digital(E_CONTROLLER_DIGITAL_A))
			btn_a = 0;
		screen::print(E_TEXT_MEDIUM, 0, "%9f", cata_1.get_temperature());
		screen::print(E_TEXT_MEDIUM, 1, "%9f", cata_2.get_temperature());
	}
}
