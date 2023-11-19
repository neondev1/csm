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
#define SENSOR_VIS		9
#define SENSOR_GYRO		10

#define ADI_INTAKE		72
#define ADI_WALL_L		71
#define ADI_WALL_R		70

#define CATA_STOP	34900
#define PI			3.1416

Controller master(E_CONTROLLER_MASTER);

Drivetrain drive(MOTOR_LF, MOTOR_RF, MOTOR_LR, MOTOR_RR);
Motor
	cata_1(MOTOR_CATA1, E_MOTOR_GEAR_RED, 1),
	cata_2(MOTOR_CATA2, E_MOTOR_GEAR_RED, 0),
	intake(MOTOR_INTAKE, E_MOTOR_GEAR_BLUE, 0);

Rotation c_rot(SENSOR_ROT);
Vision vision(SENSOR_VIS);
Imu gyro(SENSOR_GYRO);

ADIDigitalOut _intake(ADI_INTAKE);
ADIDigitalOut _wall_l(ADI_WALL_L);
ADIDigitalOut _wall_r(ADI_WALL_R);

int btn_l1 = 0, btn_r1 = 0, btn_r2 = 0, btn_a = 0, btn_x = 0, btn_y = 0;
int int_m = 0, int_p = 0, cata = 0, cata_hold = 0, cata_stop = 0, wall = 0, dir = 1;

double* _cos = (double*)0;
vision_signature_s_t tri_sig1 =
	Vision::signature_from_utility(1, -6311, -1093, -3702, -6471, -2387, -4429, 1.000, 0);
vision_signature_s_t tri_sig2 =
	Vision::signature_from_utility(2, -6241, -1133, -3688, -7117, -327, -3722, 1.000, 0);

void initialize(void) {
	_cos = (double*)malloc(180 * sizeof(double));
	for (int i = 0; i < 180; i++)
		_cos[i] = cos((double)i * PI / 180.0);
	c_rot.set_data_rate(50);
	vision.set_signature(1, &tri_sig1);
	vision.set_signature(2, &tri_sig2);
	intake.set_brake_mode(E_MOTOR_BRAKE_COAST);
	//gyro.reset(1);
}

void disabled(void) {}
void competition_initialize(void) {}

void autonomous(void) {
	//movet(&drive, 2000, -rpm[E_MOTOR_GEAR_GREEN]);
	Task cata_ctrl {[] {
		Motor tmp_cata_1(MOTOR_CATA1);
		Motor tmp_cata_2(MOTOR_CATA2);
		Rotation tmp_c_rot(SENSOR_ROT);
		for (;; delay(10)) {
			Task::notify_take(1, TIMEOUT_MAX);
			tmp_cata_1 = 100;
			tmp_cata_2 = 100;
			for (; tmp_c_rot.get_angle() != PROS_ERR
			   && (tmp_c_rot.get_angle() < 30000 || tmp_c_rot.get_angle() > CATA_STOP); delay(10));
			tmp_cata_1 = 30;
			tmp_cata_2 = 30;
		}
	}};
	motor_brake_mode_e_t def_brake = drive.lf.get_brake_mode();
	drive.lf.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	drive.rf.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	drive.lr.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	drive.rr.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	drive.tare_position();
	vel_ctrl_t vc{};
	vis_params_t vp{};
	int_p = 1;
	_intake.set_value(1);
	movevc(&drive, &vc, 1025.0);
	turn(&drive, &vc, 0.0, -110);
	vel_ctrl(&drive, &vc);
	intake = -127;
	delay(300);
	_intake.set_value(0);
	movet(&drive, 700, (int)rpm[E_MOTOR_GEAR_GREEN]);
	intake = 0;
	cata_ctrl.notify();
	vp = {
		-rpm[E_MOTOR_GEAR_GREEN] / 4.0,
		-rpm[E_MOTOR_GEAR_GREEN],
		
		40, 0, 0, 0,
		0, 0,
		0, 250, 0, 0,
		
		1, 0, 0, 0,
		0, 0, 0, 0,
		0, 1, 0, 0,
		1, 50
	};
	track(&drive, &vision, &vp);
	_intake.set_value(1);
	intake = 127;
	movet(&drive, 575, (int)rpm[E_MOTOR_GEAR_GREEN]);
	delay(200);
	//if (gyro.get_heading() != PROS_ERR_F)
	//	turnh_(&drive, &gyro, 160, (int)rpm[E_MOTOR_GEAR_GREEN]);
	//else
		turnvc(&drive, &vc, 0.0, -185);
	movevc(&drive, &vc, 587.5);
	turnvc(&drive, &vc, 0.0, 45);
	movet(&drive, 750, (int)rpm[E_MOTOR_GEAR_GREEN]);
	cata_1 = 127;
	cata_2 = 127;
	movet(&drive, 500, (int)rpm[E_MOTOR_GEAR_GREEN] / 3);
	delay(1000);
	movevc(&drive, &vc, -50.0);
	cata_1 = 0;
	cata_2 = 0;
	turnvc(&drive, &vc, 0.0, 80);
	drive.move_velocity(-rpm[E_MOTOR_GEAR_GREEN] / 2);
	drive.lf.set_brake_mode(def_brake);
	drive.rf.set_brake_mode(def_brake);
	drive.lr.set_brake_mode(def_brake);
	drive.rr.set_brake_mode(def_brake);
}

void opcontrol(void) {
	drive.move_velocity(0);
	for (;; delay(10)) {
		int l_stick = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int r_stick = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		int l_motor = dir * (l_stick + r_stick * dir);
		int r_motor = dir * (l_stick - r_stick * dir);
		drive.lf = l_motor;
		drive.rf = r_motor;
		drive.lr = l_motor;
		drive.rr = r_motor;
		/*
		if (!btn_l1 && master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			btn_l1 = 1;
			int_m = 127 * !int_m;
			if (int_m) {
				cata_hold = 1;
				cata_stop = 1;
			}
		} else if (btn_l1 && !master.get_digital(E_CONTROLLER_DIGITAL_L1))
			btn_l1 = 0;
		*/
		if (master.get_digital(E_CONTROLLER_DIGITAL_L1))
			intake = 127;
		else if (master.get_digital(E_CONTROLLER_DIGITAL_L2))
			intake = -127;
		else
			intake = 0;
		/*
		else
			intake = int_m;
		*/
		if (!btn_r1 && master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			btn_r1 = 1;
			if (cata = !cata) {
				cata_stop = 0;
				cata_1 = 127;
				cata_2 = 127;
			} else if (cata_hold) {
				cata_stop = 1;
				cata_1 = 100;
				cata_2 = 100;
			} else {
				cata_stop = 0;
				cata_1 = 0;
				cata_2 = 0;
			}
		} else if (btn_r1 && !master.get_digital(E_CONTROLLER_DIGITAL_R1))
			btn_r1 = 0;
		if (!btn_r2 && master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			btn_r2 = 1;
			cata_hold = !cata_hold;
			if (!cata) {
				if (cata_hold) {
					cata_stop = 1;
					cata_1 = 100;
					cata_2 = 100;
				} else {
					cata_stop = 0;
					cata_1 = 0;
					cata_2 = 0;
				}
			}
		} else if (btn_r2 && !master.get_digital(E_CONTROLLER_DIGITAL_R2))
			btn_r2 = 0;
		if (cata && !l_stick && !r_stick)
			drive.move_velocity(20);
		else if (!cata && !l_stick && !r_stick)
			drive.move_velocity(0);
		if (cata_stop) {
			if (c_rot.get_angle() == PROS_ERR
			|| (c_rot.get_angle() > 30000 && c_rot.get_angle() < CATA_STOP)) {
				cata_1 = 30;
				cata_2 = 30;
				cata_stop = 0;
			}
		}
		if (master.get_digital(E_CONTROLLER_DIGITAL_UP))
			dir = 1;
		else if (master.get_digital(E_CONTROLLER_DIGITAL_DOWN))
			dir = -1;
		if (!btn_a && master.get_digital(E_CONTROLLER_DIGITAL_A)) {
			btn_a = 1;
			int_p = !int_p;
			_intake.set_value(int_p);
		} else if (btn_a && !master.get_digital(E_CONTROLLER_DIGITAL_A))
			btn_a = 0;
		if (!btn_x && master.get_digital(E_CONTROLLER_DIGITAL_X)) {
			btn_x = 1;
			wall = !wall;
			_wall_l.set_value(wall);
			_wall_r.set_value(wall);
		} else if (btn_x && !master.get_digital(E_CONTROLLER_DIGITAL_X))
			btn_x = 0;
		//if (master.get_digital(E_CONTROLLER_DIGITAL_RIGHT))
		//	autonomous();
		if (master.get_digital(E_CONTROLLER_DIGITAL_UP))
			dir = 1;
		else if (master.get_digital(E_CONTROLLER_DIGITAL_DOWN))
			dir = -1;
	}
}
