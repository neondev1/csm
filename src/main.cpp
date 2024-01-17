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

#define ADI_INTAKE		65
#define ADI_CLIMB		66
#define ADI_WALL		67
#define ADI_BALANCE		68
// #define ADI_WALL_L	71
// #define ADI_WALL_R	70

#define CATA_STOP		35500
#define CATA_STOP_VEL	112
#define PI				3.1416
#define SKILL_CYCLE		40000

// #define DISABLE_IMU
#define SKILL_DEBUG

#define GYRO_ERR(gyro) gyro_err || gyro.get_heading() == PROS_ERR_F

Controller master(E_CONTROLLER_MASTER);

Drivetrain drive(MOTOR_LF, MOTOR_RF, MOTOR_LR, MOTOR_RR);
Motor
	cata_1(MOTOR_CATA1, E_MOTOR_GEAR_RED, 1),
	cata_2(MOTOR_CATA2, E_MOTOR_GEAR_RED, 0),
	intake(MOTOR_INTAKE, E_MOTOR_GEAR_BLUE, 0);

Rotation c_rot(SENSOR_ROT);
Vision vision(SENSOR_VIS);
Imu gyro(SENSOR_GYRO);

ADIDigitalOut
	_intake(ADI_INTAKE),
	_climb(ADI_CLIMB),
	_wall(ADI_WALL),
	_balance(ADI_BALANCE);

int btn_l1 = 0, btn_r1 = 0, btn_r2 = 0, btn_a = 0, btn_b = 0, btn_x = 0, btn_y = 0, gyro_err = 1;
int int_m = 0, int_p = 0, cata = 0, cata_hold = 0, cata_stop = 0, wall = 0, dir = 1;
int program = 0;

double* _cos = (double*)0;
vision_signature_s_t tri_sig1 =
	Vision::signature_from_utility(1, -6311, -1093, -3702, -6471, -2387, -4429, 1.000, 0);
vision_signature_s_t tri_sig2 =
	Vision::signature_from_utility(2, -6241, -1133, -3688, -7117, -327, -3722, 1.000, 0);

void initialize(void) {
	lcd::initialize();
	lcd::set_background_color(20, 20, 25);
	lcd::set_text_color(220, 220, 220);
	lcd::print(0, "Initializing...");
	_cos = (double*)malloc(180 * sizeof(double));
	for (int i = 0; i < 180; i++)
		_cos[i] = cos((double)i * PI / 180.0);
	c_rot.set_data_rate(50);
	vision.set_signature(1, &tri_sig1);
	vision.set_signature(2, &tri_sig2);
	lcd::clear_line(0);
	lcd::print(0, "Calibrating IMU...");
	lcd::print(1, "This process should take <10 s");
#ifndef DISABLE_IMU
	int i;
	for (i = 0; i < 3;) {
		gyro.reset(1);
		delay(1000);
		int roll = gyro.get_roll(),
			pitch = gyro.get_pitch(),
			yaw = gyro.get_yaw();
		delay(4000);
		if (absf(gyro.get_roll() - roll) > 1.0
		 || absf(gyro.get_pitch() - pitch) > 1.0
		 || absf(gyro.get_yaw() - yaw) > 0.5) {
			lcd::clear_line(1);
			lcd::print(1, "Attempt %d", ++i);
			continue;
		}
		gyro_err = 0;
		lcd::clear_line(0);
		lcd::clear_line(1);
		lcd::print(0, "Ready");
		break;
	}
#endif
	if (i == 3 || gyro.get_rotation() == PROS_ERR_F) {
		gyro_err = 1;
		lcd::clear_line(0);
		lcd::clear_line(1);
		lcd::print(0, "IMU error during initialization");
	}
	lcd::clear_line(1);
	lcd::print(1, "Selected program: NEAR");
	lcd::register_btn0_cb([] {
		Imu tmp_gyro(SENSOR_GYRO);
		program = 0;
		lcd::clear_line(1);
		gyro.tare_rotation();
		lcd::print(1, "Selected program: NEAR");
		delay(1000);
		lcd::print(6, "IMU rotation: %5f", tmp_gyro.get_rotation());
	});
	lcd::register_btn1_cb([] {
		Imu tmp_gyro(SENSOR_GYRO);
		program = 1;
		lcd::clear_line(1);
		gyro.tare_rotation();
		lcd::print(1, "Selected program: FAR");
		delay(1000);
		lcd::print(6, "IMU rotation: %5f", tmp_gyro.get_rotation());
	});
	lcd::register_btn2_cb([] {
		Imu tmp_gyro(SENSOR_GYRO);
		program = 2;
		lcd::clear_line(1);
		gyro.tare_rotation();
		lcd::print(1, "Selected program: SKILL");
		delay(1000);
		lcd::print(6, "IMU rotation: %5f", tmp_gyro.get_rotation());
	});
}

void disabled(void) {}
void competition_initialize(void) {}

void autonomous(void) {
	int itime = (int)millis();
	Task cata_ctrl {[] {
		Motor tmp_cata_1(MOTOR_CATA1);
		Motor tmp_cata_2(MOTOR_CATA2);
		Rotation tmp_c_rot(SENSOR_ROT);
		for (;; delay(10)) {
			Task::notify_take(1, TIMEOUT_MAX);
			tmp_cata_1 = CATA_STOP_VEL;
			tmp_cata_2 = CATA_STOP_VEL;
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
	gyro.tare_rotation();
	switch (program) {
		case 0:
			_intake.set_value(1);
			int_p = 1;
			movevc(&drive, &vc, 1100.0);
			turnvc(&drive, &vc, 0.0, -100);
			intake = -127;
			delay(300);
			//_intake.set_value(0);
			movet(&drive, 800, rpm[E_MOTOR_GEAR_GREEN]);
			intake = 0;
			cata_ctrl.notify();
			vp = {
				-rpm[E_MOTOR_GEAR_GREEN] / 4,
				-rpm[E_MOTOR_GEAR_GREEN],
				
				40, 0, 0, 0,
				0, 0,
				0, 250, 0, 0,
				
				1, 0, 0, 0,
				0, 0, 0, 0,
				0, 1, 0, 0,
				1, 500, 75
			};
			track(&drive, &vision, &vp);
			//_intake.set_value(1);
			intake = 127;
			delay(100);
			movevc(&drive, &vc, 375.0);
			movevc(&drive, &vc, -200.0);
			delay(100);
			/*
			if (GYRO_ERR(gyro))
				turnvc(&drive, &vc, 0.0, -185);
			else
			*/
			turnh(&drive, &gyro, 165, rpm[E_MOTOR_GEAR_GREEN] * 2 / 3);
			movevc(&drive, &vc, 250.0);
			/*
			if (GYRO_ERR(gyro))
				turnvc(&drive, &vc, 0.0, 50);
			else
			*/
			turn(&drive, &gyro, 50);
			cata_1 = 127;
			cata_2 = 127;
			movet(&drive, 1000, rpm[E_MOTOR_GEAR_GREEN]);
			cata_ctrl.notify();
			movet(&drive, 500, rpm[E_MOTOR_GEAR_GREEN] / 2);
			movet(&drive, 350, rpm[E_MOTOR_GEAR_GREEN] / 3);
			delay(500);
			/*
			if (GYRO_ERR(gyro))
				turnvc(&drive, &vc, 0.0, 20);
			else
			*/
			movet(&drive, 100, -rpm[E_MOTOR_GEAR_GREEN]);
			delay(200);
			turn(&drive, &gyro, 15);
			delay(300);
			for (int i = 0; i++ < 5; delay(50)) {
				cata_1 = 127;
				cata_2 = 127;
			}
			cata_1 = 0;
			cata_2 = 0;
			delay(100);
			/* Violates rule - revised
			if (GYRO_ERR(gyro))
				turnvc(&drive, &vc, 0.0, 45);
			else
				turnh(&drive, &gyro, gyro.get_rotation() + 40, rpm[E_MOTOR_GEAR_GREEN]);
			cata_1 = 0;
			cata_2 = 0;
			movet(&drive, 1400, -rpm[E_MOTOR_GEAR_GREEN]);
			_intake.set_value(0);
			int_p = 0;
			*/
			turn(&drive, &gyro, -130);
			_intake.set_value(0);
			int_p = 0;
			movet(&drive, 1200, rpm[E_MOTOR_GEAR_GREEN]);
			movet(&drive, 300, rpm[E_MOTOR_GEAR_GREEN] / 2);
			break;
		case 1:
			_intake.set_value(1);
			int_p = 1;
			movet(&drive, 1200, -rpm[E_MOTOR_GEAR_GREEN]);
			turn(&drive, &gyro, 20);
			movet(&drive, 700, rpm[E_MOTOR_GEAR_GREEN]);
			turnh(&drive, &gyro, -75);
			movet(&drive, 300, -rpm[E_MOTOR_GEAR_GREEN]);
			intake = 127;
			movet(&drive, 800, rpm[E_MOTOR_GEAR_GREEN] / 2);
			delay(500);
			turn(&drive, &gyro, -40);
			intake = -127;
			/* Violates rule - revised
			movet(&drive, 1500, -rpm[E_MOTOR_GEAR_GREEN]);
			intake = 0;
			_intake.set_value(0);
			int_p = 0;
			*/
			delay(500);
			intake = 0;
			turn(&drive, &gyro, 180);
			_intake.set_value(0);
			int_p = 0;
			movet(&drive, 1500, rpm[E_MOTOR_GEAR_GREEN]);
			break;
		case 2:
			_intake.set_value(1);
			int_p = 1;
			intake = -127;
			movet(&drive, 1200, -rpm[E_MOTOR_GEAR_GREEN]);
			movet(&drive, 200, rpm[E_MOTOR_GEAR_GREEN]);
			movet(&drive, 400, -rpm[E_MOTOR_GEAR_GREEN]);
			movet(&drive, 500, rpm[E_MOTOR_GEAR_GREEN]);
			turnh(&drive, &gyro, -90);
			movet(&drive, 200, rpm[E_MOTOR_GEAR_GREEN]);
			drive.rf.move_velocity(5);
			drive.rr.move_velocity(5);
#ifndef SKILL_DEBUG
			for (int i = 0; i++ < SKILL_CYCLE / 10; delay(10)) {         
				cata_1 = 127;
				cata_2 = 127;
				lcd::print(3, "CATA_1: %5d mA", cata_1.get_current_draw());
				lcd::print(4, "CATA_2: %5d mA", cata_2.get_current_draw());
			}
#else
			delay(100);
			itime -= 39900;
#endif
			/* Old code that keeps getting stuck on the bar :(
			turnh(&drive, &gyro, 100, rpm[E_MOTOR_GEAR_GREEN]);
			cata_1 = 0;
			cata_2 = 0;
			movet(&drive, 2500, rpm[E_MOTOR_GEAR_GREEN]);
			turnh(&drive, &gyro, 60, rpm[E_MOTOR_GEAR_GREEN]);
			movet(&drive, 1200, -rpm[E_MOTOR_GEAR_GREEN]);
			movet(&drive, 5000, rpm[E_MOTOR_GEAR_GREEN]);
			movet(&drive, 500, -rpm[E_MOTOR_GEAR_GREEN]);
			turnh(&drive, &gyro, gyro.get_heading() + 180, rpm[E_MOTOR_GEAR_GREEN]);
			movet(&drive, 1000, -rpm[E_MOTOR_GEAR_GREEN]);
			movet(&drive, 500, rpm[E_MOTOR_GEAR_GREEN]);
			intake = 127;
			*/
			turn(&drive, &gyro, -50);
			movevc(&drive, &vc, -680.0);
			intake = 0;
			turn(&drive, &gyro, 32);
			movevc(&drive, &vc, -1700.0);
			turnvc(&drive, &vc, -800.0, 90);
			movet(&drive, 1000, -rpm[E_MOTOR_GEAR_GREEN]);
			movet(&drive, 400, rpm[E_MOTOR_GEAR_GREEN]);
			turn(&drive, &gyro, -90);
			movevc(&drive, &vc, 300.0);
			turn(&drive, &gyro, -15);
			turnvc(&drive, &vc, 1000.0, -65);
			turnh(&drive, &gyro, 273);
			movet(&drive, 1100, -rpm[E_MOTOR_GEAR_GREEN] / 2);
			movet(&drive, 500, rpm[E_MOTOR_GEAR_GREEN] / 2);
			turnh(&drive, &gyro, 273);
			movet(&drive, 1500, -rpm[E_MOTOR_GEAR_GREEN]);
			movevc(&drive, &vc, 1000);
			turnh(&drive, &gyro, 160);
			movevc(&drive, &vc, 1250);
			turnh(&drive, &gyro, 227);
			movet(&drive, 1100, -rpm[E_MOTOR_GEAR_GREEN] / 2);
			movet(&drive, 500, rpm[E_MOTOR_GEAR_GREEN] / 2);
			turnh(&drive, &gyro, 227);
			movet(&drive, 1500, -rpm[E_MOTOR_GEAR_GREEN]);
			movevc(&drive, &vc, 1000);
			turnh(&drive, &gyro, 160);
			movevc(&drive, &vc, -700);
			turnh(&drive, &gyro, 250);
			movet(&drive, 1000, -rpm[E_MOTOR_GEAR_GREEN]);
			break;
	}
	drive.lf.set_brake_mode(def_brake);
	drive.rf.set_brake_mode(def_brake);
	drive.lr.set_brake_mode(def_brake);
	drive.rr.set_brake_mode(def_brake);
	lcd::print(7, "Auto execution time: %d ms", millis() - itime);
}

void opcontrol(void) {
	drive.move_velocity(0);
	if (program == 2) {
		_intake.set_value(1);
		int_p = 1;
		movet(&drive, 1200, -rpm[E_MOTOR_GEAR_GREEN]);
		movet(&drive, 200, rpm[E_MOTOR_GEAR_GREEN]);
		movet(&drive, 400, -rpm[E_MOTOR_GEAR_GREEN]);
		movet(&drive, 500, rpm[E_MOTOR_GEAR_GREEN]);
		turnh(&drive, &gyro, -90, rpm[E_MOTOR_GEAR_GREEN]);
		movet(&drive, 200, rpm[E_MOTOR_GEAR_GREEN]);
	}
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
		if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			intake = 127;
			cata_hold = 1;
			cata_stop = 1;
			cata_1 = CATA_STOP_VEL;
			cata_2 = CATA_STOP_VEL;
		}
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
				cata_1 = CATA_STOP_VEL;
				cata_2 = CATA_STOP_VEL;
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
					cata_1 = CATA_STOP_VEL;
					cata_2 = CATA_STOP_VEL;
				} else {
					cata_stop = 0;
					cata_1 = 0;
					cata_2 = 0;
				}
			}
		} else if (btn_r2 && !master.get_digital(E_CONTROLLER_DIGITAL_R2))
			btn_r2 = 0;
		if (cata && !l_stick && !r_stick) {
// #ifndef SKILL_DEBUG
			if (program < 2) {
/*
#else
			if (program == 1) {
#endif // SKILL_DEBUG
#if 0
}
#endif
*/
				drive.lf.move_velocity(5);
				drive.lr.move_velocity(5);
			} else {
				drive.rf.move_velocity(5);
				drive.rr.move_velocity(5);
			}
		}
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
			_climb.set_value(0);
		} else if (btn_a && !master.get_digital(E_CONTROLLER_DIGITAL_A))
			btn_a = 0;
		if (!btn_b && master.get_digital(E_CONTROLLER_DIGITAL_B)) {
			btn_b = 1;
			if (int_p) {
				_intake.set_value(0);
				_climb.set_value(0);
			}
			else {
				int_p = 1;
				_intake.set_value(1);
				_climb.set_value(1);
			}
		} else if (btn_b && !master.get_digital(E_CONTROLLER_DIGITAL_B))
			btn_b = 0;
		if (!btn_x && master.get_digital(E_CONTROLLER_DIGITAL_X)) {
			btn_x = 1;
			wall = !wall;
			_wall.set_value(wall);
			//_wall_l.set_value(wall);
			//_wall_r.set_value(wall);
		} else if (btn_x && !master.get_digital(E_CONTROLLER_DIGITAL_X))
			btn_x = 0;
		if (master.get_digital(E_CONTROLLER_DIGITAL_Y))
			_balance.set_value(1);
		if (master.get_digital(E_CONTROLLER_DIGITAL_RIGHT))
			autonomous();
		if (master.get_digital(E_CONTROLLER_DIGITAL_UP))
			dir = 1;
		else if (master.get_digital(E_CONTROLLER_DIGITAL_DOWN))
			dir = -1;
		lcd::print(3, "CATA_1: %5.0f deg C", cata_1.get_temperature());
		lcd::print(4, "CATA_2: %5.0f deg C", cata_2.get_temperature());
	}
}
