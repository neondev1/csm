#include <atomic>
#include <iomanip>
#include <sstream>
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
// #define ADI_BALANCE	68
// #define ADI_WALL_L	71
// #define ADI_WALL_R	70

#define CATA_STOP		35500
#define CATA_STOP_VEL	112
#define PI				3.1416
#define SKILL_CYCLE		27500

#define AUTON_TEST_BTN E_CONTROLLER_DIGITAL_RIGHT
#define AUTON_TESTER
// #define DISABLE_IMU
// #define SKILL_DEBUG

// #define GYRO_ERR(gyro) gyro_err || gyro.get_heading() == PROS_ERR_F

Controller master(E_CONTROLLER_MASTER);

Drivetrain drive(MOTOR_LF, MOTOR_RF, MOTOR_LR, MOTOR_RR);
Motor
	cata_1(MOTOR_CATA1, E_MOTOR_GEAR_RED, 1),
	cata_2(MOTOR_CATA2, E_MOTOR_GEAR_RED, 0),
	intake(MOTOR_INTAKE, E_MOTOR_GEAR_BLUE, 0);

Imu gyro(SENSOR_GYRO);
Rotation rot(SENSOR_ROT);
Vision vision(SENSOR_VIS);

ADIDigitalOut
	_intake(ADI_INTAKE),
	_climb(ADI_CLIMB),
	_wall(ADI_WALL);

int btn_l1 = 0, btn_r1 = 0, btn_r2 = 0, btn_a = 0, btn_b = 0, btn_x = 0, btn_y = 0, gyro_err = 1;
int int_m = 0, int_p = 0, cata = 0, cata_hold = 0, cata_stop = 0, wall = 0, dir = 1;
std::atomic<int> program;
std::atomic<int> auton;
std::atomic<int> driver;
char* name;

double* _cos = (double*)0;
vision_signature_s_t
	tri_sig_1 =
		Vision::signature_from_utility(1, -6311, -1093, -3702, -6471, -2387, -4429, 1.000, 0),
	tri_sig_2 =
		Vision::signature_from_utility(2, -6241, -1133, -3688, -7117, -327, -3722, 1.000, 0);

void initialize(void) {
	driver = 0;
	auton = 0;
	program = 0;
	name = "NEAR ";
	master.clear();
	delay(50);
	master.set_text(0, 0, "init");
	delay(500);
	lcd::initialize();
	lcd::set_background_color(20, 20, 25);
	lcd::set_text_color(220, 220, 220);
	lcd::print(0, "Initializing...");
	_cos = (double*)malloc(180 * sizeof(double));
	for (int i = 0; i < 180; i++)
		_cos[i] = cos((double)i * PI / 180.0);
	rot.set_data_rate(50);
	vision.set_signature(1, &tri_sig_1);
	vision.set_signature(2, &tri_sig_2);
	lcd::clear_line(0);
#ifndef DISABLE_IMU
	master.set_text(1, 0, "imu");
	delay(50);
	lcd::print(0, "Calibrating IMU...");
	lcd::print(1, "This process should take <10 s");
	int i;
	for (i = 0; i < 3;) {
		gyro.reset(1);
		delay(1000);
		int roll = gyro.get_roll(),
			pitch = gyro.get_pitch(),
			yaw = gyro.get_yaw();
		for (int j = 0; j++ < 400; delay(10)) {
			if (master.get_digital(E_CONTROLLER_DIGITAL_UP)
			 && master.get_digital(E_CONTROLLER_DIGITAL_DOWN)
			 && master.get_digital(E_CONTROLLER_DIGITAL_LEFT)
			 && master.get_digital(E_CONTROLLER_DIGITAL_RIGHT)) {
				gyro_err = 1;
				lcd::clear_line(0);
				lcd::clear_line(1);
				lcd::print(0, "IMU calibration cancelled");
				master.set_text(2, 0, "cancel");
				i = -2;
#ifdef AUTON_TESTER
				for (; master.get_digital(AUTON_TEST_BTN););
#endif // AUTON_TESTER
				break;
			}
		}
		if (i < 0)
			break;
		if (fabs(gyro.get_roll() - roll) > 1.0
		 || fabs(gyro.get_pitch() - pitch) > 1.0
		 || fabs(gyro.get_yaw() - yaw) > 0.5) {
			lcd::clear_line(1);
			lcd::print(1, "Attempt %d", ++i);
			continue;
		}
		gyro_err = 0;
		lcd::clear_line(0);
		lcd::clear_line(1);
		lcd::print(0, "Ready");
		master.set_text(2, 0, "done");
		break;
	}
#endif
	if (i == 3 || gyro.get_rotation() == PROS_ERR_F) {
		gyro_err = 1;
		lcd::clear_line(0);
		lcd::clear_line(1);
		lcd::print(0, "IMU error during initialization");
		master.set_text(2, 0, "fail");
	}
	delay(500);
	master.clear();
	delay(50);
	lcd::clear_line(1);
	lcd::print(1, "Selected program: NEAR");
	Task info {[] {
		Controller tmp_master(E_CONTROLLER_MASTER);
		Motor tmp_cata_1(MOTOR_CATA1), tmp_cata_2(MOTOR_CATA2);
		Imu tmp_gyro(SENSOR_GYRO);
		master.clear();
		delay(50);
		std::ostringstream oss;
		oss.str("");
		for (;; delay(100)) {
			if (driver) {
				oss.str("");
				oss.clear();
				break;
			}
			if (!auton) {
				if (tmp_master.get_digital(E_CONTROLLER_DIGITAL_LEFT))
					program = 0;
				else if (tmp_master.get_digital(E_CONTROLLER_DIGITAL_UP))
					program = 1;
				else if (tmp_master.get_digital(E_CONTROLLER_DIGITAL_RIGHT))
					program = 2;
				if (tmp_master.get_digital(E_CONTROLLER_DIGITAL_DOWN))
					tmp_gyro.tare_rotation();
			}
			switch (program) {
				case 0:
					name = "NEAR ";
					break;
				case 1:
					name = "FAR ";
					break;
				case 2:
					name = "SKILL ";
					break;
			}
			oss << (program == 2 ? "2:00 " : "1:45 ") << name << std::setw(3) << battery::get_capacity() << "%";
			tmp_master.set_text(0, 0, oss.str());
			oss.str("");
			oss.clear();
			delay(50);
			oss << "CATA 1/" << (int)tmp_cata_1.get_temperature()
				<<    "c 2/" << (int)tmp_cata_2.get_temperature() << "c";
			tmp_master.set_text(1, 0, oss.str());
			oss.str("");
			oss.clear();
			delay(50);
			tmp_master.clear_line(2);
			if (!auton) {
				delay(50);
				double rotation = fabs(tmp_gyro.get_rotation());
				if (rotation < 1.0)
					oss << "IMU " << std::setprecision(2) << rotation << " 28101A";
				else
					oss << "IMU ERR 28101A";
				tmp_master.set_text(2, 0, oss.str());
				oss.str("");
				oss.clear();
			}
		}
	}};
	lcd::register_btn0_cb([] {
		Imu tmp_gyro(SENSOR_GYRO);
		program = 0;
		lcd::clear_line(1);
		gyro.tare_rotation();
		name = "NEAR ";
		lcd::print(1, "Selected program: NEAR");
		delay(1000);
		lcd::print(6, "IMU rotation: %5f", tmp_gyro.get_rotation());
	});
	lcd::register_btn1_cb([] {
		Imu tmp_gyro(SENSOR_GYRO);
		program = 1;
		lcd::clear_line(1);
		gyro.tare_rotation();
		name = "FAR ";
		lcd::print(1, "Selected program: FAR");
		delay(1000);
		lcd::print(6, "IMU rotation: %5f", tmp_gyro.get_rotation());
	});
	lcd::register_btn2_cb([] {
		Imu tmp_gyro(SENSOR_GYRO);
		program = 2;
		lcd::clear_line(1);
		gyro.tare_rotation();
		name = "SKILL ";
		lcd::print(1, "Selected program: SKILL");
		delay(1000);
		lcd::print(6, "IMU rotation: %5f", tmp_gyro.get_rotation());
	});
}

void disabled(void) {}
void competition_initialize(void) {}

void autonomous(void) {
	auton = 1;
	int start = (int)millis();
	Task cata_ctrl {[] {
		Motor tmp_cata_1(MOTOR_CATA1), tmp_cata_2(MOTOR_CATA2);
		Rotation tmp_rot(SENSOR_ROT);
		for (;; delay(10)) {
			Task::notify_take(1, TIMEOUT_MAX);
			tmp_cata_1 = CATA_STOP_VEL;
			tmp_cata_2 = CATA_STOP_VEL;
			for (; tmp_rot.get_angle() != PROS_ERR
			   && (tmp_rot.get_angle() < 30000 || tmp_rot.get_angle() > CATA_STOP); delay(10));
			if (tmp_rot.get_angle() == PROS_ERR)
				delay(1000);
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
			movevc(&drive, &vc, 950.0, 0b10);
			turn(&drive, &gyro, -100.0);
			intake = -127;
			delay(300);
			_intake.set_value(0);
			movet(&drive, 800, rpm[G]);
			intake = 0;
			cata_ctrl.notify();
			/* Vision Sensor died :(
			vp = {
				-rpm[G] / 4,
				-rpm[G],
				
				40, 0, 0, 0,
				0, 0,
				0, 250, 0, 0,
				
				1, 0, 0, 0,
				0, 0, 0, 0,
				0, 1, 0, 0,
				1, 500, 75
			};
			track(&drive, &vision, &vp);
			*/
			movet(&drive, 350, -rpm[G]);
			turn(&drive, &gyro, 145.0);
			//_intake.set_value(1);
			intake = 127;
			delay(100);
			movevc(&drive, &vc, 200.0);
			movevc(&drive, &vc, -200.0);
			delay(100);
			turnh(&drive, &gyro, 160.0);
			movevc(&drive, &vc, 575.0, 0b01);
			turn(&drive, &gyro, 50.0);
			cata_1 = 127;
			cata_2 = 127;
			movet(&drive, 750, rpm[G], 0b00);
			cata_ctrl.notify();
			drive.decel(rpm[G] / 3);
			movet(&drive, 400, rpm[G] / 3);
			delay(300);
			movet(&drive, 150, -rpm[G]);
			delay(200);
			turn(&drive, &gyro, 15.0);
			delay(300);
			for (int i = 0; i++ < 5; delay(50)) {
				cata_1 = 127;
				cata_2 = 127;
			}
			cata_1 = 0;
			cata_2 = 0;
			delay(100);
			/* Violates rule - revised
			turnh(&drive, &gyro, gyro.get_rotation() + 40, rpm[G]);
			cata_1 = 0;
			cata_2 = 0;
			movet(&drive, 1400, -rpm[G]);
			_intake.set_value(0);
			int_p = 0;
			*/
			turn(&drive, &gyro, -130.0);
			_intake.set_value(0);
			int_p = 0;
			intake = -127;
			movet(&drive, 1200, rpm[G]);
			movet(&drive, 300, rpm[G] / 2);
			break;
		case 1:
			// Currently broken
			_intake.set_value(1);
			int_p = 1;
			movet(&drive, 1200, -rpm[G]);
			turn(&drive, &gyro, 20.0);
			movet(&drive, 700, rpm[G]);
			turnh(&drive, &gyro, -90.0);
			movet(&drive, 300, -rpm[G]);
			intake = 127;
			movet(&drive, 800, rpm[G] / 2);
			delay(500);
			movet(&drive, 100, -rpm[G]);
			turn(&drive, &gyro, -40.0);
			intake = -127;
			/* Violates rule - revised
			movet(&drive, 1500, -rpm[G]);
			intake = 0;
			_intake.set_value(0);
			int_p = 0;
			*/
			delay(500);
			intake = 0;
			turn(&drive, &gyro, 180.0);
			movet(&drive, 300, rpm[G]);
			turn(&drive, &gyro, 50.0);
			_intake.set_value(0);
			int_p = 0;
			movet(&drive, 1500, rpm[G]);
			break;
		case 2:
			//_intake.set_value(1);
			int_p = 1;
			movet(&drive, 1200, -rpm[G]);
			movet(&drive, 200, rpm[G]);
			turn(&drive, &gyro, 15.0);
			movet(&drive, 650, rpm[G]);
			turnh(&drive, &gyro, -90.0);
			movet(&drive, 300, rpm[G]);
			drive.move_r(5);
			drive.move_l(-rpm[G]);
			delay(200);
			movet(&drive, 275, rpm[G] / 2);
			drive.move_r(5);
			drive.move_l(0);
#ifndef SKILL_DEBUG
			for (int i = 0; i++ < SKILL_CYCLE / 10; delay(10)) {         
				cata_1 = 127;
				cata_2 = 127;
				lcd::print(3, "CATA_1: %5d mA", cata_1.get_current_draw());
				lcd::print(4, "CATA_2: %5d mA", cata_2.get_current_draw());
			}
			cata_1 = 0;
			cata_2 = 0;
#else
			delay(100);
			start -= SKILL_CYCLE - 100;
#endif
			/* Old code that keeps getting stuck on the bar :(
			turnh(&drive, &gyro, 100, rpm[G]);
			cata_1 = 0;
			cata_2 = 0;
			movet(&drive, 2500, rpm[G]);
			turnh(&drive, &gyro, 60, rpm[G]);
			movet(&drive, 1200, -rpm[G]);
			movet(&drive, 5000, rpm[G]);
			movet(&drive, 500, -rpm[G]);
			turnh(&drive, &gyro, gyro.get_heading() + 180, rpm[G]);
			movet(&drive, 1000, -rpm[G]);
			movet(&drive, 500, rpm[G]);
			intake = 127;
			*/
			turn(&drive, &gyro, -50.0);
			movevc(&drive, &vc, -725.0);
			turnh(&drive, &gyro, 250.0);
			cata_ctrl.notify();
			movevc(&drive, &vc, -1525.0);
			cata_1 = 0;
			cata_2 = 0;
			turnvc(&drive, &vc, -750.0, 90);
			movet(&drive, 1000, -rpm[G]);
			movet(&drive, 250, rpm[G]);
			turn(&drive, &gyro, -15.0);
			movet(&drive, 200, rpm[G]);
			turn(&drive, &gyro, -75.0);
			movevc(&drive, &vc, 550.0);
			turn(&drive, &gyro, -22.5);
			turnvc(&drive, &vc, 900.0, -55);
			movevc(&drive, &vc, -305);
			turnh(&drive, &gyro, 282.0);
			movet(&drive, 1100, -rpm[G] * 3 / 5);
			movet(&drive, 500, rpm[G] / 2);
			turnh(&drive, &gyro, 282.0);
			movet(&drive, 900, -rpm[G]);
			turnh(&drive, &gyro, 290.0);
			movet(&drive, 1000, rpm[G]);
			turnh(&drive, &gyro, 163.0);
			movevc(&drive, &vc, 1430.0);
			turnh(&drive, &gyro, 220.0);
			movet(&drive, 1100, -rpm[G] * 3 / 5);
			movet(&drive, 500, rpm[G] / 2);
			turnh(&drive, &gyro, 220.0);
			movet(&drive, 900, -rpm[G]);
			movet(&drive, 200, rpm[G]);
			turnh(&drive, &gyro, 230.0);
			movet(&drive, 1000, rpm[G]);
			turnh(&drive, &gyro, 163.0);
			movevc(&drive, &vc, -660);
			turn(&drive, &gyro, 85.0);
			_wall.set_value(1);
			movet(&drive, 800, -rpm[G] * 3 * 5);
			movet(&drive, 500, rpm[G] / 2);
			movet(&drive, 1200, -rpm[G]);
			movet(&drive, 300, rpm[G]);
			break;
	}
	drive.lf.set_brake_mode(def_brake);
	drive.rf.set_brake_mode(def_brake);
	drive.lr.set_brake_mode(def_brake);
	drive.rr.set_brake_mode(def_brake);
	lcd::print(7, "Auto execution time: %d ms", millis() - start);
}

void opcontrol(void) {
	drive.move_velocity(0);
	intake = 0;
	driver = 1;
	Task info {[] {
		Controller tmp_master(E_CONTROLLER_MASTER);
		Motor tmp_cata_1(MOTOR_CATA1), tmp_cata_2(MOTOR_CATA2);
		std::ostringstream oss;
		oss.str("");
		int start = millis();
		int total = program == 2 ? 60 : 105;
		for (;; delay(100)) {
			int time = total - (millis() - start) / 1000;
			if (time < 0)
				time = 0;
			oss << time / 60 << ":" << std::setfill('0') << std::setw(2) << time % 60
				<< " " << name << std::setfill(' ') << std::setw(3) << battery::get_capacity() << "%";
			tmp_master.set_text(0, 0, oss.str());
			oss.str("");
			oss.clear();
			delay(50);
			oss << "CATA 1/" << (int)tmp_cata_1.get_temperature()
				<<    "c 2/" << (int)tmp_cata_2.get_temperature() << "c";
			tmp_master.set_text(1, 0, oss.str());
			oss.str("");
			oss.clear();
		}
	}};
	if (program == 2) {
		_intake.set_value(1);
		int_p = 1;
		movet(&drive, 1200, -rpm[G]);
		movet(&drive, 200, rpm[G]);
		turn(&drive, &gyro, 15.0);
		movet(&drive, 650, rpm[G]);
		turnh(&drive, &gyro, -90.0);
		movet(&drive, 350, rpm[G]);
		drive.move_r(5);
		drive.move_l(-rpm[G]);
		delay(200);
		drive.move_r(5);
		movet(&drive, 275, rpm[G] / 2);
		drive.move_l(0);
	}
	for (;; delay(10)) {
		int l_stick = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int r_stick = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		int l_motor = dir * l_stick + r_stick;
		int r_motor = dir * l_stick - r_stick;
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
			intake = 0; // intake = int_m;
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
			if (program < 2)
				drive.move_l(5);
			else
				drive.move_r(5);
		}
		else if (!cata && !l_stick && !r_stick)
			drive.move_velocity(0);
		if (cata_stop) {
			if (rot.get_angle() == PROS_ERR
			|| (rot.get_angle() > 30000 && rot.get_angle() < CATA_STOP)) {
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
			int_p = !int_p;
			if (int_p) {
				_intake.set_value(1);
				_climb.set_value(1);
			}
			else {
				int_p = 1;
				_intake.set_value(0);
				_climb.set_value(0);
			}
		} else if (btn_b && !master.get_digital(E_CONTROLLER_DIGITAL_B))
			btn_b = 0;
		if (!btn_y && master.get_digital(E_CONTROLLER_DIGITAL_Y)) {
			btn_y = 1;
			cata_hold = 1;
			cata_stop = 1;
			cata_1 = CATA_STOP_VEL;
			cata_2 = CATA_STOP_VEL;
			_climb.set_value(1);
			delay(200);
			_climb.set_value(0);
		} else if (btn_y && !master.get_digital(E_CONTROLLER_DIGITAL_Y))
			btn_y = 0;
		if (!btn_x && master.get_digital(E_CONTROLLER_DIGITAL_X)) {
			btn_x = 1;
			wall = !wall;
			_wall.set_value(wall);
			//_wall_l.set_value(wall);
			//_wall_r.set_value(wall);
		} else if (btn_x && !master.get_digital(E_CONTROLLER_DIGITAL_X))
			btn_x = 0;
		/*
		if (master.get_digital(E_CONTROLLER_DIGITAL_Y))
			_balance.set_value(1);
		*/
#ifdef AUTON_TESTER
		if (master.get_digital(AUTON_TEST_BTN))
			autonomous();
#endif // AUTON_TESTER
		if (master.get_digital(E_CONTROLLER_DIGITAL_UP))
			dir = 1;
		else if (master.get_digital(E_CONTROLLER_DIGITAL_DOWN))
			dir = -1;
		lcd::print(3, "CATA_1: %5.0f deg C", cata_1.get_temperature());
		lcd::print(4, "CATA_2: %5.0f deg C", cata_2.get_temperature());
	}
}
