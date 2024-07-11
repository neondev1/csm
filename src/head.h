#ifndef CSM_HEAD_H_
#define CSM_HEAD_H_

#include "main.h"

class Drivetrain {
public:
    Drivetrain(int lf, int rf, int lm, int rm, int lr, int rr);
    void decel(int vel);
    void move_l(int vel);
    void move_r(int vel);
    void move_velocity(int vel);
    void tare_position(void);
    pros::Motor lf, rf, lm, rm, lr, rr;
};

typedef struct {
    // Target position
    double tgt_l = 0.0, tgt_r = 0.0;
    // Target velocity
    double vel_l = 0.0, vel_r = 0.0;
    // Deceleration factors
    double fact_1 = 0.8, fact_2 = 0.5;
} vel_ctrl_t;

/*
    struct vis_params_t vp = {
        // vel_l,
        // vel_r,

        // wmin, wmax, hmin, hmax,
        // left, top,
        // ctrxa, ctrxb, ctrya, ctryb,

        // wmin_flag, wmax_flag, hmin_flag, hmax_flag,
        // l_flag, r_flag, t_flag, b_flag,
        // xa_flag, xb_flag, ya_flag, yb_flag
        // obj_flag, 
    };
 */
typedef struct {
    int vel_l = 0.0;
    int vel_r = 0.0;
    int wmin = 0, wmax = 0;
    int hmin = 0, hmax = 0;
    int left = 0, top = 0;
    int ctrxa = 0, ctrxb = 0;
    int ctrya = 0, ctryb = 0;
    int wmin_flag = 0, wmax_flag = 0;
    int hmin_flag = 0, hmax_flag = 0;
    int l_flag = 0, r_flag = 0;
    int t_flag = 0, b_flag = 0;
    int xa_flag = 0, xb_flag = 0;
    int ya_flag = 0, yb_flag = 0;
    int obj_flag = 0;
    int pre, post = 0;
} vis_params_t;

#define R pros::E_MOTOR_GEAR_RED
#define G pros::E_MOTOR_GEAR_GREEN
#define B pros::E_MOTOR_GEAR_BLUE

const int rpm[3] = {
    100, 200, 600
};

extern double* _cos;

void moved(Drivetrain* motors, vel_ctrl_t* vc, double dist);
void movet(Drivetrain* motors, int time, int vel, short brake = 0b11);
void movevc(Drivetrain* motors, vel_ctrl_t* vc, double dist, short brake = 0b11);
void turnvc(Drivetrain* motors, vel_ctrl_t* vc, double dist, int angle, short brake = 0b11);
void turnm(Drivetrain* motors, vel_ctrl_t* vc, double dist, int angle);
void turnh(Drivetrain* motors, pros::Imu* gyro, double dir, int vel = rpm[G]);
void turn(Drivetrain* motors, pros::Imu* gyro, double angle, int vel = rpm[G]);
void vel_ctrl(Drivetrain* motors, vel_ctrl_t* vc, short brake = 0b11);
void track(Drivetrain* motors, pros::Vision* vision, vis_params_t* vp);

#endif // CSM_HEAD_H_
