#ifndef CSM_HEAD_H_
#define CSM_HEAD_H_

#include "main.h"

class Drivetrain {
public:
    Drivetrain(int lf, int rf, int lr, int rr);
    void move_velocity(int vel);
    void tare_position(void);
    pros::Motor lf, rf, lr, rr;
};

typedef struct {
    double tgt_l = 0.0, tgt_r = 0.0;
    double vel_l = 0.0, vel_r = 0.0;
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
    };
 */
typedef struct {
    double vel_l = 0.0;
    double vel_r = 0.0;
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
    int dly = 0;
} vis_params_t;

const double rpm[3] = {
    100.0, 200.0, 600.0
};

extern double* _cos;

void moved(Drivetrain* motors, vel_ctrl_t* vc, double dist);
void movet(Drivetrain* motors, int time, int vel);
void movevc(Drivetrain* motors, vel_ctrl_t* vc, double dist);
void turnvc(Drivetrain* motors, vel_ctrl_t* vc, double dist, int angle);
void turn(Drivetrain* motors, vel_ctrl_t* vc, double dist, int angle);
void turnh(Drivetrain* motors, pros::Imu* gyro, double dir, int vel);
void turnh_(Drivetrain* motors, pros::Imu* gyro, double dir, int vel);
void vel_ctrl(Drivetrain* motors, vel_ctrl_t* vc);
void track(Drivetrain* motors, pros::Vision* vision, vis_params_t* vp);

#endif // CSM_HEAD_H_
