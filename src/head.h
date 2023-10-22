#ifndef CSM_HEAD_H_
#define CSM_HEAD_H_

#include "main.h"

class Drivetrain {
public:
    Drivetrain(int lf, int rf, int lr, int rr);
    pros::Motor lf, rf, lr, rr;
};

typedef struct {
    double tgt_l, tgt_r;
    double vel_l, vel_r;
} vel_ctrl_t;

const double rpm[3] = {
    100.0, 200.0, 600.0
};

extern double* _cos;

void move(Drivetrain* motors, vel_ctrl_t* vc, double dist);
void turn(Drivetrain* motors, vel_ctrl_t* vc, double dist, int angle);

#endif // CSM_HEAD_H_
