#include "main.h"
#include "head.h"

#define CIRC    333.3
#define PI      3.1416
#define WIDTH   294.0

using namespace pros;

Drivetrain::Drivetrain(int lf, int rf, int lr, int rr) :
    lf(lf, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_DEGREES),
    rf(rf, E_MOTOR_GEAR_GREEN, 1, E_MOTOR_ENCODER_DEGREES),
    lr(lr, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_DEGREES),
    rr(rr, E_MOTOR_GEAR_GREEN, 1, E_MOTOR_ENCODER_DEGREES) {

    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;;                                                                               ;;
    ;;         { { { { } }               { { { } }          { { }           { } }    ;;
    ;;      { { }         { }         { }        { }        { { } }       { { } }    ;;
    ;;     { { }                     { }                    { }  { }     { }  { }    ;;
    ;;    { { }                       { }                   { }   { }   { }   { }    ;;
    ;;    { }                          { { }                { }    { } { }    { }    ;;
    ;;    { }                              { } }            { }     { { }     { }    ;;
    ;;    { } }                                { }          { }      { }      { }    ;;
    ;;     { } }                                 { }        { }               { }    ;;
    ;;      { } }         { }        { }        { }         { }               { }    ;;
    ;;         { { } } } }             { { } } }            { }               } }    ;;
    ;;                                                                               ;;
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

void Drivetrain::decel(int vel) {
    this->move_velocity(0);
    delay(50);
    this->move_velocity(vel);
}

void Drivetrain::move_l(int vel) {
    lf.move_velocity(vel);
    lr.move_velocity(vel);
}

void Drivetrain::move_r(int vel) {
    rf.move_velocity(vel);
    rr.move_velocity(vel);
}

void Drivetrain::move_velocity(int vel) {
    lf.move_velocity(vel);
    rf.move_velocity(vel);
    lr.move_velocity(vel);
    rr.move_velocity(vel);
}

void Drivetrain::tare_position() {
    lf.tare_position();
    rf.tare_position();
    lr.tare_position();
    rr.tare_position();
}

// Moves the robot in one direction for a certain length of time
void movet(Drivetrain* motors, int time, int vel, short brake) {
    motors->move_velocity(vel);
    delay(time);
    if (brake & 0b10)
        motors->move_l(0);
    if (brake & 0b01)
        motors->move_r(0);
    delay(50);
    motors->tare_position();
}

// Moves the robot in one direction for a certain distance; must call `vel_ctrl`
void moved(Drivetrain* motors, vel_ctrl_t* vc, double dist) {
    double pos = dist * 360.0 / CIRC;
    int vel = rpm[G] * (2 * (dist > 0.0) - 1);
    vc->tgt_l = pos;
    vc->tgt_r = pos;
    vc->vel_l = vel;
    vc->vel_r = vel;
    motors->tare_position();
}

// Moves the robot in one direction for a certain distance; automatically calls `vel_ctrl`
void movevc(Drivetrain* motors, vel_ctrl_t* vc, double dist, short brake) {
    *vc = {};
    moved(motors, vc, dist);
    vel_ctrl(motors, vc, brake);
    delay(50);
}

// For turning while moving; automatically calls `vel_ctrl`
void turnvc(Drivetrain* motors, vel_ctrl_t* vc, double dist, int angle, short brake) {
    *vc = {};
    turnm(motors, vc, dist, angle);
    vel_ctrl(motors, vc, brake);
    delay(50);
}

// For turning while moving; must call `vel_ctrl` afterwards
void turnm(Drivetrain* motors, vel_ctrl_t* vc, double dist, int angle) {
    int theta = abs(angle);
    int dir = angle > 0;
    if (!theta || (theta >= 180 && dist))
        return;
    int sign = dist < 0;
    double fsign = (double)(1 - sign * 2);
    int flip = dir ^ sign;
    double rad = sqrt(dist * dist / (2.0 - 2.0 * _cos[theta]));
    double out = PI * (rad + WIDTH / 2.0) * fsign * (double)theta / 180.0;
    double in = PI * (rad - WIDTH / 2.0) * fsign * (double)theta / 180.0;
    double vel = (double)rpm[G] * in / out;
    vc->tgt_l = flip ? out : in;
    vc->tgt_r = flip ? in : out;
    vc->vel_l = fsign * (flip ? rpm[G] : vel);
    vc->vel_r = fsign * (flip ? vel : rpm[G]);
    motors->tare_position();
}

// Uses the IMU to turn towards a certain heading
void turnh(Drivetrain* motors, Imu* gyro, double heading, int vel) {
    for (;;) {
        if (fabs(gyro->get_rotation() - (heading + 360.0)) < fabs(gyro->get_rotation() - heading))
            heading += 360.0;
        else if (fabs(gyro->get_rotation() - (heading - 360.0)) < fabs(gyro->get_rotation() - heading))
            heading -= 360.0;
        else
            break;
    }
    int sign = (2 * (heading > gyro->get_rotation()) - 1);
    vel = abs(vel) * sign;
    motors->move_l(vel);
    motors->move_r(-vel);
    for (; sign * (heading - gyro->get_rotation()) > 65.0; delay(10));
    vel *= 0.2;
    motors->move_l(vel);
    motors->move_r(-vel);
    for (; sign * (heading - gyro->get_rotation()) > 5.0; delay(10));
    motors->move_velocity(0);
    delay(50);
    motors->tare_position();
}

// Uses the IMU to turn the robot by a certain angle
void turn(Drivetrain* motors, Imu* gyro, double angle, int vel) {
    turnh(motors, gyro, gyro->get_rotation() + angle, vel);
}

void vel_ctrl(Drivetrain* motors, vel_ctrl_t* vc, short brake) {
    double last_l = 0, last_r = 0;
    double new_l = 0, new_r = 0;
    double sign_l = 2 * (vc->tgt_l > 0) - 1;
    double sign_r = 2 * (vc->tgt_r > 0) - 1;
    motors->tare_position();
    for (; sign_l * (vc->tgt_l - motors->lf.get_position()) > 5
        && sign_r * (vc->tgt_r - motors->rf.get_position()) > 5; delay(10)) {
        if (sign_l * (vc->tgt_l - motors->lf.get_position()) < 150) 
            new_l = vc->vel_l * vc->fact_2;
        else if (sign_l * (vc->tgt_l - motors->lf.get_position()) < 300)
            new_l = vc->vel_l * vc->fact_1;
        else
            new_l = vc->vel_l;
        if (sign_r * (vc->tgt_r - motors->rf.get_position()) < 150)
            new_r = vc->vel_r * vc->fact_2;
        else if (sign_r * (vc->tgt_r - motors->rf.get_position()) < 300)
            new_r = vc->vel_r * vc->fact_1;
        else
            new_r = vc->vel_r;
        if (new_l != last_l) {
            motors->lf.move_velocity((int)new_l);
            motors->lr.move_velocity((int)new_l);
        }
        if (new_r != last_r) {
            motors->rf.move_velocity((int)new_r);
            motors->rr.move_velocity((int)new_r);
        }
        last_l = new_l;
        last_r = new_r;
    }
    if (brake & 0b10)
        motors->move_l(0);
    if (brake & 0b01)
        motors->move_r(0);
    motors->tare_position();
}

// Uses the Vision Sensor to turn the robot until the robot is facing a Triball
void track(Drivetrain* motors, Vision* vision, vis_params_t* vp) {
    motors->lf.move_velocity(vp->vel_l);
    motors->rf.move_velocity(vp->vel_r);
    motors->lr.move_velocity(vp->vel_l);
    motors->rr.move_velocity(vp->vel_r);
    delay(vp->pre);
    for (; (vp->wmin_flag && vision->get_by_size(0).width < vp->wmin)
        || (vp->wmax_flag && vision->get_by_size(0).width > vp->wmax)
        || (vp->hmin_flag && vision->get_by_size(0).height < vp->hmin)
        || (vp->hmax_flag && vision->get_by_size(0).height > vp->hmax)
        || (vp->l_flag && vision->get_by_size(0).left_coord < vp->left)
        || (vp->r_flag && vision->get_by_size(0).left_coord > vp->left)
        || (vp->t_flag && vision->get_by_size(0).top_coord < vp->top)
        || (vp->b_flag && vision->get_by_size(0).top_coord > vp->top)
        || (vp->xa_flag && vision->get_by_size(0).x_middle_coord < vp->ctrxa)
        || (vp->xb_flag && vision->get_by_size(0).x_middle_coord > vp->ctrxb)
        || (vp->ya_flag && vision->get_by_size(0).y_middle_coord < vp->ctrya)
        || (vp->yb_flag && vision->get_by_size(0).y_middle_coord > vp->ctryb)
        || (vp->obj_flag && vision->get_object_count() < 1); delay(10));
    delay(vp->post);
    motors->move_velocity(0);
    motors->tare_position();
}
