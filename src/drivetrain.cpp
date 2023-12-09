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

double absf(double num) {
    return num * ((num > 0.0) - (num < 0.0));
}

void moved(Drivetrain* motors, vel_ctrl_t* vc, double dist) {
    double pos = dist * 360.0 / CIRC;
    int vel = rpm[E_MOTOR_GEAR_GREEN] * (2 * (dist > 0.0) - 1);
    vc->tgt_l = pos;
    vc->tgt_r = pos;
    vc->vel_l = vel;
    vc->vel_r = vel;
    motors->tare_position();
}

void movet(Drivetrain* motors, int time, int vel) {
    motors->move_velocity(vel);
    delay(time);
    motors->move_velocity(0);
    delay(50);
    motors->tare_position();
}

void movevc(Drivetrain* motors, vel_ctrl_t* vc, double dist) {
    *vc = {};
    moved(motors, vc, dist);
    vel_ctrl(motors, vc);
    delay(50);
}

void turnvc(Drivetrain* motors, vel_ctrl_t* vc, double dist, int angle) {
    *vc = {};
    turn(motors, vc, dist, angle);
    vel_ctrl(motors, vc);
    delay(50);
}

void turn(Drivetrain* motors, vel_ctrl_t* vc, double dist, int angle) {
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
    double vel = (double)rpm[E_MOTOR_GEAR_GREEN] * in / out;
    vc->tgt_l = flip ? out : in;
    vc->tgt_r = flip ? in : out;
    vc->vel_l = fsign * (flip ? rpm[E_MOTOR_GEAR_GREEN] : vel);
    vc->vel_r = fsign * (flip ? vel : rpm[E_MOTOR_GEAR_GREEN]);
    motors->tare_position();
}

void turnh(Drivetrain* motors, Imu* gyro, double heading, int vel) {
    for (;;) {
        if (absf(gyro->get_rotation() - (heading + 360.0)) < absf(gyro->get_rotation() - heading))
            heading += 360.0;
        else if (absf(gyro->get_rotation() - (heading - 360.0)) < absf(gyro->get_rotation() - heading))
            heading -= 360.0;
        else
            break;
    }
    int sign = (2 * (heading > gyro->get_rotation()) - 1);
    vel = abs(vel) * sign;
    motors->lf.move_velocity(vel);
    motors->rf.move_velocity(-vel);
    motors->lr.move_velocity(vel);
    motors->rr.move_velocity(-vel);
    for (; sign * (heading - gyro->get_rotation()) > 65.0; delay(10));
    vel *= 0.2;
    motors->lf.move_velocity(vel);
    motors->rf.move_velocity(-vel);
    motors->lr.move_velocity(vel);
    motors->rr.move_velocity(-vel);
    for (; sign * (heading - gyro->get_rotation()) > 5.0; delay(10));
    motors->move_velocity(0);
    delay(50);
    motors->tare_position();
}

void vel_ctrl(Drivetrain* motors, vel_ctrl_t* vc) {
    double last_l = 0, last_r = 0;
    double new_l = 0, new_r = 0;
    double sign_l = 2 * (vc->tgt_l > 0) - 1;
    double sign_r = 2 * (vc->tgt_r > 0) - 1;
    motors->tare_position();
    for (; sign_l * (vc->tgt_l - motors->lf.get_position()) > 5
        && sign_r * (vc->tgt_r - motors->rf.get_position()) > 5; delay(10)) {
        if (sign_l * (vc->tgt_l - motors->lf.get_position()) < 200) 
            new_l = vc->vel_l * 0.5;
        else if (sign_l * (vc->tgt_l - motors->lf.get_position()) < 400)
            new_l = vc->vel_l * 0.8;
        else
            new_l = vc->vel_l;
        if (sign_r * (vc->tgt_r - motors->rf.get_position()) < 200)
            new_r = vc->vel_r * 0.5;
        else if (sign_r * (vc->tgt_r - motors->rf.get_position()) < 400)
            new_r = vc->vel_r * 0.8;
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
    motors->move_velocity(0);
    motors->tare_position();
}

void track(Drivetrain* motors, Vision* vision, vis_params_t* vp) {
    motors->lf.move_velocity(vp->vel_l);
    motors->rf.move_velocity(vp->vel_r);
    motors->lr.move_velocity(vp->vel_l);
    motors->rr.move_velocity(vp->vel_r);
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
    delay(vp->dly);
    motors->move_velocity(0);
    motors->tare_position();
}
