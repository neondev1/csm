#include "main.h"
#include "head.h"

#define CIRC    333.3
#define PI      3.1416
#define WIDTH   320.0

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

int _abs(int num) {
    int mask = num >> sizeof(int) - 1;
    return (num ^ mask) - mask;
}
// casual patent rights infringement

void move(Drivetrain* motors, vel_ctrl_t* vc, double dist) {

}

void turn(Drivetrain* motors, vel_ctrl_t* vc, double dist, int angle) {
    int th = _abs(angle);
    int dir = angle > 0;
    if (!th || (th >= 180 && dist))
        return;
    int sign = dist < 0;
    double rad = sqrt(dist * dist / (2.0 - 2.0 * _cos[th]));
    double out = PI * (rad + WIDTH / 2) * (dist > 0 ? 1 : -1) * (double)th / 180.0;
    double in = PI * (rad - WIDTH / 2) * (dist > 0 ? 1 : -1) * (double)th / 180.0;
    double vel = rpm[E_MOTOR_GEAR_GREEN] * in / out;
    vc->tgt_l = dir ? out : in;
    vc->tgt_r = dir ? in : out;
    vc->vel_l = (1 - sign * 2) * ((dir ^ sign) ? rpm[E_MOTOR_GEAR_GREEN] : vel);
    vc->vel_r = (1 - sign * 2) * ((dir ^ sign) ? vel : rpm[E_MOTOR_GEAR_GREEN]);
    motors->lf.tare_position();
    motors->rf.tare_position();
    motors->lr.tare_position();
    motors->rr.tare_position();
}

void vel_ctrl(Drivetrain* motors, vel_ctrl_t* vc) {
    
}
