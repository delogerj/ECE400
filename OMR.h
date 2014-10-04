/* 
 * File:   OMR.h
 * Author: DeLoge
 *
 * Created on September 29, 2014, 1:27 PM
 */

#ifndef OMR_H
#define	OMR_H

// PID
extern struct controller
{
    double error_kp;
    double error_ki;
    double error_kd;

} pid;

// MOTOR
extern struct drive
{
    double rpm;
    double average_rpm[100];
    double current_A;
    double current_B;
    double dutycycle_pwm;
    double angle[2];
    int copy_POSCNT;
    int angle_position[2];
    int speed;
    int direction;

} motor;

// SENSORS
extern struct sensors
{
    double degree_bar;
    double copy_bar_inch;
    double distance_bar_inch;
    double distance_bar_cm;
    unsigned int data_accel;
    unsigned int data_current;
    int data_sound;

} sensor;

// LCD
extern struct display
{
    char line1[8];
    char line2[8];

} lcd;

// COUNTERS
extern struct vars
{
    int encoder;
    int rpm;
    int cycle_for;
    int digits;
    int debounce;
    int input_user;
    int copy_input_user;
    int direction_bar;
    int index_array;
    int i;

} counter;

#endif	/* OMR_H */

