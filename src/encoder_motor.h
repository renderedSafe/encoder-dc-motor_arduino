#pragma once
#ifndef encoder_motor_h
#define encoder_motor_h
#include <Encoder.h>
#include <Arduino.h>
#include <PID_v1.h>


enum Direction {BACKWARD = -1, FORWARD = 1};
class DcMotor
{
    public:
        DcMotor(int pin_fwd, int pin_bkwd, Direction dir=FORWARD);
        void init();
        void setDirection(Direction dir);
        void setPower(float power);
        float getPower();
        void setPwmResolution(int bits) {pwm_res = bits;}

    private:
    	int pwm_res = 8;  //number of bits in PWM resolution
    	int max_pwm_value;
        float power;
        int direction;
        int _pin_fwd, _pin_bkwd;

};

//Encoder abstraction class. Provides speed calculation. TODO: Add pulse to distance conversion
class Odometer
{
    public:
        Encoder encoder;
        //Update function manages calculations. Call this every loop. 
        bool update();
        float last_speed;
        Odometer(int enc_pin_1, int enc_pin_2, int spd_update_interval_ms=5);
    private:
        long prev_pos;
        unsigned long prev_millis;
        int calc_interval_ms;
        //Calculates speed in pulses per second. Accessed by update().  
        float getSpeed();
};

//this is a good opportunity to use inheritance
class EncoderDcMotor
{
    public:
        int max_speed;  //Max speed in pulses per second the motor is allowed to run. Unimplemented. 
        DcMotor dc_motor;
        Odometer odometer;
        PID pid_controller;
        EncoderDcMotor(int pin_fwd, int pin_bkwd, Direction dir,\
                       int enc_pin_1, int enc_pin_2, int maxspeed=6000, float kp=1.5, float ki=0.001, float kd=0.001);
        void init() {dc_motor.init();}  //one reason inheritance would be good
        //Updates current speed and computes PID output. Call this every loop. 
        void update(); 
        //Sets a target number of pulses per second to see on the encoder. 
        void setSpeed(float speed);
        //Gets the PID output value. 
        double getOutput() {return _output;}
        double getSetpoint() {return _setpoint;}
    private:
        double _setpoint, _input, _output;  //PID variables
        

};
#endif
