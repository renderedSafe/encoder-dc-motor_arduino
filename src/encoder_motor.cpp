#include "encoder_motor.h"
#include <Arduino.h>
#include <math.h>

//**********************DC Motor Class*******************

DcMotor::DcMotor(int pin_fwd, int pin_bkwd, Direction dir=FORWARD, int pwm_resolution=8)
{
    _pin_fwd = pin_fwd;
    _pin_bkwd = pin_bkwd;
    direction = dir;
    pwm_res = pwm_resolution;
    max_pwm_value = pow(2, pwm_res);  //not as readable, but if we do this calc here and store it, less math later

    
}

void DcMotor::init()
{
    pinMode(_pin_fwd, OUTPUT);
    pinMode(_pin_bkwd, OUTPUT);
}

void DcMotor::setDirection(Direction dir)
{
    direction = dir;
}

//Meant to take -1.0 to 1.0
void DcMotor::setPower(float pwr)
{
    power = pwr;  //we want to be able to know what power the motor is being run at
    float scaled_power = pwr * max_pwm_value;  //multiply 

    //handle an overflow. commanded power capped at a ratio of 1:max power
    if (abs(pwr) >= 1)
    {
        scaled_power = copysign(max_pwm_value, scaled_power); 
    }


    if (scaled_power > 0)
    {
        analogWrite(_pin_fwd, abs((int)scaled_power));
        analogWrite(_pin_bkwd, 0);
    }
    else
    {
        analogWrite(_pin_fwd, 0);
        analogWrite(_pin_bkwd, abs((int)scaled_power));
    }
}

float DcMotor::getPower()
{
    return power;
}

//*********************Odometer class************************

Odometer::Odometer(int enc_pin_1, int enc_pin_2, int spd_update_interval_ms) 
    : encoder(enc_pin_1, enc_pin_2)
{
    calc_interval_ms = spd_update_interval_ms;
    //these two values must be initialed to prevent issues with referencing them below
    prev_pos = encoder.read();
    prev_millis = millis();
    last_speed = 0;
}

bool Odometer::update()
{
    //early exit for most common situation (too early to recalc speed)
    if ((millis() - prev_millis) < calc_interval_ms)
    {
        return false;
    }

    else
    {
        last_speed = getSpeed();
        return true;
    }
}

float Odometer::getSpeed()
{
    float spd = float(encoder.read() - prev_pos)/float(millis() - prev_millis) * 1000;  //convert to pulses/second
    prev_millis = millis();
    prev_pos = encoder.read();
    return spd;
}

//************************Encoder DC Motor Class*****************
EncoderDcMotor::EncoderDcMotor(int pin_fwd, int pin_bkwd, Direction dir, int pwm_resolution=8,
                                    int enc_pin_1, int enc_pin_2, int maxspeed, float kp, float ki, float kd) : 
                                    dc_motor(DcMotor(pin_fwd, pin_bkwd, dir, pwm_resolution)),
                                    odometer(Odometer(enc_pin_1, enc_pin_2)),
                                    pid_controller(PID(&_input, &_output, &_setpoint, kp, ki, kd, DIRECT))
{
    max_speed = maxspeed;
    pid_controller.SetMode(AUTOMATIC);
    pid_controller.SetOutputLimits(-1000, 1000);
    pid_controller.SetSampleTime(5);
}

void EncoderDcMotor::update()
{
    //TODO: only do this stuff if we've gotten new odo data to work with
    // UPDATE: The below solution should accomplish this TODO. Verify to close out
    if (odometer.update())
    {
        _input = odometer.last_speed;
        pid_controller.Compute();


        //!!!!!!!!!!!!!!!HARDWARE SAFETY CODE BELOW!!!!!!!!!!!!!!!!

        //When geared brushed motors are in motion, a current applied in the opposite 
        //direction of rotation is VERY BAD for the motor. This makes sure we only apply 
        //current in the direction we're trying to move and relies on passive braking
        //rather than active control to slow the motor down. Some amount of negative 
        //current may be acceptable to control more effectively, but for now we're playing it
        //safe. 
        if ((_output<0)==(_setpoint<0))
        {
            dc_motor.setPower(_output/1000);
        }
        else
        {
            dc_motor.setPower(0);
        }
    }  
}

//TODO: set speed in mm/s instead of proportion of max speed
void EncoderDcMotor::setSpeed(float speed)
{
    _setpoint = speed * max_speed;
}
