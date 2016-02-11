#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h> 
#include <Arduino.h>


#ifndef MAX_RAMP_PWM_PER_MS_POSITIVE_DELTA // Max steep for a ramp when a zero crossing happen.  The ramp is positive (from a negative PWM to a positive PWM)
                             // The PWM has set to 0 and wait before apply the new PWM, accordingly to this max steep ramp
#define MAX_RAMP_PWM_PER_MS_POSITIVE_DELTA 0.07
//#define MAX_RAMP_PWM_PER_MS_POSITIVE_DELTA 0.02 //  Max velocity of INCREASE a PWM per msec, (NOTE:  BREAK SEEMS APPLIED BY THE ESC)
                                                //0.04 seems the max velocity, but for consitency it is better to set equal in both cases
#warning MAX_RAMP_PWM_PER_MS_POSITIVE_DELTA not defined, using default
#endif

#ifndef MAX_RAMP_PWM_PER_MS_NEGATIVE_DELTA // Max steep for a ramp when a zero crossing happen. The ramp is negative (from a positive PWM to a negative PWM)
#define MAX_RAMP_PWM_PER_MS_NEGATIVE_DELTA 0.07
//#define MAX_RAMP_PWM_PER_MS_NEGATIVE_DELTA 0.02 // Max velocity of INCREASE a PWM per msec, (NOTE: NO BREAK APPLIED BY THE ESC??)
#warning MAX_RAMP_PWM_PER_MS_NEGATIVE_DELTA not defined, using default
#endif

//#define BLDC_MTR_DEBUG 1 // it deosnt work if set in the main. mah
#define STOP_PWM_ESC 1500 // which is the stop pwm?

class Controller
{
    public:
        enum driver {L298, BTS7960, ESC};
        Controller(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB);
        unsigned int  spin(int pwm);
        int latest_spin_applied() { return this->prev_pwm_;};

    private:
        Servo motor_;
        driver motor_driver_;
        int pwm_pin_;
        int motor_pinA_;
        int motor_pinB_;
        int prev_pwm_;
        int next_pwm_;                  // USED ONLY FOR ESC
        unsigned long rampStartMillis;  // USED ONLY FOR ESC
#if BLDC_MTR_DEBUG==1
        char debug_interrupt_msg_[400];
#endif 
        
};

#endif
