#include "Motor.h"

Controller::Controller(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB):
    motor_driver_(motor_driver),
    pwm_pin_(pwm_pin),
    motor_pinA_(motor_pinA),
    motor_pinB_(motor_pinB),
    prev_pwm_(0),
    next_pwm_(0),
    rampStartMillis(0)
{
    switch (motor_driver)
    {
        case L298:
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));

            break;

        case BTS7960:
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(motor_pinB_, 0);
            analogWrite(motor_pinA_, 0);

            break;

        case ESC:
            motor_.attach(motor_pinA_);

            //ensure that the motor is in neutral state during bootup
            motor_.writeMicroseconds(1500);
            //TODO:  should I send the stop??
            break;
    }
}

unsigned int  Controller::spin(int pwm)
{
    // return 1 if spin is sent, othwerwise a 0
    int spin_possible = 1;
    switch (motor_driver_)
    {
        case L298:
            if(pwm > 0)
            {
                digitalWrite(motor_pinA_, HIGH);
                digitalWrite(motor_pinB_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pinA_, LOW);
                digitalWrite(motor_pinB_, HIGH);
            }
            analogWrite(pwm_pin_, abs(pwm));
            this->prev_pwm_ = pwm;
            break;

        case BTS7960:
            if (pwm > 0)
            {
                analogWrite(motor_pinA_, 0);
                analogWrite(motor_pinB_, abs(pwm));
            }
            else if (pwm < 0)
            {
                analogWrite(motor_pinB_, 0);
                analogWrite(motor_pinA_, abs(pwm));
            }
            else
            {
                analogWrite(motor_pinB_, 0);
                analogWrite(motor_pinA_, 0);
            }
            this->prev_pwm_ = pwm;
            break;
    
        case ESC:
          if (pwm==0 && this->prev_pwm_ != 0) { // RECEIVED A STOP, NON  IN STOP MODE, NEED TO MANAAGE A SLOW DOWN RAMP
              motor_.writeMicroseconds(STOP_PWM_ESC); //Will send a 500us value to stop the ESC. accordingly with https://www.robotshop.com/community/blog/show/rc-speed-controller-esc-arduino-library
              spin_possible = 0; // Setting flag spin is not possible
              // start ramp!
              if (this->rampStartMillis == 0) {
#if BLDC_MTR_DEBUG==1
sprintf(this->debug_interrupt_msg_ , "ESC: received PWM_0 %d with with ZC prev_pwm=%d,  starting ramp for PWM_0", pwm, this->prev_pwm_ );
#endif 
                this->next_pwm_ = 0;// this is also used to detect when a ramp starts
                this->rampStartMillis = millis(); // Take start time for the ramp
              } else {
                  int32_t deltaMillis = millis() - this->rampStartMillis; // get the delta t from the ramp start
                  int deltaPWM = this->next_pwm_ - this->prev_pwm_;
                  int possiblePWM=0;

                  // calculate the possible pwm, to verifify if the motor is halted
                  if ( (deltaPWM > 0) ) {
                     possiblePWM = round(deltaMillis * MAX_RAMP_PWM_PER_MS_POSITIVE_DELTA + this->prev_pwm_);
                  }
                  if (deltaPWM < 0) {
                     possiblePWM = round(-deltaMillis * MAX_RAMP_PWM_PER_MS_NEGATIVE_DELTA + this->prev_pwm_);
                  }
                  if (possiblePWM*this->prev_pwm_ < 0) { // changed sign, Zero reached
#if BLDC_MTR_DEBUG==1
sprintf(this->debug_interrupt_msg_ , "ESC(%d): received PWM_0 possible pwm is %d with with ZC prev_pwm=%d,  ZERO REACHED, REMOVE RAMP  PWM_0", deltaMillis, possiblePWM, this->prev_pwm_ );
#endif 
					 spin_possible = 1; // Setting flag spin to possible
                     this->next_pwm_ = 0;// this is also used to detect when a ramp starts
                     this->prev_pwm_ = 0;
                     this->rampStartMillis = 0;
                  } else {
#if BLDC_MTR_DEBUG==1
sprintf(this->debug_interrupt_msg_ , "ESC(%d): received PWM_0 possible pwm is %d with with ZC prev_pwm=%d,  DO NOTHING  PWM_0", deltaMillis, possiblePWM, this->prev_pwm_ );
#endif 
                  }
              }
              break; // EXIT ESC CONTROL   
          } 
          
          if (pwm==0 && this->prev_pwm_==0) { // AGAIN IN PWM
              motor_.writeMicroseconds(STOP_PWM_ESC);
              this->next_pwm_ = 0;// this is also used to detect when a ramp starts
              this->prev_pwm_ = 0;
              this->rampStartMillis = 0;
#if BLDC_MTR_DEBUG==1
sprintf(this->debug_interrupt_msg_ , "ESC: received PWM_0 REPETITION" );
#endif             
              break;
          } 
          
          if  (pwm*this->prev_pwm_ >= 0) { // RECEIVED A CHANGE WITHOUT ZERO CROSSING OR RECOVERY FROM A ZERO
#if BLDC_MTR_DEBUG==1
             sprintf(this->debug_interrupt_msg_ , "ESC: received PWM %d with no ZC", pwm);
#endif 
              motor_.writeMicroseconds(1500 + pwm);
              this->next_pwm_ = 0;// reset any eventual active ramp
              this->prev_pwm_ = pwm;// set the actual ramp      
              this->rampStartMillis = 0;      
              break;  // EXIT ESC CONTROL
          } 
          if  (pwm*this->prev_pwm_ < 0) { // ZERO CROSSING DETECT, sign has changed
              if (this->next_pwm_ == 0) { // It is the first time I received a ZC. Start the ramp
     //-------------- ZERO CROSSING DETECT FIRST DETECT --------------
                  motor_.writeMicroseconds(1500); // set idle
                  spin_possible = 0; // Setting flag spin is not possible
                  // start ramp!
                  this->next_pwm_ = pwm;// this is also used to detect when a ramp starts
                  this->rampStartMillis = millis(); // Take start time for the ramp
#if BLDC_MTR_DEBUG==1
                  sprintf(this->debug_interrupt_msg_ , "ESC: received PWM %d with with ZC prev_pwm=%d, first time, starting ramp ", pwm, this->prev_pwm_ );
#endif 
       // -------------- END  ZERO CROSSING DETECT FIRST DETECT --------------
              } else { // Not the first time I got a zero crossing situation
          //-------------- ZERO CROSSING DETECT FURTHER DETECT --------------
                  //1. Calculate the possible PWM that accordingly the ramp could be applied
                  int32_t deltaMillis = millis() - this->rampStartMillis; // get the delta t from the ramp start
                  int deltaPWM = this->next_pwm_ - this->prev_pwm_;
                  int possiblePWM=0;
                  if (deltaPWM > 0) { // Ramp is positive
                //-------------- ZERO CROSSING DETECT deltaPWM > 0 --------------
                      possiblePWM = round(deltaMillis * MAX_RAMP_PWM_PER_MS_POSITIVE_DELTA + this->prev_pwm_);
                      if (possiblePWM >= pwm) { // the requested pwm is accetable, 
#if BLDC_MTR_DEBUG==1
sprintf(this->debug_interrupt_msg_ , "ESC: POSITIVE deltaPWM(%d) - requested pwm vs possible pwm = %d , %d = round(deltaMillis(%d) * MAX_RAMP_PWM_PER_MS(%f) + this->prev_pwm_(%d) );  -> OK APPLY", deltaPWM, pwm, possiblePWM, deltaMillis, MAX_RAMP_PWM_PER_MS_POSITIVE_DELTA, this->prev_pwm_);
#endif 
                          motor_.writeMicroseconds(1500 + pwm);
                          this->next_pwm_ = 0;// reset any eventual active ramp
                          this->prev_pwm_ = pwm;// set the actual ramp            
                          break;  // EXIT ESC CONTROL
                      } else if (possiblePWM*this->prev_pwm_ < 0) { // what if the possible pwm cross the zero ? I can apply the reversed PWM
#if BLDC_MTR_DEBUG==1
sprintf(this->debug_interrupt_msg_ , "ESC: POSITIVE deltaPWM(%d) (ZC with possiblePWM) - requested pwm vs possible pwm = %d , %d = round(deltaMillis(%d) * MAX_RAMP_PWM_PER_MS(%f) + this->prev_pwm_(%d) );  ->  APPLY possiblePWM", deltaPWM, pwm, possiblePWM, deltaMillis, MAX_RAMP_PWM_PER_MS_POSITIVE_DELTA, this->prev_pwm_);
#endif 
                          motor_.writeMicroseconds(1500 + possiblePWM);// set the actual ramp // EXPERIMENTAL    
                          this->prev_pwm_ = possiblePWM;// set the actual ramp // EXPERIMENTAL   
                          this->next_pwm_ = 0;// reset any eventual active ramp
                          this->rampStartMillis = 0;
                          break;  // EXIT ESC CONTROL
                       } else {
#if BLDC_MTR_DEBUG==1
sprintf(this->debug_interrupt_msg_ , "ESC: POSITIVE deltaPWM(%d) - requested pwm vs possible pwm = %d , %d = round(deltaMillis(%d) * MAX_RAMP_PWM_PER_MS(%f) + this->prev_pwm_(%d) );  ->  NOT APPLY", deltaPWM, pwm, possiblePWM, deltaMillis, MAX_RAMP_PWM_PER_MS_POSITIVE_DELTA, this->prev_pwm_);
#endif 
                          spin_possible = 0; // Setting flag spin is not possible
                          break;  // EXIT ESC CONTROL
                      }  
               //-------------- END ZERO CROSSING DETECT deltaPWM > 0 --------------
                  } else { // Ramp is negative
               //-------------- ZERO CROSSING DETECT deltaPWM < 0 --------------
                      // I am assuming  deltaPWM==0 is never possible in this part of the code!
                      // if (deltaPWM==0) should be checked, NOTE: during test i didnt notice any problem. Probably is assured by the design :) 
                      possiblePWM = round(-deltaMillis * MAX_RAMP_PWM_PER_MS_NEGATIVE_DELTA + this->prev_pwm_);
                      if (possiblePWM <= pwm) { // the requested pwm is accetable,  
#if BLDC_MTR_DEBUG==1
sprintf(this->debug_interrupt_msg_ , "ESC: NEGATIVE deltaPWM(%d) - requested pwm vs possible pwm = %d , %d = round(-deltaMillis(%d) * MAX_RAMP_PWM_PER_MS(%f) + this->prev_pwm_(%d) );  ->  OK APPLY", deltaPWM, pwm, possiblePWM, deltaMillis, MAX_RAMP_PWM_PER_MS_NEGATIVE_DELTA, this->prev_pwm_);
#endif 
                          motor_.writeMicroseconds(1500 + pwm);
                          this->next_pwm_ = 0;// reset any eventual active ramp
                          this->prev_pwm_ = pwm;// set the actual ramp      
                          this->rampStartMillis = 0;      
                          break;  // EXIT ESC CONTROL
                      } else if (possiblePWM*this->prev_pwm_ < 0) { // what if the possible pwm cross the zero ? I can apply the reversed PWM
#if BLDC_MTR_DEBUG==1
sprintf(this->debug_interrupt_msg_ , "ESC: NEGATIVE deltaPWM(%d) - requested pwm vs possible pwm = %d , %d = round(-deltaMillis(%d) * MAX_RAMP_PWM_PER_MS(%f) + this->prev_pwm_(%d) );  ->  APPLY possiblePWM", deltaPWM, pwm, possiblePWM, deltaMillis, MAX_RAMP_PWM_PER_MS_NEGATIVE_DELTA, this->prev_pwm_);
#endif 
                          motor_.writeMicroseconds(1500 + pwm);// set the actual ramp // EXPERIMENTAL    
                          this->prev_pwm_ = pwm;// set the actual ramp // EXPERIMENTAL   
                      } else {
        
#if BLDC_MTR_DEBUG==1
sprintf(this->debug_interrupt_msg_ , "ESC: NEGATIVE deltaPWM(%d) - requested pwm vs possible pwm = %d , %d = round(-deltaMillis(%d) * MAX_RAMP_PWM_PER_MS(%f) + this->prev_pwm_(%d) );  ->  NOT APPLY", deltaPWM, pwm, possiblePWM, deltaMillis, MAX_RAMP_PWM_PER_MS_NEGATIVE_DELTA, this->prev_pwm_);
#endif 
                          spin_possible = 0; // Setting flag spin is not possible   
                          break;  // EXIT ESC CONTROL                  
                      }
               //-------------- ZERO CROSSING DETECT deltaPWM < 0 --------------
                  } 
                }            
          }
          break;
    }
  #if BLDC_MTR_DEBUG==1
    Serial.println("#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*"); 
    Serial.print("prev pwm= "); Serial.print(this->prev_pwm_); Serial.print(" requested pwm "); Serial.print(pwm);  Serial.print(" next pwm "); Serial.println(this->next_pwm_);
    Serial.print("msg ");Serial.println(this->debug_interrupt_msg_);
    Serial.println("#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*");  
    
  #endif 
  return spin_possible;
}
