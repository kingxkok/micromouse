#include "mbed.h"


Serial pc(SERIAL_TX, SERIAL_RX);


PwmOut LeftMotorPWMF(PA_7);
PwmOut RightMotorPWMF(PB_10);

PwmOut LeftMotorPWMB(PB_6);
PwmOut RightMotorPWMB(PC_7);

int left_travelled = 0;
int right_travelled = 0;

InterruptIn AA1(PB_3); 
InterruptIn right_encoder(PA_15); //right //b
InterruptIn left_encoder(PA_1); //left //g
InterruptIn HH8(PC_4);

void IE_right(){right_travelled++;}
void IE_left(){left_travelled++;}

Ticker Systicker; //to do error calculator and PID at regular intervals
Timer timer; //for dt calculations for Derivative controller

DigitalOut myled(LED1);

const int Kp = 1;
const int Kd = 1;
const int CORRECTION_PERIOD = 300; //in ms

int encoder_error = 0;
int prev_error = 0;
int correction = 0;
const int pwm_period = 10000;
 int base_speed = 2000;

int P_Controller(int error) {
    int correct = Kp*error;
    return correct;    
}

int D_Controller(int error){
  int dError = error - prev_error;
  int dt = timer.read_us();
  timer.reset();
  prev_error = error;
  int loc_correction = Kd*dError/dt;
  return loc_correction;

}

void systick() {
    encoder_error = right_travelled - left_travelled;
    correction = P_Controller(encoder_error) + D_Controller(encoder_error);
}



int main() {
    right_encoder.rise(&IE_right);
    right_encoder.fall(&IE_right);
    left_encoder.rise(&IE_left);
    left_encoder.fall(&IE_left);
    
    
    
    //mypwm = 0.5;
    printf("Left pwm set to %.2f %%\n", LeftMotorPWMF.read() * 100);
    
    Systicker.attach_us(&systick, 200);
    
    //Motors
    LeftMotorPWMF.period_us(pwm_period);
    RightMotorPWMF.period_us(pwm_period);
    
    while(1) {
       // delay(100);
        LeftMotorPWMF.pulsewidth_us(base_speed+correction);
        RightMotorPWMF.pulsewidth_us(base_speed-correction);
        wait_ms(CORRECTION_PERIOD); 
   //     LeftMotorPWMF.pulsewidth_us(0);
     //   RightMotorPWMF.pulsewidth_us(0);
     //   pc.printf("right: %d left: %d\r\n", right_travelled,left_travelled);
    }
 
  
}
