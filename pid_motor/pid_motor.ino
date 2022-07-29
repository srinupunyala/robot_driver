#include <PID_v1.h>
#include "encoder_motor.h"
#include <Wire.h>

#define R 0.25
#define D 0.216

//double Kp=1.21, Ki=1.3, Kd=0.5;
//double rKp=0.95, rKi=1, rKd=0.6;
// max velocity 0.5 m/s
DCEMotor left(7,8,5,2,4);
DCEMotor right(13,12,11,3,6);

volatile double rpm_in, target_rpm, pwm_out;
//double Kp=0.69, Ki=0.73, Kd=0.08;  // HERE
double Kp=1.1, Ki=0.17, Kd=0.12;

volatile double rrpm_in, rtarget_rpm, rpwm_out;
//double rKp=0.56, rKi=0.59, rKd=0.12;
double rKp=0.98, rKi=0.13, rKd=0.14;

PID myPID_l(&rpm_in, &pwm_out, &target_rpm, Kp, Ki, Kd, DIRECT);
PID myPID_r(&rrpm_in, &rpwm_out, &rtarget_rpm, rKp, rKi, rKd, DIRECT);

POS_T prev_pos = 0;
POS_T prev_pos_r = 0;
long counter_stop=0;
bool done = false;
void setup() {
  Wire.begin(8);
  Wire.onRequest(on_request);
  Wire.onReceive(on_receive);

  //done = true;
  // put your setup code here, to run once:
  Serial.begin(115200);
  myPID_l.SetMode(AUTOMATIC);
  myPID_r.SetMode(AUTOMATIC);
  left.init();
  right.init();
  attachInterrupt(digitalPinToInterrupt(2), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(3), readRightEncoder, RISING);
  left.set_direction(1);
  right.set_direction(1);
  //left.set_speed(80);
  double target_speed = 0.45;
  //target_rpm = target_speed*60/R;
  target_rpm = 0;
  rtarget_rpm = 0;
  prev_pos = left.get_position();
  prev_pos_r = right.get_position();
}

float w=0;
long prev = 0;
void loop() {
  // put your main code here, to run repeatedly:
  long curr = millis();
//  if (curr>=1000)
//    done = false;
//  if (curr >= 25000) {
//    left.set_speed(0);
//    right.set_speed(0);
//    done = true;
//  } else if (curr >= 15000) {
//    done = false;
//  } else if (curr - counter_stop >= 10000) {
//    left.set_speed(0);
//    right.set_speed(0);
//    done = true;
//  }
  double vl, vr;
  double deltaT = (double)(curr - prev);
  if (deltaT >= 200 && !done) {
    POS_T curr_pos = abs(left.get_position()); 
    POS_T curr_pos_r = abs(right.get_position()); 
    //rpm_in = ((double)(curr_pos-prev_pos)/618.75)/(deltaT/1000.0);
    //rpm_in = rpm_in*2*60.0;
    rpm_in = (300.0*1000*(double)(curr_pos-prev_pos)/(2475.0*deltaT));
    rrpm_in = (300.0*1000*(double)(curr_pos_r-prev_pos_r)/(2475.0*deltaT));
    prev = curr;
    prev_pos = curr_pos;
    prev_pos_r = curr_pos_r;
    vl = rpm_in * R/60;
    vr = rrpm_in * R/60;
  }
  //if (!done && (abs(rpm_in - target_rpm) > 0.1 || abs (rrpm_in - target_rpm) > 0.1)) {
  myPID_l.Compute();
  myPID_r.Compute();
  int control = (int)pwm_out;
  int control_r = (int)rpwm_out;
//    Serial.print("RPM-L: ");
//    Serial.print(rpm_in);
//    Serial.print("  RPM-R: ");
//    Serial.print(rrpm_in);
//    Serial.print("  W: ");
//    Serial.print((vr-vl)/D);
//    Serial.print("  control-L: ");
//    Serial.print(pwm_out);
//    Serial.print("  control-R: ");
//    Serial.println(rpwm_out);
  if (target_rpm == 0)
    control=0;
  if (rtarget_rpm == 0)
    control_r=0;
  left.set_speed(control);
  right.set_speed(control_r);
//  } else {
//    //done = true;
//  }
delay(10);
}

void readLeftEncoder() {
  left.read_encoder();
}

void readRightEncoder() {
  right.read_encoder();
}

void on_request() {
  POS_T lp = left.get_position()*-1;
  POS_T rp = right.get_position();
  unsigned char buff[8];
  for(int i=0; i<4;++i) {
    buff[i] = (lp >> (i*8)) & 0xFF;
  }
  for(int i=0; i<4;++i) {
    buff[i+4] = (rp >> (i*8)) & 0xFF;
  }
  Wire.write(buff,8);
  Serial.print("sending lp: ");
  Serial.print(lp);
  Serial.print("  rp: ");
  Serial.println(rp);
}

void on_receive(int count) {
  Serial.println("received cmd: ");
  unsigned char rbuff[3];
  for (int i=0; i<3;++i) {
    rbuff[i] = Wire.read();
  }
  target_rpm = rbuff[0];
  rtarget_rpm = rbuff[1];
  if (rbuff[2] & 1)
    left.set_direction(0);
  else
    left.set_direction(1);
  
  if (rbuff[2] & 2)
    right.set_direction(0);
  else
    right.set_direction(1);
  Serial.print(target_rpm);
  Serial.print(" ");
  Serial.print(rtarget_rpm);
  Serial.print(" ");
  Serial.println(rbuff[2]);
}
