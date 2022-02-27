
#include <PID_v1.h>
#include "Motor.h"
#include <ros.h>
#include <vacumm_hardware/WheelCmd.h>
#include <vacumm_hardware/WheelState.h>
#include <vacumm_hardware/VacummDiag.h>
#define ROS_SERIAL true

#define RIGHT_MOTOR_PIN_A 22
#define RIGHT_MOTOR_PIN_B 23
#define RIGHT_MOTOR_ENCODER_A 5
#define RIGHT_MOTOR_ENCODER_B 18
#define RIGHT_MOTOR_ENABLE_PIN 19
#define LED_PIN 2

#define LEFT_MOTOR_PIN_A 25
#define LEFT_MOTOR_PIN_B 26
#define LEFT_MOTOR_ENCODER_A 12
#define LEFT_MOTOR_ENCODER_B 13
#define LEFT_MOTOR_ENABLE_PIN 27
#define VELOCITY_TO_PULSE_MULTIPLIER 150.50

#define INTERVAL 100
#define PWM_CHANNEL 0
#define NO_COMM_MAX 50
const int TICKS_PER_REVOLUTION = 420;
const double RADS_PER_TICK_COUNT = (2 * PI) / TICKS_PER_REVOLUTION;
const double WHEEL_RADIUS = 0.04;

//wheels
volatile long right_motor_pulse = 0;
volatile long right_motor_rpm = 0;
float right_motor_act_vel = 0;
float right_motor_vel = 0.0;
int right_motor_pos = 0;
long right_motor_curr_pulse = 0;
long right_motor_pre_pulse = 0;
int right_motor_speed = 0;


volatile long left_motor_pulse = 0;
volatile long left_motor_rpm = 0;
float left_motor_act_vel = 0;
float left_motor_vel = 0.0;
int left_motor_pos = 0;
long left_motor_curr_pulse = 0;
long left_motor_pre_pulse = 0;
int left_motor_speed = 0;

//global variables
long currentMillis = 0;
long previousMillis = 0;
long no_comm_loop = 0;

//PID Settings
double right_kp = 0.6, right_ki = 3, right_kd = 0.1;
double right_input = 0.0, right_output = 0.0, right_setpoint = 0.0;

double left_kp = 0.6, left_ki = 3, left_kd = 0.1;
double left_input = 0.0, left_output = 0.0, left_setpoint = 0.0;

PID leftMotorPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);
PID rightMotorPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT );

int leftMotorPwm[22] = {255, 207, 187, 172, 159, 147, 136, 127, 117, 110, 100, 90, 82, 76, 68, 61, 53, 46, 41, 35, 25, 10};
int rightMotorPwm[22] = {178, 166, 154, 143, 134, 125, 117, 109, 100, 94, 87, 81, 74, 67, 61, 55, 49, 43, 37, 32, 25, 10};

int leftMotorPwmAC[21] = { -190, -175, -164, -150, -141, -131, -121, -111, -105, -96, -87, -80,  -72, -65, -60, -53, -44, -38, -30, -26, -10};
int rightMotorPwmAC[21] = { -255, -215, -195, -176, -162, -149, -138, -128, -118, -111, -103, -95, -88, -81, -75, -68, -60, -54, -46, -41, -30};


//Motor
Motor leftMotor(LEFT_MOTOR_PIN_A, LEFT_MOTOR_PIN_B, LEFT_MOTOR_ENCODER_A, LEFT_MOTOR_ENCODER_B, LEFT_MOTOR_ENABLE_PIN, 0);
Motor rightMotor(RIGHT_MOTOR_PIN_A, RIGHT_MOTOR_PIN_B, RIGHT_MOTOR_ENCODER_A, RIGHT_MOTOR_ENCODER_B, RIGHT_MOTOR_ENABLE_PIN, 1);


//ROS
ros::NodeHandle nh;
vacumm_hardware::WheelCmd wheel_cmd;
vacumm_hardware::WheelState wheel_state;
vacumm_hardware::VacummDiag vacumm_diag;
ros::Publisher wheel_state_pub("/vacumm/wheel_state", &wheel_state);


ros::Publisher vacumm_diag_pub("/vacumm/diag", &vacumm_diag);


void wheel_cmd_callback(const vacumm_hardware::WheelCmd& wheelCmd) {
  set_vel(wheelCmd.vel[0], wheelCmd.vel[1]);
}

void set_vel(float left_vel, float right_vel) {
  no_comm_loop = 0;

  float newLeftVel = left_vel * WHEEL_RADIUS;
  float newRightVel = right_vel * WHEEL_RADIUS;

  setLeftPID(left_vel);
  setRightPID(right_vel);

  left_motor_vel = newLeftVel;
  right_motor_vel = newRightVel;


}

void setLeftPID(float vel) {
  int absVel = abs(vel);
  if (absVel >= 3.0 || absVel == 0.0) {
    left_kp = 0.6;
    left_ki = 3;
    left_kd = 0.1;
  } else {
    left_kp = 1;
    left_ki = 1.7;
    left_kd = 0.1;
  }

  leftMotorPID.SetTunings(left_kp, left_ki, left_kd);
}

void setRightPID(float vel) {
  int absVel = abs(vel);
  if (absVel >= 3.0 || absVel == 0.0) {
    right_kp = 0.6;
    right_ki = 3;
    right_kd = 0.1;
  } else {
    right_kp = 1;
    right_ki = 1.7;
    right_kd = 0.1;
  }

  rightMotorPID.SetTunings(right_kp, right_ki, right_kd);
}



ros::Subscriber<vacumm_hardware::WheelCmd> wheel_cmd_sub("/vacumm/wheel_cmd", &wheel_cmd_callback);


void drive(int leftSpeed, int rightSpeed) {
  leftMotor.rotate(leftSpeed);
  rightMotor.rotate(rightSpeed);

}

void flashLED(int times) {
  int i = 0;

  //turn of before loop
  turnOnLed(false);

  while (i < times)
  {
    turnOnLed(true);
    delay(500);
    turnOnLed(false);
    delay(500);

    i = i + 1;
  }
}

void turnOnLed(bool turnOn) {
  if (turnOn == true)
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }
}


void IRAM_ATTR right_motor_encoder_callback() {
  int value = digitalRead(rightMotor.encoderPinB);

  if (value == 0) {
    right_motor_pulse++;
  } else {
    right_motor_pulse--;
  }

}

void IRAM_ATTR left_motor_encoder_callback() {
  int value = digitalRead(leftMotor.encoderPinB);
  if (value == 0) {
    left_motor_pulse++;
  } else {
    left_motor_pulse--;
  }

}



void setup() {

  if (!ROS_SERIAL) {
    Serial.begin(115200);
  }
  pinMode(LED_PIN, OUTPUT);
  flashLED(3);

  attachInterrupt(rightMotor.encoderPinA, right_motor_encoder_callback, RISING);
  attachInterrupt(leftMotor.encoderPinA, left_motor_encoder_callback, RISING);

  rightMotorPID.SetMode(AUTOMATIC);
  rightMotorPID.SetSampleTime(95);
  rightMotorPID.SetOutputLimits(-45, 45); // max tick that it can go at 1000 ms (0.299 * 47 ticks)
  ////
  leftMotorPID.SetMode(AUTOMATIC);
  leftMotorPID.SetSampleTime(95);
  leftMotorPID.SetOutputLimits(-45, 45);

  drive(0, 0); //reset speed to 0;

  left_input = 0;
  right_input = 0;
  left_setpoint = 0;
  right_setpoint = 0;
  left_motor_pulse = 0;
  left_motor_pre_pulse = 0;
  right_motor_pulse = 0;
  right_motor_pre_pulse = 0;


  if (ROS_SERIAL) {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(wheel_state_pub);
    nh.advertise(vacumm_diag_pub);
    nh.subscribe(wheel_cmd_sub);
  }

  //  drive(255, 255);
}

int avg(double pre, double curr) {
  if (pre == 0) {
    pre = curr;
  }

  return (pre + curr) / 2;
}

float twoDec(float value) {
  int newValue = (int) (value * 100);
  return float (newValue) / 100;
}

float leftDiff = 0;
float leftVel = 0;
float rightDiff = 0;
float rightVel = 0;
char buffers[100];
void loop() {
  // put your main code here, to run repeatedly:
  if (ROS_SERIAL) {
    nh.spinOnce();
  } else {
    readCommand();
  }

  currentMillis = millis();

  if ((millis() - previousMillis) > INTERVAL) {
    previousMillis = currentMillis;

    //    setLeftPID(left_motor_vel);
    //    setRightPID(right_motor_vel);

    noInterrupts();
    left_motor_curr_pulse = left_motor_pulse;
    right_motor_curr_pulse = right_motor_pulse;
    interrupts();
    left_input = avg(left_input, left_motor_curr_pulse - left_motor_pre_pulse);
    right_input = avg(right_input, right_motor_curr_pulse - right_motor_pre_pulse);
    //
    //    left_input = constrain(left_input, -45, 45);
    //    right_input = constrain(right_input, -45, 45);
    //
    left_motor_pre_pulse = left_motor_curr_pulse;
    right_motor_pre_pulse = right_motor_curr_pulse;


    leftVel = left_motor_vel * VELOCITY_TO_PULSE_MULTIPLIER;
    if (left_motor_vel > 0) {
      left_setpoint = ceil(leftVel);
      leftDiff = left_setpoint - leftVel;
    } else {
      left_setpoint = floor(leftVel);
      leftDiff = leftVel - left_setpoint;
    }


    rightVel = right_motor_vel * VELOCITY_TO_PULSE_MULTIPLIER;
    if (right_motor_vel > 0) {
      right_setpoint = ceil(rightVel) ;
      rightDiff = right_setpoint - rightVel;
    } else {
      right_setpoint = floor(rightVel) ;
      rightDiff = rightVel - right_setpoint;
    }



    if (left_input >= 0) {
      left_motor_act_vel = twoDec( ((left_input - leftDiff) / VELOCITY_TO_PULSE_MULTIPLIER) / WHEEL_RADIUS ) ;
    } else {
      left_motor_act_vel = twoDec( ((left_input + leftDiff) / VELOCITY_TO_PULSE_MULTIPLIER) / WHEEL_RADIUS ) ;
    }

    if (right_input >=0) {
      right_motor_act_vel = twoDec(((right_input - rightDiff) / VELOCITY_TO_PULSE_MULTIPLIER) / WHEEL_RADIUS) ;
    } else {
      right_motor_act_vel = twoDec(((right_input + rightDiff) / VELOCITY_TO_PULSE_MULTIPLIER) / WHEEL_RADIUS) ;
    }


    left_motor_pos = left_motor_curr_pulse;// * RADS_PER_TICK_COUNT;
    right_motor_pos = right_motor_curr_pulse; // * RADS_PER_TICK_COUNT;


    //
    leftMotorPID.Compute();
    rightMotorPID.Compute();

    //    left_output = left_setpoint;
    //    right_output = right_setpoint;
    //
    //    if (left_output > 0 && abs(left_output) >= 26) {
    //      left_motor_speed = leftMotorPwm[47 -(int)abs(left_output)];
    //    } else if (left_output < 0 && abs(left_output) >= 27) {
    //      left_motor_speed = leftMotorPwmAC[47 - (int)abs(left_output)];
    //    } else {
    left_motor_speed = map (left_output, -45, 45, -255, 255);
    //    }
    //
    //    if (right_output > 0 && abs(right_output) >= 26){
    //      right_motor_speed = rightMotorPwm[47 - (int)abs(right_output)];
    //    } else if (right_output < 0 && abs(right_output) >= 27){
    //      right_motor_speed = rightMotorPwmAC[47 - (int)abs(right_output)];
    //    } else {
    right_motor_speed = map(right_output, -45, 45, -255, 255);
    //    }

    if (left_motor_vel == 0 && right_motor_vel == 0){
      drive(0, 0);
    } else {
      drive(left_motor_speed, right_motor_speed);  
    }
    //    drive(left_output, right_output);
    


    if (!ROS_SERIAL) {
      Serial.print(left_setpoint);
      Serial.print(" ");
      Serial.print(left_input);
      Serial.print(" ");
      Serial.print(right_setpoint);
      Serial.print(" ");
      Serial.print(right_input);
      Serial.print(" ");
      Serial.print(left_motor_pos);
      Serial.print(" ");
      Serial.print(right_motor_pos);
      Serial.println();

    } else {
      
     
    }


    if (no_comm_loop > NO_COMM_MAX) {
      left_motor_vel = 0;
      right_motor_vel = 0;
      drive(0, 0);
    }

    no_comm_loop++;

    if (no_comm_loop == 65535) {
      no_comm_loop = NO_COMM_MAX;
    }

    if (ROS_SERIAL) {
      publish_wheel_state();
    }

  }

  delay(1);
}


void publish_wheel_state() {
  wheel_state.vel[0] = left_motor_act_vel;
  wheel_state.vel[1] = right_motor_act_vel;
  wheel_state.pos[0] = left_motor_pos;
  wheel_state.pos[1] = right_motor_pos;

  wheel_state_pub.publish(&wheel_state);


  vacumm_diag.left_setpoint = left_setpoint;
  vacumm_diag.right_setpoint = right_setpoint;
  vacumm_diag.left_input = left_input;
  vacumm_diag.right_input = right_input;
  vacumm_diag.left_output = left_output;
  vacumm_diag.right_output = right_output;
  vacumm_diag.left_pulse = left_motor_curr_pulse;
  vacumm_diag.right_pulse = right_motor_curr_pulse;
  vacumm_diag.left_vel = left_motor_vel;
  vacumm_diag.right_vel = right_motor_vel;
  vacumm_diag_pub.publish(&vacumm_diag);
  nh.spinOnce();
}


void readCommand() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    Serial.printf("Command received %s \n", command);
    if (command.substring(0, 1).equals("v")) {
      int commaIndex = command.indexOf(',');
      double x = command.substring(1, commaIndex).toDouble();
      double z = command.substring(commaIndex + 1, command.length()).toDouble();

      set_vel(x, z);

    } else if (command.substring(0, 1).equals("p")) {
      double value = command.substring(1).toDouble();
      right_kp = value;
      rightMotorPID.SetTunings(right_kp, right_ki, right_kd);

      left_kp = value;
      leftMotorPID.SetTunings(left_kp, left_ki, left_kd);
    } else if (command.substring(0, 1).equals("i")) {
      double value = command.substring(1).toDouble();
      right_ki = value;
      rightMotorPID.SetTunings(right_kp, right_ki, right_kd);

      left_ki = value;
      leftMotorPID.SetTunings(left_kp, left_ki, left_kd);
    } else if (command.substring(0, 1).equals("d")) {
      double value = command.substring(1).toDouble();
      right_kd = value;
      rightMotorPID.SetTunings(right_kp, right_ki, right_kd);

      left_kd = value;
      leftMotorPID.SetTunings(left_kp, left_ki, left_kd);
    } else if (command.substring(0, 1).equals("s")) {
      int commaIndex = command.indexOf(',');
      double x = command.substring(1, commaIndex).toDouble();
      double z = command.substring(commaIndex + 1, command.length()).toDouble();
      no_comm_loop = 0;
      drive(x, z);
    }
  }
}
