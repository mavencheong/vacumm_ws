#include "Motor.h"

//#include "PID.h"
#include <PID_v1.h>
#include "Kinematics.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>



#define MOTOR_MAX_RPM 67
#define MOTOR_MAX_RPM_TICKS 474
#define WHEEL_DISTANCE 0.235

#define RIGHT_MOTOR_PIN_A 22
#define RIGHT_MOTOR_PIN_B 23
#define RIGHT_MOTOR_ENCODER_A 12
#define RIGHT_MOTOR_ENCODER_B 13
#define RIGHT_MOTOR_ENABLE_PIN 19
#define LED_PIN 2

#define LEFT_MOTOR_PIN_A 25
#define LEFT_MOTOR_PIN_B 26
#define LEFT_MOTOR_ENCODER_A 5
#define LEFT_MOTOR_ENCODER_B 18
#define LEFT_MOTOR_ENABLE_PIN 27

#define INTERVAL 100
#define PWM_CHANNEL 0

const int TICKS_PER_REVOLUTION = 420;
const double WHEEL_RADIUS = 0.0425;
const double WHEEL_DIAMETER = WHEEL_RADIUS * 2;
const double WHEEL_CIRCIMFERENCE = 0.267; //METER
const double WHEEL_BASE = 0.235;
const double MAX_SPEED = 0.299;
const double TICKS_PER_METER = 1572.825;
const double TICKS_PER_SEC = 471;
const double TRAVELLED_PER_SEC = 0.299; // meter per sec
const double TRAVELLED_PER_REVOLUTION = 0.267; //meter
const double TRAVELLED_PER_TICK = 0.0006;  //meter

volatile long right_motor_pulse = 0;
volatile long right_motor_pre_pulse = 0;
volatile long right_motor_rpm = 0;
double right_motor_act_vel = 0;

volatile long left_motor_pulse = 0;
volatile long left_motor_pre_pulse = 0;
volatile long left_motor_rpm = 0;
double left_motor_act_vel = 0;


Motor leftMotor(LEFT_MOTOR_PIN_A, LEFT_MOTOR_PIN_B, LEFT_MOTOR_ENCODER_A, LEFT_MOTOR_ENCODER_B, LEFT_MOTOR_ENABLE_PIN, 0);
Motor rightMotor(RIGHT_MOTOR_PIN_A, RIGHT_MOTOR_PIN_B, RIGHT_MOTOR_ENCODER_A, RIGHT_MOTOR_ENCODER_B, RIGHT_MOTOR_ENABLE_PIN, 1);
Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, 0, WHEEL_DISTANCE, 8);

double right_kp = 0.5, right_ki = 2, right_kd = 0;
double right_input = 0.0, right_output = 0.0, right_setpoint = 0.0;

double left_kp = 0.5, left_ki = 2, left_kd = 0;
double left_input = 0.0, left_output = 0.0, left_setpoint = 0.0;


PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT );




void IRAM_ATTR right_motor_encoder_callback(){
  
  int value = digitalRead(rightMotor.encoderPinB);

  if (value == 0) {
      right_motor_pulse++;
  }else {
      right_motor_pulse--;  
  }  
}

void IRAM_ATTR left_motor_encoder_callback(){
  int value = digitalRead(leftMotor.encoderPinB);
  int increment = 1;
  if (value ==0){
      left_motor_pulse++;  
  } else {
      left_motor_pulse--;  
  }

}

long currentMillis = 0;
long previousMillis = 0;

float targetX;
float targetY;
float targetZ;

double target_speed_left;
double target_speed_right;

double actual_speed_left;
double actual_speed_right;


int rpm = 0;
long previous = 0;
long diff = 0;
int left_pwm = 0;
double speed_pwm_ratio = 0.001731;
double min_speed_cmd = 0.1182;
int noCommMax = 50;
int noCommLoop = 0;
float leftMotorFilter = 0;
float leftMotorPreFilter = 0;
float rightMotorFilter = 0;
float rightMotorPreFilter = 0;
Kinematics::output motorRpm;
Kinematics::velocities motorVel;

double speed = 0;



ros::NodeHandle nh;

std_msgs::Int16 left_motor_tick_count;
std_msgs::Int16 right_motor_tick_count;
//geometry_msgs::Vector3Stamped speed_msg;      

ros::Publisher rightPub("right_ticks", &right_motor_tick_count);
ros::Publisher leftPub("left_ticks", &left_motor_tick_count);


std_msgs::String chat_msg;
ros::Publisher messagePub("chatter", &chat_msg);  
geometry_msgs::Vector3Stamped speed_msg; 
ros::Publisher speedPub("speed", &speed_msg);    

void command_vel_callback(const geometry_msgs::Twist& cmdVel){
  noCommLoop = 0;

  left_motor_pulse = 0;
  left_motor_pre_pulse = 0;
  right_motor_pulse = 0;
  right_motor_pre_pulse = 0;

  char data[100];
  sprintf(data, "%f", cmdVel.linear.x);
  chat_msg.data = data;
  messagePub.publish( &chat_msg );

  doVel(cmdVel.linear.x, cmdVel.angular.z);
  
  
}

void doVel(double x, double z){
   targetX = x - (z * (WHEEL_BASE / 2));
   targetZ = x + (z * (WHEEL_BASE / 2)) ;

   targetX = constrain(targetX, -0.299, 0.299); //max speed supported by the motor in meter/sec
//   targetZ = constrain(targetZ , -1, 1);// max rad/s
}


void drive(int leftSpeed, int rightSpeed){
  leftMotor.rotate(leftSpeed);
  rightMotor.rotate(rightSpeed);
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &command_vel_callback);

void flashLED(int times)
  {
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

 void turnOnLed(bool turnOn)
  {
    if (turnOn == true)
    {
      digitalWrite(LED_PIN, HIGH);
    }
    else
    {
      digitalWrite(LED_PIN, LOW);
    }
  }

void setup() {
//  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  flashLED(3);
  attachInterrupt(rightMotor.encoderPinA, right_motor_encoder_callback, RISING);
  
  attachInterrupt(leftMotor.encoderPinA, left_motor_encoder_callback, RISING);


  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(INTERVAL);
  rightPID.SetOutputLimits(-47, 47); // max tick that it can go at 1000 ms (0.299 * 47 ticks)
////
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(INTERVAL);
  leftPID.SetOutputLimits(-47, 47);

  
  leftMotor.rotate(0);
  rightMotor.rotate(0);
  
  left_input = 0;
  right_input = 0;
  left_setpoint = 0;
  right_setpoint = 0;
  left_motor_pulse = 0;
  left_motor_pre_pulse = 0;
  right_motor_pulse = 0;
  right_motor_pre_pulse = 0;


  //leftMotor.rotate(-255);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.advertise(messagePub);
  nh.advertise(speedPub);
  nh.subscribe(subCmdVel);
//  drive(255, 255);
   
}

// 1ms = 10 ticks
// 6200 ticks per meter
//1680 ticks per cycle
// rpm 65
// max speed  = 0.270 meter / s
// wheel distance 0.235 meter. Half is 0.1175 meter
// 1 sec = 1933 ticks, 10 secs = 
//19.5 ticks per 10 milisec at 255 power
// take 3.98 round to reach 1 meter
// take 6686 ticks to reach 1 meter
//72.22 for 10 miliseconds , the max speed is 0.270 meter for the wheel able to reach in 1 secs and max ticks motor can produce is 19.5 ticks.



long left_motor_current_pulse = 0;
long right_motor_current_pulse = 0;
double left_motor_act_speed = 0;
double right_motor_act_speed = 0;
int leftSpeed=0;
int rightSpeed=0;

void loop(){
  
//  readCommand();
  nh.spinOnce();
  
  currentMillis = millis();
  
  if ((millis() - previousMillis) > INTERVAL){
    
    previousMillis = currentMillis;
    left_motor_current_pulse = left_motor_pulse;
    right_motor_current_pulse = right_motor_pulse;
       
    left_input = left_motor_current_pulse - left_motor_pre_pulse;
    right_input = right_motor_current_pulse - right_motor_pre_pulse;

   
    left_motor_pre_pulse = left_motor_current_pulse;
    right_motor_pre_pulse = right_motor_current_pulse;

    left_setpoint = targetX * 157.2;
    right_setpoint = targetZ * 157.2;

    left_motor_act_speed = left_input / 157.2;
    right_motor_act_speed = right_input / 157.2;

    leftPID.Compute();
    rightPID.Compute();

    leftSpeed = map (left_output, -47, 47, -255, 255);
    rightSpeed = map(right_output, -47, 47, -255, 255);
    drive(leftSpeed, rightSpeed);


//    Serial.print(targetX);
//    Serial.print(" ");
//    Serial.print(targetZ);
//    Serial.print(" ");
//    Serial.print(left_setpoint);
//    Serial.print(" ");
//    Serial.print(left_input);
//    Serial.print(" ");
//    Serial.print(left_output);
//    Serial.print(" ");
//    Serial.print(leftSpeed);
//    Serial.print(" ");
//    Serial.print(left_motor_act_speed);
//    Serial.print(" ");
//    Serial.print(right_setpoint);
//    Serial.print(" ");
//    Serial.print(right_input);
//    Serial.print(" ");
//    Serial.print(right_output);
//    Serial.print(" ");
//    Serial.print(rightSpeed);
//    Serial.print(" ");
//    Serial.print(right_motor_act_speed);
//    Serial.println();    

    if (noCommLoop >= noCommMax){
      targetX = 0;
      targetZ = 0;
      drive(0,0);
    }

    noCommLoop++;

    if (noCommLoop == 65535){
      noCommLoop = noCommMax;
    }

    publishSpeed(INTERVAL);
  }
  
}

//
void publishSpeed(double time){
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = left_motor_act_speed;    //left wheel speed (in m/s)
  speed_msg.vector.y = right_motor_act_speed;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speedPub.publish(&speed_msg);
  nh.spinOnce();
//  nh.loginfo("Publishing odometry");
}

unsigned long left_pre_update_time = 0;
int getLeftMotorRpm(){
  long ticks = left_motor_pulse;
  unsigned long current_time = millis();
  unsigned long dt = current_time - left_pre_update_time;

  double dtm  = (double)dt/60000;

  double delta_ticks  = ticks - left_motor_pre_pulse;
  left_motor_pre_pulse = ticks;

  left_pre_update_time = current_time;
  return (delta_ticks / TICKS_PER_REVOLUTION) /dtm ;
}

unsigned long right_pre_update_time = 0;
int getRightMotorRpm(){
  long ticks = right_motor_pulse;
  unsigned long current_time = millis();
  unsigned long dt = current_time - right_pre_update_time;

  double dtm  = (double)dt/60000;

  double delta_ticks  = ticks - right_motor_pre_pulse;
  right_motor_pre_pulse = ticks;

  right_pre_update_time = current_time;
  return (delta_ticks / TICKS_PER_REVOLUTION) /dtm ;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


void readCommand(){
  if (Serial.available()){
    String command = Serial.readStringUntil('\n');

    Serial.printf("Command received %s \n", command);
    if (command == "stop"){
      rightMotor.stop();
      leftMotor.stop();
    } else if (command.equals("reset")){
      
    } else if (command.substring(0, 1).equals("p")){
      double value = command.substring(1).toDouble();
      right_kp = value;
      rightPID.SetTunings(right_kp, right_ki, right_kd);

      left_kp = value;
      leftPID.SetTunings(left_kp, left_ki, left_kd);
    } else if (command.substring(0, 1).equals("i")){
      double value = command.substring(1).toDouble();
      right_ki = value;
      rightPID.SetTunings(right_kp, right_ki, right_kd);

      left_ki = value;
      leftPID.SetTunings(left_kp, left_ki, left_kd);
    } else if (command.substring(0, 1).equals("d")){
      double value = command.substring(1).toDouble();
      right_kd = value;
      rightPID.SetTunings(right_kp, right_ki, right_kd);

      left_kd = value;
      leftPID.SetTunings(left_kp, left_ki, left_kd);
    } else if (command.substring(0, 1).equals("s")){
      double value = command.substring(1).toInt();
      right_setpoint = value;

      left_setpoint = value;
    } else if (command.substring(0, 1).equals("v")){
      int commaIndex = command.indexOf(',');
//       left_motor_pulse = 0;
//      left_motor_pre_pulse = 0;
//      right_motor_pulse = 0;
//      right_motor_pre_pulse = 0;
      double x = command.substring(1, commaIndex).toDouble();
      double z = command.substring(commaIndex + 1, command.length()).toDouble();
      noCommLoop = 0;
      doVel(x,z);
    }else if (command.substring(0, 1).equals("r")){
      int value = command.substring(1).toInt();
      rightMotor.rotate( value);

     leftMotor.rotate(value);
    } else if (command.substring(0, 1).equals("z")){
      int commaIndex = command.indexOf(',');
      
      left_motor_pulse = command.substring(1, commaIndex).toInt();
      right_motor_pulse = command.substring(commaIndex + 1, command.length()).toInt();
      
    } else if (command.substring(0, 1).equals("x")){
      double value = command.substring(1).toInt();
      leftMotor.rotate(value);
    }
  }
}
