#include <ros.h>
#include <vacumm_hardware/WheelState.h>
#include <RotaryEncoder.h>

#define ROS_SERIAL false

#define LEFT_MOTOR_ENCODER_A 18
#define LEFT_MOTOR_ENCODER_B 19
#define RIGHT_MOTOR_ENCODER_A 4
#define RIGHT_MOTOR_ENCODER_B 5
#define INTERVAL 100



RotaryEncoder *leftEncoder = nullptr;
RotaryEncoder *rightEncoder = nullptr;


long currentMillis = 0;
long previousMillis = 0;
long leftEncoderPos = 0;
long rightEncoderPos = 0;
long leftEncoderOldPos = 0;
long rightEncoderOldPos = 0;

//ROS
ros::NodeHandle nh;
vacumm_hardware::WheelState wheel_state;
ros::Publisher wheel_state_pub("/vacumm/wheel_encoder", &wheel_state);

IRAM_ATTR void leftEncoderPosition()
{
  leftEncoder->tick(); // just call tick() to check the state.
}


IRAM_ATTR void rightEncoderPosition()
{
  rightEncoder->tick(); // just call tick() to check the state.
}


void setup() {
  // put your setup code here, to run once:
  if (!ROS_SERIAL) {
    Serial.begin(115200);
  } else {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(wheel_state_pub);
  }

  leftEncoder = new RotaryEncoder(LEFT_MOTOR_ENCODER_B, LEFT_MOTOR_ENCODER_A, RotaryEncoder::LatchMode::TWO03);
  rightEncoder = new RotaryEncoder(RIGHT_MOTOR_ENCODER_A, RIGHT_MOTOR_ENCODER_B, RotaryEncoder::LatchMode::TWO03);

  attachInterrupt(LEFT_MOTOR_ENCODER_A, leftEncoderPosition, CHANGE);
  attachInterrupt(LEFT_MOTOR_ENCODER_B, leftEncoderPosition, CHANGE);


  attachInterrupt(RIGHT_MOTOR_ENCODER_A, rightEncoderPosition, CHANGE);
  attachInterrupt(RIGHT_MOTOR_ENCODER_B, rightEncoderPosition, CHANGE);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (ROS_SERIAL) {
    nh.spinOnce();
  }

  leftEncoder->tick();
  rightEncoder->tick();
  currentMillis = millis();
  if ((millis() - previousMillis) > INTERVAL) {
    previousMillis = currentMillis;

    leftEncoderPos = leftEncoder->getPosition();
    rightEncoderPos = rightEncoder->getPosition();

    if (!ROS_SERIAL) {
      if (leftEncoderOldPos != leftEncoderPos || rightEncoderOldPos != rightEncoderPos ) {
        Serial.print("left pos:");
        Serial.print(leftEncoderOldPos);

        Serial.print(" right pos:");
        Serial.println(rightEncoderPos);

        leftEncoderOldPos = leftEncoderPos;
        rightEncoderOldPos = rightEncoderPos;
      } // if
    } else {
      wheel_state.vel[0] = 0;
      wheel_state.vel[1] = 0;
      wheel_state.pos[0] = leftEncoder->getPosition();
      wheel_state.pos[1] = rightEncoder->getPosition();

      wheel_state_pub.publish(&wheel_state);
      nh.spinOnce();
    }



  }

}
