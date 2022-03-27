#include <MPU9250_WE.h>

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>


#define ACCEL_SCALE 1 / 16384 // LSB/g
#define GYRO_SCALE 1 / 131 // LSB/(deg/s)
#define MAG_SCALE 0.6 // uT/LSB
#define G_TO_ACCEL 9.81
#define MGAUSS_TO_UTESLA 0.1
#define UTESLA_TO_TESLA 0.000001
#define MPU9250_ADDR 0x68

#define ROS_SERIAL true
//MPU9250 mpu;
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;

ros::Publisher imu_pub("/vacumm/imu", &imu_msg);
ros::Publisher mag_pub("/imu/mag", &mag_msg);
unsigned long prev_imu_time = 0;



void logInfo(char msg[]) {
  if (ROS_SERIAL) {
    nh.loginfo(msg);
  } else {
    Serial.println(msg);
  }
}

void setup() {
  if (!ROS_SERIAL) {
    Serial.begin(115200);
  } else {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(imu_pub);
    nh.advertise(mag_pub);
  }


  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(2000);

  if (!myMPU9250.init()) {

    logInfo("MPU9250 does not respond");
  }
  else {
    logInfo("MPU9250 is connected");
  }
  if (!myMPU9250.initMagnetometer()) {
    logInfo("Magnetometer does not respond");
  }
  else {
    logInfo("Magnetometer is connected");
  }

  logInfo("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");

  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_5);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);

  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_4);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(200);
}



//
xyzFloat acc;
xyzFloat gyr;
xyzFloat mag;


void displayIMU() {

  Serial.print("Acc: ");
  Serial.print(acc.x * (double) G_TO_ACCEL );
  Serial.print(", ");
  Serial.print(acc.y * (double) G_TO_ACCEL );
  Serial.print(", ");
  Serial.print(acc.z * (double) G_TO_ACCEL );
  Serial.print(" Gry: ");
  Serial.print(gyr.x * (double) DEG_TO_RAD );
  Serial.print(", ");
  Serial.print(gyr.y * (double) DEG_TO_RAD );
  Serial.print(", ");
  Serial.print(gyr.z * (double) DEG_TO_RAD );
  Serial.print(" Gry: ");
  Serial.print( (float) mag.x * (float) UTESLA_TO_TESLA );
  Serial.print(", ");
  Serial.print((float) mag.y * (float) UTESLA_TO_TESLA );
  Serial.print(", ");
  Serial.print((float) mag.z * (float) UTESLA_TO_TESLA );
  Serial.println("");
  //  Serial.println("");
  //  Serial.println("");

}
void loop() {

  if ((millis() - prev_imu_time) >= (20))
  {
    acc = myMPU9250.getGValues();
    gyr = myMPU9250.getGyrValues();
    mag = myMPU9250.getMagValues();

    if (ROS_SERIAL) {
      imu_msg.header.frame_id = "imu_link";
      imu_msg.header.stamp = nh.now();
      //        imu_msg.orientation.y = mpu.getQuaternionX() * -1.0;
      //        imu_msg.orientation.x = mpu.getQuaternionY() * -1.0;
      //        imu_msg.orientation.z = mpu.getQuaternionZ() * -1.0;
      //        imu_msg.orientation.w = mpu.getQuaternionW();

      imu_msg.linear_acceleration.y = (acc.x * (double) G_TO_ACCEL) * -1.0;
      imu_msg.linear_acceleration.x = acc.y * (double) G_TO_ACCEL;
      imu_msg.linear_acceleration.z = acc.z * (double) G_TO_ACCEL;

      imu_msg.angular_velocity.y = gyr.x * (double) DEG_TO_RAD * -1.0;
      imu_msg.angular_velocity.x = gyr.y * (double) DEG_TO_RAD;
      imu_msg.angular_velocity.z = gyr.z * (double) DEG_TO_RAD;


      mag_msg.header.frame_id = "imu_link";
      mag_msg.header.stamp = nh.now();
      mag_msg.magnetic_field.x = (float) mag.x * (float) UTESLA_TO_TESLA;
      mag_msg.magnetic_field.y = (float) mag.y * (float) UTESLA_TO_TESLA;
      mag_msg.magnetic_field.z = (float) mag.z * (float) UTESLA_TO_TESLA;


      imu_pub.publish(&imu_msg);
      mag_pub.publish(&mag_msg);
      nh.spinOnce();
    } else {
      displayIMU();
    }


    prev_imu_time = millis();
  }
}
