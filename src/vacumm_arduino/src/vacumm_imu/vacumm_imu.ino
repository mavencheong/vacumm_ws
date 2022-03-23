#include "MPU9250.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>


#define ACCEL_SCALE 1 / 16384 // LSB/g
#define GYRO_SCALE 1 / 131 // LSB/(deg/s)
#define MAG_SCALE 0.6 // uT/LSB
#define G_TO_ACCEL 9.81
#define MGAUSS_TO_UTESLA 0.1
#define UTESLA_TO_TESLA 0.000001


#define ROS_SERIAL true
MPU9250 mpu;

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;

ros::Publisher imu_pub("/vacumm/imu", &imu_msg);
ros::Publisher mag_pub("/imu/mag", &mag_msg);
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
unsigned long prev_imu_time = 0;

//void print_roll_pitch_yaw() {
//    Serial.print("Yaw, Pitch, Roll: ");
//    Serial.print(mpu.getYaw(), 2);
//    Serial.print(", ");
//    Serial.print(mpu.getPitch(), 2);
//    Serial.print(", ");
//    Serial.println(mpu.getRoll(), 2);
//    Serial.print("Acc X, Y, X");
//    Serial.print(mpu.getAccX());
//    Serial.print(", ");
//    Serial.print(mpu.getAccY());
//    Serial.print(", ");
//    Serial.println(mpu.getAccZ());
//    Serial.print("Gyro X, Y, X");
//    Serial.print(mpu.getGyroX());
//    Serial.print(", ");
//    Serial.print(mpu.getGyroY());
//    Serial.print(", ");
//    Serial.println(mpu.getGyroZ());
//}

//void print_calibration() {
//    Serial.println("< calibration parameters >");
//    Serial.println("accel bias [g]: ");
//    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
//    Serial.print(", ");
//    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
//    Serial.print(", ");
//    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
//    Serial.println();
//    Serial.println("gyro bias [deg/s]: ");
//    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
//    Serial.print(", ");
//    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
//    Serial.print(", ");
//    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
//    Serial.println();
//    Serial.println("mag bias [mG]: ");
//    Serial.print(mpu.getMagBiasX());
//    Serial.print(", ");
//    Serial.print(mpu.getMagBiasY());
//    Serial.print(", ");
//    Serial.print(mpu.getMagBiasZ());
//    Serial.println();
//    Serial.println("mag scale []: ");
//    Serial.print(mpu.getMagScaleX());
//    Serial.print(", ");
//    Serial.print(mpu.getMagScaleY());
//    Serial.print(", ");
//    Serial.print(mpu.getMagScaleZ());
//    Serial.println();
//}

//
//void setup() {
//    if (!ROS_SERIAL){
//      Serial.begin(115200);
//    }
//
//    Wire.begin(21,22);
//    Wire.setClock(400000);
//    delay(2000);
//
////    MPU9250Setting setting;
////    setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
////    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
////    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
////    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
////    setting.gyro_fchoice = 0x03;
////    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
////    setting.accel_fchoice = 0x01;
////    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
//
//    if (!mpu.setup(0x68)) {  // change to your own address
//        while (1) {
//            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
//            delay(5000);
//        }
//    }
//
//    mpu.setFilterIterations(20);
//    mpu.setAccBias(99.18, 3.43, -281.30);
//    mpu.setGyroBias(-0.55, -4.25, -1.96);
//    mpu.setMagBias(-166.27, 384.59, 138.53);
//    mpu.setMagScale(1.09, 1.01, 1.02);
//    // calibrate anytime you want to
////    /Serial.println("Accel Gyro calibration will start in 5sec.");
////    /Serial.println("Please leave the device still on the flat plane.");
////    /mpu.verbose(true);
////    /delay(5000);
//    //mpu.calibrateAccelGyro();
//
////    /Serial.println("Mag calibration will start in 5sec.");
////    /Serial.println("Please Wave device in a figure eight until done.");
////    /delay(5000);
////    /mpu.calibrateMag();
//
//
////    /print_calibration();
//    mpu.verbose(false);
//
//    if (ROS_SERIAL) {
//      nh.getHardware()->setBaud(115200);
//      nh.initNode();
//      nh.advertise(imu_pub);
//      nh.advertise(mag_pub);
//    }
//}

void displayIMU() {


  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  mpu.getHeading(&mx, &my, &mz);

  Serial.print("Acc X, Y, Z:");
  Serial.print(ax * (double) ACCEL_SCALE * G_TO_ACCEL);
  Serial.print(", ");
  Serial.print(ay * (double) ACCEL_SCALE * G_TO_ACCEL);
  Serial.print(", ");
  Serial.print(az * (double) ACCEL_SCALE * G_TO_ACCEL);
  Serial.println("");
  Serial.print("Gry X, Y, Z:");
  Serial.print(gx * (double) GYRO_SCALE * DEG_TO_RAD);
  Serial.print(", ");
  Serial.print(gy * (double) GYRO_SCALE * DEG_TO_RAD);
  Serial.print(", ");
  Serial.print(gz * (double) GYRO_SCALE * DEG_TO_RAD);
  Serial.println("");
  Serial.print("Gry X, Y, Z:");
  Serial.print( (float) mx * (float) MAG_SCALE * UTESLA_TO_TESLA);
  Serial.print(", ");
  Serial.print((float) my * (float) MAG_SCALE * UTESLA_TO_TESLA);
  Serial.print(", ");
  Serial.print((float) mz * (float) MAG_SCALE * UTESLA_TO_TESLA);
  Serial.println("");
  Serial.println("");
  Serial.println("");

}

void setup() {
  if (!ROS_SERIAL) {
    Serial.begin(115200);
  }

  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(2000);
  bool ret;
  mpu.initialize();
  ret = mpu.testConnection();
  if (ret) {
    Serial.println("MPU Connected");
  }

  if (ROS_SERIAL) {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(imu_pub);
    nh.advertise(mag_pub);
  }
}

//void loop() {
//
//  displayIMU();
//  delay(50);
//}



//
void loop() {

  if ((millis() - prev_imu_time) >= (1000 / 20))
  {

    if (mpu.testConnection()) {
      mpu.getAcceleration(&ax, &ay, &az);
      mpu.getRotation(&gx, &gy, &gz);
      mpu.getHeading(&mx, &my, &mz);


      if (ROS_SERIAL) {
        imu_msg.header.frame_id = "imu_link";
        imu_msg.header.stamp = nh.now().toSec();;
        //        imu_msg.orientation.y = mpu.getQuaternionX() * -1.0;
        //        imu_msg.orientation.x = mpu.getQuaternionY() * -1.0;
        //        imu_msg.orientation.z = mpu.getQuaternionZ() * -1.0;
        //        imu_msg.orientation.w = mpu.getQuaternionW();

        imu_msg.linear_acceleration.y = ay * (double) ACCEL_SCALE * G_TO_ACCEL;
        imu_msg.linear_acceleration.x = ax * (double) ACCEL_SCALE * G_TO_ACCEL;
        imu_msg.linear_acceleration.z = az * (double) ACCEL_SCALE * G_TO_ACCEL;

        imu_msg.angular_velocity.y = gy * (double) GYRO_SCALE * DEG_TO_RAD;
        imu_msg.angular_velocity.x = gx * (double) GYRO_SCALE * DEG_TO_RAD;
        imu_msg.angular_velocity.z = gz * (double) GYRO_SCALE * DEG_TO_RAD;


        mag_msg.header.frame_id = "imu_link";
        mag_msg.header.stamp = nh.now().toSec();
        mag_msg.magnetic_field.x = (float) mx * (float) MAG_SCALE * UTESLA_TO_TESLA;
        mag_msg.magnetic_field.y = (float) my * (float) MAG_SCALE * UTESLA_TO_TESLA;
        mag_msg.magnetic_field.z = (float) mz * (float) MAG_SCALE * UTESLA_TO_TESLA;


        imu_pub.publish(&imu_msg);
        mag_pub.publish(&mag_msg);
        nh.spinOnce();
      } else {
        displayIMU();
      }

    }
    prev_imu_time = millis();
  }
}
