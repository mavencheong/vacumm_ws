#include "MPU9250.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>


#define ROS_SERIAL true
MPU9250 mpu;

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;

ros::Publisher imu_pub("/vacumm/imu", &imu_msg);


void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
    Serial.print("Acc X, Y, X");
    Serial.print(mpu.getAccX());
    Serial.print(", ");
    Serial.print(mpu.getAccY());
    Serial.print(", ");
    Serial.println(mpu.getAccZ());
    Serial.print("Gyro X, Y, X");
    Serial.print(mpu.getGyroX());
    Serial.print(", ");
    Serial.print(mpu.getGyroY());
    Serial.print(", ");
    Serial.println(mpu.getGyroZ());
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}


void setup() {
    if (!ROS_SERIAL){
      Serial.begin(115200);  
    }
    
    Wire.begin(21,22);
    Wire.setClock(400000);
    delay(2000);

    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A4G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu.setup(0x68, setting)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    mpu.setFilterIterations(10);
    mpu.setAccBias(23.80, -2.14, -281.76);
    mpu.setGyroBias(-0.33, -4.30, -2.17);
    mpu.setMagBias(31.84, 747.82, 107.75);
    mpu.setMagScale(1.24, 0.67, 1.44);
    // calibrate anytime you want to
//    /Serial.println("Accel Gyro calibration will start in 5sec.");
//    /Serial.println("Please leave the device still on the flat plane.");
//    /mpu.verbose(true);
//    /delay(5000);
    //mpu.calibrateAccelGyro();

//    /Serial.println("Mag calibration will start in 5sec.");
//    /Serial.println("Please Wave device in a figure eight until done.");
//    /delay(5000);
//    /mpu.calibrateMag();

    
//    /print_calibration();
    mpu.verbose(false);

    if (ROS_SERIAL) {
      nh.getHardware()->setBaud(115200);
      nh.initNode();
      nh.advertise(imu_pub);
    }
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 50) {
            prev_ms = millis();
            if (ROS_SERIAL){
              imu_msg.header.frame_id = "imu_link";
              imu_msg.orientation.x = mpu.getQuaternionX();
              imu_msg.orientation.y = mpu.getQuaternionY();
              imu_msg.orientation.z = mpu.getQuaternionZ();
              imu_msg.orientation.w = mpu.getQuaternionW();
              
              imu_msg.linear_acceleration.x = mpu.getLinearAccX();
              imu_msg.linear_acceleration.y = mpu.getLinearAccY();
              imu_msg.linear_acceleration.z = mpu.getLinearAccZ();
  
              imu_msg.angular_velocity.x = mpu.getGyroX() * DEG_TO_RAD;
              imu_msg.angular_velocity.y = mpu.getGyroY() * DEG_TO_RAD;
              imu_msg.angular_velocity.z = mpu.getGyroZ()* DEG_TO_RAD;
  
              imu_pub.publish(&imu_msg);
              nh.spinOnce();
            } else {
              print_roll_pitch_yaw();
            }
        }
    }
}
