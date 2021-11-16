// Original author is waveshare, modified for ROS integration by Marcus Chu

#include "ICM20948.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <tf2/LinearMath/Quaternion.h>
#include "utilities.h"
#include <sstream>
#define PI 3.14159265359

// TODO: 
//		- Publish magnetometer readings as well

void print_imu_logs(IMU_ST_ANGLES_DATA stAngles, IMU_ST_SENSOR_DATA stGyroRawData, 
					IMU_ST_SENSOR_DATA stAccelRawData, IMU_ST_SENSOR_DATA stMagnRawData) {
	printf("\r\n /-------------------------------------------------------------/ \r\n");
	printf("\r\n Angle：Roll: %.2f     Pitch: %.2f     Yaw: %.2f \r\n",stAngles.fRoll, stAngles.fPitch, stAngles.fYaw);
	printf("\r\n Acceleration(g): X: %.3f     Y: %.3f     Z: %.3f \r\n",stAccelRawData.fX, stAccelRawData.fY, stAccelRawData.fZ);
	printf("\r\n Gyroscope(dps): X: %.3f     Y: %.3f     Z: %.3f \r\n",stGyroRawData.fX, stGyroRawData.fY, stGyroRawData.fZ);
	printf("\r\n Magnetic(uT): X: %.3f     Y: %.3f     Z: %.3f \r\n",stMagnRawData.fX, stMagnRawData.fY, stMagnRawData.fZ);
}


int main(int argc, char* argv[])
{
	IMU_EN_SENSOR_TYPE enMotionSensorType;
	IMU_ST_ANGLES_DATA stAngles;
	IMU_ST_SENSOR_DATA stGyroRawData;
	IMU_ST_SENSOR_DATA stAccelRawData;
	IMU_ST_SENSOR_DATA stMagnRawData;

	imuInit(&enMotionSensorType);
	if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
	{
		printf("Motion sersor is ICM-20948\n" );
	}
	else
	{
		printf("Motion sersor NULL\n");
	}

	ros::init(argc, argv, "camera_imu_node");
	ros::NodeHandle n;

	ros::Publisher camera_imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
	ros::Publisher camera_mag_pub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
    ros::Rate loop_rate(30);

	sensor_msgs::Imu imu_msg;
	sensor_msgs::MagneticField mag_msg;

	while(ros::ok())
	{
		imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
		// print_imu_logs(stAngles, stGyroRawData, stAccelRawData, stMagnRawData);
		// tf2::Quaternion myQuaternion;
		// myQuaternion.setRPY( stAngles.fRoll * PI/180, stAngles.fPitch * PI/180, stAngles.fYaw * PI/180);  // Create this quaternion from roll/pitch/yaw (in radians)
		// myQuaternion.normalize();
		// vector<float> orientation = euler_to_quaternion(stAngles.fRoll * PI/180 , stAngles.fPitch * PI/180, stAngles.fYaw * PI/180);

		// Build IMU Msg
		imu_msg.header.stamp = ros::Time::now();
		imu_msg.header.frame_id = "left_camera";
		// imu_msg.orientation.x = myQuaternion.x();
		// imu_msg.orientation.y = myQuaternion.y();
		// imu_msg.orientation.z = myQuaternion.z();
		// imu_msg.orientation.w = myQuaternion.w();
		// imu_msg.orientation_covariance[0] = -1;

		imu_msg.linear_acceleration.x = stAccelRawData.fZ * 9.81;
		imu_msg.linear_acceleration.y = stAccelRawData.fX * 9.81;
		imu_msg.linear_acceleration.z = stAccelRawData.fY * 9.81;
		imu_msg.linear_acceleration_covariance[0] = -1;

		imu_msg.angular_velocity.x = stGyroRawData.fZ * PI/180;
		imu_msg.angular_velocity.y = stGyroRawData.fX * PI/180;
		imu_msg.angular_velocity.z = stGyroRawData.fY * PI/180; 
		imu_msg.angular_velocity_covariance[0] = -1;
		

		// Build Mag msg
		mag_msg.header.stamp = ros::Time::now();
		mag_msg.header.frame_id = "left_camera";

		mag_msg.magnetic_field.x = stMagnRawData.fZ /1000000.0;
		mag_msg.magnetic_field.y = stMagnRawData.fX /1000000.0;
		mag_msg.magnetic_field.z = stMagnRawData.fY /1000000.0;
		mag_msg.magnetic_field_covariance[0] = -1;

		camera_imu_pub.publish(imu_msg);
		camera_mag_pub.publish(mag_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
