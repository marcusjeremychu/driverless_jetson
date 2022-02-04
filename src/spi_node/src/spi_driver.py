#!/usr/bin/env python3
import spidev
import rospy
import struct
from std_msgs.msg import Int8
from ackermann_msgs.msg import AckermannDrive
# float32 steering_angle
# float32 steering_angle_velocity
# float32 speed
# float32 acceleration
# float32 jerk

spi = None
testing = True

# rostopic pub -1 /steering ackermann_msgs/AckermannDrive -- 3.3 4.1 3.3 3.3 3.3

# float32 steering_angle          # desired virtual angle (radians)
# float32 steering_angle_velocity # desired rate of change (radians/s)

def callback(data):
    global spi
    # int8 for single dataframe, in order to have more,
    # need to use an array e.g. data=[0x00,0x01,0x02]
    s_a = list(struct.pack("f", data.steering_angle))
    
    print("steering angle", data.steering_angle)
    print("spi data array", s_a)
    spi.xfer(s_a)

def run_SPI():
    global spi
    rospy.init_node('spi_connector', anonymous=True)
    rate = rospy.Rate(30) # 30hz
    rospy.Subscriber("steering", AckermannDrive, callback)

    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 5000
    spi.mode = 0b00

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        run_SPI()
    except rospy.ROSInterruptException:
        pass