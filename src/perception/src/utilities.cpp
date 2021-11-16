#include "utilities.h"
#include <sstream>

vector<float> euler_to_quaternion(float roll, float pitch, float yaw) {
    printf("\r\n Angle：Roll: %.2f     Pitch: %.2f     Yaw: %.2f \r\n",roll, pitch, yaw);
    float qx = sin(roll/2.0) * cos(pitch/2.0) * cos(yaw/2.0) - cos(roll/2.0) * sin(pitch/2.0) * sin(yaw/2.0);
    float qy = cos(roll/2.0) * sin(pitch/2.0) * cos(yaw/2.0) + sin(roll/2.0) * cos(pitch/2.0) * sin(yaw/2.0);
    float qz = cos(roll/2.0) * cos(pitch/2.0) * sin(yaw/2.0) - sin(roll/2.0) * sin(pitch/2.0) * cos(yaw/2.0);
    float qw = cos(roll/2.0) * cos(pitch/2.0) * cos(yaw/2.0) + sin(roll/2.0) * sin(pitch/2.0) * sin(yaw/2.0);
    vector<float> quaternion {qx, qy, qz, qw};
    return quaternion;
}

vector<float> quaternion_to_euler(float x, float y, float z, float w) {
    float t0 = 2.0 * (w * x + y * y);
    float t1 = 1.0 - 2.0 * (x * x + y * y);
    float roll = atan2(t0, t1);

    float t2 = 2.0 * (w * y - z* x);
    t2 = t2 > 1.0 ? 1.0:t2;
    t2 = t2 < -1.0 ? -1.0:t2;
    float pitch = asin(t2);

    float t3 = 2.0 * (w * z + x * y);
    float t4 = 1.0 - 2.0 * (y * y + z * z);
    float yaw = atan2(t3, t4);

    vector<float> euler {roll, pitch, yaw};
    return euler;
}