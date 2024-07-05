#include <cmath>
#include <geometry_msgs/msg/quaternion.h>

#include "maths.h"

namespace maths
{
    std::array<double, 4> unpack_quat(geometry_msgs__msg__Quaternion q)
    {
        std::array<double, 4> arr = {q.x, q.y, q.z, q.w};
        return arr;
    }

    geometry_msgs__msg__Quaternion pack_quat(std::array<double, 4> q)
    {
        geometry_msgs__msg__Quaternion quat;
        quat.x = q[0];
        quat.y = q[1];
        quat.z = q[2];
        quat.w = q[3];
        return quat;
    }

    std::array<double, 4> multiply_quat(std::array<double, 4> q1, std::array<double, 4> q2){
        std::array<double, 4> q;
        q[0] = q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1];
        q[1] = q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0];
        q[2] = q1[3] * q2[2] + q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3];
        q[3] = q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2];
        return q;
    }

    std::array<double, 4> euler_to_quat(float x, float y, float z)
    {
        std::array<double, 4> q;

        float c1 = cos((y * 3.14 / 180.0) / 2);
        float c2 = cos((z * 3.14 / 180.0) / 2);
        float c3 = cos((x * 3.14 / 180.0) / 2);

        float s1 = sin((y * 3.14 / 180.0) / 2);
        float s2 = sin((z * 3.14 / 180.0) / 2);
        float s3 = sin((x * 3.14 / 180.0) / 2);

        q[0] = c1 * c2 * c3 - s1 * s2 * s3;
        q[1] = s1 * s2 * c3 + c1 * c2 * s3;
        q[2] = s1 * c2 * c3 + c1 * s2 * s3;
        q[3] = c1 * s2 * c3 - s1 * c2 * s3;

        return q;
    }

    double quat_to_yaw(geometry_msgs__msg__Quaternion q)
    {
        return quat_to_yaw(unpack_quat(q));
    }
    
        double quat_to_yaw(std::array<double, 4> q)
    {
        return atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
    }
}
