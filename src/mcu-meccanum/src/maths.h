#include <array>

namespace maths
{
std::array<double, 4> multiply_quat(std::array<double, 4> q1, std::array<double, 4> q2);

    std::array<double, 4> unpack_quat(geometry_msgs__msg__Quaternion q);
    geometry_msgs__msg__Quaternion pack_quat(std::array<double, 4> q);

    std::array<double, 4> euler_to_quat(float x, float y, float z);
    
    double quat_to_yaw(geometry_msgs__msg__Quaternion quat);
    double quat_to_yaw(std::array<double, 4> quat);
}