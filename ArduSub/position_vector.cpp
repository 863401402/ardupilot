#include "Sub.h"

// position_vector.pde related utility functions

// position vectors are Vector3f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_location_to_vector - convert lat/lon coordinates to a position vector
//  将纬度/经度坐标转换为位置向量
Vector3f Sub::pv_location_to_vector(const Location& loc)
{
    Location origin;
    if (!ahrs.get_origin(origin)) {
        origin.zero();
    }
    // 将相对于家点的高度转换为相对于EKF原点的高度
    float alt_above_origin = pv_alt_above_origin(loc.alt);  // convert alt-relative-to-home to alt-relative-to-origin
    // 获取从原点到位置点的NED（北-东-地）距离向量
    Vector3f vec = origin.get_distance_NED(loc);
    // 将纬度/经度坐标的单位从米转换为厘米
    vec.xy() *= 100;
    vec.z = alt_above_origin;
    return vec;
}

// pv_alt_above_origin - convert altitude above home to altitude above EKF origin
// 将相对于家点的高度转换为相对于EKF原点的高度
float Sub::pv_alt_above_origin(float alt_above_home_cm)
{
    Location origin;
    if (!ahrs.get_origin(origin)) {
        origin.zero();
    }
    // 计算相对于EKF原点的高度
    return alt_above_home_cm + (ahrs.get_home().alt - origin.alt);
}

