#pragma once 

#include <v0/gnss/common.hpp>
#include "std_msgs/msg/string.hpp"
#include <libgpsmm.h>

// #include "geometry_msgs/msg/pose.hpp"


#include "tf2_ros/static_transform_broadcaster_node.hpp"

using namespace std::chrono;
using GnssDataMsg = geometry_msgs::msg::TransformStamped;
using GnssData = v0::gnss::common::Tf2_transform;

namespace Types::Conversion {

/**
 * @brief converts ROS2 MSG to CommonAPI generated data type 
 * 
 * @param gps_data 
 * @return GnssData 
 */
GnssData to_capi_type(const GnssDataMsg & gps_data) {
    GnssData gnss_data;

    v0::gnss::common::Vector3 vector;
    v0::gnss::common::Quaternion orientation;

    orientation.setX(gps_data.transform.rotation.x);
    orientation.setY(gps_data.transform.rotation.y);
    orientation.setZ(gps_data.transform.rotation.z);
    orientation.setW(gps_data.transform.rotation.w);

    vector.setX(gps_data.transform.translation.x);
    vector.setY(gps_data.transform.translation.y);
    vector.setZ(gps_data.transform.translation.z);

    gnss_data.setOrientation(orientation);
    gnss_data.setVector(vector);

    return gnss_data;
}

/**
 * @brief converts CommonAPI generated data type to ROS2 data type
 * 
 * @param gps_data 
 * @return GnssData 
 */
 GnssDataMsg from_capi_type(const GnssData & gnss_data) {

    GnssDataMsg gps_data_msg;

    gps_data_msg.transform.rotation.x = gnss_data.getOrientation().getX();
    gps_data_msg.transform.rotation.y = gnss_data.getOrientation().getY();
    gps_data_msg.transform.rotation.z = gnss_data.getOrientation().getZ();
    gps_data_msg.transform.rotation.w = gnss_data.getOrientation().getW();

    
    
    


    return gps_data_msg;
 }


} // namespace TypeConversion