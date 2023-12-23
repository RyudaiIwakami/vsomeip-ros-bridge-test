#pragma once 

#include <v0/gnss/common.hpp>
#include "std_msgs/msg/string.hpp"
#include <libgpsmm.h>
#include <builtin_interfaces/msg/time.hpp>

// #include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// #include "tf2_ros/static_transform_broadcaster_node.hpp"

using namespace std::chrono;
using GnssDataMsg = sensor_msgs::msg::PointCloud2;
using GnssData = v0::gnss::common::PointCloud2;

namespace Types::Conversion {

/**
 * @brief converts ROS2 MSG to CommonAPI generated data type 
 * 
 * @param gps_data 
 * @return GnssData 
 */
GnssData to_capi_type(const GnssDataMsg & gps_data) {
    GnssData gnss_data;


    v0::gnss::common::Header header;
    v0::gnss::common::Sec_nanosec time_stamp_2;
    v0::gnss::common::PointField point_field;
    v0::gnss::common::PointCloud2 point_cloud_2;
    std::vector<v0::gnss::common::PointField> point_fields;


    time_stamp_2.setSec(gps_data.header.stamp.sec);
    time_stamp_2.setNanosec(gps_data.header.stamp.nanosec);

    header.setTime_stamp(time_stamp_2);

    header.setFrame_id(gps_data.header.frame_id);

    
    if (!gps_data.fields.empty()) {
        point_field.setName(gps_data.fields[0].name);
        point_field.setOffset(gps_data.fields[0].offset);
        point_field.setDatatype(gps_data.fields[0].datatype);
        point_field.setCount(gps_data.fields[0].count);

        point_fields.push_back(point_field); 
        gnss_data.setFields(point_fields);
    }

    // point_field.setName(gps_data.fields[0].name);
    // point_field.setOffset(gps_data.fields[0].offset);
    // point_field.setDatatype(gps_data.fields[0].datatype);
    // point_field.setCount(gps_data.fields[0].count);

    // point_fields.push_back(point_field); 

    gnss_data.setHeader(header);
    gnss_data.setHeight(gps_data.height);
    gnss_data.setWidth(gps_data.width);
    gnss_data.setIs_bigendian(gps_data.is_bigendian);
    gnss_data.setPoint_step(gps_data.point_step);
    gnss_data.setRow_step(gps_data.row_step);
    gnss_data.setData(gps_data.data);
    gnss_data.setIs_dense(gps_data.is_dense);

    
    
    return gnss_data;
}

/**
 * @brief converts CommonAPI generated data type to ROS2 data type
 * 
 * @param gps_data 
 * @return GnssData 
 */
//  GnssDataMsg from_capi_type(const GnssData & gnss_data) {

//     GnssDataMsg gps_data_msg;

//     gps_data_msg.header.stamp.sec = gnss_data.getHeader().getTime_stamp().getSec();
//     gps_data_msg.header.stamp.nanosec = gnss_data.getHeader().getTime_stamp().getNanosec();
//     gps_data_msg.header.frame_id = gnss_data.getHeader().getFrame_id();

//     gps_data_msg.child_frame_id = gnss_data.getChild_frame_id();

//     gps_data_msg.transform.rotation.x = gnss_data.getTransform().getOrientation().getX();
//     gps_data_msg.transform.rotation.y = gnss_data.getTransform().getOrientation().getY();
//     gps_data_msg.transform.rotation.z = gnss_data.getTransform().getOrientation().getZ();
//     gps_data_msg.transform.rotation.w = gnss_data.getTransform().getOrientation().getW();

//     gps_data_msg.transform.translation.x = gnss_data.getTransform().getVector().getX();
//     gps_data_msg.transform.translation.y = gnss_data.getTransform().getVector().getY();
//     gps_data_msg.transform.translation.z = gnss_data.getTransform().getVector().getZ();

    

//     // gps_data_msg.header.stamp = gnss_data.getHeader().getTime_stamp();
//     // gps_data_msg.header.frame_id = gnss_data.getHeader().getFrame_id();
//     // gps_data_msg.child_frame_id = gnss_data.getChild_frame_id();


//     // gps_data_msg.transform.rotation.x = gnss_data.getTransform().getOrientation().getX();
//     // gps_data_msg.transform.rotation.y = gnss_data.getTransform().getOrientation().getY();
//     // gps_data_msg.transform.rotation.z = gnss_data.getTransform().getOrientation().getZ();
//     // gps_data_msg.transform.rotation.w = gnss_data.getTransform().getOrientation().getW();

//     // gps_data_msg.transform.translation.x = gnss_data.getTransform().getVector().getX();
//     // gps_data_msg.transform.translation.y = gnss_data.getTransform().getVector().getY();
//     // gps_data_msg.transform.translation.z = gnss_data.getTransform().getVector().getZ();

  


    
    
//     return gps_data_msg;
//  }


} // namespace TypeConversion