/* Generated from orogen/lib/orogen/templates/typekit/ros/ROSConvertions.hpp */

#ifndef __OROGEN_GENERATED_BASE_ROS_CONVERTIONS_USER_HPP
#define __OROGEN_GENERATED_BASE_ROS_CONVERTIONS_USER_HPP

#include "Types.hpp"
#include <boost/cstdint.hpp>
#include <string>


#include <std_msgs/Time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

namespace ros_convertions {
    /** Converted types: */
    
    void toROS( ros::Time& ros, ::base::Time const& value );
    void fromROS( ::base::Time& value, ros::Time const& ros );

    void toROS( sensor_msgs::Image& ros, ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > const& value );
    void fromROS( ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame >& value, sensor_msgs::Image const& ros );
    
    void toROS( sensor_msgs::Image& ros, ::base::samples::frame::Frame const& value );
    void fromROS( ::base::samples::frame::Frame& value, sensor_msgs::Image const& ros );

    void toROS( geometry_msgs::Pose& ros, ::base::Pose const& value );
    void fromROS( ::base::Pose& value, geometry_msgs::Pose const& ros );

    void toROS( geometry_msgs::Pose& ros, ::base::Pose_m const& value );
    void fromROS( ::base::Pose_m& value, geometry_msgs::Pose const& ros );
    
    void toROS( sensor_msgs::JointState& ros, ::base::samples::Joints const& value );
    void fromROS( ::base::samples::Joints& value, sensor_msgs::JointState const& ros );

}

#endif


