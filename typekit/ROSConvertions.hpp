/* Generated from orogen/lib/orogen/templates/typekit/ros/ROSConvertions.hpp */

#ifndef __OROGEN_GENERATED_BASE_ROS_CONVERTIONS_USER_HPP
#define __OROGEN_GENERATED_BASE_ROS_CONVERTIONS_USER_HPP

#include "Types.hpp"
#include <boost/cstdint.hpp>
#include <string>


#include <std_msgs/Time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <sensor_msgs/PointCloud2.h>



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

    void toROS( trajectory_msgs::JointTrajectory& ros, ::base::JointsTrajectory const& value );
    void fromROS( ::base::JointsTrajectory& value, trajectory_msgs::JointTrajectory const& ros );

    void toROS( geometry_msgs::PoseStamped& ros, ::base::samples::RigidBodyState const& value );
    void fromROS( ::base::samples::RigidBodyState& value, geometry_msgs::PoseStamped const& ros );
    
    void toROS( geometry_msgs::PoseStamped& ros, ::base::samples::RigidBodyState_m const& value );
    void fromROS( ::base::samples::RigidBodyState_m& value, geometry_msgs::PoseStamped const& ros );

    void toROS( moveit_msgs::CollisionObject& ros, ::object_detection::PrimitiveObject const& value );
    void fromROS( ::object_detection::PrimitiveObject& value, moveit_msgs::CollisionObject const& ros );

    void toROS( sensor_msgs::PointCloud2& ros, ::base::samples::Pointcloud const& value );
    void fromROS( ::base::samples::Pointcloud& value, sensor_msgs::PointCloud2 const& ros );




}

#endif


