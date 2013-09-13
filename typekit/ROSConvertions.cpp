/* Generated from orogen/lib/orogen/templates/typekit/ros/Convertions.cpp */

#include "ROSConvertions.hpp"
#include <rtt/transports/ros/ros_convertions.hpp>
#include <boost/lexical_cast.hpp>

void ros_convertions::toROS( ros::Time& ros, ::base::Time const& value )
{
    ros.fromNSec(value.toMicroseconds() * 1000);
}
void ros_convertions::fromROS( ::base::Time& value, ros::Time const& ros )
{
    value = base::Time::fromMicroseconds(ros.toNSec() / 1000);
}

struct ModeMapping
{
    std::string ros_name;
    base::samples::frame::frame_mode_t mode;
    int data_depth;
    int pixel_size;
};

namespace frame = base::samples::frame;
typedef base::samples::frame::Frame Frame;
static ModeMapping mappings[] =
{
    { "bayer_rggb16", frame::MODE_BAYER_RGGB, 16, 8 },
    { "bayer_rggb8", frame::MODE_BAYER_RGGB, 8, 4 },
    { "bayer_bggr16", frame::MODE_BAYER_BGGR, 16, 8 },
    { "bayer_bggr8", frame::MODE_BAYER_BGGR, 8, 4 },
    { "bayer_gbrg16", frame::MODE_BAYER_GBRG, 16, 8 },
    { "bayer_gbrg8", frame::MODE_BAYER_GBRG, 8, 4 },
    { "bayer_grbg16", frame::MODE_BAYER_GRBG, 16, 8 },
    { "bayer_grbg8", frame::MODE_BAYER_GRBG, 8, 4 },
    { "rgb8", frame::MODE_RGB, 8, 3 },
    { "rgb16", frame::MODE_RGB, 16, 6 },
    { "rgba8", frame::MODE_RGB32, 8, 4 },
    { "rgba16", frame::MODE_RGB32, 16, 8 },
    { "bgr8", frame::MODE_BGR, 8, 4 },
    { "bgr16", frame::MODE_BGR, 16, 8 },
    { "mono8", frame::MODE_GRAYSCALE, 8, 1 },
    { "mono16", frame::MODE_GRAYSCALE, 16, 2 },
    { "", static_cast<frame::frame_mode_t>(0), 0, 0}
};

static std::string rosEncodingFromRock(base::samples::frame::Frame const& frame)
{
    
    for (ModeMapping const* mapping = mappings; mapping->data_depth != 0; ++mapping)
    {
        if (mapping->data_depth == (int)frame.data_depth && mapping->pixel_size == (int)frame.pixel_size && mapping->mode == frame.frame_mode)
            return mapping->ros_name;
    }
    throw ros_convertions::InvalidROSConvertion("cannot convert a Rock frame with "
            "mode=" + boost::lexical_cast<std::string>(frame.frame_mode) + ", "
            "pixel_size=" + boost::lexical_cast<std::string>(frame.pixel_size) + ", "
            "data_depth=" + boost::lexical_cast<std::string>(frame.data_depth) +
            " to a ROS frame: unknown equivalent ROS encoding");
}

static void rockEncodingFromROS(base::samples::frame::Frame& frame, sensor_msgs::Image const& image)
{
    for (ModeMapping const* mapping = mappings; mapping->data_depth != 0; ++mapping)
    {
        if (mapping->ros_name == image.encoding)
        {
            frame.frame_mode = mapping->mode;
            frame.data_depth = mapping->data_depth;
            frame.pixel_size = mapping->pixel_size;
            return;
        }
    }
    throw ros_convertions::InvalidROSConvertion("cannot convert ROS image with encoding " + image.encoding + " to a Rock frame: unrepresentable encoding in Rock");
}

void ros_convertions::toROS( sensor_msgs::Image& ros, ::base::samples::frame::Frame const& value )
{
    toROS(ros.header.stamp, value.time);
    ros.height = value.size.height;
    ros.width = value.size.width;
    ros.encoding = rosEncodingFromRock(value);
    ros.step = value.row_size;
    ros.data = value.image;
    ros.is_bigendian = false;
}

void ros_convertions::fromROS( ::base::samples::frame::Frame& value, sensor_msgs::Image const& ros )
{
    fromROS(value.time, ros.header.stamp);
    value.size.height = ros.height;
    value.size.width = ros.width;
    rockEncodingFromROS(value, ros);
    value.row_size = ros.step;
    value.image = ros.data;
}

void ros_convertions::toROS( sensor_msgs::Image& ros, ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > const& value )
{
    toROS(ros, *value);
}
void ros_convertions::fromROS( ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame >& value, sensor_msgs::Image const& ros )
{
    base::samples::frame::Frame* frame = new base::samples::frame::Frame;
    fromROS(*frame, ros);
    value.reset(frame);
}

void ros_convertions::toROS( sensor_msgs::JointState& ros, ::base::samples::Joints  const& value )
{
    toROS(ros.header.stamp, value.time);

    ros.name.resize(value.names.size());
    ros.position.resize(value.elements.size());
    ros.velocity.resize(value.elements.size());
    ros.effort.resize(value.elements.size());

    ros.name =  value.names;

    for(size_t i = 0; i < value.elements.size(); ++i)
    {
        ros.position.at(i) =  value.elements.at(i).position;
        ros.velocity.at(i) =  value.elements.at(i).speed;
        ros.effort.at(i)   =  value.elements.at(i).effort;
    }

}

void ros_convertions::fromROS( ::base::samples::Joints& value, sensor_msgs::JointState const& ros )
{
    fromROS(value.time, ros.header.stamp);

    value.names.resize(ros.name.size());
    value.names = ros.name;

    // fields are optional in the ROS type and associated arrays
    // "should have the same size or be empty"
    size_t joint_number = std::max( ros.position.size(), ros.effort.size());
    joint_number = std::max(joint_number, ros.velocity.size());

    value.elements.resize(joint_number);

    for(size_t i = 0; i < joint_number; ++i)
    {
        if(!ros.position.empty())
        {
            value.elements.at(i).position = ros.position.at(i);
        }

        if(!ros.velocity.empty())
        {
            value.elements.at(i).speed = ros.velocity.at(i);
        }

        if(!ros.effort.empty())
        {
            value.elements.at(i).effort = ros.effort.at(i);
        }
    }
}

void ros_convertions::toROS( geometry_msgs::Pose& ros, ::base::Pose  const& value )
{
        ros.position.x = value.position.x();
        ros.position.y = value.position.y();
        ros.position.z = value.position.z();
        ros.orientation.x = value.orientation.x();
        ros.orientation.y = value.orientation.y();
        ros.orientation.z = value.orientation.z();
        ros.orientation.w = value.orientation.w();
}

void ros_convertions::fromROS( ::base::Pose& value, geometry_msgs::Pose const& ros )
{
    value.position.x() = ros.position.x;
    value.position.y() = ros.position.y;
    value.position.z() = ros.position.z;
    value.orientation.x() = ros.orientation.x;
    value.orientation.y() = ros.orientation.y;
    value.orientation.z() = ros.orientation.z;
    value.orientation.w() = ros.orientation.w;
}

void ros_convertions::toROS( geometry_msgs::Pose& ros, ::base::Pose_m const& value )
{
    ros.position.x = value.position.data[0];
    ros.position.y = value.position.data[1];;
    ros.position.z = value.position.data[2];
    ros.orientation.x = value.orientation.im[0];
    ros.orientation.y = value.orientation.im[1];
    ros.orientation.z = value.orientation.im[2];
    ros.orientation.w = value.orientation.re;
}

void ros_convertions::fromROS( ::base::Pose_m& value, geometry_msgs::Pose const& ros )
{
    value.position.data[0] = ros.position.x;
    value.position.data[1] = ros.position.y;
    value.position.data[2] = ros.position.z;
    value.orientation.im[0] = ros.orientation.x;
    value.orientation.im[1] = ros.orientation.y;
    value.orientation.im[2] = ros.orientation.z;
    value.orientation.re = ros.orientation.w;
}

void ros_convertions::toROS( trajectory_msgs::JointTrajectory& ros, ::base::JointsTrajectory const& value )
{

}

void ros_convertions::fromROS( ::base::JointsTrajectory& value, trajectory_msgs::JointTrajectory const& ros )
{	

	value.elements.resize(ros.points.size());
    	
	value.names = ros.joint_names;
    	

	for(int i = 0; i < ros.points.size(); i++)
	{
		value.elements.at(i).resize(ros.points.at(i).positions.size());

			for(int j = 0; j < ros.points.at(i).positions.size(); j++)
			{	    	
				value.elements.at(i).at(j).position = ros.points.at(i).positions.at(j);

				value.elements.at(i).at(j).speed = ros.points.at(i).velocities.at(j);
		
			}		
	}
    
}

