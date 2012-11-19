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

