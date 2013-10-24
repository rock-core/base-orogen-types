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
	
    size_t joint_number = value.elements.size();

    ros.position.resize(joint_number);
    ros.velocity.resize(joint_number);
    ros.effort.resize(joint_number);

    for(size_t i = 0; i < joint_number; ++i)
    {
	if(value.elements.at(i).hasPosition())
	{
		ros.position.at(i) = value.elements.at(i).position;
	}
	if(value.elements.at(i).hasSpeed())
	{
		ros.velocity.at(i) = value.elements.at(i).speed;
	}

	if(value.elements.at(i).hasEffort())
	{
		ros.effort.at(i) = value.elements.at(i).effort;
	}

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

void ros_convertions::toROS( geometry_msgs::PoseStamped& ros, ::base::samples::RigidBodyState const& value )
{
	toROS(ros.header.stamp, value.time);
	
	ros.header.frame_id = value.sourceFrame ;

	ros.pose.position.x = value.position.x();
	ros.pose.position.y = value.position.y();
	ros.pose.position.z = value.position.z();
	ros.pose.orientation.x = value.orientation.x();
	ros.pose.orientation.y = value.orientation.y();
	ros.pose.orientation.z = value.orientation.z();
	ros.pose.orientation.w = value.orientation.w();

}
void ros_convertions::fromROS( ::base::samples::RigidBodyState& value, geometry_msgs::PoseStamped const& ros )
{
}
void ros_convertions::toROS( geometry_msgs::PoseStamped& ros, ::base::samples::RigidBodyState_m const& value )
{
	toROS(ros.header.stamp, value.time);
	
	ros.header.frame_id = value.sourceFrame ;

	ros.pose.position.x = value.position.data[0];
	ros.pose.position.y = value.position.data[1];
	ros.pose.position.z = value.position.data[2];
	ros.pose.orientation.x = value.orientation.im[0];
	ros.pose.orientation.y = value.orientation.im[1];
	ros.pose.orientation.z = value.orientation.im[2];
	ros.pose.orientation.w = value.orientation.re;
}
void ros_convertions::fromROS( ::base::samples::RigidBodyState_m& value, geometry_msgs::PoseStamped const& ros )
{
}



void ros_convertions::toROS( sensor_msgs::PointCloud2& ros, ::base::samples::Pointcloud const& value )
{
    // test mapping

    int pointcloud_size = value.points.size();

    if(pointcloud_size > 0)
    {
        
        std::cout<<"point size = "<<value.points.size()<<"  colour size = "<<value.colors.size()<<std::endl;
        //convert base::pointcloud to sensor_msgs::PointCloud2

        ros.width   = static_cast<uint32_t>(value.points.size ());
        ros.height  = 1;

        std::cout<<"ros.width = "<<ros.width<<std::endl;

        if(pointcloud_size == value.colors.size())
        {
            // fill point cloud binary data
            size_t data_size = 7 * sizeof (double) * value.points.size ();

            std::cout<<"data_size = "<<data_size<<std::endl;
            ros.data.resize(data_size);

            for (int i = 0; i < value.points.size(); i++)
            {
                ros.data.at(i*7)       = value.points.at(i)[0];
                ros.data.at((i*7)+1)   = value.points.at(i)[1];
                ros.data.at((i*7)+2)   = value.points.at(i)[2];
                ros.data.at((i*7)+3)   = value.colors.at(i)[0];
                ros.data.at((i*7)+4)   = value.colors.at(i)[1];
                ros.data.at((i*7)+5)   = value.colors.at(i)[2];
                ros.data.at((i*7)+6)   = value.colors.at(i)[3];
            }

            // fields
            ros.fields.resize(7);
            ros.fields.at(0).name       = "X";
            ros.fields.at(1).name       = "Y";
            ros.fields.at(2).name       = "Z";
            ros.fields.at(3).name       = "R";
            ros.fields.at(4).name       = "G";
            ros.fields.at(5).name       = "B";
            ros.fields.at(6).name       = "a";

            for(int i = 0; i < 7; i++)
            {
                ros.fields.at(i).offset     = i;
                ros.fields.at(i).datatype   = 8;
                ros.fields.at(i).count      = 1;
            }
        }
        else if (value.colors.size() == 0)
        {
            size_t data_size = 3 * sizeof (double) * value.points.size ();

            std::cout<<"data_size = "<<data_size<<std::endl;
            ros.data.resize(data_size);
            std::cout<<"data_size ros= "<<ros.data.size()<<std::endl;
	    
	    memcpy(&ros.data[0], &value.points[0], data_size);

            /*for (int i = 0; i < value.points.size(); i++)
            {
                ros.data.at(i*3)       = static_cast<uint_8t> value.points.at(i).x();
                ros.data.at((i*3)+1)   = value.points.at(i).y();
                ros.data.at((i*3)+2)   = value.points.at(i).z();

		//std::cout<< value.points.at(i).x()<<std::endl;
		std::cout<< ros.data.at(i*3)<<"  "<<ros.data.at((i*3)+1)<<std::endl;
            }*/
		std::cout<< ros.data.at(0)<<"  "<<ros.data.at(3)<<std::endl;

	    

            // fields
            ros.fields.resize(3);
            ros.fields.at(0).name       = "X";
            ros.fields.at(1).name       = "Y";
            ros.fields.at(2).name       = "Z";

            for(int i = 0; i < 3; i++)
            {
                ros.fields.at(i).offset     = i;
                ros.fields.at(i).datatype   = 8;
                ros.fields.at(i).count      = 1;
            }
            
        }
    
    }

}
void ros_convertions::fromROS( ::base::samples::Pointcloud& value, sensor_msgs::PointCloud2 const& ros )
{
}




