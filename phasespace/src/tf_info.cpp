#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <phasespace_msgs/MarkersStamped.h>
#include <phasespace_msgs/RigidsStamped.h>
#include "../include/phasespace/helpers.h"

#define DEFAULT_FRAME_ID "ps"
#define DEFAULT_RATE 30

////////////////////////////////////////////////////////////////////////////////

geometry_msgs::TransformStamped toTransformStamped(const phasespace_msgs::Rigid &r, ros::Time stamp)
{
    geometry_msgs::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = std::string(DEFAULT_FRAME_ID);
    t.child_frame_id = std::string(DEFAULT_FRAME_ID) + std::string("_") + r.name;
    t.transform.translation = helpers::makeVector3(r.pose.position);
    t.transform.rotation = r.pose.orientation;
    return t;
}

geometry_msgs::TransformStamped toTransformStamped(const phasespace_msgs::Marker &m, ros::Time stamp)
{
    geometry_msgs::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = std::string(DEFAULT_FRAME_ID);
    t.child_frame_id = std::string(DEFAULT_FRAME_ID) + std::string("_") + m.name;
    t.transform.translation = helpers::makeVector3(m.position);
    t.transform.rotation.w = 1.0;
    return t;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_info");
    ros::NodeHandle nh("~");

    tf2_ros::StaticTransformBroadcaster tfs_pub;
    std::vector<geometry_msgs::TransformStamped> tfs_data;
    tf2_ros::TransformBroadcaster tf_pub;
    std::vector<geometry_msgs::TransformStamped> tf_data;

    boost::function<void (const phasespace_msgs::RigidsStamped&)> cb_rigids =
            [&] (const phasespace_msgs::RigidsStamped &msg)
    {
        for (const phasespace_msgs::Rigid r : msg.data)
        {
            if (r.is_camera)
                tfs_data.push_back(toTransformStamped(r, msg.header.stamp));
            else
                tf_data.push_back(toTransformStamped(r, msg.header.stamp));
        }
    };

    boost::function<void (const phasespace_msgs::MarkersStamped&)> cb_markers =
            [&] (const phasespace_msgs::MarkersStamped &msg)
    {
        for (const phasespace_msgs::Marker m : msg.data)
            tf_data.push_back(toTransformStamped(m, msg.header.stamp));
    };

    auto sub_cameras = nh.subscribe<phasespace_msgs::RigidsStamped>("/ps_owl/cameras", 1, cb_rigids);
    auto sub_rigids = nh.subscribe<phasespace_msgs::RigidsStamped>("/ps_owl/rigids", 1, cb_rigids);
    auto sub_markers = nh.subscribe<phasespace_msgs::MarkersStamped>("/ps_owl/markers", 1, cb_markers);

    boost::function<void (const ros::TimerEvent&)> cb_timer =
            [&] (const ros::TimerEvent &e)
    {
        if (ros::ok())
        {
            if (!tfs_data.empty())
            {
                tfs_pub.sendTransform(tfs_data);
                tfs_data.clear();
            }
            if (!tf_data.empty())
            {
                tf_pub.sendTransform(tf_data);
                tf_data.clear();
            }
        }
    };

    auto duration = 1.0 / nh.param("rate", DEFAULT_RATE);
    ros::Timer timer = nh.createTimer(ros::Duration(duration), cb_timer);

    ros::spin();

    return 0;
}
