#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <phasespace_msgs/MarkersStamped.h>
#include <phasespace_msgs/RigidsStamped.h>
#include "../include/phasespace/helpers.h"

#define DEFAULT_FRAME_ID "ps"
#define DEFAULT_RATE 30

////////////////////////////////////////////////////////////////////////////////

visualization_msgs::Marker makeVisMarkerFromRigid(const phasespace_msgs::Rigid &r, ros::Time stamp)
{
    visualization_msgs::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = std::string(DEFAULT_FRAME_ID);
    m.id = r.id;
    m.pose = r.pose;
    m.action = visualization_msgs::Marker::ADD;
    if (r.is_camera)
    {
        m.lifetime = ros::Duration();
        m.ns = std::string(DEFAULT_FRAME_ID) + std::string("_camera");
        m.type = visualization_msgs::Marker::MESH_RESOURCE;
        m.mesh_resource = std::string("package://phasespace/meshes/camera_body.stl");
        m.scale = helpers::makeVector3(1.0, 1.0, 1.0);
        m.color = helpers::makeColorRGBA(0.235, 0.235, 0.235, 1.0);
    }
    else
    {
        m.lifetime = ros::Duration(0.1);
        m.ns = std::string(DEFAULT_FRAME_ID) + std::string("_rigid");
        m.type = visualization_msgs::Marker::CUBE;
        m.scale = helpers::makeVector3(0.05, 0.05, 0.01);
        m.color = helpers::makeColorRGBA(0.0, 0.0, 1.0, 1.0);
    }

    return m;
}

visualization_msgs::Marker makeVisMarkerFromMarker(const phasespace_msgs::Marker &mk, ros::Time stamp)
{
    visualization_msgs::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = std::string(DEFAULT_FRAME_ID);
    m.id = mk.id;
    m.pose.position = mk.position;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0.1);
    m.ns = std::string(DEFAULT_FRAME_ID) + std::string("_marker");
    m.type = visualization_msgs::Marker::SPHERE;
    m.scale = helpers::makeVector3(0.01, 0.01, 0.01);
    m.color = helpers::makeColorRGBA(0.0, 1.0, 0.0, 1.0);
    return m;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vis_info");
    ros::NodeHandle nh("~");

    auto pub_all = nh.advertise<visualization_msgs::MarkerArray>("all", 1);
    visualization_msgs::MarkerArray vis_data;

    boost::function<void (const phasespace_msgs::RigidsStamped&)> cb_rigids =
            [&] (const phasespace_msgs::RigidsStamped &msg)
    {
        for (const phasespace_msgs::Rigid r : msg.data)
            vis_data.markers.push_back(makeVisMarkerFromRigid(r, msg.header.stamp));
    };

    boost::function<void (const phasespace_msgs::MarkersStamped&)> cb_markers =
            [&] (const phasespace_msgs::MarkersStamped &msg)
    {
        for (const phasespace_msgs::Marker m : msg.data)
            vis_data.markers.push_back(makeVisMarkerFromMarker(m, msg.header.stamp));
    };

    auto sub_cameras = nh.subscribe<phasespace_msgs::RigidsStamped>("/ps_owl/cameras", 1, cb_rigids);
    auto sub_rigids = nh.subscribe<phasespace_msgs::RigidsStamped>("/ps_owl/rigids", 1, cb_rigids);
    auto sub_markers = nh.subscribe<phasespace_msgs::MarkersStamped>("/ps_owl/markers", 1, cb_markers);

    boost::function<void (const ros::TimerEvent&)> cb_timer =
            [&] (const ros::TimerEvent &e)
    {
        if (ros::ok())
        {
            if (!vis_data.markers.empty())
            {
                pub_all.publish(vis_data);
                vis_data.markers.clear();
            }
        }
    };

    auto duration = 1.0 / nh.param("rate", DEFAULT_RATE);
    ros::Timer timer = nh.createTimer(ros::Duration(duration), cb_timer);

    ros::spin();

    return 0;
}
