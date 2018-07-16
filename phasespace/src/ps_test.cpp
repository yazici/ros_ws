#include <iomanip>
#include <ros/ros.h>
#include <phasespace_msgs/MarkersStamped.h>
#include <eigen3/Eigen/Dense>

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ps_test");
    ros::NodeHandle nh("~");

    // Check parameters
    if (!nh.hasParam("marker1_id"))
    {
        ROS_ERROR("Parameter \"marker1_id\" was not set");
        return -1;
    }
    if (!nh.hasParam("marker2_id"))
    {
        ROS_ERROR("Parameter \"marker2_id\" was not set");
        return -1;
    }

    // Get parameters
    int marker1_id, marker2_id;
    nh.getParam("marker1_id", marker1_id);
    nh.getParam("marker2_id", marker2_id);

    // Tell the user
    std::cout << "Using marker IDs: "
              << marker1_id << " (marker1), "
              << marker2_id << " (marker2)"
              << std::endl;

    double min_dist = DBL_MAX, max_dist = 0.0;

    boost::function<void (const phasespace_msgs::MarkersStamped&)> cb_markers =
            [&] (const phasespace_msgs::MarkersStamped& msg)
    {
        Eigen::Vector3d marker1, marker2;
        bool marker1_set = false, marker2_set = false;

        for (const phasespace_msgs::Marker& m : msg.data)
        {
            if (m.id == marker1_id)
            {
                marker1 = Eigen::Vector3d(m.position.x, m.position.y, m.position.z);
                marker1_set = true;
            }
            else if (m.id == marker2_id)
            {
                marker2 = Eigen::Vector3d(m.position.x, m.position.y, m.position.z);
                marker2_set = true;
            }
        }

        if (!(marker1_set && marker2_set))
        {
            std::cerr << "Missing data for marker(s):"
                      << (marker1_set ? "" : " " + std::to_string(marker1_id))
                      << (marker2_set ? "" : " " + std::to_string(marker2_id))
                      << std::endl;
            return;
        }

        double dist = (marker1-marker2).norm();
        if (min_dist > dist)
            min_dist = dist;
        if (max_dist < dist)
            max_dist = dist;

        std::cout << std::setprecision(5)
                  << ">> Distance between markers -> "
                  << dist << " (current), "
                  << min_dist << " (min.), "
                  << max_dist << " (max.)"
                  << std::endl;
    };

    auto sub_markers = nh.subscribe<phasespace_msgs::MarkersStamped>("/ps_owl/markers", 1, cb_markers);

    ros::spin();

    return 0;
}
