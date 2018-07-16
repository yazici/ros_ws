#include <iomanip>
#include <ros/ros.h>
#include <phasespace_msgs/MarkersStamped.h>
#include <eigen3/Eigen/Dense>

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ps_align");
    ros::NodeHandle nh("~");

    // Check parameters
    if (!nh.hasParam("origin_id"))
    {
        ROS_ERROR("Parameter \"origin_id\" was not set");
        return -1;
    }
    if (!nh.hasParam("xaxis_id"))
    {
        ROS_ERROR("Parameter \"xaxis_id\" was not set");
        return -1;
    }
    if (!nh.hasParam("yaxis_id"))
    {
        ROS_ERROR("Parameter \"yaxis_id\" was not set");
        return -1;
    }
    if (!nh.hasParam("origin_offsets"))
    {
        ROS_ERROR("Parameter \"origin_offsets\" was not set");
        return -1;
    }

    // Get parameters
    int origin_id, xaxis_id, yaxis_id;
    nh.getParam("origin_id", origin_id);
    nh.getParam("xaxis_id", xaxis_id);
    nh.getParam("yaxis_id", yaxis_id);
    std::vector<double> origin_offsets;
    nh.getParam("origin_offsets", origin_offsets);
    if (origin_offsets.size() != 3)
    {
        ROS_ERROR("Failed to retrieve parameter \"origin_offsets\"");
        return -1;
    }

    // Tell the user
    std::cout << "Using marker IDs: "
              << origin_id << " (origin), "
              << xaxis_id << " (x-axis), "
              << yaxis_id << " (y-axis)"
              << std::endl;
    std::cout << "Using origin offsets: "
              << origin_offsets.at(0) << " (x-axis), "
              << origin_offsets.at(1) << " (y-axis), "
              << origin_offsets.at(2) << " (z-axis)"
              << std::endl;

    boost::function<void (const phasespace_msgs::MarkersStamped&)> cb_markers =
            [&] (const phasespace_msgs::MarkersStamped& msg)
    {
        Eigen::Vector3d origin, xaxis, yaxis;
        bool origin_set = false, xaxis_set = false, yaxis_set = false;

        for (const phasespace_msgs::Marker &m : msg.data)
        {
            if (m.id == origin_id)
            {
                origin = Eigen::Vector3d(m.position.x, m.position.y, m.position.z);
                origin_set = true;
            }
            else if (m.id == xaxis_id)
            {
                xaxis = Eigen::Vector3d(m.position.x, m.position.y, m.position.z);
                xaxis_set = true;
            }
            else if (m.id == yaxis_id)
            {
                yaxis = Eigen::Vector3d(m.position.x, m.position.y, m.position.z);
                yaxis_set = true;
            }
        }

        if (!(origin_set && xaxis_set && yaxis_set))
        {
            std::cerr << "Missing data for marker(s):"
                      << (origin_set ? "" : " " + std::to_string(origin_id))
                      << (xaxis_set ? "" : " " + std::to_string(xaxis_id))
                      << (yaxis_set ? "" : " " + std::to_string(yaxis_id))
                      << std::endl;
            return;
        }

        // Establish coordinate frame using coordinates of the specified markers
        Eigen::Vector3d xvec = xaxis - origin;
        xvec.normalize();
        Eigen::Vector3d yvec = yaxis - origin;
        yvec.normalize();
        Eigen::Vector3d zvec = xvec.cross(yvec);
        zvec.normalize();
        // Ensure the established coordinate frame is orthogonal
        yvec = zvec.cross(xvec);
        yvec.normalize();

        // Create a homogeneous matrix representing the transformation from
        // the established one into the PhaseSpace coordinate frame
        Eigen::Matrix4d tf_local_world;
        tf_local_world.col(0) = Eigen::Vector4d(xvec(0), xvec(1), xvec(2), 0);
        tf_local_world.col(1) = Eigen::Vector4d(yvec(0), yvec(1), yvec(2), 0);
        tf_local_world.col(2) = Eigen::Vector4d(zvec(0), zvec(1), zvec(2), 0);
        tf_local_world.col(3) = Eigen::Vector4d(origin(0), origin(1), origin(2), 1);

        // Apply specified coordinate offsets of the origin marker
        Eigen::Matrix4d tf_offsets = Eigen::Matrix4d::Identity();
        tf_offsets.col(3) = Eigen::Vector4d(origin_offsets.at(0), origin_offsets.at(1), origin_offsets.at(2), 1);
        tf_local_world = tf_local_world * tf_offsets;

        // Determine the inverse transformation from the PhaseSpace coordinate
        // frame to that of the established one
        Eigen::Matrix4d tf_world_local = tf_local_world.inverse();

        // Get the orientation vectors from the inverse transformation matrix
        Eigen::Vector3d xvec_inv = tf_world_local.col(0).head<3>();
        Eigen::Vector3d yvec_inv = tf_world_local.col(1).head<3>();
        Eigen::Vector3d zvec_inv = tf_world_local.col(2).head<3>();
        // Get the position vector from the inverse transformation matrix
        Eigen::Vector3d pos_inv = tf_world_local.col(3).head<3>();

        // Determine factorization as Rz*Ry*Rx -> Yaw*Pitch*Roll
        // According to https://www.geometrictools.com/Documentation/EulerAngles.pdf
        if (xvec_inv(2) > -1 && xvec_inv(2) < 1)
        {
            double yaw = atan2(xvec_inv(1), xvec_inv(0));
            double pitch = asin(-xvec_inv(2));
            double roll = atan2(yvec_inv(2), zvec_inv(2));

            std::cout << std::setprecision(10)
                      << ">> Alignment transform (x y z yaw pitch roll) -> "
                      << pos_inv.x() << " "
                      << pos_inv.y() << " "
                      << pos_inv.z() << " "
                      << yaw << " "
                      << pitch << " "
                      << roll
                      << std::endl;
        }
        else
            std::cerr << "Unable to determine a unique factorization" << std::endl;

        // Clean up and shut down
        nh.deleteParam("origin_id");
        nh.deleteParam("xaxis_id");
        nh.deleteParam("yaxis_id");
        nh.deleteParam("origin_offsets");
        ros::shutdown();
    };

    auto sub_markers = nh.subscribe<phasespace_msgs::MarkersStamped>("/ps_owl/markers", 1, cb_markers);

    ros::spin();

    return 0;
}
