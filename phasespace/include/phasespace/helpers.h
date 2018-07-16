#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>

namespace helpers
{
    geometry_msgs::Point makePoint(const double x,
                                   const double y,
                                   const double z)
    {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    geometry_msgs::Vector3 makeVector3(const double x,
                                       const double y,
                                       const double z)
    {
        geometry_msgs::Vector3 v;
        v.x = x;
        v.y = y;
        v.z = z;
        return v;
    }

    geometry_msgs::Vector3 makeVector3(const geometry_msgs::Point &p)
    {
        geometry_msgs::Vector3 v;
        v.x = p.x;
        v.y = p.y;
        v.z = p.z;
        return v;
    }

    geometry_msgs::Pose makePose(const float *data_xyz_wxyz)
    {
        geometry_msgs::Pose p;
        p.position.x = data_xyz_wxyz[0];
        p.position.y = data_xyz_wxyz[1];
        p.position.z = data_xyz_wxyz[2];
        p.orientation.w = data_xyz_wxyz[3];
        p.orientation.x = data_xyz_wxyz[4];
        p.orientation.y = data_xyz_wxyz[5];
        p.orientation.z = data_xyz_wxyz[6];
        return p;
    }

    std_msgs::ColorRGBA makeColorRGBA(const float r,
                                      const float g,
                                      const float b,
                                      const float a)
    {
        std_msgs::ColorRGBA c;
        c.r = r;
        c.g = g;
        c.b = b;
        c.a = a;
        return c;
    }
}
