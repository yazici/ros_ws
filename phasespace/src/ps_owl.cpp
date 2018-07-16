#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <phasespace/ParametersPsOwlConfig.h>
#include <phasespace_msgs/PeaksStamped.h>
#include <phasespace_msgs/MarkersStamped.h>
#include <phasespace_msgs/RigidsStamped.h>
#include <phasespace_msgs/ButtonsStamped.h>
#include <yaml-cpp/yaml.h>
#include <owl.hpp>
#include "../include/phasespace/helpers.h"

#define DEFAULT_FRAME_ID "ps"
#define DEFAULT_SERVER_NAME "localhost"
#define DEFAULT_LED_PROFILE "default"
#define DEFAULT_RATE 240
#define DEFAULT_LED_POWER 0.05
#define DEFAULT_DATA_SCALE 0.001

////////////////////////////////////////////////////////////////////////////////

void createTrackers(ros::NodeHandle& nh, OWL::Context& owl)
{
    OWL::TrackerInfoTable trackerinfo;
    OWL::MarkerInfoTable markerinfo;

    // Retrieve name of tracker configuration file
    std::string tracker_file = nh.param("tracker_file", std::string());
    if (tracker_file.empty())
    {
        ROS_WARN("No tracker configuration data specified");
        return;
    }

    // Load YAML tracker configuration data
    YAML::Node data;
    try { data = YAML::LoadFile(tracker_file); }
    catch (const YAML::BadFile &exception)
    {
        ROS_ERROR("Failed to load tracker configuration file -> %s",
                  exception.what());
        return;
    }

    try
    {
        // Get value of "trackers" node (sequence of maps)
        const YAML::Node trackers = data["trackers"];
        for (const YAML::Node tracker : trackers)
        {
            // Store tracker information
            int tracker_id = tracker["id"].as<int>();
            std::string tracker_type = tracker["type"].as<std::string>();
            std::string tracker_name = tracker["name"].as<std::string>();
            trackerinfo.push_back(OWL::TrackerInfo(tracker_id,
                                                   tracker_type,
                                                   tracker_name));

            // Get value of "markers" node (sequence of maps)
            const YAML::Node markers = tracker["markers"];
            for (const YAML::Node marker : markers)
            {
                // Store marker information
                int marker_id = marker["id"].as<int>();
                std::string marker_name = marker["name"].as<std::string>();
                std::string marker_options = marker["options"].as<std::string>();
                markerinfo.push_back(OWL::MarkerInfo(marker_id,
                                                     tracker_id,
                                                     marker_name,
                                                     marker_options));
            }
        }
    }
    catch (const YAML::Exception &exception)
    {
        ROS_ERROR("Tracker configuration file contains invalid data -> %s",
                  exception.what());
        return;
    }

    // Create trackers
    if (!owl.createTrackers(trackerinfo.data(), trackerinfo.data() + trackerinfo.size()))
    {
        ROS_ERROR("Failed to create specified trackers");
        return;
    }

    // Assign markers to created trackers
    if (!owl.assignMarkers(markerinfo.data(), markerinfo.data() + markerinfo.size()))
    {
        ROS_ERROR("Failed to assign specified markers");
        return;
    }

    std::cout << "###############################################" << std::endl;
    for (OWL::TrackerInfo ti : trackerinfo)
    {
        std::cout << "Tracker: " << ti.name
                  << " (id: " << ti.id
                  << ", type: " << ti.type
                  << ")" << std::endl;

        for (OWL::MarkerInfo mi : markerinfo)
        {
            if (mi.tracker_id == ti.id)
                std::cout << "\tMarker " << mi.id
                          << " (name: " << mi.name
                          << ", options: " << (mi.options.empty() ? "none" : mi.options)
                          << ")" << std::endl;
        }
    }
    std::cout << "###############################################" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ps_owl");
    ros::NodeHandle nh("~");

    OWL::Context owl;
    OWL::Peaks owl_peaks;
    OWL::Markers owl_markers;
    OWL::Rigids owl_rigids;
    OWL::Inputs owl_inputs;

    dynamic_reconfigure::Server<phasespace::ParametersPsOwlConfig> srv;
    dynamic_reconfigure::Server<phasespace::ParametersPsOwlConfig>::CallbackType cb_config =
            [&] (phasespace::ParametersPsOwlConfig& config, uint32_t level)
    {
        if (owl.isOpen() && (owl.property<int>("initialized") == 1))
        {
            // Update the LED power setting on the server
            if (!owl.option("system.LEDPower", std::to_string(config.owl_led_power)))
                ROS_WARN("Failed to change the LED power setting");
        }
    };
    srv.setCallback(cb_config);

    // Retrieve node parameters from parameter server
    std::string owl_server = nh.param("owl_server", std::string(DEFAULT_SERVER_NAME));
    bool owl_slave_mode = nh.param("owl_slave_mode", false);
    std::string owl_profile = nh.param("owl_profile", std::string(DEFAULT_LED_PROFILE));
    int owl_rate = nh.param("owl_rate", DEFAULT_RATE);
    double owl_led_power = nh.param("owl_led_power", DEFAULT_LED_POWER);
    double owl_scale = nh.param("owl_scale", DEFAULT_DATA_SCALE);

    // Prepare PhaseSpace initialization options
    std::string owl_init_options;
    owl_init_options += "slave=" + owl_slave_mode ? std::to_string(1) : std::to_string(0);
    // Some options are unavailable in slave mode (which ones? -> undocumented)
    if (!owl_slave_mode)
    {
        owl_init_options += " profile=" + owl_profile;
        owl_init_options += " frequency=" + std::to_string(owl_rate);
        owl_init_options += " system.LEDPower=" + std::to_string(owl_led_power);
        owl_init_options += " event.raw=1";
        owl_init_options += " event.peaks=1";
        owl_init_options += " event.markers=1";
        owl_init_options += " event.rigids=1";
//        owl_init_options += " event.markervelocities=1";
//        owl_init_options += " event.rigidvelocities=1";
        owl_init_options += " event.driverstatus=1";
    }
    owl_init_options += " scale=" + std::to_string(owl_scale);
    owl_init_options += " streaming=1";

    // Connect to PhaseSpace OWL server
    if (owl.open(owl_server) <= 0)
    {
        ROS_ERROR("OWL::Context::open() failed");
        return -1;
    }
    if (owl.initialize(owl_init_options) <= 0)
    {
        ROS_ERROR("OWL::Context::initialize() failed");
        return -1;
    }

    std::cout << "###############################################" << std::endl;
    std::cout << "PhaseSpace OWL server: " << owl_server << std::endl;
    if (!owl_slave_mode)
    {
        std::cout << "OWL profile: " << owl_profile << std::endl;
        std::cout << "OWL frequency: " << owl_rate << std::endl;
        std::cout << "OWL LED power: " << owl_led_power << std::endl;
    }
    else
        std::cout << "ATTENTION: Node is running in slave mode!" << std::endl;
    std::cout << "OWL scale factor: " << owl_scale << std::endl;
    std::cout << "###############################################" << std::endl;

    // Create OWL trackers and assign markers
    if (!owl_slave_mode) { createTrackers(nh, owl); }

    auto pub_cameras = nh.advertise<phasespace_msgs::RigidsStamped>("cameras", 1, true);
    // Publish static camera poses
    phasespace_msgs::RigidsStamped cameras;
    cameras.header.stamp = ros::Time::now();
    cameras.header.frame_id = std::string(DEFAULT_FRAME_ID);
    OWL::Cameras owl_cameras = owl.property<OWL::Cameras>("cameras");
    for (const OWL::Camera c : owl_cameras)
    {
        phasespace_msgs::Rigid r;
        r.is_camera = true;
        r.name = std::string("camera_") + std::to_string(c.id);;
        r.id = c.id;
        r.pose = helpers::makePose(c.pose);
        r.condition = c.cond;
        cameras.data.push_back(r);
    }
    pub_cameras.publish(cameras);

    auto pub_peaks = nh.advertise<phasespace_msgs::PeaksStamped>("peaks", 1);
    auto pub_rigids = nh.advertise<phasespace_msgs::RigidsStamped>("rigids", 1);
    auto pub_markers = nh.advertise<phasespace_msgs::MarkersStamped>("markers", 1);
    auto pub_buttons = nh.advertise<phasespace_msgs::ButtonsStamped>("buttons", 1);

    boost::function<void (const ros::TimerEvent&)> cb_timer =
            [&] (const ros::TimerEvent& e)
    {
        if (ros::ok() && owl.isOpen() && (owl.property<int>("initialized") == 1))
        {
            const OWL::Event* event = owl.nextEvent(1000);
            if (!event) return;

            // Process in-band events ONLY! (synchronized to optical frame)
            if (event->type_id() != OWL::Type::FRAME) return;

            // Look for OWL::Type::PEAK data
            owl_peaks.clear();
            event->find("peaks", owl_peaks);
            if (!owl_peaks.empty())
            {
                phasespace_msgs::PeaksStamped peaks;
                peaks.header.stamp = ros::Time::now();
                for (const OWL::Peak p : owl_peaks)
                {
                    phasespace_msgs::Peak peak;
                    peak.id = p.id;
                    peak.flags = p.flags;
                    peak.camera = p.camera;
                    peak.detector = p.detector;
                    peak.amp = p.amp;
                    peaks.data.push_back(peak);
                }
                pub_peaks.publish(peaks);
            }

            // Look for OWL::Type::RIGID data
            owl_rigids.clear();
            event->find("rigids", owl_rigids);
            if (!owl_rigids.empty())
            {
                phasespace_msgs::RigidsStamped rigids;
                rigids.header.stamp = ros::Time::now();
                rigids.header.frame_id = std::string(DEFAULT_FRAME_ID);
                for (const OWL::Rigid r : owl_rigids)
                {
                    // Omit invalid rigid data
                    if (r.cond <= 0) continue;

                    const OWL::TrackerInfo info = owl.trackerInfo(r.id);
                    phasespace_msgs::Rigid rigid;
                    rigid.is_camera = false;
                    rigid.id = r.id;
                    rigid.name = info.name;
                    rigid.flags = r.flags;
                    rigid.pose = helpers::makePose(r.pose);
                    rigid.condition = r.cond;
                    rigids.data.push_back(rigid);
                }
                pub_rigids.publish(rigids);
            }

            // Look for OWL::Type::MARKER data
            owl_markers.clear();
            event->find("markers", owl_markers);
            if (!owl_markers.empty())
            {
                phasespace_msgs::MarkersStamped markers;
                markers.header.stamp = ros::Time::now();
                markers.header.frame_id = std::string(DEFAULT_FRAME_ID);
                for (const OWL::Marker m : owl_markers)
                {
                    // Omit invalid marker data
                    if (m.cond <= 0) continue;

                    const OWL::MarkerInfo info = owl.markerInfo(m.id);
                    phasespace_msgs::Marker marker;
                    marker.id = m.id;
                    marker.tracker_id = info.tracker_id;
                    marker.name = info.name;
                    marker.options = info.options;
                    marker.flags = m.flags;
                    marker.position.x = m.x;
                    marker.position.y = m.y;
                    marker.position.z = m.z;
                    marker.condition = m.cond;
                    markers.data.push_back(marker);
                }
                pub_markers.publish(markers);
            }

            // Look for OWL::Type::INPUT data
            owl_inputs.clear();
            event->find("driverstatus", owl_inputs);
            if (!owl_inputs.empty())
            {
                phasespace_msgs::ButtonsStamped buttons;
                buttons.header.stamp = ros::Time::now();
                for (const OWL::Input i : owl_inputs)
                {
                    phasespace_msgs::Button button;
                    button.hw_id = i.hw_id;
                    button.id = 0;
//                    button.toggled = (((i.flags >> 5) & 0x1) == 1);
                    button.pressed = (((i.flags >> 4) & 0x1) == 1);
                    buttons.data.push_back(button);
                }
                pub_buttons.publish(buttons);
            }
        }
    };

    auto duration = 1.0 / (nh.param("rate", DEFAULT_RATE)*1.5);
    ros::Timer timer = nh.createTimer(ros::Duration(duration), cb_timer);

//    ros::AsyncSpinner spinner(2);
//    spinner.start();
    ros::spin();

    owl.done();
    owl.close();

    return 0;
}
