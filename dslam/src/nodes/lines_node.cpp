#include "localization/ScanMatcher.h"
#include "utils/luaconf.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

using namespace dslam;

ScanMatcher matcher_;
ros::Publisher marker_publisher_;
ros::Publisher marker_publisher_1_;
  ros::Subscriber scan_subscriber_;

/*
bool data_cached_ = false;

void cacheData(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  const std::size_t num_measurements = std::ceil(
      (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
  for (std::size_t i = 0; i < num_measurements; ++i)
  {
    const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(i);
  }

  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  //ROS_DEBUG("Data has been cached.");
}
*/

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  matcher_.registerScan(scan_msg);
}

void populateMarkerMsg(const std::vector<Line> &lines, 
                                           visualization_msgs::Marker &marker_msg, visualization_msgs::Marker &marker_msg_1)
{
  marker_msg.ns = "line_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;

  marker_msg_1.ns = "line_extraction";
  marker_msg_1.id = 1;
  marker_msg_1.type = visualization_msgs::Marker::POINTS;//LINE_LIST;
  marker_msg_1.scale.x = 0.1;
  marker_msg_1.color.r = 0.0;
  marker_msg_1.color.g = 1.0;
  marker_msg_1.color.b = 0.0;
  marker_msg_1.color.a = 1.0;

  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->getStart()[0];
    p_start.y = cit->getStart()[1];
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    marker_msg_1.points.push_back(p_start);

    geometry_msgs::Point p_p;
    p_p.x = cit->getCenter()[0];
    p_p.y = cit->getCenter()[1];
    p_p.z = 0;
    marker_msg_1.points.push_back(p_p);


    geometry_msgs::Point p_end;
    p_end.x = cit->getEnd()[0];
    p_end.y = cit->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
    marker_msg_1.points.push_back(p_end);
  }
  marker_msg_1.header.frame_id = "laser";
  marker_msg_1.header.stamp = ros::Time::now();
  marker_msg.header.frame_id = "laser";
  marker_msg.header.stamp = ros::Time::now();
}
const void callback(std::vector<Line>& lines)
{
  visualization_msgs::Marker marker_msg, marker_msg_1;
  populateMarkerMsg(lines, marker_msg, marker_msg_1);
  marker_publisher_.publish(marker_msg);
  marker_publisher_1_.publish(marker_msg_1);
}
void run()
{
    // Extract the lines
    matcher_.match(callback);
}

int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("Starting line_extraction_node.");

    ros::init(argc, argv, "line_extraction_node");
    ros::NodeHandle nh_local("~");

    // Parameters used by this node

    std::string scan_topic, config_file, marker_topic;
    bool pub_markers;
    
    nh_local.param<std::string>("conf_file", config_file, "config.lua");
    Configuration config(config_file);
    Configuration localisation = config.get<Configuration>("localisation", Configuration());

    matcher_.configure(localisation);

    scan_topic = config.get<std::string>("scan_topic", "/laser_scan");    
    ROS_DEBUG("scan_topic: %s", scan_topic.c_str());


    marker_topic = config.get<std::string>("line_seg_topic", "/seg_markers");

    scan_subscriber_ = nh_local.subscribe(scan_topic, 1, &laserScanCallback);
    marker_publisher_ = nh_local.advertise<visualization_msgs::Marker>(marker_topic, 1);
    marker_publisher_1_ = nh_local.advertise<visualization_msgs::Marker>("/keypoints", 1);

    double frequency = config.get<double>("frequency", 25);
    ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
    ros::Rate rate(frequency);

    while (ros::ok())
    {
        run();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
