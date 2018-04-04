#include "3rd/line_extraction/line_extraction.h"
#include "utils/luaconf.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

using namespace dslam;

LineExtraction line_extraction_;
ros::Publisher marker_publisher_;
  ros::Subscriber scan_subscriber_;
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

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  if (!data_cached_)
  {
    cacheData(scan_msg); 
    data_cached_ = true;
  }
  //ROS_DEBUG("Registe scan.");
  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
  line_extraction_.setRangeData(scan_ranges_doubles);
}

void populateMarkerMsg(const std::vector<Line> &lines, 
                                           visualization_msgs::Marker &marker_msg)
{
  marker_msg.ns = "line_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->getStart()[0];
    p_start.y = cit->getStart()[1];
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::Point p_end;
    p_end.x = cit->getEnd()[0];
    p_end.y = cit->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = "laser";
  marker_msg.header.stamp = ros::Time::now();
}

void run()
{
    // Extract the lines
  std::vector<Line> lines;
  line_extraction_.extractLines(lines);

  visualization_msgs::Marker marker_msg;
  populateMarkerMsg(lines, marker_msg);
  marker_publisher_.publish(marker_msg);
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
    Configuration xline_config = config.get<Configuration>("line_extraction", Configuration());

    scan_topic = xline_config.get<std::string>("scan_topic", "/laser_scan");    
    ROS_DEBUG("scan_topic: %s", scan_topic.c_str());

    // Parameters used by the line extraction algorithm

    double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh,
            max_line_gap, min_line_length, min_range, min_split_dist, outlier_dist;
    int min_line_points;

    bearing_std_dev = xline_config.get<double>("bearing_std_dev", 1e-3);
    line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);
    ROS_DEBUG("bearing_std_dev: %f", bearing_std_dev);

    range_std_dev = xline_config.get<double>("range_std_dev", 0.02);
    line_extraction_.setRangeVariance(range_std_dev * range_std_dev);
    ROS_DEBUG("range_std_dev: %f", range_std_dev);

    least_sq_angle_thresh = xline_config.get<double>("least_sq_angle_thresh", 1e-4);
    line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
    ROS_DEBUG("least_sq_angle_thresh: %f", least_sq_angle_thresh);

    least_sq_radius_thresh = xline_config.get<double>("least_sq_radius_thresh", 1e-4);
    line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
    ROS_DEBUG("least_sq_radius_thresh: %f", least_sq_radius_thresh);

    max_line_gap = xline_config.get<double>("max_line_gap", 0.4);
    line_extraction_.setMaxLineGap(max_line_gap);
    ROS_DEBUG("max_line_gap: %f", max_line_gap);

    min_line_length = xline_config.get<double>("min_line_length", 0.5);
    line_extraction_.setMinLineLength(min_line_length);
    ROS_DEBUG("min_line_length: %f", min_line_length);

    min_range = xline_config.get<double>("min_range", 0.4);
    line_extraction_.setMinRange(min_range);
    ROS_DEBUG("min_range: %f", min_range);

    min_split_dist = xline_config.get<double>("min_split_dist", 0.05);
    line_extraction_.setMinSplitDist(min_split_dist);
    ROS_DEBUG("min_split_dist: %f", min_split_dist);

    outlier_dist = xline_config.get<double>("outlier_dist", 0.05);
    line_extraction_.setOutlierDist(outlier_dist);
    ROS_DEBUG("outlier_dist: %f", outlier_dist);

    min_line_points = (int)xline_config.get<double>("min_line_points", 9.0);
    line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
    ROS_DEBUG("min_line_points: %d", min_line_points);

    marker_topic = config.get<std::string>("line_seg_topic", "/seg_markers");
    ROS_DEBUG("*************************************");

    scan_subscriber_ = nh_local.subscribe(scan_topic, 1, &laserScanCallback);
    marker_publisher_ = nh_local.advertise<visualization_msgs::Marker>(marker_topic, 1);

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
