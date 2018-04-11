#include "localization/ScanMatcher.h"
#include "utils/luaconf.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
using namespace dslam;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,nav_msgs::Odometry> NoCloudSyncPolicy;

LineScanMatcher matcher_;
ros::Publisher marker_publisher_;
ros::Publisher cloudpublisher_, trajectory_p_;
//message_filters::Subscriber scan_subscriber_, imu_subscriber_;
geometry_msgs::PoseArray estimated_poses_;
Eigen::Matrix4f  current_tf;
Eigen::Vector4f current_p;
std::string map_frame, laser_frame, robot_base_frame;
tf::StampedTransform laser2base;
bool tf_ok_ = false;
tf::Vector3 offset_;
void run();
void sensorCallback(const sensor_msgs::LaserScanConstPtr &scan_msg, const nav_msgs::OdometryConstPtr& odom)
{
  //ROS_DEBUG("Data found");
  matcher_.registerScan(scan_msg, odom);
  run();
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

    geometry_msgs::Point p_p;
    p_p.x = cit->getCenter()[0];
    p_p.y = cit->getCenter()[1];
    p_p.z = 0;


    geometry_msgs::Point p_end;
    p_end.x = cit->getEnd()[0];
    p_end.y = cit->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = laser_frame;
  marker_msg.header.stamp = ros::Time::now();
}
const void callback(std::vector<Line>& lines, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  visualization_msgs::Marker marker_msg;
  sensor_msgs::PointCloud2 pcloud;
  pcl::toROSMsg(cloud,pcloud); 
  pcloud.header.frame_id = robot_base_frame;
  pcloud.header.stamp = ros::Time::now();
  populateMarkerMsg(lines, marker_msg);
  marker_publisher_.publish(marker_msg);
  cloudpublisher_.publish(pcloud);
}

void publish_tranform(const geometry_msgs::Pose& pose)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
  tf::Quaternion q(pose.orientation.x, pose.orientation.y,pose.orientation.z,pose.orientation.w);
  //q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),map_frame, robot_base_frame));
}
void run()
{
    // Extract the lines
    icp_tf_t mt;
    matcher_.match(mt, callback);
    
    ROS_DEBUG("Canculate point");
    std::cout<<"tl is" << std::endl << mt.tl << std::endl;
    //current_tf = mt.tf*current_tf;

    //current_p(0) =  current_tf(0,3);
    //current_p(1) =  current_tf(1,3);
    //current_p(2) =  current_tf(2,3);
    //if(mt.tl(0) > 0.01 || mt.tl(1) > 0.01)
    current_p += mt.tl;
    //current_p(3) = 1.0;
    //current_p = mt.tf*current_p;


    std::cout << "Point is " << current_p << std::endl;
    std::cout << "Rotation is " << mt.rot.x() << " " << mt.rot.y() << " " << mt.rot.z() << " " << mt.rot.w() << std::endl;
    geometry_msgs::Pose pose;
    pose.position.x = current_p(0); // - offset_.x();
    pose.position.y = current_p(1); // - offset_.y();
    pose.position.z = current_p(2);
    pose.orientation.x = mt.rot.x();
    pose.orientation.y = mt.rot.y();
    pose.orientation.z = mt.rot.z();
    pose.orientation.w = mt.rot.w();
    estimated_poses_.poses.push_back(pose);
    estimated_poses_.header.frame_id = map_frame;
    estimated_poses_.header.stamp = ros::Time::now();
    trajectory_p_.publish(estimated_poses_);
}

int main(int argc, char **argv)
{
  current_tf = Eigen::Matrix4f::Identity();
  current_p(3) = 1.0;
  std::cout << "Current tf is " << current_tf << std::endl;
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("Starting line_extraction_node.");

    ros::init(argc, argv, "line_extraction_node");
    ros::NodeHandle nh_local("~");

    // Parameters used by this node

    std::string scan_topic, config_file, marker_topic, odom_topic;
    bool pub_markers;
    
    nh_local.param<std::string>("conf_file", config_file, "config.lua");
    Configuration config(config_file);
    Configuration localisation = config.get<Configuration>("localisation", Configuration());
    map_frame = localisation.get<std::string>("global_frame", "map");
    laser_frame = localisation.get<std::string>("laser_frame", "scan_frame");
    robot_base_frame = localisation.get<std::string>("robot_base", "base_link");
    matcher_.configure(localisation);

    scan_topic = config.get<std::string>("scan_topic", "/laser_scan");
    odom_topic = config.get<std::string>("odom_topic", "/odom"); 
    ROS_DEBUG("scan_topic: %s, IMUtopic: %s", scan_topic.c_str(), odom_topic.c_str());


    marker_topic = config.get<std::string>("line_seg_topic", "/seg_markers");

    //scan_subscriber_ = nh_local.subscribe(scan_topic, 1, &laserScanCallback);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_subscriber_(nh_local, scan_topic, 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_subscriber_(nh_local,odom_topic, 1);
    message_filters::Synchronizer<NoCloudSyncPolicy> sync(NoCloudSyncPolicy(100),scan_subscriber_,odom_subscriber_);
    sync.registerCallback(boost::bind(&sensorCallback, _1, _2));
    
    
    marker_publisher_ = nh_local.advertise<visualization_msgs::Marker>(marker_topic, 1);
    cloudpublisher_ = nh_local.advertise<sensor_msgs::PointCloud2>("/keypoints", 1);
    trajectory_p_ = nh_local.advertise<geometry_msgs::PoseArray>("/trajectory", 1);
    double frequency = config.get<double>("frequency", 25);
    ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
    ros::Rate rate(frequency);
    tf::TransformListener* listener = new tf::TransformListener();
    matcher_.setTF(listener);

    geometry_msgs::Pose pose;
    pose.position.x = 0.0; // - offset_.x();
    pose.position.y = 0.0; // - offset_.y();
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    estimated_poses_.poses.push_back(pose);
    estimated_poses_.header.frame_id = map_frame;
    estimated_poses_.header.stamp = ros::Time::now();

    while (ros::ok())
    {
        ros::spinOnce();
        if(estimated_poses_.poses.size() > 0)
          publish_tranform(estimated_poses_.poses.back());
        //rate.sleep();
    }
    return 0;
}
