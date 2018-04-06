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

using namespace dslam;

ScanMatcher matcher_;
ros::Publisher marker_publisher_;
ros::Publisher cloudpublisher_, trajectory_p_;
ros::Subscriber scan_subscriber_;
geometry_msgs::PoseArray estimated_poses_;
Eigen::Matrix4f  current_tf;
Eigen::Vector4f current_p;
std::string map_frame, laser_frame;
bool tf_ok_ = false;
tf::Vector3 offset_;
void run();
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  matcher_.registerScan(scan_msg);
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
  pcloud.header.frame_id = laser_frame;
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
  tf::Quaternion q(0,0,0,1); // q(pose.orientation.x, pose.orientation.y,pose.orientation.z,pose.orientation.w);
  //q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),map_frame, "base_link"));
}
void run()
{
    // Extract the lines
    icp_tf_t mt;
    matcher_.match(mt, callback);
    if(mt.converged && mt.fitness < 1.5)
    {
      ROS_DEBUG("Canculate point");
      std::cout<<"tf is" << std::endl << mt.tf << std::endl;
      
      current_tf = mt.tf*current_tf;
      current_p(0) =  current_tf(0,3);
      current_p(1) =  current_tf(1,3);
      current_p(2) =  current_tf(2,3);
      Eigen::Matrix3d rot;
      for(int i = 0; i < 3; i ++)
        for(int j = 0; j < 3; j ++)
          rot(i,j) = mt.tf(i,j);
      Eigen::Quaternion<double> q(rot);

      std::cout << "Point is " << current_p << std::endl;
      std::cout << "Rotation is " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      geometry_msgs::Pose pose;
      pose.position.x = current_p(0) - offset_.x();
      pose.position.y = current_p(1) - offset_.y();
      pose.position.z = current_p(2);
      //pose.orientation.x = q.x();
      //pose.orientation.y = q.y();
      //pose.orientation.z = q.z();
      //pose.orientation.w = q.w();
      estimated_poses_.poses.push_back(pose);
      estimated_poses_.header.frame_id = map_frame;
      estimated_poses_.header.stamp = ros::Time::now();
      trajectory_p_.publish(estimated_poses_);
    }
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

    std::string scan_topic, config_file, marker_topic;
    bool pub_markers;
    
    nh_local.param<std::string>("conf_file", config_file, "config.lua");
    Configuration config(config_file);
    Configuration localisation = config.get<Configuration>("localisation", Configuration());
    map_frame = localisation.get<std::string>("global_frame", "map");
    laser_frame = localisation.get<std::string>("laser_frame", "scan_frame");

    matcher_.configure(localisation);

    scan_topic = config.get<std::string>("scan_topic", "/laser_scan");    
    ROS_DEBUG("scan_topic: %s", scan_topic.c_str());


    marker_topic = config.get<std::string>("line_seg_topic", "/seg_markers");

    scan_subscriber_ = nh_local.subscribe(scan_topic, 1, &laserScanCallback);
    marker_publisher_ = nh_local.advertise<visualization_msgs::Marker>(marker_topic, 1);
    cloudpublisher_ = nh_local.advertise<sensor_msgs::PointCloud2>("/keypoints", 1);
    trajectory_p_ = nh_local.advertise<geometry_msgs::PoseArray>("/trajectory", 1);
    double frequency = config.get<double>("frequency", 25);
    ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
    ros::Rate rate(frequency);
    tf::TransformListener* listener = new tf::TransformListener();
    while (ros::ok())
    {
        if(!tf_ok_)
        {
          try
          {
            tf::StampedTransform laser2base;
            listener->lookupTransform("base_link", laser_frame, ros::Time(0), laser2base);
            offset_.setX(0);
            offset_.setY(0);
            offset_ = laser2base*offset_;
            tf_ok_ = true;
          }
          catch (tf::TransformException ex)
          {
              tf_ok_ = false;
              ROS_ERROR("Laser To Base: %s", ex.what());
          }
        }
        else
        {
          run();
          if(estimated_poses_.poses.size() > 0)
            publish_tranform(estimated_poses_.poses.back());
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
