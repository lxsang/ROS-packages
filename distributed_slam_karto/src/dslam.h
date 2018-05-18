#ifndef DSLAM_H
#define DSLAM_H

#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include "open_karto/Mapper.h"

#include "open_karto/G2OSolver.h"

#include "utils/luaconf.h"

namespace dslam {
    class DSlamKarto{
        public:
            DSlamKarto();
            ~DSlamKarto();
            void configure(Configuration &, ros::NodeHandle&);
            void publishTf();
            void publishGraph();
            void publishMap();
        private:
            void laserCallback(const sensor_msgs::LaserScan::ConstPtr&);
            bool getOdom(karto::Pose2&, const ros::Time&);
            karto::LaserRangeFinder* getRFDevice(const sensor_msgs::LaserScan::ConstPtr&);
            bool registerScan(karto::LaserRangeFinder* ,
                 const sensor_msgs::LaserScan::ConstPtr& ,
                 karto::Pose2& );
            bool  updateMap();
            tf::TransformListener tf_;
            tf::TransformBroadcaster tf_broadcaster_;
            tf::Transform fixed_to_odom_;

            bool map_init_;
            message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
            tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;

            std::string odom_frame_, fixed_frame_, base_frame_;

            boost::mutex mutex_;

            ros::Duration map_update_rate_;
            double resolution_;

            karto::Mapper* mapper_;
            karto::Dataset* database_;
            G2OSolver* optimizer_;
            
            ros::Publisher map_pub_;

            nav_msgs::OccupancyGrid map_;

            std::map<std::string, karto::LaserRangeFinder*> devices_;
            int num_scan_, throttle_scans_;
            ros::Time last_update_map_;
    };
}

#endif