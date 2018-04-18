#ifndef MAPPING_H
#define MAPPING_H

#include <map>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <angles/angles.h>
#include <ros/ros.h>
#include "3rd/raycaster/map_ray_caster.h"
#include "utils/luaconf.h"
#include "../localization/common.h"

using namespace std;
namespace dslam{
    class Mapping{
        public:
            Mapping();
            ~Mapping();
            void buildFrom(std::list<key_frame_t>&);
            void configure(Configuration&);
            nav_msgs::OccupancyGrid getMap();

        private:
            //void rotateSubMap(const nav_msgs::OccupancyGrid&,nav_msgs::OccupancyGrid&,Eigen::Quaterniond);
            //void resolveMapsize(Eigen::Vector3d,const nav_msgs::OccupancyGrid&);
            //void mergeSubmap(const nav_msgs::OccupancyGrid&,Eigen::Vector3d);
            
            bool castToObstacle(double , double , vector<size_t> &,  dslam_tf_t& _tf);
            void updatePoints(double, bool  , const vector<size_t> &, vector<int8_t> &, vector<double> &);
            void updateOccupancy(double, size_t idx, vector<int8_t> &, vector<double> &);
            void fromScan(const sensor_msgs::LaserScan &scan, dslam_tf_t& _tf);

            std::map<int, nav_msgs::OccupancyGrid> submaps_;
           //probability that a point is occupied from laser data 
            double p_occupied_w_observation_;
            // occupance probability of a free point calculated by
            // the filter
            double p_occupied_wo_observation_;
            // ray caster angle resolution
            double angle_resolution_,max_laser_range_;
            //Large log odds used with probability 0 and 1.
            double large_log_odds_;
            double max_log_odds_for_belief_;
            nav_msgs::OccupancyGrid map_;
            //!< log odds ratios for the binary Bayes filter
            std::vector<double> log_odds_;
            map_ray_caster::MapRayCaster ray_caster_;
    };
}

#endif