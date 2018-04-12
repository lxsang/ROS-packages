#ifndef LOCALMAP_BUILDER_H
#define LOCALMAP_BUILDER_H
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <angles/angles.h>
#include <ros/ros.h>
#include "3rd/raycaster/map_ray_caster.h"
#include "utils/luaconf.h"
using namespace std;
namespace dslam{
    class SubmapBuilder{
        public:
            SubmapBuilder(){};
            ~SubmapBuilder(){};
            void configure(Configuration&);
            nav_msgs::OccupancyGrid getMap();
            void fromScan(const sensor_msgs::LaserScan& scan);
        
        private:
            bool castToObstacle(double , double , vector<size_t> &);
            void updatePoints(bool , const vector<size_t> &, vector<int8_t> &, vector<double> &);
            void updateOccupancy(bool, size_t idx, vector<int8_t> &, vector<double> &);
            //probability that a point is occupied from laser data 
            double p_occupied_w_observation_;
            // occupance probability of a free point calculated by
            // the filter
            double p_occupied_wo_observation_;
            // ray caster angle resolution
            double angle_resolution_;
            //Large log odds used with probability 0 and 1.
            double large_log_odds_;
            double max_log_odds_for_belief_;
            nav_msgs::OccupancyGrid map_;
            //!< log odds ratios for the binary Bayes filter
            std::vector<double> log_odds_;
            // ray caster
            map_ray_caster::MapRayCaster ray_caster_;
    };
};
#endif