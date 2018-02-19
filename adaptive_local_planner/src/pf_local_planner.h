/**
Copyright (c) 2017 Xuan Sang LE <xsang.le@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
**/
#ifndef PF_LOCAL_PLANNER
#define PF_LOCAL_PLANNER

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Bool.h>
#include "3rd/map_builder.h"
#include "simple_ccl.h"
namespace local_planner
{

class PFLocalPlanner : public nav_core::BaseLocalPlanner
{
  public:
    /**
        * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
        * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
        * @return True if a valid velocity command was found, false otherwise
        */
    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

    /**
        * @brief  Check if the goal pose has been achieved by the local planner
        * @return True if achieved, false otherwise
        */
    bool isGoalReached();

    /**
        * @brief  Set the plan that the local planner is following
        * @param orig_global_plan The plan to pass to the local planner
        * @return True if the plan was updated successfully, false otherwise
        */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

    /**
        * @brief  Constructs the local planner
        * @param name The name to give this instance of the local planner
        * @param tf A pointer to a transform listener
        * @param costmap The cost map to use for assigning costs to local plans
        */
    void initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros);

    /**
        * @brief  Virtual destructor for the interface
        */
    ~PFLocalPlanner() {}
    PFLocalPlanner()
    {
        initialized_ = false;
        reached = false;
    }

  private:
    bool select_goal(geometry_msgs::PoseStamped *);
    bool my_pose(geometry_msgs::PoseStamped *);
    double dist(geometry_msgs::Point from, geometry_msgs::Point to);
    void adjust_velocity(tf::Vector3 *cmd);
    double get_angle(geometry_msgs::Point, geometry_msgs::Point, geometry_msgs::Point);
    map<int,geometry_msgs::Point> cc_min_dist_to_robot(tf::StampedTransform localToCmd, geometry_msgs::PoseStamped pose);

    double robot_radius,map_resolution,recovery_amplification; //, rotation;
    int min_obstacle_size_px, local_map_th, recovery_attemps, fw, fh;
    std::string goal_frame_id;
    std::string cmd_frame_id, scan_topic;
    double attractive_gain, repulsive_gain, safe_goal_dist, safe_obs_dist,max_local_goal_dist,max_linear_v, goal_tolerance, max_angular_v;
    std::vector<geometry_msgs::PoseStamped> global_plan;
    bool initialized_,verbose;
    bool reached;
    tf::TransformListener *tf;
    ros::NodeHandle private_nh;
    ros::Publisher  local_goal_pub, obstacles_pub, futur_pose_pub, local_map_pub, pf_status_pub;
    nav_msgs::OccupancyGrid local_map;
    actionlib_msgs::GoalStatusArray global_status;
    local_map::MapBuilder *map_builder;
    ros::Subscriber cmap_sub;
};
};
#endif
