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

#include "pf_local_planner.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(local_planner::PFLocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner
{

/*PFLocalPlanner::PFLocalPlanner()
    {
    }*/

map<int, geometry_msgs::Point> PFLocalPlanner::cc_min_dist_to_robot(tf::StampedTransform localToCmd, geometry_msgs::PoseStamped pose)
{
    
    map<int, geometry_msgs::Point> obs;
    SimpleCCL cclo;
    cclo.th = local_map_th;
    cclo.setMap(this->local_map);
    //cclo.print();
    if (cclo.labels.size() == 0)
        return obs;
    set<int>::iterator it;

    geometry_msgs::Point dist;
    dist.z = 0;
    dist.x = -1.0;
    dist.y = -1.0;
    // init
    for (it = cclo.labels.begin(); it != cclo.labels.end(); it++)
    {
        if (cclo.labels_tree[*it].cnt < min_obstacle_size_px) continue;
        obs[*it] = dist;
    }
    geometry_msgs::Point offset;
    offset.x = fabs(this->local_map.info.origin.position.x);
    offset.y = fabs(this->local_map.info.origin.position.y);
    double resolution = this->local_map.info.resolution;
    tf::Vector3 tmp;
    int i, j, idx, cell;
    for (i = 0; i < cclo.dw; i++)
        for (j = 0; j < cclo.dh; j++)
        {
            idx = j * cclo.dw + i;
            cell = cclo.data[idx];
            if (cell != -1)
            {
                if (cclo.labels_tree[cell].cnt < min_obstacle_size_px) continue;

                tmp.setX(i * resolution - offset.x);
                tmp.setY(j * resolution - offset.y);
                tmp = localToCmd*tmp;
                dist.x = tmp.x();
                dist.y = tmp.y();
                dist.z = this->dist(pose.pose.position, dist);
                if (obs[cell].z == 0 || obs[cell].z > dist.z)
                {
                    obs[cell] = dist;
                }
            }
        }

    map<int, geometry_msgs::Point>::iterator mit;
    geometry_msgs::PoseArray poses;
    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = this->cmd_frame_id;
    for (mit = obs.begin(); mit != obs.end(); mit++)
    {
        geometry_msgs::Pose p;
        p.position.x = mit->second.x;
        p.position.y = mit->second.y;
        p.orientation.w = 1.0;
        poses.poses.push_back(p);
    }
    obstacles_pub.publish(poses);

    return obs;
}
bool PFLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
    //return false;
    
    geometry_msgs::PoseStamped pose;
    tf::Vector3 cmd;
    std_msgs::Bool fb;
    if (!this->my_pose(&pose))
    {
        ROS_ERROR("Cannot get pose on the goal frame: %s", this->goal_frame_id.c_str());
        return false;
    }

    /*if(rotation > 0)
    {
        // do a in place rotation
        rotation -= 0.2;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0; // ?
        cmd_vel.angular.z = 1.0;
        return true;
    }*/

    geometry_msgs::PoseStamped goal;
    if (!this->select_goal(&goal))
    {
        ROS_WARN("No local goal is selected");
        reached = true;
        return true;
    }
        

    this->local_reached = false;
    tf::StampedTransform localToCmd;
    try
    {
        this->tf->lookupTransform(this->cmd_frame_id, this->local_map.header.frame_id, ros::Time(0), localToCmd);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("TF: %s - %s : %s", this->cmd_frame_id.c_str(), this->local_map.header.frame_id.c_str() , ex.what());
        return false;
    }

    /*ros::Subscriber  sub_status;
    // subcribes to related topics
    
    sub_status = private_nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10,
        [this](const actionlib_msgs::GoalStatusArray::ConstPtr &msg){
            this->global_status = *msg;
        });

    ros::spinOnce();

    for(int i = 0 ; i< global_status.status_list.size(); i++)
    {
        if(global_status.status_list[i].status == 4) // || global_status.status_list[i].status == 5)
        {
            ROS_WARN("This is not a valid global path, i will not attemp to compute velocity command for it");
            return false;
        }
    }*/

    
    double robot_to_goal = this->dist(pose.pose.position, goal.pose.position);
    double angle_diff = tf::getYaw(goal.pose.orientation) - tf::getYaw(pose.pose.orientation);
    if (robot_to_goal < goal_tolerance_linear)
    {
        cmd_vel.linear.z = 0.0;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0; // ?
        cmd_vel.angular.z = 0.0;
        if(fabs(angle_diff) < goal_tolerance_angular)
        {
            this->local_reached = true;
        }
        else
        {
            printf("NEED TO ROTATE IS: %f -> %f\n", angle_diff, goal_tolerance_angular);
            cmd_vel.angular.z = fabs(angle_diff) > max_angular_v ? (angle_diff/fabs(angle_diff))*max_angular_v: angle_diff;
        }
        fb.data = true;
        pf_status_pub.publish(fb);
        return true;
    }
    // now calculate the potential field toward the local goal
    double resolution = this->local_map.info.resolution;

    geometry_msgs::Point fatt;
    geometry_msgs::Point frep;
    fatt.x = 0.0;
    fatt.y = 0.0;
    frep.x = 0.0;
    frep.y = 0.0;

    // attractive potential

    if (robot_to_goal < safe_goal_dist)
    {
        fatt.x = attractive_gain * (goal.pose.position.x - pose.pose.position.x);
        fatt.y = attractive_gain * (goal.pose.position.y - pose.pose.position.y);
    }
    else
    {
        fatt.x = attractive_gain*(goal.pose.position.x - pose.pose.position.x) * safe_goal_dist / robot_to_goal;
        fatt.y = attractive_gain*(goal.pose.position.y - pose.pose.position.y) * safe_goal_dist / robot_to_goal;
    }

    // repulsive potential to the nearest obstacles

    int i, j, npix = 0;

    map<int, geometry_msgs::Point> obstacles = this->cc_min_dist_to_robot(localToCmd, pose);
    //return false;
    map<int, geometry_msgs::Point>::iterator mit;
    for (mit = obstacles.begin(); mit != obstacles.end(); mit++)
    {
        //double theta = atan2(mit->second.y, mit->second.x);
        if (mit->second.z <= this->safe_obs_dist)
        {
            double tmp;
            tmp = this->get_angle(goal.pose.position, mit->second, pose.pose.position);
            if(fabs(tmp) > 3.0*M_PI/5.0) continue;

            tmp = repulsive_gain * (1.0 / safe_obs_dist - 1.0 / mit->second.z) / (mit->second.z * mit->second.z);
            frep.x += tmp * mit->second.x;
            frep.y += tmp * mit->second.y;
        }
        else
        {
            frep.x += 0;
            frep.y += 0;
        }
    }

    if(verbose)
        ROS_INFO("attractive: %f, %f Respulsive: %f %f", fatt.x, fatt.y, frep.x, frep.y);
    // now calculate the velocity
    cmd.setX(fatt.x + frep.x);
    cmd.setY(fatt.y + frep.y);

    // now make a prediction of the future destination
    geometry_msgs::PoseStamped future_pose;

    int attemp = recovery_attemps;
    bool is_collision = true;
    double org_yaw,yaw, future_d, theta;
    org_yaw = atan2(cmd.y(), cmd.x()) - tf::getYaw(pose.pose.orientation);
    //this->adjust_velocity(&cmd);
    while(attemp != 0 && is_collision)
    {
        is_collision = false;
        future_pose.header.stamp = ros::Time::now();
        future_pose.header.frame_id = cmd_frame_id;
        yaw = atan2(cmd.y(), cmd.x()) - tf::getYaw(pose.pose.orientation);
        future_pose.pose.position.x = cmd.x() + (pose.pose.position.x*cos(yaw) - pose.pose.position.y*sin(yaw));
        future_pose.pose.position.y = cmd.y() + (pose.pose.position.x*sin(yaw) + pose.pose.position.y*cos(yaw));
        futur_pose_pub.publish(future_pose);
        this->adjust_velocity(&cmd);
        future_d = this->dist(future_pose.pose.position, pose.pose.position);

        for (mit = obstacles.begin(); mit != obstacles.end(); mit++)
        {
            theta = this->get_angle(future_pose.pose.position, mit->second, pose.pose.position) ; 
            if(fabs(theta) > M_PI/2.0) continue;
            double x = fabs(mit->second.z*sin(theta));
            if(x < robot_radius && mit->second.z < future_d)
            {
                double vf = recovery_amplification*repulsive_gain*(1.0 / robot_radius - 1.0 / mit->second.z) / (mit->second.z * mit->second.z);
                cmd.setX( cmd.x() + vf* mit->second.x);
                cmd.setY( cmd.y()+ vf* mit->second.y);
                is_collision = true;
                //break;
            }
        }
        attemp--;
    }

    if(fabs(yaw - org_yaw)  > M_PI/3.0)
    {
        ROS_ERROR("GOING WRONG WAY");
        is_collision = true;
    }

    this->adjust_velocity(&cmd);

    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0; // ?
    cmd_vel.angular.z = yaw;
    
    if(is_collision )
    {
        ROS_WARN("There will be a collision if i take this direction. I stop");
        fb.data = false;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
    }
    else 
    {
        fb.data = true;
        if(fabs(yaw) > max_angular_v)
        {
            cmd_vel.angular.z = (yaw/(fabs(yaw)))*max_angular_v;
        }
        else
        {
            cmd_vel.linear.x = cmd.x();
            cmd_vel.linear.y = cmd.y();
        }
        
        // check if v is so big
    }
    pf_status_pub.publish(fb);
    if(verbose)
        ROS_INFO("CMD VEL: x:%f y:%f w:%f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    return true;
}
double PFLocalPlanner::get_angle(geometry_msgs::Point v1, geometry_msgs::Point v2, geometry_msgs::Point pose)
{
    geometry_msgs::Point n1, n2;
    double d1 = this->dist(pose,v1);
    double d2 = this->dist(pose,v2);

    n1.x = (v1.x - pose.x)/d1;
    n1.y = (v1.y - pose.y)/d1;

    n2.x = (v2.x - pose.x)/d2;
    n2.y = (v2.y - pose.y)/d2;

    return acos(n1.x*n2.x + n1.y*n2.y);
}
void PFLocalPlanner::adjust_velocity(tf::Vector3 *cmd)
{
    double v = sqrt(pow(cmd->x(), 2)+ pow(cmd->y(), 2));
    if(v > max_linear_v)
    {
        double theta = atan2(cmd->y(), cmd->x());
        cmd->setX(max_linear_v*cos(theta));
        cmd->setY(max_linear_v*sin(theta));
    }
}
/*
    get my pose on the base_link frame
*/
bool PFLocalPlanner::my_pose(geometry_msgs::PoseStamped *pose)
{
    tf::StampedTransform transform;
    try
    {
        this->tf->lookupTransform(cmd_frame_id, this->cmd_frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Cannot get transform %s-%s: %s",cmd_frame_id.c_str(), this->cmd_frame_id.c_str(), ex.what());
        return false;
    }

    pose->header.stamp = ros::Time::now();
    pose->header.frame_id = cmd_frame_id;

    pose->pose.position.x = transform.getOrigin().getX();
    pose->pose.position.y = transform.getOrigin().getY();
    pose->pose.position.z = transform.getOrigin().getZ();

    pose->pose.orientation.x = transform.getRotation().getX();
    pose->pose.orientation.y = transform.getRotation().getY();
    pose->pose.orientation.z = transform.getRotation().getZ();
    pose->pose.orientation.w = transform.getRotation().getW();
    return true;
}
bool PFLocalPlanner::select_goal(geometry_msgs::PoseStamped *_goal)
{
    if (! this->local_reached)
    {
        *_goal = this->current_local_goal;
        return true;
    }
    if (this->global_plan.size() == 0)
    {
        //reached = true;
        return false;
    }
    geometry_msgs::PoseStamped pose;
    if (!this->my_pose(&pose))
    {
        ROS_ERROR("Cannot get pose on the goal frame: %s", this->goal_frame_id.c_str());
        return false;
    }

    tf::StampedTransform goalToLocal;
    try
    {
        this->tf->lookupTransform(this->cmd_frame_id, this->goal_frame_id, ros::Time(0), goalToLocal);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Goal To Local: %s", ex.what());
        return false;
    }
    std::vector<geometry_msgs::PoseStamped>::iterator it;
    double d, dx, dy;
    bool has_goal = false;
    tf::Vector3 candidate;
    tf::Quaternion orientation;
    int i=0;
    for (it = this->global_plan.begin(); it != this->global_plan.end(); it++)
    {
        candidate.setX(it->pose.position.x);
        candidate.setY(it->pose.position.y);
        orientation = tf::Quaternion(it->pose.orientation.x,it->pose.orientation.y,it->pose.orientation.z,it->pose.orientation.w);
        candidate = goalToLocal * candidate;
        geometry_msgs::Point tmp;
        tmp.x = candidate.x();
        tmp.y = candidate.y();
       // if(it->pose.position.z == 1.0) continue;
        d = this->dist(pose.pose.position,tmp);
        if (d > max_local_goal_dist)
            break;
        i++;
    }
    //if(it != global_plan.end())
    //    it->pose.position.z = 1.0
    orientation = goalToLocal*orientation;
    _goal->pose.position.x = candidate.x();
    _goal->pose.position.y = candidate.y();
    _goal->pose.orientation.x = orientation.x();
    _goal->pose.orientation.y = orientation.y();
    _goal->pose.orientation.z = orientation.z();
    _goal->pose.orientation.w = orientation.w();
    _goal->header.frame_id = this->cmd_frame_id;
    global_plan = vector<geometry_msgs::PoseStamped>(
        make_move_iterator(global_plan.begin() + i),
        make_move_iterator(global_plan.end()));
    local_goal_pub.publish(*_goal);
    this->current_local_goal = *_goal;
    return true;
}

double PFLocalPlanner::dist(geometry_msgs::Point from, geometry_msgs::Point to)
{
    // Euclidiant dist between a point and the robot
    return sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2));
}
bool PFLocalPlanner::isGoalReached()
{
    return this->reached;
}

bool PFLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
    if (!initialized_)
    {
        ROS_INFO("PFLocalPlanner hasn't initialized correctly! Quit.");
        return false;
    }
    this->global_plan.clear();
    this->global_plan = plan;
    this->reached = false;
    this->local_reached = true;
    return true;
}
void PFLocalPlanner::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
{
    /*some parameter*/
    if (!initialized_)
    {
        double dw, dh;
        std::string status_topic;
        ROS_INFO("Initialize adaptive local planner %s", name.c_str());
        private_nh = ros::NodeHandle(std::string("~") + "/" + name);
        private_nh.param<double>("robot_radius", this->robot_radius, 0.3f);
        private_nh.param<std::string>("goal_frame_id", this->goal_frame_id, "map");
        private_nh.param<std::string>("cmd_frame_id", this->cmd_frame_id, "base_link"); 
        private_nh.param<std::string>("status_topic", status_topic, "pf_status");
        //private_nh.param<std::string>("local_map_topic", this->local_map_topic, "/move_base/local_costmap/costmap");
        private_nh.param<double>("attractive_gain", this->attractive_gain, 1.0);
        private_nh.param<double>("repulsive_gain", this->repulsive_gain, 1.0);
        private_nh.param<double>("safe_goal_dist", this->safe_goal_dist, 1.0);
        private_nh.param<double>("safe_obs_dist", this->safe_obs_dist, 1.0);
        private_nh.param<double>("max_local_goal_dist", this->max_local_goal_dist, 0.5);
        private_nh.param<int>("min_obstacle_size_px", this->min_obstacle_size_px, 20);
        private_nh.param<bool>("verbose", this->verbose, true);
        private_nh.param<int>("local_map_th", local_map_th, 90);
        private_nh.param<int>("recovery_attemps", recovery_attemps, 10);
        private_nh.param<double>("goal_tolerance_linear", goal_tolerance_linear, 0.2);
        private_nh.param<double>("goal_tolerance_angular", goal_tolerance_angular, 0.2);
        private_nh.param<double>("max_linear_v", max_linear_v, 0.3);
        private_nh.param<double>("max_angular_v", max_angular_v, 1.3);
        private_nh.param<double>("recovery_amplification", recovery_amplification, 2.0);
        private_nh.param<double>("field_w", dw, 4.0);
        private_nh.param<double>("field_h", dh, 4.0);
        private_nh.param<double>("local_map_resolution", this->map_resolution, 0.05);
        private_nh.param<std::string>("scan_topic", this->scan_topic, "/scan");
        //ROS_ERROR("Scan topic is %s", this->scan_topic.c_str());
        this->fw = round(dw / this->map_resolution);
        this->fh = round(dh / this->map_resolution);
        map_builder = new local_map::MapBuilder(this->fw, this->fh, this->map_resolution);

        local_goal_pub = private_nh.advertise<geometry_msgs::PoseStamped>("local_goal", 1, true);
        futur_pose_pub = private_nh.advertise<geometry_msgs::PoseStamped>("future_pose", 1, true);
        obstacles_pub = private_nh.advertise<geometry_msgs::PoseArray>("obstacles", 1, true);
        local_map_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("pf_local_map",1,true);
        pf_status_pub =  private_nh.advertise<std_msgs::Bool>(status_topic,1,true);
        this->tf = tf;
        
        cmap_sub = private_nh.subscribe<sensor_msgs::LaserScan>(this->scan_topic, 10,
            [this](const sensor_msgs::LaserScan::ConstPtr &msg) {
                this->map_builder->grow(*msg);
                this->local_map = map_builder->getMap();
                //ROS_WARN("local map found");
                this->local_map_pub.publish(this->local_map);
           });
        initialized_ = true;
        //rotation = 2*M_PI;
    }
    else
    {
        ROS_INFO("Adaptive local planner has ben initialized, do nothing");
    }
}
};