
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Bool.h>
#include "common.h"
double goal_tolerance, frontier_tolerance;
bool random_frontier;
std::string map_frame, base_frame, goal_topic, pf_stat_topic, cmd_vel_topic, move_base_status,frontier_topic;
geometry_msgs::Point frontier, old_frontier;
tf::TransformListener *listener;
actionlib_msgs::GoalStatusArray global_status;
std::vector<geometry_msgs::Point> reached_frontiers;
std::vector<geometry_msgs::Point> global_frontiers;
bool local_status = false;



void status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    global_status = *msg;
}

void pf_status_callback(const std_msgs::Bool::ConstPtr &msg)
{
    local_status = msg->data;
}

bool frontier_blacklisting(geometry_msgs::Point p)
{
    std::vector<geometry_msgs::Point>::iterator it;

    for (it = reached_frontiers.begin(); it != reached_frontiers.end(); it++)
    {
        double dist = distance<geometry_msgs::Point, geometry_msgs::Point>(*it, p);
        if (dist < goal_tolerance)
            return true;
    }
    return false;
}


void frontier_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    global_frontiers.clear();
    for (int i = 0; i < msg->poses.size(); i++)
    {
        if(!frontier_blacklisting(msg->poses[i].position))
            global_frontiers.push_back(msg->poses[i].position);
    }
}

/*
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    // threshold the map to find frontier
    //SimpleCCL cclo;
    //cclo.th = 255;
    //cclo.setMap(*msg);
    //cclo.print();
}
*/
bool find_next_frontier()
{

    geometry_msgs::Pose pose;
    if (!my_pose(&pose, listener, map_frame, base_frame ))
    {
        //ROS_WARN("Cannot find my pose");
        return false;
    }

    double dist = distance<geometry_msgs::Point, geometry_msgs::Point>(pose.position, frontier);
    if (dist > goal_tolerance)
    {
        bool path_clear = false;
        for (int i = 0; i < global_status.status_list.size(); i++)
        {
            int stat = global_status.status_list[i].status;
            if (stat == 1 && local_status)
                path_clear  = true;

            if(stat == 4 || stat == 5) // fail
            {
                reached_frontiers.push_back(frontier);
                path_clear = false;
                break;
            }
        }
        if(path_clear) return false;
    }
    else if(frontier.x != 0 || frontier.y != 0)
    {
        reached_frontiers.push_back(frontier);
    }
    local_status = true;
    if (global_frontiers.size() == 0)
    {
        ROS_WARN("No frontier to allocate at this time");
        reached_frontiers.clear();
        return false;
    }

    if (random_frontier)
    {
        old_frontier = frontier;
        frontier = global_frontiers[rand() % global_frontiers.size()];
        return true;
    }

    // nearest frontier allocation
    double mindist = 0, fr_dist;
    bool allocated = false;
    old_frontier = frontier;
    for (int i = 0; i < global_frontiers.size(); i++)
    {
        dist = distance<geometry_msgs::Point, geometry_msgs::Point>(pose.position, global_frontiers[i]);
        fr_dist = distance<geometry_msgs::Point, geometry_msgs::Point>(old_frontier, global_frontiers[i]);
        if ((old_frontier.x == 0 && old_frontier.y == 0) || ((mindist == 0 || dist < mindist) && dist > goal_tolerance && fr_dist > frontier_tolerance))
        {
            frontier = global_frontiers[i];
            mindist = dist;
            allocated = true;
        }
    }

    return allocated;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "frontier_allocator");
    ros::NodeHandle private_nh("~");
    listener = new tf::TransformListener();
    private_nh.param<double>("goal_tolerance", goal_tolerance, 0.3);
    private_nh.param<double>("frontier_tolerance", frontier_tolerance, 0.3);
    private_nh.param<bool>("random_frontier", random_frontier, false);
    private_nh.param<std::string>("map_frame", map_frame, "map");
    private_nh.param<std::string>("base_frame", base_frame, "base_link");
    private_nh.param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");
    private_nh.param<std::string>("pf_status", pf_stat_topic, "/move_base/PFLocalPlanner/pf_status" );
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
    private_nh.param<std::string>("move_base_status", move_base_status, "/move_base/status");
    private_nh.param<std::string>("frontier_topic", frontier_topic, "/frontiers");

    ros::Subscriber sub_ph = private_nh.subscribe<geometry_msgs::PoseArray>(frontier_topic, 10, &frontier_callback);
    ros::Publisher pub_goal = private_nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    ros::Subscriber sub_status = private_nh.subscribe<actionlib_msgs::GoalStatusArray>(move_base_status, 10, &status_callback);
    ros::Publisher cmd_vel_pub = private_nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
    ros::Subscriber sub_pf_status = private_nh.subscribe<std_msgs::Bool>(pf_stat_topic, 1, &pf_status_callback);

    srand(time(NULL));
    double rotation = 6.283;
    ros::Rate loop_rate(1.0);
    while(rotation > 0)
    {
        geometry_msgs::Twist cmd_vel;
         // do a in place rotation
        rotation -= 0.5;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0; // ?
        cmd_vel.angular.z = 1.0;
        cmd_vel_pub.publish(cmd_vel);
        loop_rate.sleep();
    }

    while (ros::ok())
    {
        ros::spinOnce();
        if (find_next_frontier())
        {
            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = map_frame;
            goal.pose.position.x = frontier.x;
            goal.pose.position.y = frontier.y;
            //ROS_ERROR("Frontier %f %f", frontier.x, frontier.y);
            goal.pose.orientation.x = 0;
            goal.pose.orientation.y = 0;
            goal.pose.orientation.z = 0;
            goal.pose.orientation.w = 1;

            pub_goal.publish(goal);
        }
        loop_rate.sleep();
    }
}