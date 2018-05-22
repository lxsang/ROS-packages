#include "common.h"
#include "simple_ccl.h"

tf::TransformListener *listener;
ros::Publisher frontiers_map_pub,frontiers_pub;
std::string base_frame;
int min_frontier_size_px;


bool is_frontier(const nav_msgs::OccupancyGrid::ConstPtr & map, int x, int y)
{
    int dw,dh;
    dw = map->info.width;
    dh = map->info.height;
    
    if(x != 0 && map->data[x - 1 + y*dw] == 0) return true;
    if(y != 0 && map->data[x  + (y-1)*dw]== 0) return true;
    if(x != 0 && y != 0 && map->data[x - 1 + (y-1)*dw]== 0) return true;
    if(x != dw - 1 && y != 0 && map->data[x + 1 + (y-1)*dw]== 0) return true;
    if(x != dw - 1 && map->data[x + 1 + (y)*dw]== 0)  return true;
    if(x !=0 && y != dh - 1 && map->data[x - 1 + (y+1)*dw] == 0) return true;
    if( y != dh - 1 && map->data[x  + (y+1)*dw] == 0) return true;
    if(x != dw - 1 && y != dh - 1 && map->data[x + 1 + (y+1)*dw] == 0)  return true;
    
    return false;
}


void calcul_frontiers(const nav_msgs::OccupancyGrid::ConstPtr & gmap)
{
    geometry_msgs::Pose pose;
    if (!my_pose(&pose, listener, gmap->header.frame_id, base_frame ))
    {
        ROS_ERROR("Cannot find my pose");
        return ;
    }
    // calculate frontier
    nav_msgs::OccupancyGrid frontiers_map;
    frontiers_map.header = gmap->header;
    frontiers_map.info = gmap->info;
    frontiers_map.data.resize(gmap->info.width*gmap->info.height,-1);
    //std::fill(frontiers_map.data.begin(), frontiers_map.data.end(), -1);

    int i,j, idx;
    int8_t cell;
    for(i=0; i < gmap->info.width; i++)
        for(j = 0; j < gmap->info.height; j++)
        {
            idx = j*gmap->info.width + i;
            cell = gmap->data[idx];

            if(cell == -1 && is_frontier(gmap,i,j))
                frontiers_map.data[idx] = 100;
        }
    
    //
    
    SimpleCCL cclo;
    cclo.th = 100;
    cclo.setMap(frontiers_map);

    

    // create a pose array of potential goal

    if (cclo.labels.size() == 0)
    {
        return;
    }
        
    set<int>::iterator it;
    map<int, geometry_msgs::Point> obs;

    // init
    geometry_msgs::Point offset;
    offset.x = fabs(gmap->info.origin.position.x);
    offset.y = fabs(gmap->info.origin.position.y);
    double resolution = gmap->info.resolution;
    for (it = cclo.labels.begin(); it != cclo.labels.end(); it++)
    {
        if (cclo.labels_tree[*it].cnt < min_frontier_size_px) continue;
        geometry_msgs::Point point;
        point.z = 0;
        point.x =  ((double)cclo.labels_tree[*it].mass_x/(double)cclo.labels_tree[*it].cnt)*resolution - offset.x;
        point.y = ((double)cclo.labels_tree[*it].mass_y/(double)cclo.labels_tree[*it].cnt)*resolution - offset.y;
        obs[*it] = point;
    }
    /*
    for (i = 0; i < cclo.dw; i++)
        for (j = 0; j < cclo.dh; j++)
        {
            idx = j * cclo.dw + i;
            cell = cclo.data[idx];
            if (cell != -1)
            {
                if (cclo.labels_tree[cell].cnt < min_frontier_size_px) continue;

                dist.x = i * resolution - offset.x;
                dist.y = j * resolution -  offset.y;
                dist.z = distance<geometry_msgs::Point,geometry_msgs::Point>(pose.position, dist);
                if (obs[cell].z == 0 || obs[cell].z > dist.z)
                {
                    obs[cell] = dist;
                }
            }
        }
        */
        map<int, geometry_msgs::Point>::iterator mit;
        geometry_msgs::PoseArray poses;
        poses.header.stamp = ros::Time::now();
        poses.header.frame_id = gmap->header.frame_id;
        for (mit = obs.begin(); mit != obs.end(); mit++)
        {
            geometry_msgs::Pose p;
            p.position.x = mit->second.x;
            p.position.y = mit->second.y;
            p.orientation.w = 1.0;
            poses.poses.push_back(p);
        }
        frontiers_pub.publish(poses);
        frontiers_map_pub.publish(frontiers_map);
    //ROS_WARN("published frontier map");
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    calcul_frontiers(msg);
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "frontier_allocator");
    ros::NodeHandle private_nh("~");
    listener = new tf::TransformListener();
    double rate;
    std::string map_topic, frontiers_map_topic, frontier_topic;
    private_nh.param<std::string>("map", map_topic, "/map");
    private_nh.param<std::string>("base_frame", base_frame, "base_link");
    private_nh.param<std::string>("map", map_topic, "/map");
    private_nh.param<std::string>("frontiers_map", frontiers_map_topic, "/frontiers_map");
    private_nh.param<std::string>("frontiers_topic", frontier_topic, "/frontiers");
    private_nh.param<int>("min_frontier_size_px", min_frontier_size_px, 30);
    private_nh.param<double>("publishing_rate", rate, 0.5);
    frontiers_map_pub = private_nh.advertise<nav_msgs::OccupancyGrid>(frontiers_map_topic, 1,true);
    frontiers_pub = private_nh.advertise<geometry_msgs::PoseArray>(frontier_topic, 1, true);
    ros::Subscriber sub_ph = private_nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, map_callback);
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        ros::spinOnce();
        //loop_rate.sleep();
    }
    return 0;
}
