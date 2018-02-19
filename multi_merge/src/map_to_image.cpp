
#include "ros/ros.h"
#include "stdio.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"


const std::string save_to = "/home/mrsang/mapimage";
int cnt;

cv::Mat mapToMat(const nav_msgs::OccupancyGrid *map)
{
    cv::Mat im(map->info.height, map->info.width, CV_8UC1);

    if (map->info.height*map->info.width != map->data.size())
    {
        ROS_ERROR("Data size doesn't match width*height: width = %d, height = %d, data size = %zu", map->info.width, map->info.height, map->data.size());
    }

    // transform the map in the same way the map_saver component does
    for (size_t i=0; i < map->info.height*map->info.width; i++)
    {
        if (map->data.at(i) == 0)
        {
            im.data[i] = 254;
        }
        else
        {
            if(map->data.at(i) == 100)
            {
                im.data[i] = 0;
            }
            else
            {
                im.data[i] = 205;
            }
        }
    }

    return im;
}

nav_msgs::OccupancyGrid* matToMap(const cv::Mat mat, nav_msgs::OccupancyGrid *forInfo)
{
    nav_msgs::OccupancyGrid* toReturn = forInfo;
    for(size_t i=2; i<toReturn->info.height * toReturn->info.width;i++)
    {
        //toReturn->data.push_back(mat.data[i]);
       if(mat.data[i]== 254)//KNOWN
           toReturn->data[i]=0;
        // here it is <10 and not 0 (like in mapToMat), becouse otherwise we loose much
        //walls etc. but so we get a little of wrong information in the unkown area, what is
        // not so terrible.
        else if(mat.data[i] > -1 && mat.data[i] < 50) toReturn->data[i]=100; //WALL
       else toReturn->data[i] = -1; //UNKOWN
    }
    return toReturn;
}


void callback_convert(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    cv::Mat image =  mapToMat(new nav_msgs::OccupancyGrid(*msg));
    std::stringstream ss;
    ss << save_to << "/" << "frame_" << cnt << ".jpg";
    ROS_INFO("saved %s", ss.str().c_str());
    cv::imwrite( ss.str(), image );
    cnt++;
  
}

int main(int argc, char** argv)
{
    cnt = 0;
    ros::init(argc, argv, "map_convert");
	ros::NodeHandle n("~");
	ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid>("/local_map", 50,&callback_convert);
    
    ros::spin();
}