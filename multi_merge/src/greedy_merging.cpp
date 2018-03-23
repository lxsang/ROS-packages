/**
Copyright (c) 2017 Xuan Sang LE <xsang.le@gmail.com>

This code is based on a greedy merging implementation of Zhi Yan 
https://github.com/yzrobot/map_merging

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
#include <ros/ros.h>
#include "multi_master_bridge/MapData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"

// open CV header
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

#define UNKNOWN -1
std::string other_map_,my_map_,merged_map_topic,map_update_;
bool furious_merge;
nav_msgs::OccupancyGrid global_map;
nav_msgs::OccupancyGridPtr local_map;
geometry_msgs::Pose my_pose;
ros::Publisher  global_map_pub;
ros::Publisher  map_update_pub;
ros::Publisher test_map_pub;
std::map<std::string,  multi_master_bridge::MapData> pipeline;
double map_width_m_, map_height_m_, map_resolution_, merged_map_inlate_;

void dilate_map (int k);
void erode_map (int k);

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
       //if(mat.data[i]== 254)//KNOWN
        //   toReturn->data[i]=0;
        // here it is <10 and not 0 (like in mapToMat), becouse otherwise we loose much
        //walls etc. but so we get a little of wrong information in the unkown area, what is
        // not so terrible.
        //else if(mat.data[i] > -1 && mat.data[i] < 50) toReturn->data[i]=100; //WALL
       //else toReturn->data[i] = -1; //UNKOWN
       if(mat.data[i]== 0)
            toReturn->data[i]=100;
        else if(toReturn->data[i] != -1)
            toReturn->data[i]=0;


    }
    return toReturn;
}


void getRelativePose(geometry_msgs::Pose p, geometry_msgs::Pose q, geometry_msgs::Pose &delta, double resolution) {
  
    delta.position.x = round((p.position.x - q.position.x) / resolution);
    delta.position.y = round((p.position.y - q.position.y) / resolution);
    delta.position.z = round((p.position.z - q.position.z) / resolution);
}


void greedyMerging(int delta_x, int delta_y, int x, int y, const nav_msgs::OccupancyGrid their_map, bool furious) {
  for(int i = 0; i < (int)their_map.info.width; i++) {
    for(int j = 0; j < (int)their_map.info.height; j++) {
        if(i+delta_x + x  >= 0 && i+delta_x + x  < (int)global_map.info.width &&
	        j+delta_y + y  >= 0 && j+delta_y + y  < (int)global_map.info.height) {
            int mycell = i + delta_x + x +(j + delta_y + y)*(int)global_map.info.width;
            int theircell = i + j*(int)their_map.info.width;
            if((int)global_map.data[mycell] == UNKNOWN || ( furious && (int)their_map.data[theircell] != UNKNOWN && (int)global_map.data[mycell] != (int)their_map.data[theircell] ))
            {
                //ROS_INFO("merging...");
            global_map.data[mycell] = their_map.data[theircell];
            }
        }
    }
  }
}
void mege_pipeline(bool furious)
{
    geometry_msgs::Pose delta;
    if ( !local_map ) {
        ROS_INFO("Local map not found, wait for it");
        return;
    }
    // fill global map with local content
    global_map.data.resize(global_map.info.width*global_map.info.height, -1);
    std::fill(global_map.data.begin(), global_map.data.end(), -1);

    std::map<std::string,  multi_master_bridge::MapData>::iterator it;
    geometry_msgs::Pose offset;
    getRelativePose(local_map->info.origin, global_map.info.origin, offset, local_map->info.resolution);
    //offset.position.x = abs(offset.position.x);
    //offset.position.y = abs(offset.position.y);

    
    for(int i= 0;i < local_map->info.width ; i++)
        for(int j = 0; j < local_map->info.height; j++)
            if(local_map->data[i + j*local_map->info.width] != -1)
                global_map.data[(offset.position.y +  j)*global_map.info.width + offset.position.x + i] = local_map->data[i + j*local_map->info.width];
        

    //global_map.reset(new nav_msgs::OccupancyGrid(*local_map));
    for ( it = pipeline.begin(); it != pipeline.end(); it++ )
    {
        geometry_msgs::Pose p;
        p.position = it->second.position;
        ROS_INFO("mergin map of %s with init pose (%f,%f,%f)",it->first.c_str(), it->second.position.x,it->second.position.y,it->second.position.z);
        ROS_INFO("Get relative position");
        getRelativePose(p,my_pose, delta,global_map.info.resolution);
        ROS_INFO("Get map offset");
        getRelativePose(it->second.map.info.origin,global_map.info.origin,offset,global_map.info.resolution);
        //offset.position.x = abs(offset.position.x);
        //offset.position.y = abs(offset.position.y);
        ROS_INFO("merging");
        greedyMerging(round(delta.position.x), round(delta.position.y),offset.position.x,offset.position.y, it->second.map,furious);

    }
    ros::Time now = ros::Time::now();
    global_map.info.map_load_time = now;
    global_map.header.stamp = now;
    
    // now we process the merged map using openCV
    // first let inflate it
    dilate_map(merged_map_inlate_);
    test_map_pub.publish(global_map);
    // then convert it to Mat
    Mat img = mapToMat(&global_map);
    // extract the map skeleton
    //cvtColor( img, img, CV_BGR2GRAY );
    threshold(img,img, 0, 255, THRESH_BINARY_INV);
    Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    Mat temp;
    Mat eroded;
    Mat element = getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    bool done;		
    do
    {
        erode(img, eroded, element);
        dilate(eroded, temp, element); // 
        subtract(img, temp, temp);
        bitwise_or(skel, temp, skel);
        eroded.copyTo(img);

        done = (cv::countNonZero(img) == 0);
    } while (!done);

    threshold(skel,skel, 127, 255, THRESH_BINARY_INV);
    // convert the skel back to grid map
    matToMap(skel, & global_map);    
    dilate_map(2);
    ROS_INFO("Publishing global map");
    global_map_pub.publish(global_map);
}

void resolve_mapsize(geometry_msgs::Point theirpose,const nav_msgs::OccupancyGrid msg)
{
    double minx, miny, maxx, maxy;
    double tx, ty, lx, ly;
    geometry_msgs::Point  delta;

    delta.x = theirpose.x - my_pose.position.x;
    delta.y = theirpose.y - my_pose.position.y;
    tx = msg.info.origin.position.x + delta.x;
    ty = msg.info.origin.position.y + delta.y;

    minx =  global_map.info.origin.position.x < tx ?  global_map.info.origin.position.x: tx;
    miny = global_map.info.origin.position.y < ty ? global_map.info.origin.position.y : ty;

    tx = (double)msg.info.width*msg.info.resolution - fabs(msg.info.origin.position.x);
    tx = tx + delta.x;

    ty = (double)msg.info.height*msg.info.resolution - fabs(msg.info.origin.position.y);
    ty = ty + delta.y;

    lx = (double)global_map.info.width*global_map.info.resolution - fabs(global_map.info.origin.position.x);
    ly = (double)global_map.info.height*global_map.info.resolution - fabs(global_map.info.origin.position.y);

    maxx = lx > tx ? lx : tx;
    maxy = ly > ty ? ly : ty;

    global_map.info.width = round( (double)(maxx - minx)/global_map.info.resolution ) ;
    global_map.info.height = round( (double)(maxy - miny)/global_map.info.resolution ) ;
    global_map.info.origin.position.x = minx;
    global_map.info.origin.position.y = miny;
}

void dilate_map(int k)
{
    //nav_msgs::OccupancyGridPtr mhmap;
    //mhmap.reset(new nav_msgs::OccupancyGrid(global_map));
    int matrix[global_map.info.height*global_map.info.width];
    // traverse from top left to bottom right
    for (int i=0; i< global_map.info.height; i++){
        for (int j=0; j< global_map.info.width; j++){
            if (global_map.data[j+i* global_map.info.width] == 100){
                // first pass and pixel was on, it gets a zero
                matrix[j+i* global_map.info.width] = 0;
            }else {
                // pixel was off
                // It is at most the sum of the lengths of the array
                // away from a pixel that is on
                matrix[j+i* global_map.info.width] = global_map.info.width +  global_map.info.height;
                // or one more than the pixel to the north
                if (i>0) 
                {
                    int minv = matrix[j+i* global_map.info.width] > matrix[j+(i-1)* global_map.info.width] + 1 ? matrix[j+(i-1)* global_map.info.width] + 1: matrix[j+i* global_map.info.width];
                    matrix[j+i* global_map.info.width] = minv;
                }
                // or one more than the pixel to the west
                if (j>0) 
                {
                    int minv = matrix[j+i* global_map.info.width] > matrix[(j-1)+i* global_map.info.width] + 1 ? matrix[(j-1)+i* global_map.info.width] + 1: matrix[j+i* global_map.info.width];
                    matrix[j+i* global_map.info.width] = minv;
                }
            }
        }
    }
    // traverse from bottom right to top left
    for (int i=global_map.info.height - 1; i>=0; i--){
        for (int j=global_map.info.width-1; j>=0; j--){
            // either what we had on the first pass
            // or one more than the pixel to the south
            if (i+1< global_map.info.height)
            {
                int minv = matrix[j+i* global_map.info.width] > matrix[j+(i+1)* global_map.info.width] + 1 ? matrix[j+(i+1)* global_map.info.width] + 1: matrix[j+i* global_map.info.width];
                matrix[j+i* global_map.info.width] = minv;
            } 
            //image[i][j] = Math.min(image[i][j], image[i+1][j]+1);
            // or one more than the pixel to the east
            if (j+1< global_map.info.width)
            {
                int minv = matrix[j+i* global_map.info.width] > matrix[(j+1)+i* global_map.info.width] + 1 ? matrix[(j+1)+i* global_map.info.width] + 1: matrix[j+i* global_map.info.width];
                matrix[j+i* global_map.info.width] = minv;
            }
            //image[i][j] = Math.min(image[i][j], image[i][j+1]+1);
        }
    }
    
    for(int i = 0; i < global_map.info.height*global_map.info.width; i++)
    {
        /*if(i % global_map.info.width == 0)
            printf("\n");
        printf("%d ", global_map.data[i]);*/
        if(matrix[i] < k)
        {
            global_map.data[i] = 100;
        }
        else if(global_map.data[i] == -1)
            global_map.data[i] = -1;
        else
            global_map.data[i] = 0;
        
    }
}

void erode_map(int k)
{
   
    int matrix[global_map.info.height*global_map.info.width];
    // traverse from top left to bottom right
    for (int i=0; i< global_map.info.height; i++){
        for (int j=0; j< global_map.info.width; j++){
            if (global_map.data[j+i* global_map.info.width] == 0){
                // first pass and pixel was on, it gets a zero
                matrix[j+i* global_map.info.width] = 0;
            }else {
                // pixel was off
                // It is at most the sum of the lengths of the array
                // away from a pixel that is on
                matrix[j+i* global_map.info.width] = global_map.info.width +  global_map.info.height;
                // or one more than the pixel to the north
                if (i>0) 
                {
                    int minv = matrix[j+i* global_map.info.width] > matrix[j+(i-1)* global_map.info.width] + 1 ? matrix[j+(i-1)* global_map.info.width] + 1: matrix[j+i* global_map.info.width];
                    matrix[j+i* global_map.info.width] = minv;
                }
                // or one more than the pixel to the west
                if (j>0) 
                {
                    int minv = matrix[j+i* global_map.info.width] > matrix[(j-1)+i* global_map.info.width] + 1 ? matrix[(j-1)+i* global_map.info.width] + 1: matrix[j+i* global_map.info.width];
                    matrix[j+i* global_map.info.width] = minv;
                }
            }
        }
    }
    // traverse from bottom right to top left
    for (int i=global_map.info.height - 1; i>=0; i--){
        for (int j=global_map.info.width-1; j>=0; j--){
            // either what we had on the first pass
            // or one more than the pixel to the south
            if (i+1< global_map.info.height)
            {
                int minv = matrix[j+i* global_map.info.width] > matrix[j+(i+1)* global_map.info.width] + 1 ? matrix[j+(i+1)* global_map.info.width] + 1: matrix[j+i* global_map.info.width];
                matrix[j+i* global_map.info.width] = minv;
            } 
            //image[i][j] = Math.min(image[i][j], image[i+1][j]+1);
            // or one more than the pixel to the east
            if (j+1< global_map.info.width)
            {
                int minv = matrix[j+i* global_map.info.width] > matrix[(j+1)+i* global_map.info.width] + 1 ? matrix[(j+1)+i* global_map.info.width] + 1: matrix[j+i* global_map.info.width];
                matrix[j+i* global_map.info.width] = minv;
            }
            //image[i][j] = Math.min(image[i][j], image[i][j+1]+1);
        }
    }
    
    for(int i = 0; i < global_map.info.height*global_map.info.width; i++)
    {
        /*if(i % global_map.info.width == 0)
            printf("\n");
        printf("%d ", global_map.data[i]);*/
        if(matrix[i] < k)
        {
            global_map.data[i] = 0;
        }
        else if(global_map.data[i] == -1)
            global_map.data[i] = -1;
        else
            global_map.data[i] = 100;
        
    }
}

void register_local_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("Local map found");
    multi_master_bridge::MapData visibledata;
    visibledata.position = my_pose.position;
    visibledata.x = 0;
    visibledata.y = 0;
    visibledata.map = *msg;
    resolve_mapsize(visibledata.position, *msg);
    map_update_pub.publish(visibledata);
    local_map.reset(new nav_msgs::OccupancyGrid(*msg));
}

void register_neighbor_map(const multi_master_bridge::MapData::ConstPtr& msg)
{
    ROS_INFO("Registering neighbor map");
    resolve_mapsize(msg->position, msg->map);
    pipeline[msg->ip] = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "greedy_merging");
	ros::NodeHandle n("~");
    
    n.param<std::string>("other_map",other_map_, "/other_map");
    n.param<std::string>("my_map",my_map_, "/map");
    n.param<std::string>("map_update_topic",map_update_, "/map_update");
    n.param<std::string>("merged_map_topic",merged_map_topic, "/global_map");
    n.param<double>("merged_map_dilation",merged_map_inlate_, 4.0);
    n.param<bool>("furious_merge",furious_merge, false);
    n.param<double>("init_z",my_pose.position.z, 0.0);
	n.param<double>("init_x",my_pose.position.x, 0.0);
	n.param<double>("init_y",my_pose.position.y, 0.0);
    n.param<double>("map_resolution",map_resolution_, 0.05);
   
    global_map.info.width = 0;
    global_map.info.height = 0;
    global_map.info.resolution = map_resolution_;
    global_map.info.origin.position.x =0.0;
    global_map.info.origin.position.y = 0.0;
    global_map.info.origin.position.z = 0.0;
    global_map.info.origin.orientation.x = 0.0;
    global_map.info.origin.orientation.y = 0.0;
    global_map.info.origin.orientation.z = 0.0;
    global_map.info.origin.orientation.w = 1.0;
    // subscribe to this map
    ROS_INFO("My initial position is [%f,%f,%f]\n",my_pose.position.x,my_pose.position.y, my_pose.position.z);
    ROS_INFO("My Map topic is %s",my_map_.c_str());
    ROS_INFO("Other map topic is %s", other_map_.c_str());
    ROS_INFO("Global map topic is %s", merged_map_topic.c_str());
    ROS_INFO("Furios merge is  %d", furious_merge );
    ros::Subscriber sub = n.subscribe<multi_master_bridge::MapData>(other_map_, 50,&register_neighbor_map);
	ros::Subscriber sub1 = n.subscribe<nav_msgs::OccupancyGrid>(my_map_, 50,&register_local_map);
    local_map = nullptr;
    // publisher register
    global_map_pub = n.advertise<nav_msgs::OccupancyGrid>(merged_map_topic, 50, true);
    map_update_pub = n.advertise<multi_master_bridge::MapData>(map_update_, 50, true);
    test_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/processed_map", 50, true);

    ros::Rate r(1);
    while(ros::ok())
    {
        ros::spinOnce();
        //pb_mege_pipeline();
        mege_pipeline(furious_merge);
        r.sleep();
    }
}