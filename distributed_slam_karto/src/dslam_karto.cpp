#include "ros/ros.h"
#include "ros/console.h"
#include "dslam.h"

using namespace dslam;
DSlamKarto* slam = NULL;
void run()
{
    ros::Rate rate(25);
    while (ros::ok())
    {
        if(!slam) continue;
        slam->publishTf();
        rate.sleep();
    }
}
int main(int argc, char **argv)
{
    /*if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }*/
    ros::init(argc, argv, "dslam_karto");
    // execute config
    ros::NodeHandle nh_local("~");

    std::string config_file;
    nh_local.param<std::string>("conf_file", config_file, "config.lua");
    Configuration robot;
    // add robot configuration here
    Configuration config(config_file, robot);

    slam = new DSlamKarto();
    slam->configure(config, nh_local);

    //ros::Rate rate(25);
    boost::thread t1(&run);
    //boost::thread t2(&localmapping);
    //boost::thread t3(&publish_tranform);
    //t1.join();
    ros::spin();
    /*while (ros::ok())
    {
        ros::spinOnce();
        //publish_tranform();
        //if(estimated_poses_.poses.size() > 0)
        //rate.sleep();
    }*/

    return 0;
}
