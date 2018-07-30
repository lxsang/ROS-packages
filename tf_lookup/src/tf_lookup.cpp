#include "ros/ros.h"
//#include "tf2_msgs/TFMessage.h"
#include <tf/transform_listener.h>
#include "tf_lookup/tfLookup.h"

tf::TransformListener *listener;

bool lookup(tf_lookup::tfLookup::Request &rq,
            tf_lookup::tfLookup::Response &res)
{
    if (!listener)
        return false;

    tf::StampedTransform transform;
    try
    {
        listener->waitForTransform(rq.to, rq.from, ros::Time(0), ros::Duration(3.0) );
        listener->lookupTransform(rq.to, rq.from,ros::Time(0), transform);
        res.transform.header.stamp = ros::Time::now();
        res.transform.header.frame_id = rq.from;
        res.transform.child_frame_id = rq.to;
        res.transform.transform.translation.x = transform.getOrigin().x();
        res.transform.transform.translation.y = transform.getOrigin().y();
        res.transform.transform.translation.z = transform.getOrigin().z();
        res.transform.transform.rotation.x = transform.getRotation().getX();
        res.transform.transform.rotation.y = transform.getRotation().getY();
        res.transform.transform.rotation.z = transform.getRotation().getZ();
        res.transform.transform.rotation.w = transform.getRotation().getW();
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        //ros::Duration(1.0).sleep();
        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_lookup_server");
    ros::NodeHandle n;
    listener = new tf::TransformListener();
    ros::ServiceServer service = n.advertiseService("tf_lookup", lookup);
    ROS_INFO("Tf_lookup is on.");
    ros::spin();

    return 0;
}