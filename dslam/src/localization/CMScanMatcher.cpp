#include "CMScanMatcher.h"

namespace dslam
{
CMScanMatcher::CMScanMatcher()
{
    initialized_ = false;
    fixed_to_dom_.setIdentity();
    // unclear
}
CMScanMatcher::~CMScanMatcher()
{
    if (synchronizer_)
        delete synchronizer_;
    if (scan_sync_)
        delete scan_sync_;
    if (odom_sync_)
        delete odom_sync_;
}

void CMScanMatcher::configure(Configuration &config, ros::NodeHandle &nh)
{
    boost::mutex::scoped_lock(mutex_);

    base_frame_ = config.get<std::string>("robot_base", "base_link");
    fixed_frame_ = config.get<std::string>("global_frame", "world");
    scan_topic_ = config.get<std::string>("scan_topic", "/scan");
    odom_topic_ = config.get<std::string>("odom_topic", "/odom");

    kf_dist_linear_ = config.get<double>("keyframe_sample_linear", 0.1);
    kf_dist_angular_ = config.get<double>("keyframe_sample_angular", 10.0 * (M_PI / 180.0));
    kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;

    Configuration scnf = config.get<Configuration>("correlative_scan_matcher", Configuration());

    // resolution of the matching grid
    double kernel_resolution = (double)scnf.get<double>("kernel_resolution", 0.025);
    double kernel_radius = scnf.get<double>("kernel_radius", 0.2);
    int loop_closing_window = (int)scnf.get<double>("loop_closing_window",10);
    double max_matching_score = scnf.get<double>("max_matching_score", 0.15);
    double inlier_threshold = scnf.get<double>("inlier_threshold",2.0);
    int min_inlier = (int)scnf.get<double>("min_inlier", 7);
    max_optimization_step_ = (int)scnf.get<double>("max_optimization_step", 5);
    max_laser_range_ = scnf.get<double>("max_laser_range",10.0);
    // configuration stuff go here

    scan_sync_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, scan_topic_, 1);
    odom_sync_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, odom_topic_, 1);
    synchronizer_ = new message_filters::Synchronizer<ScanOdomSyncPolicy>(ScanOdomSyncPolicy(100), *scan_sync_, *odom_sync_);
    synchronizer_->registerCallback(boost::bind(&CMScanMatcher::scanOdomCallBack, this, _1, _2));

    
    slam_.setIdRobot(1);
    slam_.setBaseId(1000);
    slam_.init(kernel_resolution,kernel_radius,loop_closing_window,max_matching_score, inlier_threshold, min_inlier );
    //publisher

    trajectory_p_ = nh.advertise<geometry_msgs::PoseArray>("trajectory", 1);
    scan_cloud_p_ = nh.advertise<sensor_msgs::PointCloud>("lasermap", 1);
}

void CMScanMatcher::scanOdomCallBack(const sensor_msgs::LaserScanConstPtr &scan_msg, const nav_msgs::OdometryConstPtr &odom_msg)
{
    boost::mutex::scoped_lock(mutex_);
    SE2 odom;

    odom.setTranslation(Eigen::Vector2d(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y));
    odom.setRotation(Eigen::Rotation2Dd(tf::getYaw(odom_msg->pose.pose.orientation)));
    latest_odom_msg_ = odom;
    // if this is the first registration
    if (!initialized_)
    {
        try
        {
            tf::StampedTransform l2b;
            tf::TransformListener listener;
            listener.waitForTransform(base_frame_, scan_msg->header.frame_id,
                                      ros::Time::now(), ros::Duration(0.5));
            listener.lookupTransform(base_frame_, scan_msg->header.frame_id,
                                     ros::Time::now(), l2b);
            laser2base_ = SE2(l2b.getOrigin().x(),
                              l2b.getOrigin().y(),
                              tf::getYaw(l2b.getRotation()));

            //latest_odom_msg_ = odom;
            last_used_odom_msg_ = odom;
            current_estimated_ = odom;
            odom_frame_ = odom_msg->header.frame_id;
            RobotLaser *laser = convertScan(odom, scan_msg);
            slam_.setInitialData(current_estimated_,laser);
            ROS_DEBUG("First add scan");
            initialized_ = true;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("cg_mrslam: %s", ex.what());
            return;
        }
        // get the laser to base tf
    }
    else
    {
        SE2 odom_diff = last_used_odom_msg_.inverse() * latest_odom_msg_;
        current_estimated_ *= odom_diff;
        last_used_odom_msg_ = latest_odom_msg_;

        double dx = slam_.lastVertex()->estimate().translation().x() - current_estimated_.translation().x();
        double dy = slam_.lastVertex()->estimate().translation().y() - current_estimated_.translation().y();
        double sq_dist = dx*dx + dy*dy;
        if ((sq_dist > kf_dist_linear_sq_) ||
            (fabs(slam_.lastVertex()->estimate().rotation().angle() - current_estimated_.rotation().angle()) > kf_dist_angular_))
        {
            ROS_DEBUG("add data");
            // add new node and constrains
            RobotLaser *laser = convertScan(odom, scan_msg);
            slam_.addDataSM(current_estimated_, laser);
            slam_.findConstraints();
            slam_.optimize(max_optimization_step_);
            // find constrains
            current_estimated_ = slam_.lastVertex()->estimate();
            // calculate transformation
            SE2 _f2o = current_estimated_*last_used_odom_msg_.inverse();
            fixed_to_dom_.setOrigin(tf::Vector3(_f2o.translation().x(), _f2o.translation().y(), 0.0));
            tf::Quaternion q;
            q.setRPY(0.0, 0.0, _f2o.rotation().angle());
            fixed_to_dom_.setRotation(q);
        }
    }
}

RobotLaser *CMScanMatcher::convertScan(SE2 odom, const sensor_msgs::LaserScanConstPtr &scan_msg)
{
    boost::mutex::scoped_lock(mutex_);
    LaserParameters lparams(0, scan_msg->ranges.size(), scan_msg->angle_min, scan_msg->angle_increment, scan_msg->range_max, 0.1, 0);
    lparams.laserPose = laser2base_;

    RobotLaser *rlaser = new RobotLaser;
    rlaser->setLaserParams(lparams);
    rlaser->setOdomPose(odom);
    std::vector<double> ranges(scan_msg->ranges.size());
    for (size_t i = 0; i < scan_msg->ranges.size(); i++)
    {
        if(scan_msg->ranges[i] > max_laser_range_)
            ranges[i] = -1;
        else
            ranges[i] = scan_msg->ranges[i];
    }
    rlaser->setRanges(ranges);
    rlaser->setTimestamp(scan_msg->header.stamp.sec + scan_msg->header.stamp.nsec * pow(10, -9));
    rlaser->setLoggerTimestamp(rlaser->timestamp());
    rlaser->setHostname("hostname");
    return rlaser;
}

void CMScanMatcher::publishTf()
{
    if (!initialized_)
        return;
    //ROS_DEBUG("publishing tf %s -> %s", fixed_frame_.c_str(), odom_frame_.c_str());
    tf::StampedTransform transform_msg(fixed_to_dom_, ros::Time::now(), fixed_frame_, odom_frame_);
    tf_broadcaster_.sendTransform(transform_msg);
}

void CMScanMatcher::publishGraph()
{
    boost::mutex::scoped_lock(mutex_);
    if (!initialized_)
        return;

    geometry_msgs::PoseArray traj;
    sensor_msgs::PointCloud pcloud;
    traj.poses.resize(slam_.graph()->vertices().size());
    pcloud.points.clear();
    int i = 0;
    for (OptimizableGraph::VertexIDMap::iterator it = slam_.graph()->vertices().begin(); it != slam_.graph()->vertices().end(); ++it)
    {
        VertexSE2 *v = (VertexSE2 *)(it->second);
        traj.poses[i].position.x = v->estimate().translation().x();
        traj.poses[i].position.y = v->estimate().translation().y();
        traj.poses[i].position.z = 0;
        traj.poses[i].orientation = tf::createQuaternionMsgFromYaw(v->estimate().rotation().angle());

        RobotLaser *laser = dynamic_cast<RobotLaser *>(v->userData());
        if (laser)
        {
            RawLaser::Point2DVector vscan = laser->cartesian();
            SE2 trl = laser->laserParams().laserPose;
            SE2 transf = v->estimate() * trl;
            RawLaser::Point2DVector wscan;
            ScanMatcher::applyTransfToScan(transf, vscan, wscan);

            size_t s = 0;
            while (s < wscan.size())
            {
                geometry_msgs::Point32 point;
                point.x = wscan[s].x();
                point.y = wscan[s].y();
                pcloud.points.push_back(point);

                s = s + 10;
            }
        }
        i++;
    }

    traj.header.frame_id = fixed_frame_;
    traj.header.stamp = ros::Time::now();
    pcloud.header.frame_id = traj.header.frame_id;
    pcloud.header.stamp = ros::Time::now();
    //ROS_DEBUG("publishing graph");
    scan_cloud_p_.publish(pcloud);
    trajectory_p_.publish(traj);
}
} // namespace dslam