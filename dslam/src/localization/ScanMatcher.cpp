#include "ScanMatcher.h"

namespace dslam
{
ScanMatcher::ScanMatcher()
{
    // **** state variables

    f2b_.setIdentity();
    f2b_kf_.setIdentity();
    input_.laser[0] = 0.0;
    input_.laser[1] = 0.0;
    input_.laser[2] = 0.0;

    // Initialize output_ vectors as Null for error-checking
    output_.cov_x_m = 0;
    output_.dx_dy1_m = 0;
    output_.dx_dy2_m = 0;

    current_kf_.index = 0;
    current_kf_.f2b.setIdentity();
    current_kf_.odom.setIdentity();
    current_kf_.b2l.setIdentity();
}
void ScanMatcher::configure(Configuration &config, ros::NodeHandle &nh)
{
    Configuration scnf = config.get<Configuration>("scan_matcher", Configuration());

    // parameters
    base_frame_ = config.get<std::string>("robot_base", "base_link");
    fixed_frame_ = config.get<std::string>("global_frame", "world");
    imu_topic_ = config.get<std::string>("imu_topic", "/imu");
    scan_topic_ = config.get<std::string>("scan_topic", "/scan");
    odom_topic_ = config.get<std::string>("odom_topic", "/odom");
    // **** keyframe params: when to generate the keyframe scan
    // if either is set to 0, reduces to frame-to-frame matching
    kf_dist_linear_ = config.get<double>("keyframe_sample_linear", 0.1);
    kf_dist_angular_ = config.get<double>("keyframe_sample_angular", 10.0 * (M_PI / 180.0));
    kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;

    // **** What predictions are available to speed up the ICP?
    // 1) imu - [theta] from imu yaw angle - /imu topic
    // 2) odom - [x, y, theta] from wheel odometry - /odom topic
    // If more than one is enabled, priority is imu > odom

    confidence_factor_ = scnf.get<double>("confidence_factor",3.0);
    max_laser_range_ = scnf.get<double>("max_laser_range", 10.0);
    use_imu_ = scnf.get<bool>("use_imu", true);
    use_odom_ = scnf.get<bool>("use_odom", true);

    position_covariance_.resize(3);
    std::fill(position_covariance_.begin(), position_covariance_.end(), 1e-9);

    orientation_covariance_.resize(3);
    std::fill(orientation_covariance_.begin(), orientation_covariance_.end(), 1e-9);

    // **** CSM parameters - comments copied from algos.h (by Andrea Censi)

    // Maximum angular displacement between scans
    input_.max_angular_correction_deg = scnf.get<double>("max_angular_correction_deg", 45.0);

    // Maximum translation between scans (m)
    input_.max_linear_correction = scnf.get<double>("max_linear_correction", 0.50);

    // Maximum ICP cycle iterations
    input_.max_iterations = scnf.get<double>("max_iterations", 10);

    // A threshold for stopping (m)
    input_.epsilon_xy = scnf.get<double>("epsilon_xy", 0.000001);

    // A threshold for stopping (rad)
    input_.epsilon_theta = scnf.get<double>("epsilon_theta", 0.000001);

    // Maximum distance for a correspondence to be valid
    input_.max_correspondence_dist = scnf.get<double>("max_correspondence_dist", 0.3);

    // Noise in the scan (m)
    input_.sigma = scnf.get<double>("sigma", 0.010);

    // Use smart tricks for finding correspondences.
    input_.use_corr_tricks = scnf.get<bool>("use_corr_tricks", true) ? 1 : 0;

    // Restart: Restart if error is over threshold
    input_.restart = scnf.get<bool>("restart", false) ? 1 : 0;

    // Restart: Threshold for restarting
    input_.restart_threshold_mean_error = scnf.get<double>("restart_threshold_mean_error", 0.01);

    // Restart: displacement for restarting. (m)
    input_.restart_dt = scnf.get<double>("restart_dt", 1.0);

    // Restart: displacement for restarting. (rad)
    input_.restart_dtheta = scnf.get<double>("restart_dtheta", 0.1);

    // Max distance for staying in the same clustering
    input_.clustering_threshold = scnf.get<double>("clustering_threshold", 0.25);

    // Number of neighbour rays used to estimate the orientation
    input_.orientation_neighbourhood = (int)scnf.get<double>("orientation_neighbourhood", 20);

    // If 0, it's vanilla ICP
    input_.use_point_to_line_distance = scnf.get<bool>("use_point_to_line_distance", true) ? 1 : 0;

    // Discard correspondences based on the angles
    input_.do_alpha_test = scnf.get<bool>("do_alpha_test", false) ? 1 : 0;

    // Discard correspondences based on the angles - threshold angle, in degrees
    input_.do_alpha_test_thresholdDeg = scnf.get<double>("do_alpha_test_thresholdDeg", 20.0);

    // Percentage of correspondences to consider: if 0.9,
    // always discard the top 10% of correspondences with more error
    input_.outliers_maxPerc = scnf.get<double>("outliers_maxPerc", 0.90);

    // Parameters describing a simple adaptive algorithm for discarding.
    //  1) Order the errors.
    //  2) Choose the percentile according to outliers_adaptive_order.
    //     (if it is 0.7, get the 70% percentile)
    //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
    //     with the value of the error at the chosen percentile.
    //  4) Discard correspondences over the threshold.
    //  This is useful to be conservative; yet remove the biggest errors.
    input_.outliers_adaptive_order = scnf.get<double>("outliers_adaptive_order", 0.7);
    input_.outliers_adaptive_mult = scnf.get<double>("outliers_adaptive_mult", 2.0);

    // If you already have a guess of the solution, you can compute the polar angle
    // of the points of one scan in the new position. If the polar angle is not a monotone
    // function of the readings index, it means that the surface is not visible in the
    // next position. If it is not visible, then we don't use it for matching.
    input_.do_visibility_test = scnf.get<bool>("do_visibility_test", false) ? 1 : 0;

    // no two points in laser_sens can have the same corr.
    input_.outliers_remove_doubles = scnf.get<bool>("outliers_remove_doubles", true)?1:0;

    // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
    input_.do_compute_covariance = scnf.get<bool>("do_compute_covariance", false) ? 1 : 0;

    // Checks that find_correspondences_tricks gives the right answer
    input_.debug_verify_tricks = scnf.get<bool>("debug_verify_tricks", false) ? 1 : 0;

    // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
    // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
    input_.use_ml_weights = scnf.get<bool>("use_ml_weights", false) ? 1 : 0;

    // If 1, the field 'readings_sigma' in the second scan is used to weight the
    // correspondence by 1/sigma^2
    input_.use_sigma_weights = scnf.get<bool>("use_sigma_weights", false) ? 1 : 0;

    // turn off the gls error handler that may cause the node to quit
    gsl_set_error_handler_off();
    // subscribers

    if (use_imu_)
        imu_subscriber_ = nh.subscribe(imu_topic_, 1, &ScanMatcher::imuCallback, this);

    if (use_odom_)
    {
        scan_sync_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, scan_topic_, 1);
        odom_sync_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, odom_topic_, 1);
        synchronizer_ = new message_filters::Synchronizer<ScanOdomSyncPolicy>(ScanOdomSyncPolicy(1), *scan_sync_, *odom_sync_);
        synchronizer_->registerCallback(boost::bind(&ScanMatcher::scanOdomCallBack, this, _1, _2));
    }
    else
    {
        scan_subscriber_ = nh.subscribe(scan_topic_, 1, &ScanMatcher::scanCallback, this);
    }
}
ScanMatcher::~ScanMatcher()
{
    if(synchronizer_) delete synchronizer_;
    if(scan_sync_) delete scan_sync_;
    if(odom_sync_) delete odom_sync_;
}

void ScanMatcher::imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    boost::mutex::scoped_lock(mutex_);
    latest_imu_msg_ = *imu_msg;
    if (!received_imu_)
    {
        last_used_imu_msg_ = *imu_msg;
        received_imu_ = true;
    }
}

void ScanMatcher::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    boost::mutex::scoped_lock(mutex_);
    latest_odom_msg_ = *odom_msg;
    if (!received_odom_)
    {
        last_used_odom_msg_ = *odom_msg;
        received_odom_ = true;
    }
}
void ScanMatcher::scanOdomCallBack(const sensor_msgs::LaserScanConstPtr &scan_msg, const nav_msgs::OdometryConstPtr &odom_msg)
{
    boost::mutex::scoped_lock(mutex_);
    latest_odom_msg_ = *odom_msg;
    if (!received_odom_)
    {
        last_used_odom_msg_ = *odom_msg;
        received_odom_ = true;
    }
    scanCallback(scan_msg);
}
void ScanMatcher::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // **** if first scan, cache the tf from base to the scanner
    boost::mutex::scoped_lock(mutex_);
    if (!initialized_)
    {
        createCache(scan_msg); // caches the sin and cos of all angles

        // cache the static tf from base to laser
        if (!getBaseToLaserTf(scan_msg->header.frame_id))
        {
            ROS_WARN("Skipping scan");
            return;
        }

        laserScanToLDP(scan_msg, prev_ldp_scan_);
        //last_icp_time_ = scan_msg->header.stamp;
        initialized_ = true;
    }
    current_kf_.scan = *scan_msg;
    current_kf_.b2l = base_to_laser_;
    LDP curr_ldp_scan;
    laserScanToLDP(scan_msg, curr_ldp_scan);
    processScan(curr_ldp_scan, scan_msg->header.stamp);
}

void ScanMatcher::processScan(LDP &curr_ldp_scan, const ros::Time &time)
{
    boost::mutex::scoped_lock(mutex_);
    ros::WallTime start = ros::WallTime::now();

    // CSM is used in the following way:
    // The scans are always in the laser frame
    // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
    // The new scan (currLDPScan) has a pose equal to the movement
    // of the laser in the laser frame since the last scan
    // The computed correction is then propagated using the tf machinery

    prev_ldp_scan_->odometry[0] = 0.0;
    prev_ldp_scan_->odometry[1] = 0.0;
    prev_ldp_scan_->odometry[2] = 0.0;

    prev_ldp_scan_->estimate[0] = 0.0;
    prev_ldp_scan_->estimate[1] = 0.0;
    prev_ldp_scan_->estimate[2] = 0.0;

    prev_ldp_scan_->true_pose[0] = 0.0;
    prev_ldp_scan_->true_pose[1] = 0.0;
    prev_ldp_scan_->true_pose[2] = 0.0;

    input_.laser_ref = prev_ldp_scan_;
    input_.laser_sens = curr_ldp_scan;

    // **** estimated change since last scan

    double pr_ch_x, pr_ch_y, pr_ch_a;
    getPrediction(pr_ch_x, pr_ch_y, pr_ch_a);

    // the predicted change of the laser's position, in the fixed frame

    tf::Transform pr_ch;
    createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, pr_ch);

    // account for the change since the last kf, in the fixed frame

    pr_ch = pr_ch * (f2b_ * f2b_kf_.inverse());

    // the predicted change of the laser's position, in the laser frame

    tf::Transform pr_ch_l;
    pr_ch_l = laser_to_base_ * f2b_.inverse() * pr_ch * f2b_ * base_to_laser_;

    input_.first_guess[0] = pr_ch_l.getOrigin().getX();
    input_.first_guess[1] = pr_ch_l.getOrigin().getY();
    input_.first_guess[2] = tf::getYaw(pr_ch_l.getRotation());

    // If they are non-Null, free covariance gsl matrices to avoid leaking memory
    if (output_.cov_x_m)
    {
        gsl_matrix_free(output_.cov_x_m);
        output_.cov_x_m = 0;
    }
    if (output_.dx_dy1_m)
    {
        gsl_matrix_free(output_.dx_dy1_m);
        output_.dx_dy1_m = 0;
    }
    if (output_.dx_dy2_m)
    {
        gsl_matrix_free(output_.dx_dy2_m);
        output_.dx_dy2_m = 0;
    }

    // *** scan match - using point to line icp from CSM

    sm_icp(&input_, &output_);
    tf::Transform corr_ch;
    //f2b_ = f2b_kf_*pr_ch;
    if (output_.valid)
    {
        // the correction of the laser's position, in the laser frame
        tf::Transform corr_ch_l;
        createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);
        corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;
        // the correction of the base's position, in the base frame
        // update the pose in the world frame
        f2b_ = f2b_kf_ * corr_ch;

        double dx = f2b_.getOrigin().getX() - f2b_kf_.getOrigin().getX();
        double dy = f2b_.getOrigin().getY() - f2b_kf_.getOrigin().getY();
        double corr_len = sqrt(pow(dx,2) +pow(dy, 2));
        double odom_len = sqrt(pr_ch_x*pr_ch_x + pr_ch_y*pr_ch_y);
        double score = corr_len/odom_len;
        score = score < 0.5?odom_len/corr_len:score;
        double dir = dx*pr_ch_x + dy*pr_ch_y;
        
        
        if(use_odom_ && (score > confidence_factor_ || dir < 0))
        {
            ROS_WARN("Odom len %f - estimate len %f: score %f\n", odom_len, corr_len, corr_len/odom_len );
            createTfFromXYTheta(f2b_kf_.getOrigin().getX() + pr_ch_x,
            f2b_kf_.getOrigin().getY() + pr_ch_y,
            tf::getYaw(f2b_kf_.getRotation()) + pr_ch_a ,f2b_);
        }
        // **** publish

        /*
        if (publish_pose_with_covariance_)
        {
            // unstamped PoseWithCovariance message
            geometry_msgs::PoseWithCovariance::Ptr pose_with_covariance_msg;
            pose_with_covariance_msg = boost::make_shared<geometry_msgs::PoseWithCovariance>();
            tf::poseTFToMsg(f2b_, pose_with_covariance_msg->pose);

            if (input_.do_compute_covariance)
            {
                pose_with_covariance_msg->covariance = boost::assign::list_of(gsl_matrix_get(output_.cov_x_m, 0, 0))(0)(0)(0)(0)(0)(0)(gsl_matrix_get(output_.cov_x_m, 0, 1))(0)(0)(0)(0)(0)(0)(static_cast<double>(position_covariance_[2]))(0)(0)(0)(0)(0)(0)(static_cast<double>(orientation_covariance_[0]))(0)(0)(0)(0)(0)(0)(static_cast<double>(orientation_covariance_[1]))(0)(0)(0)(0)(0)(0)(gsl_matrix_get(output_.cov_x_m, 0, 2));
            }
            else
            {
                pose_with_covariance_msg->covariance = boost::assign::list_of(static_cast<double>(position_covariance_[0]))(0)(0)(0)(0)(0)(0)(static_cast<double>(position_covariance_[1]))(0)(0)(0)(0)(0)(0)(static_cast<double>(position_covariance_[2]))(0)(0)(0)(0)(0)(0)(static_cast<double>(orientation_covariance_[0]))(0)(0)(0)(0)(0)(0)(static_cast<double>(orientation_covariance_[1]))(0)(0)(0)(0)(0)(0)(static_cast<double>(orientation_covariance_[2]));
            }

            pose_with_covariance_publisher_.publish(pose_with_covariance_msg);
        }
        if (publish_pose_with_covariance_stamped_)
        {
            // stamped Pose message
            geometry_msgs::PoseWithCovarianceStamped::Ptr pose_with_covariance_stamped_msg;
            pose_with_covariance_stamped_msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

            pose_with_covariance_stamped_msg->header.stamp = time;
            pose_with_covariance_stamped_msg->header.frame_id = fixed_frame_;

            tf::poseTFToMsg(f2b_, pose_with_covariance_stamped_msg->pose.pose);

            if (input_.do_compute_covariance)
            {
                pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of(gsl_matrix_get(output_.cov_x_m, 0, 0))(0)(0)(0)(0)(0)(0)(gsl_matrix_get(output_.cov_x_m, 0, 1))(0)(0)(0)(0)(0)(0)(static_cast<double>(position_covariance_[2]))(0)(0)(0)(0)(0)(0)(static_cast<double>(orientation_covariance_[0]))(0)(0)(0)(0)(0)(0)(static_cast<double>(orientation_covariance_[1]))(0)(0)(0)(0)(0)(0)(gsl_matrix_get(output_.cov_x_m, 0, 2));
            }
            else
            {
                pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of(static_cast<double>(position_covariance_[0]))(0)(0)(0)(0)(0)(0)(static_cast<double>(position_covariance_[1]))(0)(0)(0)(0)(0)(0)(static_cast<double>(position_covariance_[2]))(0)(0)(0)(0)(0)(0)(static_cast<double>(orientation_covariance_[0]))(0)(0)(0)(0)(0)(0)(static_cast<double>(orientation_covariance_[1]))(0)(0)(0)(0)(0)(0)(static_cast<double>(orientation_covariance_[2]));
            }

            pose_with_covariance_stamped_publisher_.publish(pose_with_covariance_stamped_msg);
        }
        */
    }
    else
    {
        corr_ch.setIdentity();
        // fall back to odom
        createTfFromXYTheta(f2b_kf_.getOrigin().getX() + pr_ch_x,
            f2b_kf_.getOrigin().getY() + pr_ch_y,
            tf::getYaw(f2b_kf_.getRotation()) + pr_ch_a ,f2b_);
        ROS_WARN("Error in scan matching");
    }
     // publish tf
    if(use_odom_)
    {
        tf::Quaternion q1(
            latest_odom_msg_.pose.pose.orientation.x,
            latest_odom_msg_.pose.pose.orientation.y,
            latest_odom_msg_.pose.pose.orientation.z,
            latest_odom_msg_.pose.pose.orientation.w);
        createTfFromXYTheta(latest_odom_msg_.pose.pose.position.x,latest_odom_msg_.pose.pose.position.y,
            tf::getYaw(q1), current_kf_.odom );
    }
    
    // select key frame
    tf::Transform diff = f2b_ *current_kf_.f2b.inverse();
    double dist = 
        pow(f2b_.getOrigin().getX() - current_kf_.f2b.getOrigin().getX(), 2)
        + 
        pow(f2b_.getOrigin().getY() - current_kf_.f2b.getOrigin().getY(),2);
    double yaw = fabs(
        tf::getYaw(current_kf_.f2b.getRotation())
        -
        tf::getYaw( f2b_.getRotation() )
    );
    
    //ROS_DEBUG("kf_dist: %f %f\n", dist, yaw);

    if(keyframes.empty() || dist >= kf_dist_linear_sq_ || yaw >= kf_dist_angular_)
    {
        current_kf_.f2b = f2b_;
        keyframes.push_back(current_kf_);
        current_kf_.index++;
    }
   
    // **** swap old and new
    if(prev_ldp_scan_)
        ld_free(prev_ldp_scan_);
    prev_ldp_scan_ = curr_ldp_scan;
    f2b_kf_ = f2b_;
    // now let see when we recorded the keyframe
    
    /*if (newKeyframeNeeded(corr_ch))
    {
        // generate a keyframe
        ld_free(prev_ldp_scan_);
        prev_ldp_scan_ = curr_ldp_scan;
        f2b_kf_ = f2b_;
    }
    else
    {
        ld_free(curr_ldp_scan);
    }

    
    */
    // **** statistics
    //last_icp_time_ = time;
    double dur = (ros::WallTime::now() - start).toSec() * 1e3;
    ROS_DEBUG("Scan matcher total duration: %.1f ms", dur);
}

void ScanMatcher::publishTf()
{
    if(keyframes.empty()) return;

    if(use_odom_)
    {
        auto it = keyframes.back();
        tf::Transform f2o;
        f2o = it.f2b*it.odom.inverse();
        tf::StampedTransform transform_msg(f2o, ros::Time::now(), fixed_frame_, latest_odom_msg_.header.frame_id );
        tf_broadcaster_.sendTransform(transform_msg);
    }
    else
    {
        tf::StampedTransform transform_msg(f2b_, ros::Time::now(), fixed_frame_, base_frame_ );
        tf_broadcaster_.sendTransform(transform_msg);
    }
}
void ScanMatcher::getLastKnownPose(geometry_msgs::Pose& pose)
{
    pose.position.x =  f2b_.getOrigin().getX();
    pose.position.y =  f2b_.getOrigin().getY();
    pose.position.z = 0.0;

    pose.orientation.x = f2b_.getRotation().x();
    pose.orientation.y = f2b_.getRotation().y();
    pose.orientation.z = f2b_.getRotation().z();
    pose.orientation.w = f2b_.getRotation().w();
}

void ScanMatcher::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg,LDP &ldp)
{
    unsigned int n = scan_msg->ranges.size();
    ldp = ld_alloc_new(n);
    //ROS_WARN("max laser rang is %f\n", max_laser_range_);
    for (unsigned int i = 0; i < n; i++)
    {
        // calculate position in laser frame

        double r = scan_msg->ranges[i];

        if (r > scan_msg->range_min && r < max_laser_range_) //scan_msg->range_max
        {
            // fill in laser scan data

            ldp->valid[i] = 1;
            ldp->readings[i] = r;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1; // for invalid range
        }

        ldp->theta[i] = scan_msg->angle_min + i * scan_msg->angle_increment;

        ldp->cluster[i] = -1;
    }

    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[n - 1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}

void ScanMatcher::createCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    a_cos_.clear();
    a_sin_.clear();

    for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
    {
        double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        a_cos_.push_back(cos(angle));
        a_sin_.push_back(sin(angle));
    }

    input_.min_reading = scan_msg->range_min;
    input_.max_reading = scan_msg->range_max;
}

bool ScanMatcher::getBaseToLaserTf(const std::string &frame_id)
{
    ros::Time t = ros::Time::now();

    tf::StampedTransform base_to_laser_tf;
    try
    {
        tf_listener_.waitForTransform(
            base_frame_, frame_id, t, ros::Duration(1.0));
        tf_listener_.lookupTransform(
            base_frame_, frame_id, t, base_to_laser_tf);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
        return false;
    }
    base_to_laser_ = base_to_laser_tf;
    laser_to_base_ = base_to_laser_.inverse();

    return true;
}

// returns the predicted change in pose (in fixed frame)
// since the last time we did icp
void ScanMatcher::getPrediction(double &pr_ch_x, double &pr_ch_y,
                                double &pr_ch_a)
{
    boost::mutex::scoped_lock(mutex_);

    // **** base case - no input available, use zero-motion model
    pr_ch_x = 0.0;
    pr_ch_y = 0.0;
    pr_ch_a = 0.0;


    // **** use odometry
    if (use_odom_ && received_odom_)
    {
        pr_ch_x = latest_odom_msg_.pose.pose.position.x -
                  last_used_odom_msg_.pose.pose.position.x;

        pr_ch_y = latest_odom_msg_.pose.pose.position.y -
                  last_used_odom_msg_.pose.pose.position.y;

        tf::Quaternion q(
            last_used_odom_msg_.pose.pose.orientation.x,
            last_used_odom_msg_.pose.pose.orientation.y,
            last_used_odom_msg_.pose.pose.orientation.z,
            last_used_odom_msg_.pose.pose.orientation.w);
        tf::Quaternion q1(
            latest_odom_msg_.pose.pose.orientation.x,
            latest_odom_msg_.pose.pose.orientation.y,
            latest_odom_msg_.pose.pose.orientation.z,
            latest_odom_msg_.pose.pose.orientation.w);
        pr_ch_a = tf::getYaw(q1*q.inverse()) ;

        /*if (pr_ch_a >= M_PI)
            pr_ch_a -= 2.0 * M_PI;
        else if (pr_ch_a < -M_PI)
            pr_ch_a += 2.0 * M_PI;
        */
        last_used_odom_msg_ = latest_odom_msg_;
    }

    // **** use imu
    if (use_imu_ && received_imu_)
    {
        pr_ch_a = tf::getYaw(latest_imu_msg_.orientation) -
                tf::getYaw(last_used_imu_msg_.orientation);

        if (pr_ch_a >= M_PI)
            pr_ch_a -= 2.0 * M_PI;
        else if (pr_ch_a < -M_PI)
            pr_ch_a += 2.0 * M_PI;

        last_used_imu_msg_ = latest_imu_msg_;
    }
}

void ScanMatcher::createTfFromXYTheta(
    double x, double y, double theta, tf::Transform &t)
{
    t.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    t.setRotation(q);
}
}


/*
typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void LaserScanMatcher::PointCloudToLDP(const PointCloudT::ConstPtr& cloud,
                                             LDP& ldp)
{
  double max_d2 = cloud_res_ * cloud_res_;

  PointCloudT cloud_f;

  cloud_f.points.push_back(cloud->points[0]);

  for (unsigned int i = 1; i < cloud->points.size(); ++i)
  {
    const PointT& pa = cloud_f.points[cloud_f.points.size() - 1];
    const PointT& pb = cloud->points[i];

    double dx = pa.x - pb.x;
    double dy = pa.y - pb.y;
    double d2 = dx*dx + dy*dy;

    if (d2 > max_d2)
    {
      cloud_f.points.push_back(pb);
    }
  }

  unsigned int n = cloud_f.points.size();

  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame
    if (is_nan(cloud_f.points[i].x) || is_nan(cloud_f.points[i].y))
    {
      ROS_WARN("Laser Scan Matcher: Cloud input contains NaN values. \
                Please use a filtered cloud input.");
    }
    else
    {
      double r = sqrt(cloud_f.points[i].x * cloud_f.points[i].x +
                      cloud_f.points[i].y * cloud_f.points[i].y);

      if (r > cloud_range_min_ && r < cloud_range_max_)
      {
        ldp->valid[i] = 1;
        ldp->readings[i] = r;
      }
      else
      {
        ldp->valid[i] = 0;
        ldp->readings[i] = -1;  // for invalid range
      }
    }

    ldp->theta[i] = atan2(cloud_f.points[i].y, cloud_f.points[i].x);
    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void LaserScanMatcher::cloudCallback (const PointCloudT::ConstPtr& cloud)
{
  // **** if first scan, cache the tf from base to the scanner

  std_msgs::Header cloud_header = pcl_conversions::fromPCL(cloud->header);

  if (!initialized_)
  {
    // cache the static tf from base to laser
    if (!getBaseToLaserTf(cloud_header.frame_id))
    {
      ROS_WARN("Skipping scan");
      return;
    }

    PointCloudToLDP(cloud, prev_ldp_scan_);
    last_icp_time_ = cloud_header.stamp;
    initialized_ = true;
  }

  LDP curr_ldp_scan;
  PointCloudToLDP(cloud, curr_ldp_scan);
  processScan(curr_ldp_scan, cloud_header.stamp);
}

*/