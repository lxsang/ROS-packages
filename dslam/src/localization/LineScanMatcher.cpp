#include "ScanMatcher.h"

namespace dslam
{
LineScanMatcher::LineScanMatcher(){
    nscan_ = 0;
    first_match_ = true;
    tf_=nullptr;
    tf_ok_=false;
    last_features_.orientation= Eigen::Quaterniond::Identity();
    //yaw=0.0;
    //last_know_position_(0) = 0.0;
    //last_know_position_(1) = 0.0;
    //last_know_position_(2) = 0.0;
    current_kf_.tf.rotation = Eigen::Quaterniond::Identity();
    current_kf_.diff.rotation = Eigen::Quaterniond::Identity();
    current_kf_.tf.translation = Eigen::Vector3d(0.0,0.0,0.0);
    current_kf_.diff.translation = Eigen::Vector3d(0.0,0.0,0.0);
}
void LineScanMatcher::configure(Configuration &cnf)
{
    Configuration xline_config = cnf.get<Configuration>("line_extraction", Configuration());
    double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh,
        max_line_gap, min_line_length, min_range, min_split_dist, outlier_dist;
    int min_line_points;

    bearing_std_dev = xline_config.get<double>("bearing_std_dev", 1e-3);
    line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);
    PCL_INFO("bearing_std_dev: %f\n", bearing_std_dev);

    range_std_dev = xline_config.get<double>("range_std_dev", 0.02);
    line_extraction_.setRangeVariance(range_std_dev * range_std_dev);
    PCL_INFO("range_std_dev: %f\n", range_std_dev);

    least_sq_angle_thresh = xline_config.get<double>("least_sq_angle_thresh", 1e-4);
    line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
    PCL_INFO("least_sq_angle_thresh: %f\n", least_sq_angle_thresh);

    least_sq_radius_thresh = xline_config.get<double>("least_sq_radius_thresh", 1e-4);
    line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
    PCL_INFO("least_sq_radius_thresh: %f\n", least_sq_radius_thresh);

    max_line_gap = xline_config.get<double>("max_line_gap", 0.4);
    line_extraction_.setMaxLineGap(max_line_gap);
    PCL_INFO("max_line_gap: %f\n", max_line_gap);

    min_line_length = xline_config.get<double>("min_line_length", 0.5);
    line_extraction_.setMinLineLength(min_line_length);
    PCL_INFO("min_line_length: %f\n", min_line_length);

    min_range = xline_config.get<double>("min_range", 0.4);
    line_extraction_.setMinRange(min_range);
    PCL_INFO("min_range: %f\n", min_range);

    min_split_dist = xline_config.get<double>("min_split_dist", 0.05);
    line_extraction_.setMinSplitDist(min_split_dist);
    PCL_INFO("min_split_dist: %f\n", min_split_dist);

    outlier_dist = xline_config.get<double>("outlier_dist", 0.05);
    line_extraction_.setOutlierDist(outlier_dist);
    PCL_INFO("outlier_dist: %f\n", outlier_dist);

    min_line_points = (int)xline_config.get<double>("min_line_points", 9.0);
    line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
    PCL_INFO("min_line_points: %d\n", min_line_points);

    line_scale_ = xline_config.get<double>("line_scale", 0.05);

    Configuration icpcnf = cnf.get<Configuration>("icp", Configuration());
    double max_dist, max_iter, epsilon, eufitness;
    max_dist = icpcnf.get<double>("max_distance", 0.05);
    PCL_INFO("ICP: max dist %f\n", max_dist);
    icp.setMaxCorrespondenceDistance(max_dist);

    max_iter = icpcnf.get<double>("max_iteration", 50);
    PCL_INFO("ICP: max iteration %f\n", max_iter);
    icp.setMaximumIterations((int)max_iter);

    epsilon = icpcnf.get<double>("tf_epsilon", 1e-6);
    PCL_INFO("ICP: tf_epsilon: %f\n", epsilon);
    icp.setTransformationEpsilon(epsilon);

    eufitness = icpcnf.get<double>("eu_fitness", 0.5);
    PCL_INFO("ICP: the euclidean distance difference epsilon %f\n", eufitness);
    icp.setEuclideanFitnessEpsilon(eufitness);

    sample_fitness_ = icpcnf.get<double>("sample_fitness", 0.2);
    PCL_INFO("ICP: The sample distance %f\n", sample_fitness_);

    //translation_tolerance_ = icpcnf.get<double>("translation_tolerance", 1e-6);

    global_frame_ = cnf.get<std::string>("global_frame", "/map");
    laser_frame_ = cnf.get<std::string>("laser_frame", "/laser_scan");
    robot_base_frame_ = cnf.get<std::string>("robot_base", "base_link");
    //odom_frame_ = cnf.get<std::string>("odom_frame", "/odom");
    keyframe_sample_linear_ = cnf.get<double>("keyframe_sample_linear", 0.5);
    keyframe_sample_angular_ = cnf.get<double>("keyframe_sample_angular", 0.5);
    PCL_INFO("Map frame is %s, robot frame is %s, and laser frame is %s\n", global_frame_.c_str(), robot_base_frame_.c_str() ,laser_frame_.c_str());
    PCL_INFO("*************************************\n");

    //icp.setRANSACOutlierRejectionThreshold (0.06); // TODO
}

void LineScanMatcher::cacheData(sensor_msgs::LaserScan &scan_msg)
{
    std::vector<double> bearings, cos_bearings, sin_bearings;
    std::vector<unsigned int> indices;
    const std::size_t num_measurements = std::ceil(
        (scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);
    for (std::size_t i = 0; i < num_measurements; ++i)
    {
        const double b = scan_msg.angle_min + i * scan_msg.angle_increment;
        bearings.push_back(b);
        cos_bearings.push_back(cos(b));
        sin_bearings.push_back(sin(b));
        indices.push_back(i);
    }

    line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
    //ROS_DEBUG("Data has been cached.");
}
void LineScanMatcher::registerScan(sensor_msgs::LaserScan &scan_msg, nav_msgs::Odometry& odom)
{
    Eigen::Quaterniond q;
    q.x() = odom.pose.pose.orientation.x;
    q.y() = odom.pose.pose.orientation.y;
    q.z() = odom.pose.pose.orientation.z;
    q.w() = odom.pose.pose.orientation.w;
    current_feature_.orientation = q;
    if(nscan_ == 0)
    {
        //printf("SCAN MATCHER: First scan \n");
        cacheData(scan_msg);
        
    }
    nscan_++;
    current_feature_.position(0) = odom.pose.pose.position.x;
    current_feature_.position(1) = odom.pose.pose.position.y;
    current_feature_.position(2) = odom.pose.pose.position.z;
    std::vector<double> scan_ranges_doubles(scan_msg.ranges.begin(), scan_msg.ranges.end());
    line_extraction_.setRangeData(scan_ranges_doubles);
    
}

double LineScanMatcher::getYaw()
{
    auto euler = last_features_.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    auto euler1 = current_feature_.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    return (euler1[2] - euler[2]);
}
void LineScanMatcher::alignLastFeature(pcl::PointCloud<pcl::PointXYZ> &ret)
{
    Eigen::Matrix3f M;
    M = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())
        *Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY())
        *Eigen::AngleAxisf(-getYaw(), Eigen::Vector3f::UnitZ()); 
    ret.width = last_features_.cloud.width;
    ret.height = last_features_.cloud.height;
    ret.is_dense = last_features_.cloud.is_dense;
    Eigen::Vector3d offset = current_feature_.position - last_features_.position;
    ret.points.resize(ret.width);
    for(int i = 0; i < last_features_.cloud.width; i++)
    {
        Eigen::Vector3f v(last_features_.cloud.points[i].x, last_features_.cloud.points[i].y , last_features_.cloud.points[i].z);
        v = M*v;
        //Eigen::Vector3d rotatedV = rotatedP.vec();
        ret.points[i].x = v.x() - offset.x();
        ret.points[i].y = v.y() - offset.y();
        ret.points[i].z = 0.0;//v.z() - offset.z();
    }
}
void LineScanMatcher::match(const void (*callback)(std::vector<Line>&, pcl::PointCloud<pcl::PointXYZ>& cloud))
{
    //mt.converged = false;
    //mt.fitness = 0.0;
    //mt.rot = Eigen::Quaterniond::Identity();
    // extract the line
    try
    {
        if(!tf_ok_)
        {
            if(!tf_) return;
            tf_->lookupTransform(robot_base_frame_, laser_frame_, ros::Time(0), laser2base_);
            tf_ok_ = true;
        }
        
        std::vector<Line> lines;
        line_extraction_.extractLines(lines);
        if(lines.size() == 0) return;
        // convert lines to point cloud

        if(first_match_) // first registration
        {
            linesToPointCloud(lines, last_features_.cloud, laser2base_);
            last_features_.orientation = current_feature_.orientation;
            last_features_.position = current_feature_.position;
            //printf("SCAN MATCHER: Regist firts match\n");
            first_match_ = false;
            //mt.converged = true;
            return;
        }
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud, aligned_feature;
        linesToPointCloud(lines, current_feature_.cloud, laser2base_);
        icp.setInputSource (current_feature_.cloud.makeShared());
        alignLastFeature(aligned_feature);
        icp.setInputTarget (aligned_feature.makeShared());
        icp.align(aligned_cloud);
        if(callback)
            callback(lines, aligned_cloud);
        
        Eigen::Vector3d offset = current_feature_.position - last_features_.position;
        //offset(2) = 0.0;
        Eigen::Matrix4f _tf = icp.getFinalTransformation();
        //last_know_position_ += offset;
        current_kf_.tf.translation += offset;

        current_kf_.tf.rotation *= Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
        *Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
        *Eigen::AngleAxisd(getYaw(), Eigen::Vector3d::UnitZ());
        //current_kf_.tf.rotation.normalize();
        //mt.rot = current_feature_.orientation;
        //Eigen::Matrix3d M;
        //M = mt.rot;
        //mt.tl = M*mt.tl;
        last_features_ = current_feature_;
        //mt.fitness = icp.getFitnessScore();
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        if(icp.hasConverged() && icp.getFitnessScore() < sample_fitness_)
        {
            Eigen::Matrix3d rot;
            for(int i = 0; i < 3; i ++)
                for(int j = 0; j < 3; j ++)
                rot(i,j) = _tf(i,j);
            q = rot;
            //double y = rot.eulerAngles(0,1,2)[2]/2.0;
            //mt.rot = mt.rot*q;
            q.normalize();
            current_kf_.tf.rotation *= q;
            //current_kf_.tf.rotation.normalize();
            //mt.tl = _tf*mt.tl;
            current_kf_.tf.translation(0) += _tf(0,3);
            current_kf_.tf.translation(1) += _tf(1,3);
            current_kf_.tf.translation(2) += _tf(2,3);

            current_kf_.diff.translation(0) += _tf(0,3);
            current_kf_.diff.translation(1) += _tf(1,3);
            current_kf_.diff.translation(2) += _tf(2,3);
            current_kf_.diff.rotation *= q;
            //current_kf_.diff.rotation.normalize();
            //mt.converged = true;
            printf("Converge: %d fitness:%f\n", icp.hasConverged (), icp.getFitnessScore () );
        }
        //mt.position = last_know_position_;
        
        
        double dist = sqrt( pow(current_kf_.tf.translation(0), 2) + pow(current_kf_.tf.translation(1),2) );
        double yaw = fabs(current_kf_.tf.rotation.toRotationMatrix().eulerAngles(0, 1, 2)[2]);
        if(dist >= keyframe_sample_linear_)// || yaw >= keyframe_sample_angular_)
        {
            keyframes_.push_back(current_kf_);
            current_kf_.tf.rotation = Eigen::Quaterniond::Identity();
            current_kf_.diff.rotation = Eigen::Quaterniond::Identity();
            current_kf_.tf.translation = Eigen::Vector3d(0.0,0.0,0.0);
            current_kf_.diff.translation = Eigen::Vector3d(0.0,0.0,0.0);
        }
        //transform_.setOrigin( tf::Vector3(last_know_position_(0) - current_feature_.position(0), 
        //last_know_position_(1) - current_feature_.position(1), 
        //last_know_position_(2) - current_feature_.position(2)));
        //tf::Quaternion qad(q.x(), q.y(),q.z(),q.w());
        //q.setRPY(0, 0, msg->theta);
        //transform_.setRotation(qad);
        //tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),global_frame_, odom_frame_));
    }
    catch (tf::TransformException ex)
    {
        //PCL_ERROR("Scan matching error: %s\n", ex.what());
    }

}

void LineScanMatcher::getTransform(tf::Transform& transform)
{
    auto it = keyframes_.begin();
    dslam_tf_t _tf;
    _tf.translation = Eigen::Vector3d(0.0,0.0,0.0);
    _tf.rotation = Eigen::Quaterniond::Identity();
    while(it != keyframes_.end())
    {
        _tf.translation += it->diff.translation;
        _tf.rotation *= it->diff.rotation;
        //_tf.rotation.normalize();
        it++;
    }
    transform.setOrigin( tf::Vector3(_tf.translation(0),_tf.translation(1),0.0));
    //tf::Quaternion qad(0.0, 0.0,0.0,1.0);
    tf::Quaternion qad(_tf.rotation.x(), _tf.rotation.y(),_tf.rotation.z(),_tf.rotation.w());
    transform.setRotation(qad);
}
void LineScanMatcher::getLastKnowPose(geometry_msgs::Pose& pose)
{
    auto it = keyframes_.begin();
    dslam_tf_t _tf;
    _tf.translation = Eigen::Vector3d(0.0,0.0,0.0);
    _tf.rotation = Eigen::Quaterniond::Identity();
    while(it != keyframes_.end())
    {
        _tf.translation += it->tf.translation;
        _tf.rotation *= it->tf.rotation;
        //_tf.rotation.normalize();
        it++;
    }
    pose.position.x = _tf.translation(0);
    pose.position.y = _tf.translation(1);
    pose.position.z = _tf.translation(2);

    pose.orientation.x = _tf.rotation.x();
    pose.orientation.y = _tf.rotation.y();
    pose.orientation.z = _tf.rotation.z();
    pose.orientation.w = _tf.rotation.w();
}
void LineScanMatcher::linesToPointCloud(std::vector<Line>& lines, pcl::PointCloud<pcl::PointXYZ>& cloud, tf::StampedTransform& laser2base)
{
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //cloud.width = lines.size()*3;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(0);
    int size = 0;
    //printf("Line scale is %d\n", line_scale_);
    for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
    {
        std::vector<pcl::PointXYZ> points;
        cit->asPointCloud(points, laser2base, (int)(cit->length()/line_scale_));
        size += points.size();
        cloud.points.insert(cloud.points.end(), points.begin(), points.end());
    }
    pcl::PointXYZ ori;
    ori.x = 0.0;
    ori.y = 0.0;
    cloud.points.push_back(ori);
    cloud.width = cloud.points.size();
}
}