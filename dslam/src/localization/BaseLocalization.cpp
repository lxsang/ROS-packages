#include "ICPLocalization.h"

namespace dslam
{
BaseLocalization::BaseLocalization(){
    nscan_ = 0;
    first_match_ = true;
    tf_=nullptr;
    tf_ok_=false;
    last_features_.orientation= Eigen::Quaterniond::Identity();
    current_kf_.tf.rotation = Eigen::Quaterniond::Identity();
    current_kf_.diff.rotation = Eigen::Quaterniond::Identity();
    current_kf_.tf.translation = Eigen::Vector3d(0.0,0.0,0.0);
    current_kf_.diff.translation = Eigen::Vector3d(0.0,0.0,0.0);
    kf_idx_ = 0;
}
BaseLocalization::~BaseLocalization()
{
}
void BaseLocalization::extractLines(std::vector<Line> &lines_)
{
    std::vector<Line> lines;
    line_extraction_.extractLines(lines);
    boost::array<double, 2> org;
    org[0] = 0.0;
    org[1] = 0.0;
    for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
    {
        Line l(org, cit->getStart());
        if(l.length() > max_line_dist_)
            l = Line(org, cit->getEnd());
        if(l.length() < max_line_dist_)
            lines_.push_back(l);
        lines_.push_back(*cit);
    }
}
void BaseLocalization::configure(Configuration &cnf)
{
    Configuration xline_config = cnf.get<Configuration>("line_extraction", Configuration());
    double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh,
        max_line_gap, min_line_length, min_range, min_split_dist, outlier_dist;
    int min_line_points;

    publish_odom_ = cnf.get<bool>("publish_odom", false);
    odom_frame_ = cnf.get<std::string>("odom_frame", "/odom");
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
    max_line_dist_ = xline_config.get<double>("max_dist", 10.0);

    Configuration icpcnf = cnf.get<Configuration>("icp", Configuration());
    double max_dist, max_iter, epsilon, eufitness;
    bool use_nl = icpcnf.get<bool>("use_non_linear", true);
    PCL_INFO("ICP: User icp nonlinear %d\n", use_nl);

    if(use_nl)
        icp = &icp_nl_;
    else
        icp = &icp_ln_;

    max_dist = icpcnf.get<double>("max_distance", 0.05);
    PCL_INFO("ICP: max dist %f\n", max_dist);
    icp->setMaxCorrespondenceDistance(max_dist);

    max_iter = icpcnf.get<double>("max_iteration", 50);
    PCL_INFO("ICP: max iteration %f\n", max_iter);
    icp->setMaximumIterations((int)max_iter);

    epsilon = icpcnf.get<double>("tf_epsilon", 1e-6);
    PCL_INFO("ICP: tf_epsilon: %f\n", epsilon);
    icp->setTransformationEpsilon(epsilon);

    eufitness = icpcnf.get<double>("eu_fitness", 0.5);
    PCL_INFO("ICP: the euclidean distance difference epsilon %f\n", eufitness);
    icp->setEuclideanFitnessEpsilon(eufitness);

    sample_fitness_ = icpcnf.get<double>("sample_fitness", 0.2);
    PCL_INFO("ICP: The sample distance %f\n", sample_fitness_);

    //translation_tolerance_ = icpcnf.get<double>("translation_tolerance", 1e-6);

    global_frame_ = cnf.get<std::string>("global_frame", "/map");
    laser_frame_ = cnf.get<std::string>("laser_frame", "/laser_scan");
    robot_base_frame_ = cnf.get<std::string>("robot_base", "base_link");
    keyframe_sample_linear_ = cnf.get<double>("keyframe_sample_linear", 0.5);
    keyframe_sample_angular_ = cnf.get<double>("keyframe_sample_angular", 0.5);
    PCL_INFO("Map frame is %s, robot frame is %s, and laser frame is %s\n", global_frame_.c_str(), robot_base_frame_.c_str() ,laser_frame_.c_str());
    PCL_INFO("*************************************\n");

    //icp->setRANSACOutlierRejectionThreshold (0.06); // TODO
}
bool BaseLocalization::match(const void (*callback)(std::vector<Line>&, pcl::PointCloud<pcl::PointXYZ>&))
{
    bool v = __match(callback);
    if(publish_odom_) publishOdomTF();
    return v;
}
void BaseLocalization::cacheData(sensor_msgs::LaserScan &scan_msg)
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
void BaseLocalization::registerScan(sensor_msgs::LaserScan &scan_msg, nav_msgs::Odometry& odom)
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
    current_kf_.scan = scan_msg;
    if(tf_ok_)
    {
        current_kf_.base2laser.translation(0) = laser2base_.getOrigin().getX();
        current_kf_.base2laser.translation(1) = laser2base_.getOrigin().getY();
        current_kf_.base2laser.translation(2) = laser2base_.getOrigin().getZ();

        current_kf_.base2laser.rotation.x() = laser2base_.getRotation().x();
        current_kf_.base2laser.rotation.y() = laser2base_.getRotation().y();
        current_kf_.base2laser.rotation.z() = laser2base_.getRotation().z();
        current_kf_.base2laser.rotation.w() = laser2base_.getRotation().w();
    }
}
void BaseLocalization::publishOdomTF()
{
    if(!publish_odom_) return;
    updatePose();
    dslam_tf_t _tf;
    _tf.translation(0) = pose_.position.x + current_kf_.tf.translation(0);
    _tf.translation(1) = pose_.position.y + current_kf_.tf.translation(1);
    _tf.translation(2) = 0.0;

    _tf.rotation.x() = pose_.orientation.x;
    _tf.rotation.y() = pose_.orientation.y;
    _tf.rotation.z() = pose_.orientation.z;
    _tf.rotation.w() = pose_.orientation.w;
    
    _tf.rotation = _tf.rotation*current_kf_.tf.rotation;

    //std::cout << "Publishing Odom "  << std::endl;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(_tf.translation(0),_tf.translation(1),0.0));
    tf::Quaternion qad(_tf.rotation.x(), _tf.rotation.y(),_tf.rotation.z(),_tf.rotation.w());
    transform.setRotation(qad);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame_, robot_base_frame_));
}
void BaseLocalization::updatePose()
{
    if(keyframes.size() == 0) return;
    auto it = keyframes.begin();
    dslam_tf_t _tf;
    _tf.translation = Eigen::Vector3d(0.0,0.0,0.0);
    _tf.rotation = Eigen::Quaterniond::Identity();
    while(it != keyframes.end())
    {
        _tf.translation += it->tf.translation;
        _tf.rotation *= it->tf.rotation;
        //_tf.rotation.normalize();
        if(publish_odom_)
        {
            it->diff.translation = _tf.translation;
            it->diff.rotation = _tf.rotation;
        }
        it++;
    }
    pose_.position.x = _tf.translation(0);
    pose_.position.y = _tf.translation(1);
    pose_.position.z = _tf.translation(2);

    pose_.orientation.x = _tf.rotation.x();
    pose_.orientation.y = _tf.rotation.y();
    pose_.orientation.z = _tf.rotation.z();
    pose_.orientation.w = _tf.rotation.w();
}
void BaseLocalization::getTransform(tf::Transform& transform)
{
    
    updatePose();
    dslam_tf_t diff = keyframes.back().diff;

    transform.setOrigin(tf::Vector3(pose_.position.x - diff.translation(0),pose_.position.y-diff.translation(1),0.0));
    //tf::Quaternion qad(0.0, 0.0,0.0,1.0);
    Eigen::Quaterniond q ;
    q.x() = pose_.orientation.x;
    q.y() = pose_.orientation.y;
    q.z() = pose_.orientation.z;
    q.w() = pose_.orientation.w;
    q = q*diff.rotation.inverse();
    tf::Quaternion qad(q.x(), q.y(),q.z(),q.w());
    transform.setRotation(qad);
}
void BaseLocalization::getLastKnownPose(geometry_msgs::Pose& pose)
{
    pose = pose_;
}
void BaseLocalization::linesToPointCloud(std::vector<Line>& lines, pcl::PointCloud<pcl::PointXYZ>& cloud, tf::StampedTransform& laser2base)
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
        cit->asPointCloud(points, laser2base, (int)(cit->length()/line_scale_), max_line_dist_ );
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