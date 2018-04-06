#include "ScanMatcher.h"

namespace dslam
{
void ScanMatcher::configure(Configuration &cnf)
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

    sample_dist_ = icpcnf.get<double>("sample_distance", 0.2);
    PCL_INFO("ICP: The sample distance %f\n", sample_dist_);

    global_frame_ = cnf.get<std::string>("global_frame", "/map");
    laser_frame_ = cnf.get<std::string>("laser_frame", "/laser_scan");
    PCL_INFO("Map frame is %s, and laser frame is %s\n", global_frame_.c_str(), laser_frame_.c_str());
    PCL_INFO("*************************************\n");

    //  icp.setRANSACOutlierRejectionThreshold (0.6);
}

void ScanMatcher::cacheData(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    std::vector<double> bearings, cos_bearings, sin_bearings;
    std::vector<unsigned int> indices;
    const std::size_t num_measurements = std::ceil(
        (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
    for (std::size_t i = 0; i < num_measurements; ++i)
    {
        const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
        bearings.push_back(b);
        cos_bearings.push_back(cos(b));
        sin_bearings.push_back(sin(b));
        indices.push_back(i);
    }

    line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
    //ROS_DEBUG("Data has been cached.");
}
void ScanMatcher::registerScan(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if(nscan_ == 0)
    {
        //printf("SCAN MATCHER: First scan \n");
        cacheData(scan_msg);
    }
    //printf("SCAN MATCHER: Register scan \n");
    std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
    line_extraction_.setRangeData(scan_ranges_doubles);
    nscan_++;
}
void ScanMatcher::match(icp_tf_t& mt, const void (*callback)(std::vector<Line>&, pcl::PointCloud<pcl::PointXYZ>& cloud))
{
    mt.converged = false;
    mt.fitness = 0.0;
    mt.tf = Eigen::Matrix4f::Identity();
    // extract the line
    std::vector<Line> lines;
    line_extraction_.extractLines(lines);
    if(lines.size() == 0) return;
    // convert lines to point cloud

    if(first_match_) // first registration
    {
        linesToPointCloud(lines, last_features_);
        //printf("SCAN MATCHER: Regist firts match\n");
        first_match_ = false;
        mt.converged = true;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ> new_scan;
    linesToPointCloud(lines, new_scan);
    icp.setInputSource (new_scan.makeShared());
    icp.setInputTarget (last_features_.makeShared());
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud);
    if(callback)
        callback(lines, aligned_cloud);
    mt.tf = icp.getFinalTransformation();
    mt.fitness = icp.getFitnessScore();
    if(icp.hasConverged() ) // && mt.fitness >= sample_dist_ 
    {
        mt.converged = true;
        last_features_ = new_scan;
        //printf("Converge: %d fitness:%f\n", icp.hasConverged (), icp.getFitnessScore () );
    }

}

void ScanMatcher::linesToPointCloud(std::vector<Line>& lines, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //cloud.width = lines.size()*3;
    cloud.height = 1;
    cloud.is_dense = true;
    //cloud.points.resize(cloud.width);
    int size = 0;
    for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
    {
        std::vector<pcl::PointXYZ> points;
        cit->asPointCloud(points);
        size += points.size();
        cloud.points.insert(cloud.points.end(), points.begin(), points.end());
    }
    cloud.width = cloud.points.size();
}
}