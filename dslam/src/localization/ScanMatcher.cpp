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
    printf("bearing_std_dev: %f\n", bearing_std_dev);

    range_std_dev = xline_config.get<double>("range_std_dev", 0.02);
    line_extraction_.setRangeVariance(range_std_dev * range_std_dev);
    printf("range_std_dev: %f\n", range_std_dev);

    least_sq_angle_thresh = xline_config.get<double>("least_sq_angle_thresh", 1e-4);
    line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
    printf("least_sq_angle_thresh: %f\n", least_sq_angle_thresh);

    least_sq_radius_thresh = xline_config.get<double>("least_sq_radius_thresh", 1e-4);
    line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
    printf("least_sq_radius_thresh: %f\n", least_sq_radius_thresh);

    max_line_gap = xline_config.get<double>("max_line_gap", 0.4);
    line_extraction_.setMaxLineGap(max_line_gap);
    printf("max_line_gap: %f\n", max_line_gap);

    min_line_length = xline_config.get<double>("min_line_length", 0.5);
    line_extraction_.setMinLineLength(min_line_length);
    printf("min_line_length: %f\n", min_line_length);

    min_range = xline_config.get<double>("min_range", 0.4);
    line_extraction_.setMinRange(min_range);
    printf("min_range: %f\n", min_range);

    min_split_dist = xline_config.get<double>("min_split_dist", 0.05);
    line_extraction_.setMinSplitDist(min_split_dist);
    printf("min_split_dist: %f\n", min_split_dist);

    outlier_dist = xline_config.get<double>("outlier_dist", 0.05);
    line_extraction_.setOutlierDist(outlier_dist);
    printf("outlier_dist: %f\n", outlier_dist);

    min_line_points = (int)xline_config.get<double>("min_line_points", 9.0);
    line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
    printf("min_line_points: %d\n", min_line_points);

    Configuration icpcnf = cnf.get<Configuration>("icp", Configuration());
    double max_dist, max_iter, epsilon, eufitness;
    max_dist = icpcnf.get<double>("max_distance", 0.05);
    printf("ICP: max dist %f\n", max_dist);
    icp.setMaxCorrespondenceDistance(max_dist);

    max_iter = icpcnf.get<double>("max_iteration", 50);
    printf("ICP: max iteration %f\n", max_iter);
    icp.setMaximumIterations((int)max_iter);

    epsilon = icpcnf.get<double>("tf_epsilon", 1e-6);
    printf("ICP: tf_epsilon: %f\n", epsilon);
    icp.setTransformationEpsilon(epsilon);

    eufitness = icpcnf.get<double>("eu_fitness", 0.5);
    printf("ICP: the euclidean distance difference epsilon %f\n", eufitness);
    icp.setEuclideanFitnessEpsilon(eufitness);
    printf("*************************************\n");

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
Eigen::Matrix4f ScanMatcher::match(const void (*callback)(std::vector<Line>&))
{
    Eigen::Matrix4f tf;
    // extract the line
    std::vector<Line> lines;
    line_extraction_.extractLines(lines);
    if(lines.size() == 0) return tf;
    // convert lines to point cloud
    if(callback)
        callback(lines);
    if(first_match_) // first registration
    {
        last_features_ = linesToPointCloud(lines);
        //printf("SCAN MATCHER: Regist firts match\n");
        first_match_ = false;
        return tf;
    }
    pcl::PointCloud<pcl::PointXYZ> new_scan = linesToPointCloud(lines);
    icp.setInputCloud (new_scan.makeShared());
    icp.setInputTarget (last_features_.makeShared());
    pcl::PointCloud<pcl::PointXYZ> new_scan_registered;
    icp.align(new_scan_registered);
    last_features_ = new_scan;
    return icp.getFinalTransformation();
}

pcl::PointCloud<pcl::PointXYZ> ScanMatcher::linesToPointCloud(std::vector<Line>& lines)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = lines.size()*3;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(cloud.width);
    int i = 0;
    for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
    {
        boost::array<double, 2> point = cit->getCenter();

        cloud.points[i].x = cit->getStart()[0];
        cloud.points[i].x = cit->getStart()[1];
        cloud.points[i].z = 0.0;

        cloud.points[i+1].x = point[0];
        cloud.points[i+1].x = point[1];
        cloud.points[i+1].z = 0.0;

        cloud.points[i+2].x = cit->getEnd()[0];
        cloud.points[i+2].x = cit->getEnd()[1];
        cloud.points[i+2].z = 0.0;
        i+=3;
    }
    return cloud;
}
}