#include "PFLocalization.h"

namespace dslam
{
using namespace Eigen;
PFLocalization::PFLocalization()
{
    nscan_ = 0;
    first_match_ = true;
    tf_ = nullptr;
    tf_ok_ = false;
    last_features_.orientation = Quaterniond::Identity();
    //yaw=0.0;
    //last_know_position_(0) = 0.0;
    //last_know_position_(1) = 0.0;
    //last_know_position_(2) = 0.0;
    current_kf_.tf.rotation = Quaterniond::Identity();
    current_kf_.diff.rotation = Quaterniond::Identity();
    current_kf_.tf.translation = Vector3d(0.0, 0.0, 0.0);
    current_kf_.diff.translation = Vector3d(0.0, 0.0, 0.0);
}
PFLocalization::~PFLocalization()
{
    if (pf_filter_)
        delete pf_filter_;
    if (sys_model_)
        delete sys_model_;
    if (meas_model_)
        delete meas_model_;
    if(sys_pdf_)
        delete sys_pdf_;
    if(meas_pdf_)
        delete meas_pdf_;
    if(prior_discr_)
        delete prior_discr_;
}
void PFLocalization::configure(Configuration &cnf)
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
    PCL_INFO("Map frame is %s, robot frame is %s, and laser frame is %s\n", global_frame_.c_str(), robot_base_frame_.c_str(), laser_frame_.c_str());
    PCL_INFO("*************************************\n");

    //icp.setRANSACOutlierRejectionThreshold (0.06); // TODO

    Configuration pf_conf = cnf.get<Configuration>("pf", Configuration());

    double MU_SYSTEM_NOISE_X = pf_conf.get<double>("MU_SYSTEM_NOISE_X", 0.0);
    double MU_SYSTEM_NOISE_Y = pf_conf.get<double>("MU_SYSTEM_NOISE_Y", 0.0);
    double MU_SYSTEM_NOISE_THETA = pf_conf.get<double>("MU_SYSTEM_NOISE_THETA", 0.0);
    double SIGMA_SYSTEM_NOISE_X = pf_conf.get<double>("SIGMA_SYSTEM_NOISE_X", 4e-3);
    double SIGMA_SYSTEM_NOISE_Y = pf_conf.get<double>("SIGMA_SYSTEM_NOISE_Y", 4e-3);
    double SIGMA_SYSTEM_NOISE_THETA = pf_conf.get<double>("SIGMA_SYSTEM_NOISE_THETA", 1e-3);

    double MU_MEAS_NOISE_X = pf_conf.get<double>("MU_MEAS_NOISE_X", 0.0);
    double MU_MEAS_NOISE_Y = pf_conf.get<double>("MU_MEAS_NOISE_Y", 0.0);
    double MU_MEAS_NOISE_THETA = pf_conf.get<double>("MU_MEAS_NOISE_THETA", 0.0);
    double SIGMA_MEAS_NOISE_X = pf_conf.get<double>("SIGMA_MEAS_NOISE_X", 2e-4);
    double SIGMA_MEAS_NOISE_Y = pf_conf.get<double>("SIGMA_MEAS_NOISE_Y", 2e-4);
    double SIGMA_MEAS_NOISE_THETA = pf_conf.get<double>("SIGMA_MEAS_NOISE_THETA", 2e-4);

    // Initial estimate of position and orientation
    double PRIOR_MU_X = pf_conf.get<double>("PRIOR_MU_X", -1e-2);
    double PRIOR_MU_Y = pf_conf.get<double>("PRIOR_MU_Y", 1e-2);
    double PRIOR_MU_THETA = pf_conf.get<double>("PRIOR_MU_THETA", M_PI/120);	//M_PI/4
    // Initial covariances of position and orientation
    double PRIOR_COV_X = pf_conf.get<double>("PRIOR_COV_X", pow(2e-4,2));
    double PRIOR_COV_Y = pf_conf.get<double>("PRIOR_COV_Y", pow(2e-4,2));
    double PRIOR_COV_THETA = pf_conf.get<double>("PRIOR_COV_THETA", pow(M_PI/360.0,2) );
    int NUM_SAMPLES  = (int)pf_conf.get<double>("NUM_SAMPLES",150);
    // particles filter
    /****************************
   * NonLinear system model      *
   ***************************/

    // create gaussian
    ColumnVector sys_noise_Mu(3);
    sys_noise_Mu(1) = MU_SYSTEM_NOISE_X;
    sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y;
    sys_noise_Mu(3) = MU_SYSTEM_NOISE_THETA;

    SymmetricMatrix sys_noise_Cov(3);
    sys_noise_Cov = 0.0;
    sys_noise_Cov(1, 1) = SIGMA_SYSTEM_NOISE_X;
    sys_noise_Cov(1, 2) = 0.0;
    sys_noise_Cov(1, 3) = 0.0;
    sys_noise_Cov(2, 1) = 0.0;
    sys_noise_Cov(2, 2) = SIGMA_SYSTEM_NOISE_Y;
    sys_noise_Cov(2, 3) = 0.0;
    sys_noise_Cov(3, 1) = 0.0;
    sys_noise_Cov(3, 2) = 0.0;
    sys_noise_Cov(3, 3) = SIGMA_SYSTEM_NOISE_THETA;

    Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

    // create the nonlinear system model
    sys_pdf_ = new NonLinearSystemPDF(system_Uncertainty);
    sys_model_ = new SystemModel<ColumnVector>(sys_pdf_);

    // create gaussian
    ColumnVector meas_noise_Mu(3);
    meas_noise_Mu(1) = MU_MEAS_NOISE_X;
    meas_noise_Mu(2) = MU_MEAS_NOISE_Y;
    meas_noise_Mu(3) = MU_MEAS_NOISE_THETA;

    SymmetricMatrix meas_noise_Cov(3);
    meas_noise_Cov = 0.0;
    meas_noise_Cov(1, 1) = SIGMA_MEAS_NOISE_X;
    meas_noise_Cov(1, 2) = 0.0;
    meas_noise_Cov(1, 3) = 0.0;
    meas_noise_Cov(2, 1) = 0.0;
    meas_noise_Cov(2, 2) = SIGMA_MEAS_NOISE_Y;
    meas_noise_Cov(2, 3) = 0.0;
    meas_noise_Cov(3, 1) = 0.0;
    meas_noise_Cov(3, 2) = 0.0;
    meas_noise_Cov(3, 3) = SIGMA_MEAS_NOISE_THETA;

    Gaussian meas_Uncertainty(meas_noise_Mu, meas_noise_Cov);
    meas_pdf_ = new NonLinearMeasurementPDF(meas_Uncertainty);
    meas_model_ = new MeasurementModel<ColumnVector, ColumnVector>(meas_pdf_);

    // Continuous Gaussian prior (for Kalman filters)
    ColumnVector prior_Mu(3);
    prior_Mu(1) = PRIOR_MU_X;
    prior_Mu(2) = PRIOR_MU_Y;
    prior_Mu(3) = PRIOR_MU_THETA;
    SymmetricMatrix prior_Cov(3);
    prior_Cov(1, 1) = PRIOR_COV_X;
    prior_Cov(1, 2) = 0.0;
    prior_Cov(1, 3) = 0.0;
    prior_Cov(2, 1) = 0.0;
    prior_Cov(2, 2) = PRIOR_COV_Y;
    prior_Cov(2, 3) = 0.0;
    prior_Cov(3, 1) = 0.0;
    prior_Cov(3, 2) = 0.0;
    prior_Cov(3, 3) = PRIOR_COV_THETA;
    Gaussian prior_cont(prior_Mu, prior_Cov);

    // Discrete prior for Particle filter (using the continuous Gaussian prior)
    vector<Sample<ColumnVector>> prior_samples(NUM_SAMPLES);
    prior_discr_ = new MCPdf<ColumnVector>(NUM_SAMPLES, 3);
    prior_cont.SampleFrom(prior_samples, NUM_SAMPLES, CHOLESKY, NULL);
    prior_discr_->ListOfSamplesSet(prior_samples);

    pf_filter_ = new BootstrapFilter<ColumnVector, ColumnVector>(prior_discr_, 0, NUM_SAMPLES / 4.0);
}

void PFLocalization::cacheData(sensor_msgs::LaserScan &scan_msg)
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
void PFLocalization::registerScan(sensor_msgs::LaserScan &scan_msg, nav_msgs::Odometry &odom)
{
    Quaterniond q;
    q.x() = odom.pose.pose.orientation.x;
    q.y() = odom.pose.pose.orientation.y;
    q.z() = odom.pose.pose.orientation.z;
    q.w() = odom.pose.pose.orientation.w;
    current_feature_.orientation = q;
    if (nscan_ == 0)
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
}

void PFLocalization::alignLastFeature(pcl::PointCloud<pcl::PointXYZ> &ret)
{
    Matrix3d M;
    M = last_features_.orientation * current_feature_.orientation.inverse();
    ret.width = last_features_.cloud.width;
    ret.height = last_features_.cloud.height;
    ret.is_dense = last_features_.cloud.is_dense;
    Vector3d offset = current_feature_.position - last_features_.position;
    ret.points.resize(ret.width);
    for (int i = 0; i < last_features_.cloud.width; i++)
    {
        Vector3d v(last_features_.cloud.points[i].x, last_features_.cloud.points[i].y, last_features_.cloud.points[i].z);
        v = M * v;
        //Vector3d rotatedV = rotatedP.vec();
        ret.points[i].x = v.x() - offset.x();
        ret.points[i].y = v.y() - offset.y();
        ret.points[i].z = 0.0; //v.z() - offset.z();
    }
}
bool PFLocalization::match(const void (*callback)(std::vector<Line> &, pcl::PointCloud<pcl::PointXYZ> &cloud))
{
    try
    {
        if (!tf_ok_)
        {
            if (!tf_)
                return false;
            tf_->lookupTransform(robot_base_frame_, laser_frame_, ros::Time(0), laser2base_);
            tf_ok_ = true;
        }

        std::vector<Line> lines;
        line_extraction_.extractLines(lines);
        if (lines.size() == 0)
            return false;
        // convert lines to point cloud

        if (first_match_) // first registration
        {
            linesToPointCloud(lines, last_features_.cloud, laser2base_);
            last_features_.orientation = current_feature_.orientation;
            last_features_.position = current_feature_.position;
            //printf("SCAN MATCHER: Regist firts match\n");
            first_match_ = false;
            //mt.converged = true;
            return false;
        }
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        linesToPointCloud(lines, current_feature_.cloud, laser2base_);
        icp.setInputSource(current_feature_.cloud.makeShared());
        icp.setInputTarget(last_features_.cloud.makeShared());
        icp.align(aligned_cloud);
        if (callback)
            callback(lines, aligned_cloud);

        // get the input
        ColumnVector input(3);
        Vector3d offset = current_feature_.position - last_features_.position;
        
        input(1) = offset(0);
        input(2) = offset(1);

        input(3) = 
            (current_feature_.orientation
            *last_features_.orientation.inverse())
            .toRotationMatrix().eulerAngles(0,1,2)(2);

        Matrix4f _tf = icp.getFinalTransformation();

        ColumnVector measurement;
        measurement = input;
        last_features_ = current_feature_;
        if (icp.hasConverged() && icp.getFitnessScore() < sample_fitness_)
        {
            Matrix3d rot = Matrix3d::Identity();
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                    rot(i, j) = _tf(i, j);
            measurement(3) = rot.eulerAngles(0,1,2)(2);
            measurement(1) = _tf(0, 3);
            measurement(2) = _tf(1, 3);

            printf("Converge: %d fitness:%f\n", icp.hasConverged(), icp.getFitnessScore());
        }
        pf_filter_->Update(sys_model_, input, meas_model_, measurement);
        //mt.position = last_know_position_;
        
    }
    catch (tf::TransformException ex)
    {
        PCL_ERROR("Scan matching error: %s\n", ex.what());
        return false;
    }
    return true;
}

void PFLocalization::getTransform(tf::Transform &transform)
{
    MCPdf<ColumnVector> *posterior = pf_filter_->PostGet();
    std::vector<WeightedSample<ColumnVector>> samples = posterior->ListOfSamplesGet();
    int index = 0;
    double maxWeight = -INFINITY;
    for (int i = 0; i < samples.size(); i++)
        if (samples.at(i).WeightGet() > maxWeight)
        {
            index = i;
            maxWeight = samples.at(i).WeightGet();
        }

    ColumnVector _tf = samples.at(index).ValueGet();

    Quaterniond q =
        AngleAxisd(0.0, Vector3d::UnitX())
        * AngleAxisd(0.0, Vector3d::UnitY())
        * AngleAxisd(_tf(3), Vector3d::UnitZ());
    transform.setOrigin(tf::Vector3(_tf(1), _tf(2), 0.0));
    
    //tf::Quaternion qad(0.0, 0.0,0.0,1.0);
    tf::Quaternion qad(q.x(), q.y(), q.z(), q.w());
    transform.setRotation(qad);
}
void PFLocalization::getLastKnowPose(geometry_msgs::Pose &pose)
{
    MCPdf<ColumnVector> *posterior = pf_filter_->PostGet();
    std::vector<WeightedSample<ColumnVector>> samples = posterior->ListOfSamplesGet();
    int index = 0;
    double maxWeight = -INFINITY;
    for (int i = 0; i < samples.size(); i++)
        if (samples.at(i).WeightGet() > maxWeight)
        {
            index = i;
            maxWeight = samples.at(i).WeightGet();
        }

    ColumnVector _tf = samples.at(index).ValueGet();

    Quaterniond q =
        AngleAxisd(0.0, Vector3d::UnitX())
        * AngleAxisd(0.0, Vector3d::UnitY())
        * AngleAxisd(_tf(3), Vector3d::UnitZ());

    pose.position.x = _tf(1);
    pose.position.y = _tf(2);
    pose.position.z = 0.0;

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
}
void PFLocalization::linesToPointCloud(std::vector<Line> &lines, pcl::PointCloud<pcl::PointXYZ> &cloud, tf::StampedTransform &laser2base)
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
        cit->asPointCloud(points, laser2base, (int)(cit->length() / line_scale_));
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