#include "PFLocalization.h"

namespace dslam
{
using namespace Eigen;
PFLocalization::PFLocalization():BaseLocalization()
{
    last_estimate_pose_.translation = Vector3d(0.0, 0.0, 0.0);
    last_estimate_pose_.rotation = Quaterniond::Identity();
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
    BaseLocalization::configure(cnf);
    //icp->setRANSACOutlierRejectionThreshold (0.06); // TODO

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

    //meas_linear_tol_ = pf_conf.get<double>("MEAS_LINEAR_TOLERANCE", 0.0);
    //meas_angular_tol_ = pf_conf.get<double>("MEAS_ANGULAR_TOLERANCE", 0.0);

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
    //printf("======INIT PF FILTER Finish\n");
}
void PFLocalization::alignLastFeature(pcl::PointCloud<pcl::PointXYZ> &ret)
{
    Eigen::Matrix3d M;
    M = last_features_.orientation*current_feature_.orientation.inverse(); 
    ret.width = last_features_.cloud.width;
    ret.height = last_features_.cloud.height;
    ret.is_dense = last_features_.cloud.is_dense;
    Eigen::Vector3d offset = current_feature_.position - last_features_.position;
    ret.points.resize(ret.width);
    for(int i = 0; i < last_features_.cloud.width; i++)
    {
        Eigen::Vector3d v(last_features_.cloud.points[i].x, last_features_.cloud.points[i].y , last_features_.cloud.points[i].z);
        v = M*v;
        //Eigen::Vector3d rotatedV = rotatedP.vec();
        ret.points[i].x = v.x() - offset.x();
        ret.points[i].y = v.y() - offset.y();
        ret.points[i].z = 0.0;//v.z() - offset.z();
    }
}
bool PFLocalization::__match(const void (*callback)(std::vector<Line> &, pcl::PointCloud<pcl::PointXYZ> &cloud))
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
        extractLines(lines);
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
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud,aligned_feature;
        linesToPointCloud(lines, current_feature_.cloud, laser2base_);
        alignLastFeature(aligned_feature);
        icp->setInputSource(current_feature_.cloud.makeShared());
        icp->setInputTarget(aligned_feature.makeShared());
        icp->align(aligned_cloud);
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
        input(3) = angles::normalize_angle(input(3));
        
        //input(1) = fabs(input(1)) > meas_linear_tol_?input(1):0.0;
        //input(2) = fabs(input(2)) > meas_linear_tol_?input(2):0.0;
        //input(3) = fabs(input(3)) > meas_angular_tol_?input(3):0.0;
        Matrix4f _tf = icp->getFinalTransformation();

        ColumnVector measurement;
        measurement = input;
        last_features_ = current_feature_;
        if (icp->hasConverged() && icp->getFitnessScore() < sample_fitness_)
        {
            Matrix3d rot = Matrix3d::Identity();
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                    rot(i, j) = _tf(i, j);
            double theta = angles::normalize_angle(rot.eulerAngles(0,1,2)(2));
            Vector3d tl;
            tl(0) =  _tf(0, 3);
            tl(1) = _tf(1,3);
            tl(2) = 0.0;
            tl = last_features_.orientation.toRotationMatrix()*tl;
            measurement(3) = theta;
            measurement(1) = tl(0);
            measurement(2) = tl(1);

            //measurement(3) = fabs(theta) > meas_angular_tol_?theta:0.0;
            //measurement(1) = fabs(_tf(0, 3)) > meas_linear_tol_?_tf(0, 3):0.0;
            //measurement(2) = fabs(_tf(1, 3)) > meas_linear_tol_?_tf(1, 3):0.0;

            //printf("Converge: %d fitness:%f\n", icp->hasConverged(), icp->getFitnessScore());
        }
        else
        {
            ROS_WARN("ICP is not converged\n");
        }
        pf_filter_->Update(sys_model_, input, meas_model_, measurement);
        //mt.position = last_know_position_;
        dslam_tf_t es_pose;
        getLastEstimatedPose(es_pose);
        current_kf_.tf.translation = es_pose.translation - last_estimate_pose_.translation;
        current_kf_.tf.rotation = es_pose.rotation*last_estimate_pose_.rotation.inverse();
        current_kf_.diff.translation = current_feature_.position;
        current_kf_.diff.rotation = current_feature_.orientation;
        current_kf_.index = kf_idx_;
        double dist = sqrt( pow(current_kf_.tf.translation(0), 2) + pow(current_kf_.tf.translation(1),2) );
        double yaw = fabs(current_kf_.tf.rotation.toRotationMatrix().eulerAngles(0, 1, 2)[2]);

        // transform
        //transform_.setOrigin( tf::Vector3(es_pose.translation(0) - current_feature_.position(0),es_pose.translation(1) - current_feature_.position(1),0.0));
        //Quaterniond __q = es_pose.rotation*current_feature_.orientation.inverse();
        //tf::Quaternion qad(__q.x(), __q.y(),__q.z(),__q.w());
        //transform_.setRotation(qad);

        if(keyframes.empty() || dist >= keyframe_sample_linear_ || yaw >= keyframe_sample_angular_)
        {
            //current_kf_.scan = current_scan_;
            //pf_filter_->Resample();
            keyframes.push_back(current_kf_);
            kf_idx_++;
            current_kf_.tf.rotation = Eigen::Quaterniond::Identity();
            current_kf_.diff.rotation = Eigen::Quaterniond::Identity();
            current_kf_.tf.translation = Eigen::Vector3d(0.0,0.0,0.0);
            current_kf_.diff.translation = Eigen::Vector3d(0.0,0.0,0.0);
            last_estimate_pose_ = es_pose;
            return true;
        }
    }
    catch (tf::TransformException ex)
    {
        PCL_ERROR("Scan matching error: %s\n", ex.what());
        return false;
    }
    return false;
}
void PFLocalization::visualize(ros::Publisher& pub)
{
    MCPdf<ColumnVector> *posterior = pf_filter_->PostGet();
    std::vector<WeightedSample<ColumnVector>> samples = posterior->ListOfSamplesGet();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(samples.size());
    cloud.width = samples.size();
    for (int i = 0; i < samples.size(); i++)
    {
        ColumnVector pos = samples.at(i).ValueGet();
        cloud.points[i].x = pos(1);
        cloud.points[i].y = pos(2);
        cloud.points[i].z = 0.0;
    }
    sensor_msgs::PointCloud2 pcloud;
    pcl::toROSMsg(cloud, pcloud);
    pcloud.header.frame_id = global_frame_;
    pcloud.header.stamp = ros::Time::now();
    pub.publish(pcloud);

}
void PFLocalization::getLastEstimatedPose( dslam_tf_t& pose)
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

    pose.translation(0) = _tf(1);
    pose.translation(1) = _tf(2);
    pose.translation(2) = 0.0;

    pose.rotation = q;
}
}