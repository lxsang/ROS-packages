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
    odom_frame_ = cnf.get<std::string>("odom_frame", "/odom");
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
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        linesToPointCloud(lines, current_feature_.cloud, laser2base_);
        icp->setInputSource(current_feature_.cloud.makeShared());
        icp->setInputTarget(last_features_.cloud.makeShared());
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
            measurement(3) = rot.eulerAngles(0,1,2)(2);
            measurement(1) = _tf(0, 3);
            measurement(2) = _tf(1, 3);

            printf("Converge: %d fitness:%f\n", icp->hasConverged(), icp->getFitnessScore());
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
        if(keyframes.empty() || dist >= keyframe_sample_linear_ || yaw >= keyframe_sample_angular_)
        {
            //current_kf_.scan = current_scan_;
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
/*
void PFLocalization::getTransform(tf::Transform &transform)
{
    if(!tf_) return;
    try{
        tf::StampedTransform robot2odom;
        tf_->lookupTransform(odom_frame_,robot_base_frame_, ros::Time(0), robot2odom );
        geometry_msgs::Pose pose;
        getLastKnowPose(pose);

        tf::Vector3 tl(0.0, 0.0, 0.0);
        tl.setX( pose.position.x - robot2odom.getOrigin().getX() );
        tl.setY(pose.position.y - robot2odom.getOrigin().getY() );
        tl.setZ(pose.position.z - robot2odom.getOrigin().getZ() );

        transform.setOrigin(tl);

        Quaterniond q;
        q.x() = pose.orientation.x;
        q.y() = pose.orientation.y;
        q.z() = pose.orientation.z;
        q.w() = pose.orientation.w;

        Quaterniond p;
        p.x() = robot2odom.getRotation().x();
        p.y() = robot2odom.getRotation().y();
        p.z() = robot2odom.getRotation().z();
        p.w() = robot2odom.getRotation().w();

        q = q*p.inverse();

        //tf::Quaternion qad(0.0, 0.0,0.0,1.0);
        tf::Quaternion qad(q.x(), q.y(), q.z(), q.w());
        transform.setRotation(qad);
    }
    catch(tf::TransformException e){}
    
}
*/
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
/*
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
*/
}