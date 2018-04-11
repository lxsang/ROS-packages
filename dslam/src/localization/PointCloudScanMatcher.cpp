#include "ScanMatcher.h"

namespace dslam{
    PointCloudScanMatcher::PointCloudScanMatcher()
    {
        first_match_ = true;
        last_features_.orientation= Eigen::Quaterniond::Identity();
        tf_ = nullptr;
        //yaw=0.0;
    }
    PointCloudScanMatcher::~PointCloudScanMatcher()
    {

    }

    double PointCloudScanMatcher::getYaw()
    {
        auto euler = last_features_.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        auto euler1 = current_feature_.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        double yaw = euler1[2] - euler[2];
        return yaw;
    }

    void PointCloudScanMatcher::configure(Configuration& cnf)
    {
        Configuration xpc_config = cnf.get<Configuration>("point_cloud_extraction", Configuration());
        max_range_ = xpc_config.get<double>("max_range",10.0);

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
        robot_base_frame_ = cnf.get<std::string>("robot_base", "base_link");
        PCL_INFO("Map frame is %s, robot frame is %s, and laser frame is %s\n", global_frame_.c_str(), robot_base_frame_.c_str() ,laser_frame_.c_str());
        PCL_INFO("*************************************\n");
    }
    void PointCloudScanMatcher::registerScan(const sensor_msgs::LaserScan::ConstPtr & scan, geometry_msgs::Quaternion orientation)
    {
        Eigen::Quaterniond q;
        q.x() = orientation.x;
        q.y() = orientation.y;
        q.z() =  orientation.z;
        q.w() = orientation.w;
        
        current_feature_.orientation = q*last_features_.orientation.inverse();

        sensor_msgs::PointCloud2 cloud;
        if(tf_)
        {
            projector_.transformLaserScanToPointCloud(robot_base_frame_,*scan,cloud,*tf_, laser_geometry::channel_option::Default);
            pcl::fromROSMsg(cloud,current_feature_.cloud);
            //printf("project data %d\n", current_feature_.cloud.points.size());
        }
    }
    void PointCloudScanMatcher::match(icp_tf_t& mt, const void (*callback)(pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>&))
    {
        mt.converged = false;
        mt.fitness = 0.0;
        mt.rot = Eigen::Quaterniond::Identity();
        mt.tl(0) = 0.0;
        mt.tl(1) = 0.0;
        mt.tl(2) = 0.0;
        // extract the line
        if(!tf_) return;
        // convert lines to point cloud
        if(current_feature_.cloud.points.size() == 0) return;
        if(first_match_) // first registration
        {
            last_features_ = current_feature_;
            //printf("SCAN MATCHER: Regist firts match\n");
            first_match_ = false;
            mt.converged = true;
            return;
        }
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud, aligned_feature;
        icp.setInputSource (current_feature_.cloud.makeShared());
        rotateLastFeature(aligned_feature);
        icp.setInputTarget (aligned_feature.makeShared());
        icp.align(aligned_cloud);
        if(callback)
            callback(current_feature_.cloud, aligned_feature);
        
        Eigen::Matrix4f _tf = icp.getFinalTransformation();
        mt.tf = _tf;
        mt.tl(0) = _tf(0,3);
        mt.tl(1) = _tf(1,3);
        mt.tl(2) = _tf(2,3);
        //p.w() = 0;
        //p.vec() = mt.tl;
        
        Eigen::Matrix3d rot;
        for(int i = 0; i < 3; i ++)
            for(int j = 0; j < 3; j ++)
            rot(i,j) = _tf(i,j);
        Eigen::Quaterniond q(rot);
        mt.rot = current_feature_.orientation*q;
        Eigen::Matrix3d M;
        M = mt.rot;
        mt.tl = M*mt.tl;
        mt.fitness = icp.getFitnessScore();
        if(icp.hasConverged())
        {
            last_features_ = current_feature_;
        }
        if(icp.hasConverged())
        {
            mt.converged = true;
        }
        else
        {
            PCL_INFO("Fitness is %f\n", mt.fitness);
        }
        
    }

    void PointCloudScanMatcher::rotateLastFeature(pcl::PointCloud<pcl::PointXYZ>& ret)
    {
        Eigen::Matrix3f M;
        M = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())
            *Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY())
            *Eigen::AngleAxisf(getYaw(), Eigen::Vector3f::UnitZ()); 
        ret.width = last_features_.cloud.width;
        ret.height = last_features_.cloud.height;
        ret.is_dense = last_features_.cloud.is_dense;
        ret.points.resize(ret.width);
        for(int i = 0; i < last_features_.cloud.width; i++)
        {
            Eigen::Vector3f v(last_features_.cloud.points[i].x, last_features_.cloud.points[i].y, last_features_.cloud.points[i].z);
            v = M*v;
            //Eigen::Vector3d rotatedV = rotatedP.vec();
            ret.points[i].x = v.x();
            ret.points[i].y = v.y();
            ret.points[i].z = v.z();
        }

    }

}