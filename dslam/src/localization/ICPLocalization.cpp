#include "ICPLocalization.h"

namespace dslam
{
ICPLocalization::ICPLocalization():BaseLocalization(){
}
void ICPLocalization::configure(Configuration& cnf)
{
    BaseLocalization::configure(cnf);
}
void ICPLocalization::alignLastFeature(pcl::PointCloud<pcl::PointXYZ> &ret)
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
bool ICPLocalization::match(const void (*callback)(std::vector<Line>&, pcl::PointCloud<pcl::PointXYZ>& cloud))
{
    //mt.converged = false;
    //mt.fitness = 0.0;
    //mt.rot = Eigen::Quaterniond::Identity();
    // extract the line
    try
    {
        if(!tf_ok_)
        {
            if(!tf_) return false;
            tf_->lookupTransform(robot_base_frame_, laser_frame_, ros::Time(0), laser2base_);
            tf_ok_ = true;
        }
        
        std::vector<Line> lines;
        extractLines(lines);
        if(lines.size() == 0) return false;
        // convert lines to point cloud

        if(first_match_) // first registration
        {
            linesToPointCloud(lines, last_features_.cloud, laser2base_);
            last_features_.orientation = current_feature_.orientation;
            last_features_.position = current_feature_.position;
            //printf("SCAN MATCHER: Regist firts match\n");
            first_match_ = false;
            //mt.converged = true;
            return false;
        }
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud, aligned_feature;
        linesToPointCloud(lines, current_feature_.cloud, laser2base_);
        icp->setInputSource (current_feature_.cloud.makeShared());
        alignLastFeature(aligned_feature);
        icp->setInputTarget (aligned_feature.makeShared());
        icp->align(aligned_cloud);
        if(callback)
            callback(lines, aligned_feature);
        
        Eigen::Vector3d offset = current_feature_.position - last_features_.position;
        //offset(2) = 0.0;
        Eigen::Matrix4f _tf = icp->getFinalTransformation();
        //last_know_position_ += offset;
        current_kf_.tf.translation += offset;

        current_kf_.tf.rotation *= current_feature_.orientation*last_features_.orientation.inverse();
        //current_kf_.tf.rotation.normalize();
        //mt.rot = current_feature_.orientation;
        //Eigen::Matrix3d M;
        //M = mt.rot;
        //mt.tl = M*mt.tl;
        last_features_ = current_feature_;
        //mt.fitness = icp->getFitnessScore();
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        if(icp->hasConverged() && icp->getFitnessScore() < sample_fitness_)
        {
            Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
            for(int i = 0; i < 2; i ++)
                for(int j = 0; j < 2; j ++)
                rot(i,j) = _tf(i,j);
            q = rot;
            //double y = rot.eulerAngles(0,1,2)[2]/2.0;
            //mt.rot = mt.rot*q;
            current_kf_.tf.rotation *= q;
            //current_kf_.tf.rotation.normalize();
            //mt.tl = _tf*mt.tl;
            current_kf_.tf.translation(0) += _tf(0,3);
            current_kf_.tf.translation(1) += _tf(1,3);
            //current_kf_.tf.translation(2) += _tf(2,3);

            
            //current_kf_.diff.rotation.normalize();
            //mt.converged = true;
            printf("Converge: %d fitness:%f\n", icp->hasConverged (), icp->getFitnessScore () );
        }
        //mt.position = last_know_position_;
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
            return true;
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
    return false;
}
}