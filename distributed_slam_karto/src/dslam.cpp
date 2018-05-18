#include "dslam.h"

namespace dslam
{
DSlamKarto::DSlamKarto()
{
    fixed_to_odom_.setIdentity();
    num_scan_ = 0;
    last_update_map_ = ros::Time(0,0);
    map_init_ = true;
}
DSlamKarto::~DSlamKarto()
{
    if (optimizer_)
        delete optimizer_;
    if (mapper_)
        delete mapper_;
    if (database_)
        delete database_;
    if (scan_filter_)
        delete scan_filter_;
    if (scan_filter_sub_)
        delete scan_filter_sub_;
}
void DSlamKarto::configure(Configuration &config, ros::NodeHandle &nh)
{
    bool use_robust_kernel = config.get<bool>("use_robust_kernel", false);
    odom_frame_ = config.get<std::string>("odom_frame", "odom");
    fixed_frame_ = config.get<std::string>("fixed_frame", "map");
    base_frame_ = config.get<std::string>("base_frame", "base_link");
    std::string map_topic = config.get<std::string>("map_topic", "/map");
    throttle_scans_ = (int)config.get<double>("throttle_scans", 1.0);

    map_update_rate_.fromSec(config.get<double>("map_update_rate", 5.0));
    resolution_ = config.get<double>("resolution", 0.05);

    publish_graph_ = config.get<bool>("publish_graph", false);

    mapper_ = new karto::Mapper();
    database_ = new karto::Dataset();

    mapper_->setParamUseScanMatching(config.get<bool>("use_scan_matching", true));
    mapper_->setParamUseScanBarycenter(config.get<bool>("use_scan_barycenter", true));
    mapper_->setParamMinimumTravelDistance(config.get<double>("minimum_travel_distance", 0.2));
    mapper_->setParamMinimumTravelHeading(config.get<double>("minimum_travel_heading", 0.2));
    mapper_->setParamScanBufferSize((int)config.get<double>("scan_buffer_size", 70));
    mapper_->setParamScanBufferMaximumScanDistance(config.get<double>("scan_buffer_maximum_scan_distance", 20));
    mapper_->setParamLinkMatchMinimumResponseFine(config.get<double>("link_match_minimum_response_fine", 0.7));
    mapper_->setParamLinkScanMaximumDistance(config.get<double>("link_scan_maximum_distance", 8.0));
    mapper_->setParamLoopSearchMaximumDistance(config.get<double>("loop_search_maximum_distance", 6.0));
    mapper_->setParamDoLoopClosing(config.get<bool>("do_loop_closing", true));
    mapper_->setParamLoopMatchMinimumChainSize((int)config.get<double>("loop_match_minimum_chain_size", 10));
    mapper_->setParamLoopMatchMaximumVarianceCoarse(config.get<double>("loop_match_maximum_variance_coarse", 0.4));
    mapper_->setParamLoopMatchMinimumResponseCoarse(config.get<double>("loop_match_minimum_response_coarse", 0.5));
    mapper_->setParamLoopMatchMinimumResponseFine(config.get<double>("loop_match_minimum_response_fine", 0.7));

    mapper_->setParamCorrelationSearchSpaceDimension(config.get<double>("correlation_search_space_dimension", 0.3));
    mapper_->setParamCorrelationSearchSpaceResolution(config.get<double>("correlation_search_space_resolution", 0.01));
    mapper_->setParamCorrelationSearchSpaceSmearDeviation(config.get<double>("correlation_search_space_smear_deviation", 0.03));

    //loop closure
    mapper_->setParamLoopSearchSpaceDimension(config.get<double>("loop_search_space_dimension", 8.0));
    mapper_->setParamLoopSearchSpaceResolution(config.get<double>("loop_search_space_resolution", 0.05));
    mapper_->setParamLoopSearchSpaceSmearDeviation(config.get<double>("loop_search_space_smear_deviation", 0.025));

    // front-end scanmatcher
    mapper_->setParamDistanceVariancePenalty(config.get<double>("distance_variance_penalty", 0.3));
    mapper_->setParamAngleVariancePenalty(config.get<double>("angle_variance_penalty", 0.349));
    mapper_->setParamFineSearchAngleOffset(config.get<double>("fine_search_angle_offset", 0.00349));
    mapper_->setParamCoarseSearchAngleOffset(config.get<double>("coarse_search_angle_offset", 0.349));
    mapper_->setParamCoarseAngleResolution(config.get<double>("coarse_angle_resolution", 0.0349));
    mapper_->setParamMinimumAnglePenalty(config.get<double>("minimum_angle_penalty", 0.9));
    mapper_->setParamMinimumDistancePenalty(config.get<double>("minimum_distance_penalty", 0.5));
    mapper_->setParamUseResponseExpansion(config.get<bool>("use_response_expansion", false));

    map_.info.resolution = resolution_;
    map_.info.origin.position.x = 0.0;
    map_.info.origin.position.y = 0.0;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.x = 0.0;
    map_.info.origin.orientation.y = 0.0;
    map_.info.origin.orientation.z = 0.0;
    map_.info.origin.orientation.w = 1.0;

    optimizer_ = new G2OSolver();
    optimizer_->useRobustKernel(use_robust_kernel);

    mapper_->SetScanSolver(optimizer_);

    // subscriber to topic
    std::string scan_topic = config.get<std::string>("scan_topic", "/base_scan");

    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, scan_topic, 5);
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
    scan_filter_->registerCallback(boost::bind(&DSlamKarto::laserCallback, this, _1));

    std::string constraints_topic = config.get<std::string>("constraints_topic", "/constraints");
    // publisher
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(map_topic, 1);
    if(publish_graph_)
    {
        constraint_pub_ = nh.advertise< visualization_msgs::Marker>(constraints_topic, 10);
    }
}
void DSlamKarto::publishTf()
{
    //ROS_DEBUG("Publishing tf %s -> %s", fixed_frame_.c_str(), odom_frame_.c_str());
    tf::StampedTransform transform_msg(fixed_to_odom_, ros::Time::now(), fixed_frame_, odom_frame_);
    tf_broadcaster_.sendTransform(transform_msg);
}
void DSlamKarto::publishGraph()
{
    if(!publish_graph_ || !optimizer_) return;
    std::vector<Eigen::Vector2d> nodes;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > edges;
    optimizer_->getGraph(nodes, edges);

    visualization_msgs::Marker points, line_list;

    points.header.frame_id =  line_list.header.frame_id = fixed_frame_;
    points.header.stamp =  line_list.header.stamp = ros::Time::now();
    points.ns =  line_list.ns = "points_and_lines";
    points.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w  = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_list.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.04;
    points.scale.y = 0.04;

    line_list.scale.x = 0.01;
    // Points are green
    points.color.b = 1.0f;
    points.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    geometry_msgs::Point p;
    for(auto it = nodes.begin(); it != nodes.end(); it++)
    {
        
        p.x = (*it)(0);
        p.y = (*it)(1);
        p.z = 0;
        points.points.push_back(p);
    }

    for(auto it = edges.begin(); it != edges.end(); it++)
    {
        p.x = it->first(0);
        p.y = it->first(1);
        p.z = 0;
        line_list.points.push_back(p);

        p.x = it->second(0);
        p.y = it->second(1);
        p.z = 0;
        line_list.points.push_back(p);
    }

    // publish the message
    constraint_pub_.publish(points);
    constraint_pub_.publish(line_list);
}
void DSlamKarto::laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    boost::mutex::scoped_lock lock(mutex_);
    num_scan_++;
    if (!map_init_ && num_scan_ % throttle_scans_ != 0)
        return;
    karto::LaserRangeFinder *dev = getRFDevice(scan);
    if (!dev)
    {
        ROS_WARN("Laser device is invalid: %s", scan->header.frame_id.c_str());
        return;
    }
    karto::Pose2 odom;

    if (registerScan(dev, scan, odom))
    {
        // scan is valid
        // update the map if it fall to a time interval
        if ( map_init_ || scan->header.stamp - last_update_map_ > map_update_rate_)
        {
            // update the map
            // TODO
            if(updateMap())
            {
                map_pub_.publish(map_);
                last_update_map_ = scan->header.stamp;
                map_init_ = false;
            }
        }
    }

}
bool DSlamKarto::getOdom(karto::Pose2 &odom, const ros::Time &t)
{
    tf::StampedTransform b2o;
    try
    {
        tf_.waitForTransform(odom_frame_, base_frame_, t, ros::Duration(0.5));
        tf_.lookupTransform(odom_frame_, base_frame_, t, b2o);
        odom = karto::Pose2(
            b2o.getOrigin().x(),
            b2o.getOrigin().y(),
            tf::getYaw(b2o.getRotation()));
        return true;
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Cannot get laser to baser tf: %s", ex.what());
        return false;
    }
}

bool DSlamKarto::updateMap()
{
    //boost::mutex::scoped_lock lock(mutex_);
    karto::OccupancyGrid *kmap =
        karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), resolution_);
    if (!kmap)
        return false;

    kt_int32s kw = kmap->GetWidth();
    kt_int32s kh = kmap->GetHeight();
    karto::Vector2<kt_double> offset = kmap->GetCoordinateConverter()->GetOffset();
    // reposition the map if needed
    if (map_.info.width != (unsigned int)kw ||
        map_.info.height != (unsigned int)kh ||
        map_.info.origin.position.x != offset.GetX() ||
        map_.info.origin.position.y != offset.GetY())
    {
        map_.info.origin.position.x = offset.GetX();
        map_.info.origin.position.y = offset.GetY();
        map_.info.width = kw;
        map_.info.height = kh;
        map_.data.resize(map_.info.width * map_.info.height, -1);
    }

    for (kt_int32s i=0; i<kh; i++)
        for (kt_int32s j=0; j<kw; j++) 
        {
            // Getting the value at position x,y
            kt_int8u value = kmap->GetValue(karto::Vector2<kt_int32s>(j, i));
            // convert to ros format
            switch(value)
            {
                case karto::GridStates_Unknown:
                    map_.data[j + i*map_.info.width] = -1;
                    break;
                case karto::GridStates_Occupied:
                    map_.data[j + i*map_.info.width] = 100;
                    break;
                case karto::GridStates_Free:
                    map_.data[j + i*map_.info.width] = 0;
                    break;
                default:
                    break;
            }
        }
    // update header
    map_.header.stamp = ros::Time::now();
    map_.header.frame_id = fixed_frame_;
    delete kmap;
    return true;
}
karto::LaserRangeFinder *DSlamKarto::getRFDevice(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    if (devices_.find(scan->header.frame_id) == devices_.end())
    {
        // device not found
        // create new one
        // 1. get laser to base transformation
        tf::StampedTransform l2b;
        try
        {
            tf_.waitForTransform(base_frame_, scan->header.frame_id,
                                      scan->header.stamp, ros::Duration(0.5));
            tf_.lookupTransform(base_frame_, scan->header.frame_id,
                                     scan->header.stamp, l2b);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Cannot get laser to baser tf: %s", ex.what());
            return NULL;
        }
        // 2.create new device
        karto::LaserRangeFinder *dev =
            karto::LaserRangeFinder::CreateLaserRangeFinder(
                karto::LaserRangeFinder_Custom,
                karto::Name(scan->header.frame_id));
        dev->SetOffsetPose(karto::Pose2(l2b.getOrigin().x(),
                                        l2b.getOrigin().y(),
                                        tf::getYaw(l2b.getRotation())));
        dev->SetMinimumRange(scan->range_min);
        dev->SetMaximumRange(scan->range_max);
        dev->SetMinimumAngle(scan->angle_min);
        dev->SetMaximumAngle(scan->angle_max);
        dev->SetAngularResolution(scan->angle_increment);

        devices_[scan->header.frame_id] = dev;
        database_->Add(dev);
    }

    return devices_[scan->header.frame_id];
}
bool DSlamKarto::registerScan(karto::LaserRangeFinder *dev,
                              const sensor_msgs::LaserScan::ConstPtr &scan,
                              karto::Pose2 &odom)
{
    if (!getOdom(odom, scan->header.stamp))
        return false;
    std::vector<kt_double> ranges;
    for (std::vector<float>::const_iterator it = scan->ranges.begin(); it != scan->ranges.end(); it++)
    {
        ranges.push_back(*it);
    }
    // create karto range scan
    karto::LocalizedRangeScan *kscan = new karto::LocalizedRangeScan(dev->GetName(), ranges);
    kscan->SetOdometricPose(odom);
    kscan->SetCorrectedPose(odom);

    // now process the scan using karto front end and g2o optimizer back end
    bool valid = mapper_->Process(kscan);
    if (valid)
    {
        // this scan is valid
        // calculate the fixed to odom transformation
        karto::Pose2 estimated = kscan->GetCorrectedPose();
        tf::Transform f2b = tf::Transform(
            tf::createQuaternionFromRPY(0, 0, estimated.GetHeading()),
            tf::Vector3(estimated.GetX(), estimated.GetY(), 0.0));
        tf::Transform o2b = tf::Transform(
            tf::createQuaternionFromRPY(0, 0, odom.GetHeading()),
            tf::Vector3(odom.GetX(), odom.GetY(), 0.0));
        fixed_to_odom_ = f2b * o2b.inverse();
        database_->Add(kscan);
    }
    else
        delete kscan;

    return valid;
}

} // namespace dslam