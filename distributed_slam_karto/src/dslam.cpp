#include "dslam.h"

namespace dslam
{
DSlamKarto::DSlamKarto()
{
    fixed_to_odom_.setIdentity();
    //num_scan_ = 0;
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
    /*if (scan_filter_)
        delete scan_filter_;
    if (scan_filter_sub_)
        delete scan_filter_sub_;
        */
}
void DSlamKarto::configure(Configuration &config, ros::NodeHandle &nh)
{
    bool use_robust_kernel = config.get<bool>("use_robust_kernel", false);
    odom_frame_ = config.get<std::string>("odom_frame", "odom");
    fixed_frame_ = config.get<std::string>("fixed_frame", "map");
    //base_frame_ = config.get<std::string>("base_frame", "base_link");
    std::string map_topic = config.get<std::string>("map_topic", "/map");
    //throttle_scans_ = (int)config.get<double>("throttle_scans", 1.0);

    map_update_rate_.fromSec(config.get<double>("map_update_rate", 5.0));
    resolution_ = config.get<double>("resolution", 0.05);

    publish_graph_ = config.get<bool>("publish_graph", false);

    offset_id_ = (unsigned int)config.get<double>("graph_offset_id",0.0);

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

    std::string graph_topic = config.get<std::string>("graph_topic", "/graph");
    std::string srv_name = config.get<std::string>("graph_sync_service", "dslam_sync_graph");
    // publisher
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(map_topic, 1);
    if(publish_graph_)
    {
        //constraint_pub_ = nh.advertise< visualization_msgs::Marker>(constraints_topic, 5);
        graph_pub_ = nh.advertise<distributed_slam_karto::Graph>(graph_topic, 2);
        sync_srv_ = nh.advertiseService(srv_name, &DSlamKarto::syncGraphService, this);
    }
}

void DSlamKarto::transformPose(tf::StampedTransform& g2g, geometry_msgs::Point& p)
{
    tf::Transform pose(tf::createQuaternionFromRPY(0,0,p.z),tf::Vector3(p.x,p.y,0));
    pose = g2g*pose;
    p.x = pose.getOrigin().getX();
    p.y = pose.getOrigin().getY();
    p.z = tf::getYaw(pose.getRotation());
}

bool DSlamKarto::syncGraphService(distributed_slam_karto::SyncGraph::Request& rq, distributed_slam_karto::SyncGraph::Response & res)
{
    boost::mutex::scoped_lock lock(mutex_);
    // check if transform is available
    tf::StampedTransform g2g;
    if(!mapper_ || !tf_) return false;
    try
    {
        tf_->waitForTransform(fixed_frame_, rq.graph.frame_id,ros::Time::now() , ros::Duration(0.5));
        tf_->lookupTransform(fixed_frame_, rq.graph.frame_id, ros::Time::now() , g2g);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("Cannot get fixed to fixed tf: %s", ex.what());
        return false;
    }
    // add laser device if not exist
    for(auto it = rq.graph.devices.begin(); it != rq.graph.devices.end(); it++)
        if(devices_.find(it->frame_id) == devices_.end())
        {
            // add new device
            karto::LaserRangeFinder *dev =
            karto::LaserRangeFinder::CreateLaserRangeFinder(
                karto::LaserRangeFinder_Custom,
                karto::Name(it->frame_id));
            dev->SetOffsetPose(karto::Pose2(it->offset.x,it->offset.y,it->offset.z));
            dev->SetMinimumRange(it->min_range);
            dev->SetMaximumRange(it->max_range);
            dev->SetMinimumAngle(it->min_angle);
            dev->SetMaximumAngle(it->max_angle);
            dev->SetAngularResolution(it->resolution);
            devices_[it->frame_id] = dev;
            database_->Add(dev);
        }
    // add vertex
    for(auto it = rq.graph.vertices.begin(); it != rq.graph.vertices.end(); it++)
    {
        karto::Vertex<karto::LocalizedRangeScan>* v;
        //check if vertex exist
        if((v=getVertex(it->dev_name,it->state_id)) != NULL)
        {
            //ROS_WARN("Vertex %d:%d exist", v->GetObject()->GetStateId(), v->GetObject()->GetUniqueId());
            continue;
        }
        ROS_INFO("Adding node %d of device %s", it->state_id, it->dev_name.c_str());
        std::vector<kt_double> ranges;
        for(auto vit = it->ranges.begin(); vit != it->ranges.end(); vit++) ranges.push_back(*vit);
        // create karto range scan
        karto::LocalizedRangeScan *kscan = new karto::LocalizedRangeScan(karto::Name(it->dev_name), ranges);
        transformPose(g2g, it->odom_pose);
        kscan->SetOdometricPose(karto::Pose2(it->odom_pose.x,it->odom_pose.y,it->odom_pose.z));
        transformPose(g2g, it->corrected_pose);
        kscan->SetCorrectedPose(karto::Pose2(it->corrected_pose.x,it->corrected_pose.y,it->corrected_pose.z));
        //kscan->SetUniqueId();
        //kscan->SetStateId(it->state_id);
        // add vertex
        mapper_->GetMapperSensorManager()->AddScan(kscan, it->id + rq.graph.offset_id, it->state_id);
        mapper_->GetGraph()->AddVertex(kscan);
        database_->Add(kscan);
        // TODO: generate constrains if exist
    }
    for(auto it = rq.graph.edges.begin(); it != rq.graph.edges.end(); it++)
    {
        // find vertex
        karto::Vertex<karto::LocalizedRangeScan>* v1 = getVertex(it->dev_name,it->source_id);
        karto::Vertex<karto::LocalizedRangeScan>* v2 = getVertex(it->dev_name,it->target_id);
        if(!v1)
        {
            ROS_WARN("Cannot find vertex %d of devices %s", it->source_id, it->dev_name.c_str());
            continue;
        }
        if(!v2)
        {
            ROS_WARN("Cannot find vertex %d of devices %s", it->target_id, it->dev_name.c_str());
            continue;
        }
        // check if the edge exist
        bool has_edge = false;
        karto::Edge<karto::LocalizedRangeScan>* pEdge;
        for(auto eit = v1->GetEdges().begin(); eit!= v1->GetEdges().end(); eit++)
        {
            pEdge = *eit;

            if (pEdge->GetTarget() == v2)
            {
                has_edge = true;
                break;
            }
        }
        if(has_edge)
        {
            //ROS_WARN("Edge %d->%d exist", v1->GetObject()->GetUniqueId(), v2->GetObject()->GetUniqueId());
            continue;
        }
        // create new edge
        ROS_INFO("Add Edge %d->%d", v1->GetObject()->GetUniqueId(), v2->GetObject()->GetUniqueId());
        pEdge = new karto::Edge<karto::LocalizedRangeScan>(v1, v2);
        // restore the link
        karto::LinkInfo* link = new karto::LinkInfo();
        karto::Matrix3 cov;
        for(int i = 0; i< 3; i++)
            for(int j=0; j < 3; j ++)
                cov(i,j) = it->link.covariance[i*3+j];
        transformPose(g2g, it->link.pose1);
        transformPose(g2g, it->link.pose2);
        link->setLink(karto::Pose2(it->link.pose1.x,it->link.pose1.y,it->link.pose1.z ),
                    karto::Pose2(it->link.pose2.x,it->link.pose2.y,it->link.pose2.z),
                    cov);
        pEdge->SetLabel(link);
        mapper_->GetGraph()->AddEdge(pEdge);
        // add to optimizer
        if(optimizer_)
            optimizer_->AddConstraint(pEdge);
    }
    res.sync = true;
    // add edge
    return true;
}
void DSlamKarto::publishTf()
{
    //ROS_DEBUG("Publishing tf %s -> %s", fixed_frame_.c_str(), odom_frame_.c_str());
    tf::StampedTransform transform_msg(fixed_to_odom_, ros::Time::now(), fixed_frame_, odom_frame_);
    tf_broadcaster_.sendTransform(transform_msg);
}
void DSlamKarto::publishMap()
{
    boost::mutex::scoped_lock lock(mutex_);
    if(map_init_) return;
    map_pub_.publish(map_);
}
void DSlamKarto::getDevices(std::vector<distributed_slam_karto::LaserDevice> &devs)
{
    for ( auto mit = devices_.begin(); mit != devices_.end(); mit++ )
    {
        distributed_slam_karto::LaserDevice dev;
        dev.frame_id = mit->first;
        dev.min_range = mit->second->GetMinimumRange();
        dev.max_range = mit->second->GetMaximumRange();
        dev.min_angle = mit->second->GetMinimumAngle();
        dev.max_angle = mit->second->GetMaximumAngle();
        dev.resolution = mit->second->GetAngularResolution();
        dev.offset.x = mit->second->GetOffsetPose().GetX();
        dev.offset.y = mit->second->GetOffsetPose().GetY();
        dev.offset.z = mit->second->GetOffsetPose().GetHeading();
        devs.push_back(dev);
    }
}
karto::Vertex<karto::LocalizedRangeScan>* DSlamKarto::getVertex(std::string n, int state_id)
{
    //boost::mutex::scoped_lock lock(mutex_);
    if(!mapper_->GetGraph()) return NULL;
    karto::Name name(n);
    std::map<karto::Name,std::vector<karto::Vertex<karto::LocalizedRangeScan>*>> vertices;
    vertices = mapper_->GetGraph()->GetVertices();

    if(vertices.find(name) == vertices.end() || state_id > vertices[name].size() - 1 )
    {
        return NULL;
    }

    return vertices[name][state_id];
}
void DSlamKarto::publishGraph()
{
    if(!publish_graph_ || !mapper_->GetGraph()) return;
    std::map<karto::Name,std::vector<karto::Vertex<karto::LocalizedRangeScan>*>> vertices;
    std::vector<karto::Edge<karto::LocalizedRangeScan>*> edges;
    vertices = mapper_->GetGraph()->GetVertices();
    edges = mapper_->GetGraph()->GetEdges();

    distributed_slam_karto::Graph msg;
    msg.offset_id = offset_id_;
    msg.frame_id = fixed_frame_;
    getDevices(msg.devices);
    // get all devices

    //std::vector<Eigen::Vector2d> nodes;
    //std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > edges;
    //optimizer_->getGraph(nodes, edges);

    /*visualization_msgs::Marker points, line_list;

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
    */
    // vertex

    for ( auto mit = vertices.begin(); mit != vertices.end(); mit++ )
    {
       for(auto it = mit->second.begin(); it != mit->second.end(); it++)
        {
            distributed_slam_karto::Vertex vtx;
            //vertex
            vtx.id = (*it)->GetObject()->GetUniqueId();
            vtx.state_id = (*it)->GetObject()->GetStateId();
            vtx.dev_name = mit->first.ToString();
            vtx.odom_pose.x = (*it)->GetObject()->GetOdometricPose().GetX();
            vtx.odom_pose.y = (*it)->GetObject()->GetOdometricPose().GetY();
            vtx.odom_pose.z = (*it)->GetObject()->GetOdometricPose().GetHeading();
            vtx.corrected_pose.x = (*it)->GetObject()->GetCorrectedPose().GetX();
            vtx.corrected_pose.y = (*it)->GetObject()->GetCorrectedPose().GetY();
            vtx.corrected_pose.z = (*it)->GetObject()->GetCorrectedPose().GetHeading();
            kt_double* data  = (*it)->GetObject()->GetRangeReadings();
            for(int i=0; i <  (*it)->GetObject()->GetNumberOfRangeReadings(); i++)
                vtx.ranges.push_back(data[i]);
            msg.vertices.push_back(vtx);

            /*p.x = (*it)->GetObject()->GetCorrectedPose().GetX();
            p.y = (*it)->GetObject()->GetCorrectedPose().GetY();
            p.z = 0;
            points.points.push_back(p);*/
        } 
    }
    

    for(auto it = edges.begin(); it != edges.end(); it++)
    {
        distributed_slam_karto::Edge edge;
        edge.source_id = (*it)->GetSource()->GetObject()->GetStateId();
        edge.target_id = (*it)->GetTarget()->GetObject()->GetStateId();
        edge.dev_name = (*it)->GetSource()->GetObject()->GetSensorName().ToString();
        karto::LinkInfo* link= (karto::LinkInfo*)(*it)->GetLabel();
        edge.link.pose1.x = link->GetPose1().GetX();
        edge.link.pose1.y = link->GetPose1().GetY();
        edge.link.pose1.z = link->GetPose1().GetHeading();
        edge.link.pose2.x = link->GetPose2().GetX();
        edge.link.pose2.y = link->GetPose2().GetY();
        edge.link.pose2.z = link->GetPose2().GetHeading();
        for(int i = 0; i < 3; i++)
            for(int j= 0; j<3; j++)
                edge.link.covariance.push_back(link->GetCovariance()(i,j));
        msg.edges.push_back(edge);

        /*p.x = (*it)->GetSource()->GetObject()->GetCorrectedPose().GetX();
        p.y = (*it)->GetSource()->GetObject()->GetCorrectedPose().GetY();
        p.z = 0;
        line_list.points.push_back(p);

        p.x = (*it)->GetTarget()->GetObject()->GetCorrectedPose().GetX();
        p.y = (*it)->GetTarget()->GetObject()->GetCorrectedPose().GetY();
        p.z = 0;
        line_list.points.push_back(p);*/
    }

    // publish the message
    //constraint_pub_.publish(points);
    //constraint_pub_.publish(line_list);
    graph_pub_.publish(msg);
}
void DSlamKarto::laserCallback(const AnnotatedScan& aScan)
{
    boost::mutex::scoped_lock lock(mutex_);
    /*if(num_scans_.find(aScan.scan->header.frame_id) == num_scans_.end())
        num_scans_[aScan.scan->header.frame_id] = 1;
    else
        num_scans_[aScan.scan->header.frame_id]++;
    if (!map_init_ && num_scans_[aScan.scan->header.frame_id] % throttle_scans_ != 0)
        return;
    */
    karto::LaserRangeFinder *dev = getRFDevice(aScan);
    if (!dev)
    {
        ROS_WARN("Laser device is invalid: %s", aScan.scan->header.frame_id.c_str());
        return;
    }
    karto::Pose2 odom;
    if (!getOdom(odom, aScan)) return;
    if (registerScan(dev,aScan, odom))
    {
        // scan is valid
        // update the map if it fall to a time interval
        if ( map_init_ || aScan.scan->header.stamp - last_update_map_ > map_update_rate_)
        {
            // update the map
            // TODO
            if(updateMap())
            {
                last_update_map_ = aScan.scan->header.stamp;
                map_init_ = false;
            }
        }
    }

}
void DSlamKarto::laserCallback(const AnnotatedScan& aScan, karto::Pose2& fixed_pose)
{
    boost::mutex::scoped_lock lock(mutex_);
    /*if(num_scans_.find(aScan.scan->header.frame_id) == num_scans_.end())
        num_scans_[aScan.scan->header.frame_id] = 1;
    else
        num_scans_[aScan.scan->header.frame_id]++;
    if (!map_init_ && num_scans_[aScan.scan->header.frame_id] % throttle_scans_ != 0)
        return;
    */
    karto::LaserRangeFinder *dev = getRFDevice(aScan);
    if (!dev)
    {
        ROS_WARN("Laser device is invalid: %s", aScan.scan->header.frame_id.c_str());
        return;
    }
    tf::Transform pose(tf::createQuaternionFromRPY(0,0,fixed_pose.GetHeading()),
                                           tf::Vector3(fixed_pose.GetX(),fixed_pose.GetY(),0));
    if(!tf_) return;
    try
    {
        tf::StampedTransform f2o;
        tf_->waitForTransform(odom_frame_, fixed_frame_,ros::Time::now() , ros::Duration(0.1));
        tf_->lookupTransform(odom_frame_, fixed_frame_, ros::Time::now() , f2o);
        pose = f2o*pose;
    }
    catch (tf::TransformException ex)
    {
        //ROS_WARN("Cannot get fixed to odom tf: %s", ex.what());
        return;
    }
    // odom pose
    double yaw = tf::getYaw(pose.getRotation());

    karto::Pose2 karto_pose = 
            karto::Pose2(pose.getOrigin().x(),
                        pose.getOrigin().y(),
                        yaw);
    if (registerScan(dev,aScan, karto_pose))
    {
        // scan is valid
        // update the map if it fall to a time interval
        if ( map_init_ || aScan.scan->header.stamp - last_update_map_ > map_update_rate_)
        {
            // update the map
            // TODO
            if(updateMap())
            {
                last_update_map_ = aScan.scan->header.stamp;
                map_init_ = false;
            }
        }
    }

}
bool DSlamKarto::getOdom(karto::Pose2 &odom, const AnnotatedScan& aScan)
{
    tf::StampedTransform b2o;
    if(!tf_) return false;
    try
    {
        tf_->waitForTransform(odom_frame_, aScan.base_frame_id, aScan.scan->header.stamp, ros::Duration(0.5));
        tf_->lookupTransform(odom_frame_, aScan.base_frame_id, aScan.scan->header.stamp, b2o);
        odom = karto::Pose2(
            b2o.getOrigin().x(),
            b2o.getOrigin().y(),
            tf::getYaw(b2o.getRotation()));
        return true;
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Cannot get base to odom tf: %s", ex.what());
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
karto::LaserRangeFinder *DSlamKarto::getRFDevice(const AnnotatedScan& scan)
{
    if (devices_.find(scan.scan->header.frame_id) == devices_.end())
    {
        // device not found
        // create new one
        // 1. get laser to base transformation
        tf::StampedTransform l2b;
        if(!tf_) return NULL;
        try
        {
            tf_->waitForTransform(scan.base_frame_id, scan.scan->header.frame_id,
                                      scan.scan->header.stamp, ros::Duration(0.5));
            tf_->lookupTransform(scan.base_frame_id, scan.scan->header.frame_id,
                                     scan.scan->header.stamp, l2b);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Cannot get laser to base tf: %s", ex.what());
            return NULL;
        }
        // 2.create new device
        karto::LaserRangeFinder *dev =
            karto::LaserRangeFinder::CreateLaserRangeFinder(
                karto::LaserRangeFinder_Custom,
                karto::Name(scan.scan->header.frame_id));
        dev->SetOffsetPose(karto::Pose2(l2b.getOrigin().x(),
                                        l2b.getOrigin().y(),
                                        tf::getYaw(l2b.getRotation())));
        dev->SetMinimumRange(scan.scan->range_min);
        dev->SetMaximumRange(scan.scan->range_max);
        dev->SetMinimumAngle(scan.scan->angle_min);
        dev->SetMaximumAngle(scan.scan->angle_max);
        dev->SetAngularResolution(scan.scan->angle_increment);

        devices_[scan.scan->header.frame_id] = dev;
        database_->Add(dev);
    }

    return devices_[scan.scan->header.frame_id];
}
bool DSlamKarto::registerScan(karto::LaserRangeFinder *dev,
                              const AnnotatedScan& aScan,
                              karto::Pose2 &odom)
{
    std::vector<kt_double> ranges;
    for (std::vector<float>::const_iterator it = aScan.scan->ranges.begin(); it != aScan.scan->ranges.end(); it++)
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