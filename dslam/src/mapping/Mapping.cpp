#include "Mapping.h"

namespace dslam
{
using namespace Eigen;
Mapping::Mapping()
{
}
Mapping::~Mapping()
{
}
void Mapping::configure(Configuration &cnf)
{
    //submap_builder_.configure(subcnf);
    std::string frame_ = cnf.get<std::string>("map_frame", "/map");
    double init_w = cnf.get<double>("init_width", 1.0);
    double init_h = cnf.get<double>("init_height", 1.0);
    double resolution = cnf.get<double>("resolution", 0.01);

    p_occupied_w_observation_ = cnf.get<double>("p_occupied_with_observation", 0.7);
    p_occupied_wo_observation_ = cnf.get<double>("p_occupied_without_observation", 0.2);
    angle_resolution_ = cnf.get<double>("angle_resolution", M_PI / 360);
    large_log_odds_ = cnf.get<double>("large_log_odds", 90.0);
    max_log_odds_for_belief_ = cnf.get<double>("max_log_odds_for_belief", 30.0);
    max_laser_range_ = cnf.get<double>("max_laser_range", 10.0);
    ray_caster_.setMaxRange(max_laser_range_/resolution);
    int w = round(init_w / resolution);
    int h = round(init_h / resolution);
    map_.header.frame_id = frame_;
    map_.info.width = w;
    map_.info.height = h;
    map_.info.resolution = resolution;
    map_.info.origin.position.x = -static_cast<double>(w) / 2 * resolution;
    map_.info.origin.position.y = -static_cast<double>(h) / 2 * resolution;
    map_.info.origin.position.z = 0.0;

    map_.info.origin.orientation.x = 0.0;
    map_.info.origin.orientation.y = 0.0;
    map_.info.origin.orientation.z = 0.0;
    map_.info.origin.orientation.w = 1.0;
    //map_.data.assign(w * h, -1);
    //log_odds_.assign(w * h, 0);
    // Fill in the lookup cache.
    /*const double angle_start = -M_PI;
    const double angle_end = angle_start + 2 * M_PI - 1e-6;
    for (double a = angle_start; a <= angle_end; a += angle_resolution_)
    {
        ray_caster_.getRayCastToMapBorder(a, h, w, 0.9 * angle_resolution_);
    }*/
}
void Mapping::buildFrom(std::list<key_frame_t> &keyframes)
{
    map_.data.assign(map_.info.width * map_.info.height, -1);
    log_odds_.assign(map_.info.width * map_.info.height, 0);
    auto it = keyframes.begin();
    dslam_tf_t _tf, _tf1;
    _tf.translation = Eigen::Vector3d(0.0, 0.0, 0.0);
    _tf.rotation = Eigen::Quaterniond::Identity();
    while (it != keyframes.end())
    {
        //map_.data.assign(map_.info.width * map_.info.height, -1);
        //log_odds_.assign(map_.info.width * map_.info.height, 0);
        _tf.translation += it->tf.translation;
        _tf.rotation *= it->tf.rotation;

        _tf1.rotation = _tf.rotation*it->base2laser.rotation;
        // rotate base2laser vector around origin
        Vector3d rotated_v;
        double angle = angles::normalize_angle(_tf.rotation.toRotationMatrix().eulerAngles(0,1,2)[2]);
        double x = it->base2laser.translation(0);
        double y = it->base2laser.translation(1);

        rotated_v(0) = (x * cos(angle)) - ( (- y) * sin(angle));
        rotated_v(1) = ((- y) * cos(angle)) - (x  * sin(angle)) ;
        rotated_v(2) = 0.0;
        
        _tf1.translation = _tf.translation + rotated_v;
        
        fromScan(it->scan,_tf1);
        // now merge the submap to the global map using _tf
        //nav_msgs::OccupancyGrid out;
        //rotateSubMap(submaps_[it->index], out, _tf.rotation);
        //resolveMapsize(_tf.translation, out);
        //mergeSubmap(out, _tf.translation);
        it++;
    }
}
nav_msgs::OccupancyGrid Mapping::getMap()
{
    return map_;
}
/*
void Mapping::rotateSubMap(const nav_msgs::OccupancyGrid &in, nav_msgs::OccupancyGrid &out, Quaterniond rot)
{
    Matrix3d mat;
    mat = rot;
    int size = round(sqrt(2 * pow(in.info.width, 2)));
    out.header = in.header;
    out.info.width = size;
    out.info.height = size;
    out.info.resolution = in.info.resolution;
    out.info.origin.position.x = 0.0;
    out.info.origin.position.y = (double)((size / 2.0) * in.info.resolution);
    out.info.origin.position.z = 0.0;

    out.info.origin.orientation.x = 0.0;
    out.info.origin.orientation.y = 0.0;
    out.info.origin.orientation.z = 0.0;
    out.info.origin.orientation.w = 1.0;
    out.data.resize(size * size, -1);
    // now convert each pixel to coordinate
    // rotate it, then corvert the result to
    // pixel position
    double offset = (double)in.info.width / 2.0;
    double roffset = (double)size / 2.0;
    Vector3d xy, rxy;
    for (int i = 0; i < in.info.width; i++)
        for (int j = 0; j < in.info.height; j++)
        {
            xy(0) = ((double)i - offset) * in.info.resolution;
            xy(1) = ((double)j - offset) * in.info.resolution;
            xy(2) = 0.0;
            rxy = mat * xy;
            int ri = round(rxy(0) / in.info.resolution + roffset);
            int rj = round(rxy(1) / in.info.resolution + roffset);
            if (ri < 0 || rj < 0)
            {
                printf("Warning: pixel index (%d,%d) is invalid\n", ri, rj);
                continue;
            }
            out.data[ri + rj * size] = in.data[i + j * in.info.width];
        }
}
void Mapping::resolveMapsize(Eigen::Vector3d theirpose, const nav_msgs::OccupancyGrid &msg)
{
    double minx, miny, maxx, maxy;
    double tx, ty, lx, ly;
    geometry_msgs::Point delta;

    delta.x = theirpose(0);
    delta.y = theirpose(1);
    tx = msg.info.origin.position.x + delta.x;
    ty = msg.info.origin.position.y + delta.y;

    minx = map_.info.origin.position.x < tx ? map_.info.origin.position.x : tx;
    miny = map_.info.origin.position.y < ty ? map_.info.origin.position.y : ty;

    tx = (double)msg.info.width * msg.info.resolution - fabs(msg.info.origin.position.x);
    tx = tx + delta.x;

    ty = (double)msg.info.height * msg.info.resolution - fabs(msg.info.origin.position.y);
    ty = ty + delta.y;

    lx = (double)map_.info.width * map_.info.resolution - fabs(map_.info.origin.position.x);
    ly = (double)map_.info.height * map_.info.resolution - fabs(map_.info.origin.position.y);

    maxx = lx > tx ? lx : tx;
    maxy = ly > ty ? ly : ty;

    int w = round((double)(maxx - minx) / map_.info.resolution);
    int h = round((double)(maxy - miny) / map_.info.resolution);
    if (w > map_.info.width || h > map_.info.height)
    {
        map_.info.width = w;
        map_.info.height = h;
        map_.info.origin.position.x = minx;
        map_.info.origin.position.y = miny;
    }
}*/
void Mapping::fromScan(const sensor_msgs::LaserScan &scan, dslam_tf_t& _tf)
{
    const int ncol = map_.info.width;

    int width = map_.info.width;
    int height = map_.info.height;
    //log_odds_.assign(map_.info.width * map_.info.height, 0);
    // Update occupancy.
    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const double angle = angles::normalize_angle(scan.angle_min + i * scan.angle_increment);
        vector<size_t> pts;
        const bool obstacle_in_map = castToObstacle(angle, scan.ranges[i], pts, _tf);
        if (pts.empty())
        {
            continue;
        }
        if (obstacle_in_map)
        {
            // The last point is the point with obstacle.
            const size_t last_pt = pts.back();
            updateOccupancy(p_occupied_w_observation_, last_pt, map_.data, log_odds_);
            pts.pop_back();
            updatePoints(p_occupied_wo_observation_  ,false, pts, map_.data, log_odds_);
        } 
        else
        {
            updatePoints(p_occupied_wo_observation_  ,true, pts, map_.data, log_odds_);
        }
        // The remaining points are in free space.
        
    }
}
void Mapping::updatePoints(double p, bool desc, const vector<size_t> &indexes, vector<int8_t> &occupancy, vector<double> &log_odds)
{
    double _p = p;
    int s = indexes.size();
    s = desc?s/2:s;
    for (int i = 0; i < s; i++)
    {
        if(desc)
        {
           _p = 0.5*i/s;
        }
        updateOccupancy(_p, indexes[i], occupancy, log_odds);
    }
}
void Mapping::updateOccupancy(double p, size_t idx, vector<int8_t> &occupancy, vector<double> &log_odds)
{
    if (idx >= occupancy.size())
    {
        return;
    }

    if (occupancy.size() != log_odds.size())
    {
        ROS_ERROR("occupancy and count do not have the same number of elements");
        return;
    }

    // Update log_odds.
    /*double p; // Probability of being occupied knowing current measurement.
    if (occupied)
    {
        p = p_occupied_w_observation_;
    }
    else
    {
        p = p_occupied_wo_observation_;
    }*/
    // Original formula: Table 4.2, "Probabilistics robotics", Thrun et al., 2005:
    // log_odds[idx] = log_odds[idx] +
    //     std::log(p * (1 - p_occupancy) / (1 - p) / p_occupancy);
    // With p_occupancy = 0.5, this simplifies to:
    log_odds[idx] += std::log(p / (1 - p));
    if (log_odds[idx] < -large_log_odds_)
    {
        log_odds[idx] = -large_log_odds_;
    }
    else if (log_odds[idx] > large_log_odds_)
    {
        log_odds[idx] = large_log_odds_;
    }
    // Update occupancy.
    if (log_odds[idx] < -max_log_odds_for_belief_)
    {
        occupancy[idx] = 0;
    }
    else if (log_odds[idx] > max_log_odds_for_belief_)
    {
        occupancy[idx] = 100;
    }
    else
    {
        occupancy[idx] = static_cast<int8_t>(lround((1 - 1 / (1 + std::exp(log_odds[idx]))) * 100));
        /*if(occupancy[idx] > 60)
            occupancy[idx] = 100;
        else if(occupancy[idx] < 40)
            occupancy[idx] = 0;
        else
        {
            log_odds[idx] = 0;
            occupancy[idx] = -1;
        }*/
    }
}

bool Mapping::castToObstacle(double angle, double range, vector<size_t> &raycast,  dslam_tf_t& _tf)
{
    // Do not consider a 0-length range.

    if (range < 1e-10)
    {
        raycast.clear();
        return false;
    }
    Vector3d pos = _tf.translation;
    int x0 = round(pos(0)/map_.info.resolution + map_.info.width/2);
    int y0 = round(pos(1)/map_.info.resolution + map_.info.height/2);
    double yaw = angles::normalize_angle(angle + _tf.rotation.toRotationMatrix().eulerAngles(0,1,2)[2]);
    const vector<size_t> &ray_to_map_border = ray_caster_.getRayCastToMapBorderFrom(yaw,
                                                                                map_.info.height, map_.info.width, 1.1 * angle_resolution_, x0, y0);
    // range in pixel length. The ray length in pixels corresponds to the number
    // of pixels in the bresenham algorithm.
    const size_t pixel_range = round(range * max(abs(std::cos(yaw)), abs(std::sin(yaw))) / map_.info.resolution);
    size_t raycast_size;
    bool obstacle_in_map = pixel_range < ray_to_map_border.size();
    //printf(" RANGE IS %d and obstacle_in_map IS %d\n ", pixel_range, ray_to_map_border.size() );
    if (obstacle_in_map)
    {
        raycast_size = pixel_range;
    }
    else
    {
        raycast_size = ray_to_map_border.size();
    }
    raycast.clear();
    raycast.reserve(raycast_size);
    for (size_t i = 0; i < raycast_size; ++i)
    {
        raycast.push_back(ray_to_map_border[i]);
    }

    return obstacle_in_map;
}
}