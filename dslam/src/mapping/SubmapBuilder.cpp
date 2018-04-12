#include "SubmapBuilder.h"

namespace dslam
{
void SubmapBuilder::configure(Configuration &config)
{
    p_occupied_w_observation_ = config.get<double>("p_occupied_with_observation", 0.7);
    p_occupied_wo_observation_ = config.get<double>("p_occupied_without_observation", 0.2);
    angle_resolution_ = config.get<double>("angle_resolution", M_PI / 360);
    large_log_odds_ = config.get<double>("large_log_odds", 90.0);
    max_log_odds_for_belief_ = config.get<double>("max_log_odds_for_belief", 30.0);
    int width, height;
    double resolution = config.get<double>("resolution", 0.07);
    width = height = round(config.get<double>("max_laser_range", 8.0)/resolution);
   
    // init the map
    map_.header.frame_id = "laser";
    map_.info.width = width;
    map_.info.height = height;
    map_.info.resolution = resolution;
    map_.info.origin.position.x = 0.0;
    map_.info.origin.position.y = -static_cast<double>(height) / 2 * resolution;
    map_.info.origin.orientation.w = 1.0;
   
}
nav_msgs::OccupancyGrid SubmapBuilder::getMap()
{
    return map_;
}
void SubmapBuilder::fromScan(const sensor_msgs::LaserScan &scan)
{
    const int ncol = map_.info.width;
    
    int width = map_.info.width;
    int height = map_.info.height;
    map_.data.assign(width * height, -1);
    log_odds_.assign(width * height, 0);
    
    
    // Fill in the lookup cache.
    const double angle_start = -M_PI;
    const double angle_end = angle_start + 2 * M_PI - 1e-6;
    for (double a = angle_start; a <= angle_end; a += angle_resolution_)
    {
        ray_caster_.getRayCastToMapBorder(a, height, width, 0.9 * angle_resolution_);
    }
    // Update occupancy.
    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const double angle = angles::normalize_angle(scan.angle_min + i * scan.angle_increment);
        vector<size_t> pts;
        const bool obstacle_in_map = castToObstacle(angle, scan.ranges[i], pts);
        if (pts.empty())
        {
            continue;
        }
        if (obstacle_in_map)
        {
            // The last point is the point with obstacle.
            const size_t last_pt = pts.back();
            updateOccupancy(true, last_pt, map_.data, log_odds_);
            pts.pop_back();
        }
        // The remaining points are in free space.
        updatePoints(false, pts, map_.data, log_odds_);
    }
}
void SubmapBuilder::updatePoints(bool occupied, const vector<size_t> &indexes, vector<int8_t> &occupancy, vector<double> &log_odds)
{
    vector<size_t>::const_iterator idx = indexes.begin();
    for (; idx != indexes.end(); ++idx)
    {
        updateOccupancy(occupied, *idx, occupancy, log_odds);
    }
}
void SubmapBuilder::updateOccupancy(bool occupied, size_t idx, vector<int8_t> &occupancy, vector<double> &log_odds)
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
    double p; // Probability of being occupied knowing current measurement.
    if (occupied)
    {
        p = p_occupied_w_observation_;
    }
    else
    {
        p = p_occupied_wo_observation_;
    }
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
    }
}

bool SubmapBuilder::castToObstacle(double angle, double range, vector<size_t> &raycast)
{
    // Do not consider a 0-length range.
    
    if (range < 1e-10)
    {
        raycast.clear();
        return false;
    }

    const vector<size_t> &ray_to_map_border = ray_caster_.getRayCastToMapBorder(angle,
                                                                                map_.info.height, map_.info.width, 1.1 * angle_resolution_);
    // range in pixel length. The ray length in pixels corresponds to the number
    // of pixels in the bresenham algorithm.
    const size_t pixel_range = lround(range * max(abs(std::cos(angle)), abs(std::sin(angle))) / map_.info.resolution);
    size_t raycast_size;
    bool obstacle_in_map = pixel_range < ray_to_map_border.size();
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
};