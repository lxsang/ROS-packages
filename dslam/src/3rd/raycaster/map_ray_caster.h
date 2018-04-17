#ifndef MAP_RAY_CASTER_MAP_RAY_CASTER_H
#define MAP_RAY_CASTER_MAP_RAY_CASTER_H

#include <math.h> /* for lround, std::lround not in C++99. */
#include <cmath>
#include <map>
#include <vector>

#include <angles/angles.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>


#include "ray_caster_utils.h"

namespace map_ray_caster
{

typedef std::map<double, std::vector<size_t> > RayLookup;

class MapRayCaster
{
  public:

    MapRayCaster(const int occupied_threshold = 60);

    void laserScanCast(const nav_msgs::OccupancyGrid& map, sensor_msgs::LaserScan& scan);

    const std::vector<size_t>& getRayCastToMapBorder(const double angle, const size_t nrow, const size_t ncol, const double tolerance =0);
    const std::vector<size_t>& getRayCastToMapBorderFrom(const double angle, const size_t nrow, const size_t ncol, const double tolerance,  int x0,  int y0);
    size_t lookupSize() const {return raycast_lookup_.size();}
    void setMaxRange(double v){ max_range_ = v; };
  private:

    RayLookup::const_iterator angleLookup(const double angle, const double tolerance);

    int occupied_threshold_;
    size_t ncol_; //!< Map width used in the cache.
    size_t nrow_; //!< Map height used in the cache.
    RayLookup raycast_lookup_;
    double max_range_;
};

} // namespace map_ray_caster

#endif // MAP_RAY_CASTER_MAP_RAY_CASTER_H
