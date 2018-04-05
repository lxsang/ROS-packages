#ifndef SCANMATCHER_H
#define SCANMATCHER_H

#include <pcl/registration/icp.h>
#include <pcl/common/projection_matrix.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include "3rd/line_extraction/line_extraction.h"
#include "utils/luaconf.h"

namespace dslam {
    class ScanMatcher{

        public:
            ScanMatcher(){nscan_ = 0;first_match_ = true;};
            ~ScanMatcher(){};
            void configure(Configuration&);
            void registerScan(const sensor_msgs::LaserScan::ConstPtr &);
            Eigen::Matrix4f match( const void (*)(std::vector<Line>&));
            
        private:
            void cacheData(const sensor_msgs::LaserScan::ConstPtr &);
            pcl::PointCloud<pcl::PointXYZ> linesToPointCloud(std::vector<Line>&);
            pcl::PointCloud<pcl::PointXYZ> last_features_;
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            LineExtraction line_extraction_;
            int nscan_;
            bool first_match_;
    };
}

#endif