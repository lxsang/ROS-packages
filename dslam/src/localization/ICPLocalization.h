#ifndef ICP_LOCALIZATION_H
#define ICP_LOCALIZATION_H
#include "BaseLocalization.h"
namespace dslam {
    class ICPLocalization: public BaseLocalization{

        public:
            ICPLocalization();
            virtual ~ICPLocalization(){};
            virtual void configure(Configuration&);
            virtual void visualize(ros::Publisher&){};
        protected:
            virtual bool __match(const void (*)(std::vector<Line>&, pcl::PointCloud<pcl::PointXYZ>&));

        private:
            void alignLastFeature(pcl::PointCloud<pcl::PointXYZ>&);
    };
}

#endif