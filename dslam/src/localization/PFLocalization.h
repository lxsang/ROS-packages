#ifndef PFLocalization_H
#define PFLocalization_H

// Particle filter
#include "BaseLocalization.h"
// Particle filter
#include <filter/bootstrapfilter.h>
#include "NonLinearSystemPDF.h"
#include "NonLinearMeasurementPDF.h"

 // Default Number of Samples


namespace dslam {
    using namespace BFL;
    using namespace MatrixWrapper;

    class PFLocalization: public BaseLocalization{
        public:
            PFLocalization();
            virtual ~PFLocalization();
            virtual void configure(Configuration&);
            virtual bool match(const void (*)(std::vector<Line>&, pcl::PointCloud<pcl::PointXYZ>&));

        private:
            void getLastEstimatedPose(dslam_tf_t &);
            std::string odom_frame_;
            dslam_tf_t last_estimate_pose_;
            BootstrapFilter<ColumnVector,ColumnVector>* pf_filter_;
            SystemModel<ColumnVector>* sys_model_;
            MeasurementModel<ColumnVector,ColumnVector>* meas_model_;
            NonLinearSystemPDF* sys_pdf_;
            NonLinearMeasurementPDF* meas_pdf_;
            MCPdf<ColumnVector>* prior_discr_;
    };
}

#endif