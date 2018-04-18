#include "NonLinearSystemPDF.h"

namespace BFL{
    using namespace MatrixWrapper;
    using namespace Eigen;
    NonLinearSystemPDF::NonLinearSystemPDF(const Gaussian & noise)
    :ConditionalPdf<ColumnVector,ColumnVector>(3,2)
    {
        noise_ = noise;
    }
    NonLinearSystemPDF::~NonLinearSystemPDF(){}

    bool NonLinearSystemPDF::SampleFrom(Sample<ColumnVector>& samp, int method, void * args) const
    {
        // calculate the expected translation from last
        // known pose using the odometry data
        ColumnVector state = ConditionalArgumentGet(0);
        ColumnVector odom_offset = ConditionalArgumentGet(1);
        
        state += odom_offset;
        state(3) = angles::normalize_angle(state(3));
        Sample<ColumnVector> noise;
        noise_.SampleFrom(noise,method, args);

        samp.ValueSet(state + noise.ValueGet());
        return true;
    }
}