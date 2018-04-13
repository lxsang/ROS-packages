#include "NonLinearMeasurementPDF.h"


namespace BFL{
    using namespace MatrixWrapper;
    NonLinearMeasurementPDF::NonLinearMeasurementPDF(const Gaussian& noise)
    :ConditionalPdf<ColumnVector, ColumnVector>(3,1)
    {
        noise_ = noise;
    }
    Probability NonLinearMeasurementPDF::ProbabilityGet(const ColumnVector& measurement) const
    {
        ColumnVector state = ConditionalArgumentGet(0);

        return noise_.ProbabilityGet(measurement);
    }
}