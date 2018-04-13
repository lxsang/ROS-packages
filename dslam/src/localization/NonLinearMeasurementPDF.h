#ifndef NonLinearMeasurementPDF_H
#define NonLinearMeasurementPDF_H


#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>
#include "common.h"
namespace BFL {
    class NonLinearMeasurementPDF: public ConditionalPdf<MatrixWrapper::ColumnVector,MatrixWrapper::ColumnVector>
    {
        public:
            NonLinearMeasurementPDF(const Gaussian&);
            virtual ~NonLinearMeasurementPDF(){};
            virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;
        private:
            Gaussian noise_;
    };
} 


#endif