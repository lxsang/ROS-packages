#ifndef NonLinearSystemPDF_H
#define NonLinearSystemPDF_H
#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>
#include <Eigen/Dense>
#include <angles/angles.h>
namespace BFL
{
    class NonLinearSystemPDF: public  ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
    {
        public:
            NonLinearSystemPDF( const Gaussian&);
            virtual ~NonLinearSystemPDF();
            virtual bool SampleFrom (Sample<MatrixWrapper::ColumnVector>& sample, int method=DEFAULT, void * args=NULL) const;

        private:
            Gaussian noise_;
    };
}
#endif