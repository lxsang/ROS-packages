

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;
Mat src; Mat img;
void Dilation( int dilation_elem, int dilation_size )
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( img, img, element );
}

int main( int argc, char** argv )
{
    src = imread( argv[1], 1 );

    /// Convert image to gray and blur it
    cvtColor( src, img, CV_BGR2GRAY );
    threshold(img,img, 0, 255, THRESH_BINARY_INV);
    Dilation(2, 3);
    imshow( "Dilation Demo", img );
    waitKey(0);
    Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    Mat temp;
    Mat eroded;

    Mat element = getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;		
    do
    {
        erode(img, eroded, element);
        dilate(eroded, temp, element); // 
        subtract(img, temp, temp);
        bitwise_or(skel, temp, skel);
        eroded.copyTo(img);

        done = (cv::countNonZero(img) == 0);
    } while (!done);

    //dilate(skel, skel, element);
    //erode(skel, skel, element);
    threshold(skel,skel, 127, 255, THRESH_BINARY_INV);
    imshow( "source", src );
    imshow( "skel", skel );
    waitKey(0);
}