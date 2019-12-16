#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include <iostream>
#include <sstream>

using namespace std;
using namespace cv;




class HarrisDetector {
	private:
 // 32-bit float image of corner strength
 cv::Mat cornerStrength;
 // 32-bit float image of thresholded corners
 cv::Mat cornerTh;
  // image of local maxima (internal)
 cv::Mat localMax;
 // size of neighborhood for derivatives smoothing
 int neighbourhood;
 // aperture for gradient computation
 int aperture;
 // Harris parameter
 double k;
 // maximum strength for threshold computation
 double maxStrength;
 // calculated threshold (internal)
 double threshold;
 // size of neighborhood for non-max suppression
 int nonMaxSize;
 // kernel for non-max suppression
 cv::Mat kernel;
 public:
 HarrisDetector() : neighbourhood(3), aperture(3),
 k(0.01), maxStrength(0.0),
 threshold(0.01), nonMaxSize(3) {
 // create kernel used in non-maxima suppression
 setLocalMaxWindowSize(nonMaxSize);
}

void setLocalMaxWindowSize(int a){
	nonMaxSize=a;
	return;
	}
// Compute Harris corners
 void detect(const cv::Mat& image) {
 // Harris computation
 cv::cornerHarris(image,cornerStrength,
 neighbourhood,// neighborhood size
 aperture, // aperture size
 k); // Harris parameter
 // internal threshold computation
 double minStrength; // not used
 cv::minMaxLoc(cornerStrength,
 &minStrength,&maxStrength);
 // local maxima detection
 cv::Mat dilated; // temporary image
 cv::dilate(cornerStrength,dilated,cv::Mat());
 cv::compare(cornerStrength,dilated,
 localMax,cv::CMP_EQ);
 } 
 
 // Get the corner map from the computed Harris values
 cv::Mat getCornerMap(double qualityLevel) {
 cv::Mat cornerMap;
 // thresholding the corner strength
 threshold= qualityLevel*maxStrength;
 cv::threshold(cornerStrength,cornerTh,
 threshold,255,cv::THRESH_BINARY);
 // convert to 8-bit image
 cornerTh.convertTo(cornerMap,CV_8U);
 // non-maxima suppression
 cv::bitwise_and(cornerMap,localMax,cornerMap);
 return cornerMap;
 }
 
 // Get the feature points from the computed Harris values
 void getCorners(std::vector<cv::Point> &points,
 double qualityLevel) {
 // Get the corner map
 cv::Mat cornerMap= getCornerMap(qualityLevel);
 // Get the corners
 getCorners(points, cornerMap);
 }
 
 // Get the feature points from the computed corner map
 void getCorners(std::vector<cv::Point> &points,
 const cv::Mat& cornerMap) {
 // Iterate over the pixels to obtain all features
 for( int y = 0; y < cornerMap.rows; y++ ) {
 const uchar* rowPtr = cornerMap.ptr<uchar>(y);
 for( int x = 0; x < cornerMap.cols; x++ ) {
 // if it is a feature point
 if (rowPtr[x]) {
	  points.push_back(cv::Point(x,y));
 }
 }
 }
 }
 
  // Draw circles at feature point locations on an image
 void drawOnImage(cv::Mat &image,
 const std::vector<cv::Point> &points,
 cv::Scalar color= cv::Scalar(255,255,255),
 int radius=3, int thickness=2) {
 std::vector<cv::Point>::const_iterator it=
 points.begin();
 // for all corners
 while (it!=points.end()) {
 // draw a circle at each corner location
 cv::circle(image,*it,radius,color,thickness);
 ++it;
 }
 }
 
 
};
/// Global variables


/** @function main */
int main( int argc, char** argv )
{
Mat src, src_gray,dst;
Mat srcCopy;
		
	VideoCapture vcap(0); 
	if(!vcap.isOpened())  
	        return -1;
	vcap >> src;
	
	
	
	char q=waitKey(30);
	while(1){
		if (q=='q')	return 0;
	vcap >> src;	
	src.copyTo(srcCopy);
	cvtColor( src, src_gray, CV_BGR2GRAY );
  dst.create( src.size(), src.type() );
	
///-------------------Harris	
	
	// Create Harris detector instance
 HarrisDetector harris;
 // Compute Harris values
 harris.detect(src_gray);
 // Detect Harris corners
 std::vector<cv::Point> pts;
 harris.getCorners(pts,0.01);
 // Draw Harris corners
 harris.drawOnImage(srcCopy,pts);
 imshow("with circles 1",srcCopy);
 src.copyTo(srcCopy);
 
///-------------------Good features to track 
 
 // Compute good features to track
 cvtColor( src, src_gray, CV_BGR2GRAY );
  std::vector<cv::Point> corners;
 cv::goodFeaturesToTrack(src_gray,corners,
 50, // maximum number of corners to be returned
 0.01, // quality level
 10); // minimum allowed distance between points
 harris.drawOnImage(srcCopy,corners);

 imshow("with circles 2",srcCopy);
 src.copyTo(srcCopy);


///-------------------surf 
 cvtColor( src, src_gray, CV_BGR2GRAY );
 
  // vector of keypoints
 std::vector<cv::KeyPoint> keypoints;
 // Construct the SURF feature detector object
 cv::SurfFeatureDetector surf(
 2500.); // threshold
 // Detect the SURF features
 surf.detect(src_gray,keypoints);
 
 Mat featureImage;
 Mat featureImage2;
 // Draw the keypoints with scale and orientation information
 cv::drawKeypoints(srcCopy, // original image
 keypoints, // vector of keypoints
 featureImage, // the resulting image
 cv::Scalar(255,255,255), // color of the points
 cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); //flag
 imshow("with circles 3",featureImage);
 src.copyTo(srcCopy);
 cv::drawKeypoints(srcCopy, // original image
 keypoints, // vector of keypoints
 featureImage2, // the resulting image
 cv::Scalar(255,255,255), // color of the points
 cv::DrawMatchesFlags::DEFAULT); //flag
 imshow("with circles 4",featureImage2);
  q=waitKey(30);
 
}
	// Detect Harris Corners
	cv::Mat cornerStrength;
	cv::cornerHarris(src_gray,cornerStrength,
	3, // neighborhood size
	3, // aperture size
	0.01); // Harris parameter
	// threshold the corner strengths
	
	imshow("step1",cornerStrength);
	waitKey(0);
	cv::Mat harrisCorners;
	double threshold= 0.0001;
	cv::threshold(cornerStrength,harrisCorners,
	threshold,255,cv::THRESH_BINARY);
	imshow("step2",harrisCorners);
	waitKey(0);
 


  return 0;
  }
