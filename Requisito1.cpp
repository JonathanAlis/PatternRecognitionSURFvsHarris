#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include <iostream>
#include <sstream>

using namespace std;
using namespace cv;



Point p1;
Point p2;
bool clicou1;
bool clicou2;
int pixSize;
Mat withLine;
Mat withGrid;
Mat frame;
Mat src, src_gray,dst;
Mat srcCopy;

void CallBackFunc(int event, int x, int y, int flags, void* userdata){
     if  ( event == EVENT_LBUTTONDOWN ){          
		  if(clicou1 && !clicou2){
			p2=Point(x,y);
			clicou2=true;
			cout << "Segundo clique: (" << x << ", " << y << ")" << endl;
			pixSize=sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
			rectangle( withLine,p1,p2,Scalar( 255, 255, 255 ),2,3);		
			imshow("Defina a regiao", withLine);   
		  }
		  
		  if(!clicou1){
			p1=Point(x,y);
			clicou1=true;
			cout << "Primeiro clique: (" << x << ", " << y << ")" << endl;
		  }
     }
     else if ( event == EVENT_MOUSEMOVE )     {
          //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
		  if(clicou1 && !clicou2){
			p2=Point(x,y);			
			pixSize=sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
			//line( withLine,p1,p2,Scalar( 255, 255, 255 ),2,3);
			//imshow("ClickToSize", withLine);   
		  }	
     }
     
}


void definirRegiao(const Mat& myImage){
	char repeat='y';
	
	while (repeat=='y'){
	clicou1=false;
	clicou2=false;
	int pixSize=0;
	myImage.copyTo(withLine);
	
	namedWindow("Defina a regiao", 1);
	//set the callback function for any mouse event
	
	setMouseCallback("Defina a regiao", CallBackFunc, NULL);
    
    //rectangle( withLine,p1,p2,Scalar( 255, 255, 255 ),2,3);
    
    imshow("Defina a regiao", withLine);   
    cout<<"quer refazer? Aperte y se sim."<<endl;
    repeat=waitKey(0);
	}
    
    
}




/// classe harris detector


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
















/// MAIN

int main( int argc, char** argv )
{
//Mat src, src_gray,dst;
//Mat srcCopy, object;
		
	VideoCapture vcap(0); 
	if(!vcap.isOpened())  
	        return -1;
	vcap >> src;

	int frame_width=   vcap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height=   vcap.get(CV_CAP_PROP_FRAME_HEIGHT);
	
	definirRegiao(src);
	
	Mat roi(src,Rect(p1,p2));
	Mat objeto;
	objeto.create(roi.size(),roi.type());
	roi.copyTo(objeto);
	
	Mat objGray;
	cvtColor( objeto, objGray, CV_BGR2GRAY );
	imshow("escolhido",objeto);

	vector<Point> harris_points;
	vector<Point> GFtT_points;
	vector<KeyPoint> surf_points;
	
	
	// Construction of the SURF descriptor extractor
 cv::SurfDescriptorExtractor surfDesc;
 // Extraction of the SURF descriptors
 cv::Mat descriptors1, descriptors2;
 
 
 
	///-------------------Harris	
	{
		Mat objCopy;
	objeto.copyTo(objCopy);
	// Create Harris detector instance
 HarrisDetector harris;
 // Compute Harris values
 harris.detect(objGray);
 // Detect Harris corners
 std::vector<cv::Point> pts;
 harris.getCorners(pts,0.01);
 // Draw Harris corners
 harris.drawOnImage(objCopy,pts);
 imshow("harris on object",objCopy);
 objeto.copyTo(objCopy);
	
	harris_points.swap(pts);
///-------------------Good features to track 
 cvtColor( objeto, objGray, CV_BGR2GRAY );
 // Compute good features to track
 std::vector<cv::Point> corners;
 cv::goodFeaturesToTrack(objGray,corners,
 50, // maximum number of corners to be returned
 0.01, // quality level
 10); // minimum allowed distance between points
 harris.drawOnImage(objCopy,corners);

 imshow("good features to track on object",objCopy);
 objeto.copyTo(objCopy);
GFtT_points.swap(corners);	
	
///-------------------surf 
 cvtColor( objeto, objGray, CV_BGR2GRAY ); 
  // vector of keypoints
 std::vector<cv::KeyPoint> keypoints;
 // Construct the SURF feature detector object
 cv::SurfFeatureDetector surf(
 2500.); // threshold
 // Detect the SURF features
 surf.detect(objGray,keypoints);
 surfDesc.compute(objCopy,keypoints,descriptors1);
 
 Mat featureImage;
 Mat featureImage2;
 // Draw the keypoints with scale and orientation information
 cv::drawKeypoints(objCopy, // original image
 keypoints, // vector of keypoints
 featureImage, // the resulting image
 cv::Scalar(255,255,255), // color of the points
 cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); //flag
 imshow("surf, rich points",featureImage);
 objeto.copyTo(objCopy);
  cv::drawKeypoints(objCopy, // original image
 keypoints, // vector of keypoints
 featureImage2, // the resulting image
 cv::Scalar(255,255,255), // color of the points
 cv::DrawMatchesFlags::DEFAULT); //flag
 
 
 
 imshow("surf",featureImage2);
	surf_points.swap(keypoints);
}
cout<<"Pontos gerados no objeto escolhido."<<endl;

///////////////// looooooooooo...ooop
	while(1){
			char q=waitKey(30);
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
 surfDesc.compute(src_gray,keypoints,descriptors2);
 
 
 
  BFMatcher matcher(NORM_L2);
  std::vector< DMatch > matches;
  matcher.match( descriptors1, descriptors2, matches );

  //-- Draw matches
  Mat img_matches;
  drawMatches( objeto, surf_points, src, keypoints, matches, img_matches );

  //-- Show detected matches
  imshow("Surf Matches", img_matches );
  
  
  
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
  
  for(int i=0;i<keypoints.size();i++){
	  cout<<"(x,y)=("<<keypoints[i].pt.x<<","<<keypoints[i].pt.y<<")"<<endl;
	  cout<<"size= "<<keypoints[i].size<<endl;
	  cout<<"angle= "<<keypoints[i].angle<<endl;
	  cout<<"response= "<<keypoints[i].response<<endl;
	  cout<<"octave= "<<keypoints[i].octave<<endl;
	  cout<<"class_id= "<<keypoints[i].class_id<<endl<<endl;
	  }
 
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
