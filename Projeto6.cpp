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
			src.copyTo(srcCopy);
			rectangle( srcCopy,p1,p2,Scalar( 0, 0, 255 ),2,3);
			imshow("Defina a regiao", srcCopy); 
		  }	
     }
     
}


char definirRegiao(const Mat& myImage){
	
	clicou1=false;
	clicou2=false;
	int pixSize=0;
	myImage.copyTo(withLine);
	
	namedWindow("Defina a regiao", 1);
		
	setMouseCallback("Defina a regiao", CallBackFunc, NULL);
    imshow("Defina a regiao", withLine);   
    cout<<"Clique duas vezes pra escolher a região."<<endl;
    cout<<"quer refazer? Aperte y se sim."<<endl;
    return waitKey(0);
    
}




/// MAIN

int main( int argc, char** argv )
{
//Mat src, src_gray,dst;
//Mat srcCopy, object;
		
	VideoCapture vcap(0); 
	if(!vcap.isOpened())  
	        return -1;
	
	int frame_width=   vcap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height=   vcap.get(CV_CAP_PROP_FRAME_HEIGHT);
            
	char repeat='y';
	while(repeat=='y'){
		vcap >> src;
		repeat=definirRegiao(src);
	}
	Mat roi(src,Rect(p1,p2));
	
	
	Mat objeto;
	objeto.create(roi.size(),roi.type());
	roi.copyTo(objeto);
	
	Mat objGray;
	cvtColor( objeto, objGray, CV_BGR2GRAY );
	imshow("escolhido",objeto);
cout<<"Objeto escolhido, posicao:"<<p1<<" e "<<p2<<", tamanho="<<p2-p1<<endl;
	
 
	///-------------------Harris	

	double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = true;
  double k = 0.04;
  int maxCorners=100;
  
		// detecting keypoints on object
  
GoodFeaturesToTrackDetector detectorH(maxCorners, qualityLevel,
                                 minDistance, blockSize,
                                 useHarrisDetector, k);                                 
                                 
detectorH.create("HARRIS");
int min_features=40;
int max_features=50;
//int iteracoes=10;
//DynamicAdaptedFeatureDetector ajuste(min_fearutes, max_features, iteracoes,detectorH)
vector<KeyPoint> harris1, harris2;
detectorH.detect(objeto, harris1);

bool sair=false;
if(harris1.size()<=max_features && harris1.size()>=min_features ) sair=true;
while(!sair){
	
	//detectorH.detect(objeto, harris1);
	cout<<"tam before="<<harris1.size()<<endl;
	if(harris1.size()<min_features){
		
		qualityLevel-=0.001;
		if(qualityLevel>0){
			harris1.clear();
			GoodFeaturesToTrackDetector temp(maxCorners, qualityLevel,
                                 minDistance, blockSize,
                                 useHarrisDetector, k);
			temp.detect(objeto, harris1);
			detectorH=temp;
		}else{sair=true;}
		
	}
	if(harris1.size()>max_features){
		qualityLevel+=0.001;
		harris1.clear();
		GoodFeaturesToTrackDetector temp(maxCorners, qualityLevel,
                                 minDistance, blockSize,
                                 useHarrisDetector, k);
        temp.detect(objeto, harris1);
		detectorH=temp;
	}
	for(int i=0;i<harris1.size();i++){
		cout<<harris1[i].pt<<", ";
		}
	cout<<"tam after="<<harris1.size()<<endl;
	
	char q=waitKey(10);	
	if(q=='q') sair=true;	
	if(harris1.size()<=max_features && harris1.size()>=min_features ) sair=true;
	
}

cout<<"Harris, pontos encontrados ="<<harris1.size()<<endl;

// computing descriptors
BriefDescriptorExtractor extractorH;
Mat descriptorsH1, descriptorsH2;
extractorH.compute(objeto, harris1, descriptorsH1);

cout<<"desc="<<descriptorsH1.rows<<" "<<descriptorsH1.cols<<endl;





///---------------SURF

// detecting keypoints
SurfFeatureDetector detectorS(800);
vector<KeyPoint> surf1, surf2;
detectorS.detect(objeto, surf1);

// computing descriptors
SurfDescriptorExtractor extractorS;
Mat descriptorsS1, descriptorsS2;
extractorS.compute(objeto, surf1, descriptorsS1);




///////////////// looooooooooo...ooop
	while(1){
			char q=waitKey(30);
		if (q=='q')	return 0;
	vcap >> src;	
	src.copyTo(srcCopy);
	cvtColor( src, src_gray, CV_BGR2GRAY );
  dst.create( src.size(), src.type() );

double t = (double)getTickCount();
detectorH.detect(srcCopy, harris2);
extractorH.compute(srcCopy, harris2, descriptorsH2);

t = ((double)getTickCount() - t)/getTickFrequency();  
cout << "Tempo harris em segundos: " << t << endl;    
 
 
// matching descriptors
BFMatcher matcherH(NORM_L2);
vector<DMatch> matchesH;
matcherH.match(descriptorsH1, descriptorsH2, matchesH);

if(matchesH.size()>25){
std::nth_element(matchesH.begin(), // initial position
 matchesH.begin()+24, // position of the sorted element
 matchesH.end()); // end position
 // remove all elements after the 25th
 matchesH.erase(matchesH.begin()+25, matchesH.end()); 
}


// drawing the results
namedWindow("matches Harris", 1);
Mat img_matches;
drawMatches(objeto, harris1, srcCopy, harris2, matchesH, img_matches);
imshow("matches Harris", img_matches);

	
	
	

src.copyTo(srcCopy);

t = (double)getTickCount();
detectorS.detect(srcCopy, surf2);
extractorS.compute(srcCopy, surf2, descriptorsS2);

t = ((double)getTickCount() - t)/getTickFrequency();  
cout << "Tempo surf em segundos: " << t << endl;     
// matching descriptors
BFMatcher matcherS(NORM_L2);
vector<DMatch> matchesS;
matcherS.match(descriptorsS1, descriptorsS2, matchesS);

if(matchesS.size()>25){
std::nth_element(matchesS.begin(), // initial position
 matchesS.begin()+24, // position of the sorted element
 matchesS.end()); // end position
 // remove all elements after the 25th
 matchesS.erase(matchesS.begin()+25, matchesS.end()); 
} 

 
 
// drawing the results
namedWindow("matches Surf", 1);
drawMatches(objeto, surf1, srcCopy, surf2, matchesS, img_matches);
imshow("matches Surf", img_matches);



}

  return 0;
  }
