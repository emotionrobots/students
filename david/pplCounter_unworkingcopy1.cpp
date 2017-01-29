#define __PPLCOUNTER_CPP__
#include "pplCounter.h"
#include <climits>
#include <algorithm>
#include <iostream>
#include <cmath>   
#include "opencv2/opencv.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
//#include "opencv2/core.hpp"
//#include "opencv2/imgproc.hpp"

pplCounter::pplCounter(int w, int h) : TOFApp(w, h)
{
	_setBackground = false;
	initDisplay();
}

void pplCounter::initDisplay()
{
	namedWindow( "Amplitude", WINDOW_NORMAL );
	namedWindow( "Binary", WINDOW_NORMAL );
	namedWindow( "Morph", WINDOW_NORMAL );
	namedWindow( "Draw", WINDOW_NORMAL );
	namedWindow( "Controls", WINDOW_NORMAL);

	_ampGain = 100;
	_ampThresh = 3;
	_depthThresh = 2;
	_minContourArea = 500;
	_maxContourArea=1000;
	_aspectRatio = 100;

	createTrackbar("Amplitude Gain",   "Controls", &_ampGain, 100);
	createTrackbar("Amplitude Thresh", "Controls", &_ampThresh, 100);
	createTrackbar("Depth Threshold",  "Controls", &_depthThresh, 500);
	createTrackbar("MinContour Area",  "Controls", &_minContourArea, 5000);
	createTrackbar("Aspect Ratio (Y/X)%",  "Controls", &_aspectRatio, 500);
}


Mat pplCounter::clipBackground(float dThr, float iThr)
{
	Mat dMat = Mat::zeros( _iMat.size(), CV_32FC1 );
	Mat fMat = _bkgndMat-_dMat;
	for (int i = 0; i < _dMat.rows; i++) 
	{
		for (int j = 0; j < _dMat.cols; j++) 
		{
			if (fMat.at<float>(i,j) < 0.0f) fMat.at<float>(i,j) = 0.0f;
			dMat.at<float>(i,j) = (_iMat.at<float>(i,j) > iThr &&  fMat.at<float>(i,j) > dThr) ? 255.0 : 0.0;
		}
	}
	return dMat;
}

void pplCounter::resetBackground()
{
   _setBackground = false;
}

//Finds center of contour
cv::Point pplCounter::getCenter(vector<cv::Point> v)
{
	int sumy=0;
	int sumx=0;
	for(int i=0;i<v.size();i++)
	{
		sumy+=v[i].y;
		sumx+=v[i].x;
	}
	int number=v.size();
	float cntrx=(float)sumx/(float)number;
	float cntry=(float)sumy/(float)number;
	cv::Point center=cv::Point(cntrx,cntry);
	return center;
}

//Finds the distance between 2 points
float pplCounter::dist(cv::Point p, cv::Point p2)
{
	float diffx=(float)p2.x-(float)p.x;
	float diffy=(float)p2.y-(float)p.y;
	float _dist=sqrt(diffx*diffx+diffy*diffy);
	return _dist;
}

//Finds the matching index in a _fifo
int pplCounter::pointOfInsertion(vector<Path> p,Path p2)
{
	int match=0;
	for(int i=0;i<p.size();i++)
	{
		//Checking for a match in x and y coordinates
		if(p[i].getPos().x==p2.getPos().x && p[i].getPos().y==p2.getPos().y)
		{
			match=i;
			break;
		}
	}
	return match;
}

//Finds the matching index in a contours vector
int pplCounter::cpointOfInsertion(vector<vector<cv::Point>> p,vector<cv::Point> p2)
{
	int match=0;
	for(int i=0;i<p.size();i++)
	{
		//Checking for a match in x and y coordinates
		if(getCenter(p[i]).x==getCenter(p2).x && getCenter(p[i]).y==getCenter(p2).y)
		{
			match=i;
			break;
		}
	}
	return match;
}

//Finds if a contour is a reasonable person based on cross-sectional area
bool pplCounter::isPerson(vector<cv::Point> p)
{
	if(contourArea(p)>_minContourArea && contourArea(p))
	{
		return true;
	}
	else
	{
		return false;
	}
}

//See if a contour has possibly exited the frame
bool pplCounter::hasExited(Path p)
{
	bool truthval=false;
	//If distance from exit edge is close enough
	//to an arbitrary value
	if(abs((float)p.getPos().x-0.0f)<.1)
	{
		if((float)p.getVel().x<0.0f)
		{
			truthval=true;
		}
	}
	if(abs((float)p.getPos().x-320.0f)<.1)
	{

		if((float)p.getVel().x>0.0f)
		{
			truthval=true;
		}
	}
	if(abs((float)p.getPos().y-0.0f)<.1)
	{
		if((float)p.getVel().y<0.0f)
		{
			truthval=true;
		}
	}
	if(abs((float)p.getPos().y-240.0f)<.1)
	{
		if((float)p.getVel().y>0.0f)
		{
			truthval=true;
		}
	}
	return truthval;
}

//Using kmeans to see if there has been possible clustering
Mat pplCounter::hasClustered(Mat m,int n)
{
	Mat _clustered,_labels;
	kmeans(m,n,_labels,cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT,10,1.0),
			3,KMEANS_RANDOM_CENTERS,_clustered);
	return _clustered;
}
CV_32FC1
//Creates a matrix suitable for input to kmeans 
Mat pplCounter::createKMat(Mat morphMat)
{
	int num=0;
	int _i=0;
	for(int i=0;i<morphMat.rows;i++)
	{
		for(int j=0;j<morphMat.cols;j++)
		{
			if(morphMat.at<float>(i,j)==255.0f)
			{
				num++;
			}
		}
	}
	Mat _k=Mat::zeros(num,2,CV_32FC1);
	for(int i=0;i<morphMat.rows;i++)
	{
		for(int j=0;j<morphMat.cols;j++)
		{
			if(morphMat.at<float>(i,j)==255.0f)
			{
				_k.at<float>(_i,0)=(float)i;
				_k.at<float>(_i,1)=(float)j;
				_i++;
			}
		}
	}
	return _k;
	
}

//Checking if kmeans calculation of centroid positions match
//The original calculations of the contour centers
bool pplCounter::checkCentersMatch(vector<Path> fifo,Mat cntrs)
{
	bool done=false;
	bool truthval=true;
	float minDist=1000.0f;	//A maximum float value
	int pinsertion;
	int cinsertion;
	int miniindex;
	Path min_p;
	cv::Point _targ;
	while(!done)
	{
		for(Path p:fifo)			
		{
			for(int i=0;i<cntrs.rows;i++)
			{
				if(dist(p.getPos(),cv::Point(cntrs.at(i,0),cntrs.at(i,1)))<minDist)
				{
					minDist=dist(p.getPos(),cv::Point(cntrs.at(i,0),cntrs.at(i,1)));	
					min_p=p;
					_targ=cv::Point(cntrs.at(i,0),cntrs.at(i,1));
					miniindex=i;
				}
			}
		}
		//if the minDist exceeds a mininum value, then the centroid is too far
		//from a reasonable previous contour's position, so no clustering has occurred
		if(minDist>.1)
		{
			done=true;
			truthval=false;
		}
		fifo.erase(pointOfInsertion(fifo,min_p));
		cntrs=matErase(miniindex,cntrs);
		//If either vector is empty, stop the while loop
		if(fifo.size()==0 && cntrs.row==0)
		{
			done=true;
		}
	}
	return truthval;
}

//Removes a row from Mat
Mat pplCounter::matErase(int i,Mat cntrs)
{
	Mat matIn=cntrs;    //Matrix of which a row will be deleted
	int row=i;          //Row to delete
	Mat matOut;   //Result: matIn less that one row
	Size size=cntrs.size();
	if (row>0) //Copy everything above that one row
	{
	    cv::Rect rect(0,0,size.width,row);
	    matIn(rect).copyTo(matOut(rect));
	}
	if (row<size.height-1) //Copy everything below that one row
	{
		cv::Rect rect1(0,row+1,size.width,size.height-row-1);
	    cv::Rect rect2(0,row,size.width,size.height-row-1);
		matIn(rect1).copyTo(matOut(rect2));
	}
	return matOut;
}
void pplCounter::update(Frame *frm)
{
	vector< vector<cv::Point> > contours;
	vector<Vec4i> hierarchy;
		 RNG rng(12345);
	if (getFrameType() == DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME) 
	{
		// Create amplitude and depth Mat
		vector<float> zMap, iMap;
		XYZIPointCloudFrame *frm = dynamic_cast<XYZIPointCloudFrame *>(frame);
		for (int i=0; i< frm->points.size(); i++) 
		{
			zMap.push_back(frm->points[i].z);
			iMap.push_back(frm->points[i].i);
		}
		_iMat = Mat(getDim().height, getDim().width, CV_32FC1, iMap.data());
		_dMat = Mat(getDim().height, getDim().width, CV_32FC1, zMap.data()); 
	
		// Apply amplitude gain
		_iMat = (float)_ampGain*_iMat;

		// Update background as required
		if (!_setBackground) 
		{
			_dMat.copyTo(_bkgndMat);
			_setBackground = true;
			cout << endl << "Updated background" << endl;
		}

		// Find foreground by subtraction and convert to binary 
		// image based on amplitude and depth thresholds
		Mat fMat = clipBackground((float)_depthThresh/100.0, (float)_ampThresh/100.0);
		//Cloning fmat for purpose of kmeans algorithm later
		Mat forKMeans=fmat.clone();
		// Apply morphological open to clean up image
		fMat.convertTo(_bMat, CV_8U, 255.0);
		Mat morphMat = _bMat.clone();
		Mat element = getStructuringElement( 0, Size(5,5), cv::Point(1,1) );
		morphologyEx(_bMat, morphMat, 2, element);
	
		// Draw contours that meet a "person" requirement
		Mat drawing = Mat::zeros( _dMat.size(), CV_8UC3 );
		Mat im_with_keypoints = Mat::zeros( _iMat.size(), CV_8UC3 );
		cvtColor(_dMat*0.5, drawing, CV_GRAY2RGB);

		int peopleCount = 0;

		// Find all contours
		findContours(morphMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

		//Running code to log trajectories of contour entities

		//Camera just starts up and logs begin to record
		if(_fifo2.size()==0)
		{
			int basicsize=contours.size();
			_fifo2.resize(basicsize);
			for(int i=0;i<contours.size();i++)
			{
				if(isPerson(contours[i])
				{
					Path input=new Path(300);
					input.addPos(getCenter(contours[i]));
					_fifo2[i].pushback(input);
				}
				else
				{
					_fifo2.resize(basicsize--);
				}
			}
		}
		
		//The general process of trajectory log updating
		else
		{
			//Instantiating vital variables for the below code to run
			bool done=false;
			float minDist=1000.0f;	//A maximum float value
			int pinsertion;
			int cinsertion;
			int numCentroids;
	
			Path insertthing;
			vector<cv::Point> min_c;	
			vector<Path> min_p;
			//c_fifo2 and ccountours will gradually decrease in elements
			//as elements are removed through the loop
			vector<vector<cv::Point>> ccountours(contours);
			vector<Path> c_fifo2(_fifo2);
		
			while(!done)
			{
				for(vector<cv::Point> c:ccountours)
				{
					for(Path p:c_fifo2)
					{
						if(dist(p.getPos(),getCenter(c))<minDist)
						{
							minDist=dist(p.getPos(),getCenter(c));	
							min_c=c;
							min_p=p;
						}
					}
				}
				//Adding updated contour data to _fifo
				pinsertion=pointOfInsertion(_fifo2,min_p);
				cinsertion=cpointOfInsertion(contours,min_c);
				_fifo2[pinsertion].addPos(getCenter(contours[cinsertion]));
				//Erasing matched elements
				c_fifo2.erase(pinsertion);
				ccountours.erase(cinsertion);
				//If either vector is empty, stop the while loop
				if(c_fifo2.size()==0 || ccountours.size()==0)
				{
					done=true;
				}
				
				//Interpret residual elements in either vectors
				if(!ccountours.size()==0)
				{
					//This means new "contours" or people have walked in
					for(vector<cv::Point> c:ccountours)
					{
						if(isPerson(c))
						{
							//If c passes basic human cross sectional area
							//requirements, then it is another human
							insertthing=new Path(300);
							insertthing.addPos(getCenter(c));
							_fifo.push_back(insertthing);
						}
	
					}
				}
				else if(!c_fifo2.size()==0)
				{
					for(Path p:c_fifo2)
					{
						//Testing if some "contours" exited, causing the drop
						//in quantity of contour vector elements
						if(hasExited(p))
						{
							_fifo2.erase(pointOfInsertion(_fifo2,p));
						}
						//Checking if leftover _fifo element has possibly
						//clustered in with another element through kmeans
						else
						{
							Mat inputMat=createKMat(forKMeans);
							Mat cntrs=hasClustered(inputMat,_fifo2.size());
							numCentroids=cntrs.rows;
							//Might need to check with a previous frame version
							//of _fifo2
							if(!checkCentersMatch(_fifo2,cntrs))
							{
								_fifo2.erase(pointOfInsertion(_fifo2,p));
							}
						}
						//Can add further checks with analyzing trajectory history
						//of contour
						
					}
				}
	
				
			}
		}
		peopleCount=_fifo2.size();
		putText(drawing, "Count = "+to_string(peopleCount), cv::Point(200, 50), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));
		imshow("Binary", _bMat);
		imshow("Amplitude", _iMat); 
		imshow("Draw", drawing);
		imshow("Morph", morphMat);
	
	}
}
#undef __PPLCOUNTER_CPP__
