#define __PPLCOUNTER_CPP__
#include "pplCounter.h"
#include <climits>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
//#include <cmath> 
using namespace std;

pplCounter::pplCounter(int w, int h) : TOFApp(w, h)
{
	_setBackground=false;
	_willCalibrate=false;
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
	createTrackbar("MaxContour Area",  "Controls", &_maxContourArea, 10000);
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
 
//Finds if a contour is a reasonable person based on cross-sectional area
bool pplCounter::isPerson(vector<cv::Point> &p)
{
	bool rc=false;
	int minX=INT_MAX,minY=INT_MAX;
	int maxX=0,maxY=0,sumX=0,sumY=0;
	int dx=0,dy=0;

   // Find biometric statistics
	for (int i=0; i<p.size(); i++) 
	{
		minX=min(minX,p[i].x);
		minY=min(minY,p[i].y);
		maxX=max(maxX,p[i].x);
		maxY=max(maxY,p[i].y);
		sumX+=p[i].x; 
		sumY+=p[i].y;
	}
	
	dx=maxX-minX;
	dy=maxY-minY;
	
	if(contourArea(p)>_minContourArea && contourArea(p)<_maxContourArea)
	{
		if (dx>0) 
		{
			float ratio=(float)dy/(float)dx;
			//cout << "ratio = " << ratio << endl;
			if (ratio>(float)_aspectRatio/100.0) 
			{
				rc = true;
			}
		}
	}
	else
	{
		rc=false;
	}
	return rc;
}

void pplCounter::update(Frame *frame)
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
			cout << endl << "Updated background for calibration process." << endl;
		}
		if(!_willCalibrate)
		{
			cout << "Please begin your calibration through the trackbar." << endl;
			cout << "Any edits or preferences will be automatically saved at the end." << endl;
			_willCalibrate=true;
		}

		// Find foreground by subtraction and convert to binary 
		// image based on amplitude and depth thresholds
		Mat fMat = clipBackground((float)_depthThresh/100.0, (float)_ampThresh/100.0);
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
		findContours(morphMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, 	cv::Point(0,0));
	
		//Code checking if a contour is a human based upon cross sectional area	
		for(int i=0;i<contours.size();i++)
		{
			if(isPerson(contours[i]))
			{
				peopleCount++;
			}
		}	
		
		putText(drawing, "Count = "+to_string(peopleCount), cv::Point(200, 50), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));
		imshow("Binary", _bMat);
		imshow("Amplitude", _iMat); 
		imshow("Draw", drawing);
		imshow("Morph", morphMat);
		//Writing calibration preferences to a .txt document
		ofstream calibrationSettings;
		calibrationSettings.open("calibrationSettings.txt");
		calibrationSettings << _ampGain << endl;
		calibrationSettings << _depthThresh << endl;
		calibrationSettings << _ampThresh << endl;
		calibrationSettings << _minContourArea << endl;
		calibrationSettings << _maxContourArea << endl;
		calibrationSettings << _aspectRatio << endl;
		calibrationSettings.close();
		
	}
}
#undef __PPLCOUNTER_CPP__
