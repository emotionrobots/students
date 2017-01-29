#define __PPLCOUNTER_CPP__
#include "pplCounterCalibration.h"
#include <climits>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath> 
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;

pplCounter::pplCounter(int w, int h) : TOFApp(w, h)
{
	_setBackground=false;
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
	_minDist=30;
	_checkminDist=0;
	_multiplier=1;
	_analyzeBackground=1;
	
	createTrackbar("Amplitude Gain",   "Controls", &_ampGain, 100);
	createTrackbar("Amplitude Thresh", "Controls", &_ampThresh, 100);
	createTrackbar("Depth Threshold",  "Controls", &_depthThresh, 500);
	createTrackbar("Depth Threshold Max",  "Controls", &_depthThreshmax, 500);
	createTrackbar("Depth mat multiplier",  "Controls", &_multiplier,100);
	createTrackbar("MinContour Area",  "Controls", &_minContourArea, 5000);
	createTrackbar("MaxContour Area",  "Controls", &_maxContourArea, 10000);
	createTrackbar("Aspect Ratio (Y/X)%",  "Controls", &_aspectRatio, 500);
	createTrackbar("Mininum Distance",  "Controls", &_checkminDist, 1);
	createTrackbar("Analzye Background",  "Controls", &_analyzeBackground, 1);

	cout << "This is the calibration program for the people counter device." << endl;
	cout << "Please alter the trackbar to your setting." << endl;
	cout << "When you are prepared to determine minDist, please slide the trackbar to 1." << endl;
	cout << "Then have a person walk across the frame, and press ctrl c immediately afterwards to automatically save data." << endl;
}	


Mat pplCounter::clipBackground(float dThr,float iThr,float dThrTop)
{
	Mat dMat=Mat::zeros(_iMat.size(),CV_32FC1);
	Mat fMat=_bkgndMat-_dMat;

	
	for(int i=0;i<_dMat.rows;i++) 
	{
		for(int j=0;j<_dMat.cols;j++) 
		{
			if(fMat.at<float>(i,j)<0.0f) 
			{
				fMat.at<float>(i,j)=0.0f;
			}
			dMat.at<float>(i,j)=(_iMat.at<float>(i,j)>=iThr && (fMat.at<float>(i,j)>dThr && fMat.at<float>(i,j)<dThrTop)) ? 255.0:0.0;
		}
	}
	return dMat;
}

void pplCounter::resetBackground()
{
   _setBackground=false;
}
 
//Finds if a contour is a reasonable person based on cross-sectional area
bool pplCounter::isPerson(vector<cv::Point> p)
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

//Gets the center of a contour
cv::Point pplCounter::getCenter(vector<cv::Point> v)
{
	int sumy=0;
	int sumx=0;
	cv::Point center;
	for(int i=0;i<v.size();i++)
	{
		sumy+=v[i].y;
		sumx+=v[i].x;
	}
	int number=v.size();
	if(number>0)
	{
		float cntrx=(float)sumx/(float)number;
		float cntry=(float)sumy/(float)number;
		center=cv::Point(cntrx,cntry);
	}
	else
	{
		center=cv::Point(0,0);
	}
	return center;
}

//Finds distance between 2 points
float pplCounter::dist(cv::Point p, cv::Point p2)
{
	float diffx=(float)p2.x-(float)p.x;
	float diffy=(float)p2.y-(float)p.y;
	float _dist=sqrt(diffx*diffx+diffy*diffy);
	return _dist;
}

//Finds the position where an element fits in a vector
int pplCounter::getIndex(vector<Path*> t,Path *p)
{
	int index=0;
	for(int i=0;i<t.size();i++)
	{
		if(dist(t[i]->getPos(),p->getPos())==0)
		{
			index=i;
			break;	
		}
	}
	return index;
	
}

int pplCounter::getIndex(vector<vector<cv::Point>> t,vector<cv::Point> p)
{
	int index=0;
	for(int i=0;i<t.size();i++)
	{
		if(dist(getCenter(t[i]),getCenter(p))==0)
		{
			index=i;
			break;	
		}
	}
	return index;
	
}

int pplCounter::getIndex(vector<cv::Point> t,cv::Point p)
{
	int index=0;
	for(int i=0;i<t.size();i++)
	{
		if(t[i].x==p.x && t[i].y==p.y)
		{
			index=i;
			break;	
		}
	}
	return index;
	
}

//Sees if a trajectory of a contour has the potential of exiting
bool pplCounter::hasExited(Path *p)
{
	bool truthval=false;
	//If distance from exit edge is close enough
	//to an arbitrary value
	if(abs((float)p->getPos().x-0.0f)<_minDist/4)
	{
		if((float)p->getVel().x<0.0f)
		{
			truthval=true;
		}
	}
	if(abs((float)p->getPos().x-160.0f)<_minDist/4)
	{

		if((float)p->getVel().x>0.0f)
		{
			truthval=true;
		}
	}
	if(abs((float)p->getPos().y-0.0f)<_minDist/4)
	{
		if((float)p->getVel().y<0.0f)
		{

			truthval=true;
		}
	}
	if(abs((float)p->getPos().y-120.0f)<_minDist/4)
	{
		if((float)p->getVel().y>0.0f)
		{
			truthval=true;
		}
	}
	return truthval;
}

//Creates a matrix suitable for input to kmeans 
Mat pplCounter::createKMat(Mat morphMat2)
{
	int num=0;
	int _i=0;
	for(int i=0;i<morphMat2.rows;i++)
	{
		for(int j=0;j<morphMat2.cols;j++)
		{
			if(morphMat2.at<float>(i,j)==255.0)
			{
				num++;
			}
		}
	}
	Mat _k=Mat::zeros(num,2,CV_32F);
	for(int i=0;i<morphMat2.rows;i++)
	{
		for(int j=0;j<morphMat2.cols;j++)
		{
			if(morphMat2.at<float>(i,j)==255.0)
			{
				_k.at<float>(_i,0)=(float)i;
				_k.at<float>(_i,1)=(float)j;
				_i++;
			}
		}
	}
	return _k;
	
}


//Using kmeans to see if there has been possible clustering
vector<cv::Point> pplCounter::getClusterCenters(Mat m,int n)
{
	Mat _clustered,_labels;
	vector<cv::Point> container;
	kmeans(m,n,_labels,cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT,100,1.0),
			3,KMEANS_RANDOM_CENTERS,_clustered);
	for(int i=0;i<_clustered.rows;i++)
	{
		container.push_back(cv::Point(_clustered.at<float>(i,1),_clustered.at<float>(i,0)));
	}
	return container;
}

//Checks for potential collision between 2 contours
bool pplCounter::hasCollided(Path *traj,Path *traj2)
{
	if(traj->getVel().x+traj2->getVel().x<abs(traj->getVel().x)+abs(traj2->getVel().x) || traj->getVel().y+traj2->getVel().y<abs(traj->getVel().y)+abs(traj2->getVel().y))
	{
		return true;
	}
	else
	{
		return false;
	}
}

//Checking if kmeans calculation of centroid positions match
//The original calculations of the contour centers
bool pplCounter::checkCentersMatch(vector<Path*> pop,vector<cv::Point> cntrs)
{
	bool done=false;
	bool truthval=true;
	float minDist=_minDist;
	int miniindex;
	Path *min_p=new Path(300);
	cv::Point min_cntr;
	while(!done)
	{
		for(Path *traj:pop)		
		{
			for(cv::Point cntr:cntrs)
			{
				if(dist(traj->getPos(),cntr)<minDist)
				{
					minDist=dist(traj->getPos(),cntr);
					min_p=traj;
					min_cntr=cntr;
				}
			}
		}
		//if the minDist exceeds a mininum value, then the centroid is too far
		//from a reasonable previous contour's position, so no clustering has occurred
		if(minDist==_minDist)
		{
			done=true;
			truthval=false;
		}
		
		//Account for residual again
		pop.erase(pop.begin()+getIndex(pop,min_p));
		cntrs.erase(cntrs.begin()+getIndex(cntrs,min_cntr));

		//If either vector is empty, stop the while loop
		if(pop.size()==0 && cntrs.size()==0)
		{
			done=true;
		}
	}
	return truthval;
}

int pplCounter::getCloseIndex(Path *traj,vector<cv::Point> cntrs)
{	
	int index=0;
	for(int i=0;i<cntrs.size();i++)
	{
		if(dist(traj->getPos(),cntrs[i])<_minDist)
		{
			index=i;
			break;
		}
	}
	return index;

}

int pplCounter::numPersons(vector<cv::Point> v)
{
	int num=1;
	for(int i=2;i<20;i++)
	{
		if(contourArea(v)/i<_maxContourArea && contourArea(v)/i>_minContourArea)
		{
			num=i;
		}
	}
	return num;
}
void pplCounter::update(Frame *frame)
{
	cout << "running" << endl;
	vector< vector<cv::Point> > contours;
	vector<Vec4i> hierarchy;
	RNG rng(12345);
	bool passed=false;
	bool done=false;
	int xc=0;
	float totalIntensity=0;
	float totalElevation=0;
	ofstream depthMatData;
	ofstream intMatData;
	
	if (getFrameType()==DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME) 
	{
		// Create amplitude and depth Mat
		vector<float> zMap, iMap;
		XYZIPointCloudFrame *frm=dynamic_cast<XYZIPointCloudFrame *>(frame);
		if(_analyzeBackground==1)
		{
			//depthMatData.open("depthMatData.txt");
			//intMatData.open("intMatData.txt");
			for (int i=0;i< frm->points.size();i++) 
			{
				zMapCalib.push_back(frm->points[i].z);
				iMapCalib.push_back(frm->points[i].i);
				//depthMatData << frm->points[i].z << endl;
				//intMatData << frm->points[i].i << endl;
			}
			//intMatData.close();
			//depthMatData.close();
			_iMat=Mat(getDim().height,getDim().width,CV_32F,iMapCalib.data());
			_dMat=Mat(getDim().height,getDim().width,CV_32F,zMapCalib.data());
			/*for(int i=0;i<frm->points.size();i++) 
			{
				totalIntensity+=frm->points[i].z;
				totalElevation+=frm->points[i].i;
			}
			avgIntensity=(float)totalIntensity/frm->points.size();
			avgElevation=(float)totalElevation/frm->points.size();	
			for(int i=0;i<frm->points.size();i++) 
			{
				zMap.push_back(avgElevation);
				iMap.push_back(avgIntensity);
			}
			_iMat=Mat(getDim().height,getDim().width,CV_32F,iMap.data());
			_dMat=Mat(getDim().height,getDim().width,CV_32F,zMap.data()); 
			cout << avgElevation << endl;*/
		}
		else
		{
			for(int i=0;i<frm->points.size();i++) 
			{
				if(abs(frm->points[i].i-iMapCalib[i])>.01)
				{
					iMap.push_back(frm->points[i].i);
				}
				if(abs(frm->points[i].i-iMapCalib[i])<=.01)
				{
					iMap.push_back(0);
				}
				if(abs(frm->points[i].z-zMapCalib[i])>.3)
				{
					zMap.push_back(255.0);
				}
				if(abs(frm->points[i].z-zMapCalib[i])<=.3)
				{
					zMap.push_back(0);
				}
			}
			_iMat=Mat(getDim().height,getDim().width,CV_32F,iMap.data());
			_dMat=Mat(getDim().height,getDim().width,CV_32F,zMap.data());
		
		}
		 
		/*for(int i=0;i<_dMat.rows;i++)
		{
			for(int j=0;j<_dMat.cols;j++)
			{	
				if(_dMat.at<float>(i,j)>0)
				{
					float angle=(float)asin(dist(cv::Point(j,i),cv::Point(80,60))/_dMat.at<float>(i,j));
					float newdist=(float)cos(angle)*_dMat.at<float>(i,j);
					_dMat.at<float>(i,j)=newdist;
				}
				
			}
		}*/
		
		/*for (int i=0;i< frm->points.size();i++) 
		{
			zMap.push_back(frm->points[i].z);
			iMap.push_back(frm->points[i].i);
		}
		_iMat=Mat(getDim().height,getDim().width,CV_32F,iMap.data());
		_dMat=Mat(getDim().height,getDim().width,CV_32F,zMap.data()); 
		*/
		// Apply amplitude gain
		_iMat=(float)_ampGain*_iMat;
		// Update background as required
		if (!_setBackground) 
		{
			_dMat.copyTo(_bkgndMat);
			_setBackground = true;
			cout << endl << "Updated background for people counting process." << endl;
		}
		// Find foreground by subtraction and convert to binary 
		// image based on amplitude and depth thresholds
		Mat fMat=_dMat.clone();//clipBackground((float)_depthThresh/100.0,(float)_ampThresh/100.0,(float)_depthThreshmax/100.0);
		Mat forKMeans=fMat.clone();
		// Apply morphological open to clean up image
		fMat.convertTo(_bMat,CV_8U,255.0);
		Mat morphMat=_bMat.clone();
		Mat element=getStructuringElement(0,Size(5,5),cv::Point(1,1) );
		morphologyEx(_bMat,morphMat,2,element);
	
		// Draw contours that meet a "person" requirement
		Mat drawing=Mat::zeros(_dMat.size(),CV_8UC3 );
		Mat im_with_keypoints=Mat::zeros(_iMat.size(),CV_8UC3);
		cvtColor(_dMat*(float)_multiplier/10,drawing,CV_GRAY2RGB);
		Mat cloneofMorph=morphMat.clone();
		// Find all contours
		findContours(cloneofMorph,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,cv::Point(0,0));
		for(int i=0;i<contours.size();i++)
		{
			
			if(isPerson(contours[i]))
			{
				drawContours(drawing,contours,i,Scalar(0,255,0),2,8,vector<Vec4i>(),0,cv::Point());
			}
			else
			{
				drawContours(drawing,contours,i,Scalar(0,0,255),2,8,vector<Vec4i>(),0,cv::Point());
				
			}
		}
		/*if(contours.size()>0)
		{
			for(vector<cv::Point> contour:contours)
			{
				if(!isPerson(contour))
				{
					contours.erase(contours.begin()+getIndex(contours,contour));
				}
			}
		}*/
		
		
		if(_population.size()==0)
		{
			int num;
			for(int i=0;i<contours.size();i++)
			{
				if(isPerson(contours[i]))
				{
					Path *input=new Path(300);
					input->addPos(getCenter(contours[i]));
					_population.push_back(input);
	
				}
			}
		}
		else
		{
			//Only one person should ever enter the frame
			Path *min_traj=new Path(300);
			vector<cv::Point> min_contour;
			vector<vector<cv::Point>> contourscopy(contours);
			vector<Path*> _populationcopy(_population);
			while(!done)
			{	
				for(vector<cv::Point> contour:contourscopy)
				{
					if(isPerson(contour))
					{
						for(Path *traj:_populationcopy)
						{
							if(dist(getCenter(contour),traj->getPos())<_minDist)
							{
								min_traj=traj;
								min_contour=contour;
								passed=true;
							}
						}
					}
				}
				if(passed)
				{
					_population[getIndex(_population,min_traj)]->addPos(getCenter(min_contour));
					_populationcopy.erase(_populationcopy.begin()+getIndex(_populationcopy,min_traj));
					contourscopy.erase(contourscopy.begin()+getIndex(contourscopy,min_contour));
				}
				if(_populationcopy.size()==0 || contourscopy.size()==0)
				{
					done=true;
				}
				if(!passed)
				{
					done=true;
				}
			}
		}
		if(contours.size()==0)
		{
			for(Path *traj:_population)
			{
				_population.erase(_population.begin()+getIndex(_population,traj));
			}
		}
		ofstream calibrationSettings;
		calibrationSettings.open("calibrationSettings2.txt");
		calibrationSettings << _ampGain << endl;
		calibrationSettings << _depthThresh << endl;
		calibrationSettings << _ampThresh << endl;
		calibrationSettings << _minContourArea << endl;
		calibrationSettings << _maxContourArea << endl;
		calibrationSettings << _aspectRatio << endl;
		if(_checkminDist==1 && _population.size()>0)
		{
			calibrationSettings << (int)dist(cv::Point(0,0),cv::Point(_population[0]->getVel().x,_population[0]->getVel().x))+1 << endl;
		}
		calibrationSettings.close();

		putText(drawing,"Count = "+to_string(_population.size())+","+to_string(contours.size()),cv::Point(70,10),FONT_HERSHEY_PLAIN,.75,Scalar(0,0,255));
		imshow("Binary",_bMat);
		imshow("Amplitude",_iMat); 
		imshow("Draw",drawing);
		imshow("Morph",morphMat);
	
	}
	
}
#undef __PPLCOUNTER_CPP__
