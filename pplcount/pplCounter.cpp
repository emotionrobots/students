#define __PPLCOUNTER_CPP__
#include "pplCounter.h"
#include <climits>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath> 
#include <chrono>
#include <ctime>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;

pplCounter::pplCounter(int w, int h) : TOFApp(w, h)
{
	_setBackground=false;
	initDisplay();
	_analyzeBackground=0;
	_rightExit=0;
	_leftExit=0;	
	_topExit=0;
	_bottomExit=0;
	_go=true;
	startOnce=true;
	enterCase=0;
	exitCase=0;
	_personsClear=0;
	
	for(int i=0;i<getDim().height;i++)
	{
		for(int j=0;j<getDim().width;j++)
		{
			avgMap.push_back(0.0);
			stdDev.push_back(0.0);
			avgMap2.push_back(0.0);
			stdDev2.push_back(0.0);
			iMapCalib.push_back(0.0);
		}
	}
	
}

void pplCounter::initDisplay()
{
	//namedWindow( "Amplitude", WINDOW_NORMAL );
	//namedWindow( "Binary", WINDOW_NORMAL );
	//namedWindow( "Morph", WINDOW_NORMAL );
	//namedWindow( "Controls", WINDOW_NORMAL);
	namedWindow( "Binary", WINDOW_NORMAL );
	namedWindow( "Actual", WINDOW_NORMAL );
	string line;
	//Reading from calibrationSettings.txt
	ifstream calibrationSettings;
	calibrationSettings.open("calibrationSettings.txt");
	ifstream intMatData;
	ifstream depthMatData;
	intMatData.open("intMatData.txt");
	depthMatData.open("depthMatData.txt");
	
	if(calibrationSettings.is_open())
	{
		while(getline(calibrationSettings,line))
		{
			int x;
			stringstream convert(line);
			convert >> x;
			holder.push_back(x);
		}
		calibrationSettings.close();
	}
	if(holder.size()==6)
	{
		//_minDist and Contour area are primary discriminating factors in tracking; other factors are unused or insignificant
		_minContourArea=holder[3];
		_maxContourArea=holder[4];
		_aspectRatio=holder[5];
		_minDist=20;
		
	}
	else
	{
		_minContourArea=0;
		_maxContourArea=0;
		_aspectRatio=0;
		_minDist=0;
		cout << "Calibration is incomplete and/or contains an error." << endl;
		cout << "Please recalibrate the program." << endl;
	}
	cout << "Amplitude gain setting: "+to_string(_ampGain) << endl;
	cout << "Depth threshold setting: "+to_string(_depthThresh) << endl;
	cout << "Amplitude Threshold setting: "+to_string(_ampThresh) << endl;
	cout << "Mininum Contour Area setting: "+to_string(_minContourArea) << endl;
	cout << "Maximum Contour Area setting: "+to_string(_maxContourArea) << endl;
	cout << "Aspect Ratio setting: "+to_string(_aspectRatio) << endl;
	cout << "Mininum Distance: "+to_string(_minDist) << endl;
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
	
	//Determining if a human is in frame
	if(contourArea(p)>_minContourArea && contourArea(p)<_maxContourArea)
	{
		if (dx>0) 
		{
			float ratio=(float)dy/(float)dx;
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

//Finds index the position where an element fits in a vector
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

//Finds index position of a contour in a vector of contours
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

//Finds index position of a point in a vector of points
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

//Sees if a trajectory potentially exited to the right of frame
//Could look at directional velocity as well
bool pplCounter::hasExitedRight(Path *p)
{
	bool truthval=false;
	//If distance from exit edge is close enough
	//to an arbitrary value
	if(abs((float)p->getPos().x-160.0f)<20.0f)
	{
		if((float)p->getVel().x>0.0f)
		{
			truthval=true;
		}
	}

	return truthval;
}

//Checks if person has potentially exited left
bool pplCounter::hasExitedLeft(Path *p)
{
	bool truthval=false;
	if(abs((float)p->getPos().x-0.0f)<20.0f)
	{
		if((float)p->getVel().x<0.0f)
		{
			truthval=true;
		}
	}
	return truthval;
}
//Checks if person has potentially exited to the top of frame
bool pplCounter::hasExitedTop(Path *p)
{
	bool truthval=false;

	if(abs((float)p->getPos().y-0.0f)<20.0f)
	{
		if((float)p->getVel().y<0.0f)
		{
			truthval=true;
		}
	}
	return truthval;
}
//Checks if person has potentially exited to the bottom of frame
bool pplCounter::hasExitedBottom(Path *p)
{
	bool truthval=false;

	if(abs((float)p->getPos().y-120.0f)<20.0f)
	{
		if((float)p->getVel().y>0.0f)
		{
			truthval=true;
		}
	}
	return truthval;
}

//Clear recorded people counts
void pplCounter::clearCounts()
{
	ofstream settings1;
	const char *path1="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/15a.txt";
	settings1.open(path1);
	string data(to_string(0));
	settings1 << data;
	settings1.close();

	ofstream settings2;
	const char *path2="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/30a.txt";
	settings2.open(path2);
	settings2 << "0";
	settings2.close();

	ofstream settings3;
	const char *path3="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/45a.txt";
	settings3.open(path3);
	settings3 << "0";
	settings3.close();

	ofstream settings4;
	const char *path4="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/60a.txt";
	settings4.open(path4);
	settings4 << "0";
	settings4.close();

	ofstream settings5;
	const char *path5="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/75a.txt";
	settings5.open(path5);
	settings5 << "0";
	settings5.close();
	
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

//Checking if kmeans calculation of centroid positions match
//The original calculations of the contour centers
bool pplCounter::checkCentersMatch(vector<Path*> pop,vector<cv::Point> cntrs)
{
	bool done=false;
	bool truthval=true;
	float minDist=_minDist;
	int miniindex;
	Path *min_p=new Path(30);
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
		//if the minDist exceeds a mininum value, then the centroid is too far from a reasonable previous contour's position, so no clustering has occurred
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

//Finds the index of a trajectory in a vector of center points
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

//Estimate people in large, unquantified cluster
int pplCounter::numPersons(vector<cv::Point> v)
{
	int num=1;
	for(int i=1;i<20;i++)
	{
		if(contourArea(v)/i<_maxContourArea && contourArea(v)/i>_minContourArea)
		{
			num=i;
		}
	}
	return num;
}

//Primary enginer of the algorithm; methodically tracks position and determines potential clustering cases
void pplCounter::update(Frame *frame)
{
	cout << "running" << endl;
	vector< vector<cv::Point> > contours;
	vector<Vec4i> hierarchy;
	RNG rng(12345);
	bool passed=false;
	bool done=false;
	int numBlobbed=0;
	float depth=0;
	int iter=0;
	string line2;
	if (getFrameType()==DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME) 
	{
		// Create amplitude and depth Mat
		vector<float> zMap, iMap;
		XYZIPointCloudFrame *frm=dynamic_cast<XYZIPointCloudFrame *>(frame);
		if(startOnce==true && _analyzeBackground==501)
		{
			startTime=chrono::system_clock::now();
			startOnce=false;
			clearCounts();
			
		}
		//Important calibration phase for determining elevation points across the frame
		for (int i=0;i< frm->points.size();i++) 
		{
			iMapCalib[i]=frm->points[i].z;
		}
		if(_analyzeBackground<250)
		{
			for (int i=0;i< frm->points.size();i++) 
			{
				float a=avgMap[i];
				avgMap[i]=a+frm->points[i].z;
			}
		}
		//Calculating average elevation at each point in frame
		if(_analyzeBackground==250)
		{
			for(int i=0;i<avgMap.size();i++)
			{
				float a=avgMap[i];
				avgMap[i]=(float)a/250.0;
			}
		}
		//Determinig Standard deviation of elevation points
		if(_analyzeBackground>250 && _analyzeBackground<=500)
		{
			for(int i=0;i<frm->points.size();i++)
			{
				float a=stdDev[i];
				stdDev[i]=a+(avgMap[i]-frm->points[i].z)*(avgMap[i]-frm->points[i].z);
			}
			
		}
		//Calculating a standard deviation
		if(_analyzeBackground==500)
		{
			for(int i=0;i<frm->points.size();i++)
			{
				float a=stdDev[i];
				stdDev[i]=(float)sqrt(a/250.0);
			}
			
		}
		//Next few conditional statements are determining automatic calibration data if nobody stands in frame
		if(_personsClear<250 && _analyzeBackground==501)
		{
			for (int i=0;i< frm->points.size();i++) 
			{
				float a=avgMap2[i];
				avgMap2[i]=a+frm->points[i].z;
			}
		}
		if(_personsClear==250 && _analyzeBackground==501)
		{
			for(int i=0;i<avgMap.size();i++)
			{
				float a=avgMap2[i];
				avgMap2[i]=(float)a/250.0;
			}
		}
		if((_personsClear>250 && _personsClear<=500) && _analyzeBackground==501)
		{
			for(int i=0;i<frm->points.size();i++)
			{
				float a=stdDev2[i];
				stdDev2[i]=a+(avgMap2[i]-frm->points[i].z)*(avgMap2[i]-frm->points[i].z);
			}
			
		}
		if(_personsClear==500 && _analyzeBackground==501)
		{
			for(int i=0;i<frm->points.size();i++)
			{
				float a=stdDev2[i];
				stdDev2[i]=(float)sqrt(a/250.0);
			}
			
		}
		if(_personsClear==501 && _analyzeBackground==501)
		{
			float suma=0;
			for(int i=0;i<frm->points.size();i++)
			{
				suma+=avgMap2[i];
			}
			for(int i=0;i<frm->points.size();i++)
			{
				if(true)
				{
				stdDev[i]=stdDev2[i];	
				avgMap[i]=avgMap2[i];
				}
			}
		}
		//Producing a binary image resembling calibrated elevation data
		if(_analyzeBackground>=500)
		{
		
			for(int i=0;i<frm->points.size();i++)
			{
				if(frm->points[i].z<avgMap[i]-5*stdDev[i])
				{
					zMapCalib.push_back(255.0);
				}
				else
				{
					zMapCalib.push_back(0.0);
				}
			}	
			_dMat=Mat(getDim().height,getDim().width,CV_32F,zMapCalib.data());
			_iMat=Mat(getDim().height,getDim().width,CV_32F,iMapCalib.data());
			iMapCalib.clear();
			zMapCalib.clear();
		}
		//Primary tracking algorithms
		if(_analyzeBackground>=501)
		{
			string lineaa2;
			int enteraa2;
			int exitaa3;
			string lineaa3;
			ifstream onlinesettingsaa2;
			const char *pathyaa2="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/maxcontour.txt";
			onlinesettingsaa2.open(pathyaa2);
			if(onlinesettingsaa2.is_open())
			{
				while(getline(onlinesettingsaa2,lineaa2))
				{
					int xaa2;
					stringstream convert(lineaa2);
					convert >> xaa2;
					enteraa2=xaa2;
					_maxContourArea=enteraa2;
				}		
				onlinesettingsaa2.close();
			}
			ifstream onlinesettingsaa3;
			const char *pathyaa3="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/mincontour.txt";
			onlinesettingsaa3.open(pathyaa3);
			if(onlinesettingsaa3.is_open())
			{
				while(getline(onlinesettingsaa3,lineaa3))
				{
					int xaa3;
					stringstream convert(lineaa3);
					convert >> xaa3;
					exitaa3=xaa3;
					_minContourArea=exitaa3;
				}		
				onlinesettingsaa3.close();
			}			
			if (!_setBackground) 
			{
				_dMat.copyTo(_bkgndMat);
				_setBackground = true;
				cout << endl << "Updated background for people counting process." << endl;
			}
			Mat fMat=_dMat.clone();
			Mat forKMeans=fMat.clone();
			// Apply morphological open to clean up image
			fMat.convertTo(_bMat,CV_8U,255.0);
			Mat morphMat=_bMat.clone();
			Mat element=getStructuringElement(0,Size(5,5),cv::Point(1,1) );
			morphologyEx(_bMat,morphMat,2,element);
			// Draw contours that meet a "person" requirement
			Mat drawing=Mat::zeros(_dMat.size(),CV_8UC3 );
			Mat drawing2=Mat::zeros(_iMat.size(),CV_8UC3);
			cvtColor(_dMat*0.5,drawing,CV_GRAY2RGB);
			cvtColor(_iMat*.2,drawing2,CV_GRAY2RGB);
			Mat cloneofMorph=morphMat.clone();
			// Find all contours
			findContours(cloneofMorph,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,cv::Point(0,0));
			if(_population.size()==0)
			{
				int num;
				for(int i=0;i<contours.size();i++)
				{
					if(isPerson(contours[i]))
					{
						Path *input=new Path(30);
						input->addPos(getCenter(contours[i]));
						_population.push_back(input);
					}
					//Checking for clustered entry of more than 1 person into frame
					else
					{
						//Utilizing Kmeans to estimate persons and positions in large clusters
						if(!contourArea(contours[i])<_maxContourArea)
						{
							if(numPersons(contours[i])>1)
							{
								Mat inputMat=createKMat(forKMeans);
								vector<cv::Point> cntrs=getClusterCenters(inputMat,numPersons(contours[i]));
								for(cv::Point cntr:cntrs)
								{
									Path *newtraj=new Path(30);
									newtraj->addPos(cntr);
									_population.push_back(newtraj);
								}
							}
						}
					}
				}
			}
			//General process of trajectory logging
			else
			{
				Path *min_traj=new Path(30);
				vector<cv::Point> min_contour;
				vector<vector<cv::Point>> contourscopy(contours);
				vector<Path*> _populationcopy(_population);
				while(!done)
				{	
					float minDistTemp=_minDist;
					for(vector<cv::Point> contour:contourscopy)
					{
						for(int x=0;x<contours.size();x++)
						{
							drawContours(drawing,contours,x,Scalar(0,0,255),2,8,vector<Vec4i>(),0,cv::Point());
						}
						for(int x=0;x<_population.size();x++)
						{
							cv::rectangle(drawing2,cv::Point((_population[x]->getPos()).x-1,(_population[x]->getPos()).y-1),cv::Point((_population[x]->getPos()).x+1,(_population[x]->getPos()).y+1),Scalar(0,255,0),3);
							cv::rectangle(drawing,cv::Point((_population[x]->getPos()).x-1,(_population[x]->getPos()).y-1),cv::Point((_population[x]->getPos()).x+1,(_population[x]->getPos()).y+1),Scalar(0,255,0),3);
						}
						for(Path *traj:_populationcopy)
						{
							//Matching process of contours and trajectories
							if(dist(getCenter(contour),traj->getPos())<minDistTemp)
							{
								//Check for null
								minDistTemp=dist(getCenter(contour),traj->getPos());
								min_traj=traj;
								min_contour=contour;
								passed=true;
							}
						}
					}
			
					//Utilize for loop with integer indexes; Update positions of person trajectories
					if(passed)
					{
					
						_population[getIndex(_population,min_traj)]->addPos(getCenter(min_contour));
						cout << "Position of person "+to_string(getIndex(_population,min_traj)+1)+": ("+to_string(_population[getIndex(_population,min_traj)]->getPos().x)+","+to_string(_population[getIndex(_population,min_traj)]->getPos().y)+")" << endl;
						cout << "Size of Path _fifo: "+to_string(_population[getIndex(_population,min_traj)]->getSize()) << endl;
						_populationcopy.erase(_populationcopy.begin()+getIndex(_populationcopy,min_traj));
						//Create a bag that contains the residual index
						contourscopy.erase(contourscopy.begin()+getIndex(contourscopy,min_contour));
						passed=false;
					}
					if(_populationcopy.size()==0 || contourscopy.size()==0)
					{
						done=true;
					}
					if(iter>=contours.size()*_population.size())
					{
						done=true;
					}
					iter++;
					for(int x=0;x<_population.size();x++)
					{
						int _popsize=_population[x]->getSize();
						for(int y=0;y<_popsize-1;y++)
						{
							cv::line(drawing2,_population[x]->getPos(y),_population[x]->getPos(y+1),Scalar(255,255,255));
							cv::line(drawing,_population[x]->getPos(y),_population[x]->getPos(y+1),Scalar(255,0,0));
						}
					}
				
				}
		
				//Now checking any residuals					
				//This means new contours have popped into frame					
				if(!contourscopy.size()==0)
				{
					//Don't consider condition of what's left in contours exceeds maxcontoutarea
					for(vector<cv::Point> contour:contourscopy)
					{
						if(isPerson(contour))
						{
							Path *newtraj=new Path(30);
							newtraj->addPos(getCenter(contour));
							_population.push_back(newtraj);
						}
						//Consider case if large cluster popped into frame
						else
						{
							if(numPersons(contour)>1)
							{
								Mat inputMat=createKMat(forKMeans);
								vector<cv::Point> cntrs=getClusterCenters(inputMat,numPersons(contour));
								for(cv::Point cntr:cntrs)
								{
									Path *newtraj=new Path(30);
									newtraj->addPos(cntr);
									_population.push_back(newtraj);
								}
							}
						}
					}
				}
				
				//This means people have possibly clustered together or left the frame
				if(!_populationcopy.size()==0)
				{	
					//First check the case where someone simply exits
					for(Path *traj:_populationcopy)
					{	
						//Testing the case where someone simply exited frame
						string linea2;
						int entera2;
						int exita3;
						string linea3;
						ifstream onlinesettingsa2;
						const char *pathya2="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/enterpos.txt";
						onlinesettingsa2.open(pathya2);
						if(onlinesettingsa2.is_open())
						{
							while(getline(onlinesettingsa2,linea2))
							{
								int xa2;
								stringstream convert(linea2);
								convert >> xa2;
								entera2=xa2;
							}		
							onlinesettingsa2.close();
						}
						ifstream onlinesettingsa3;
						const char *pathya3="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/exitpos.txt";
						onlinesettingsa3.open(pathya3);
						if(onlinesettingsa3.is_open())
						{
							while(getline(onlinesettingsa3,linea3))
							{
								int xa3;
								stringstream convert(linea3);
								convert >> xa3;
								exita3=xa3;
							}		
							onlinesettingsa3.close();
						}
						if(hasExitedRight(traj))
						{
							_population.erase(_population.begin()+getIndex(_population,traj));
							_rightExit++;	
						}
						else if(hasExitedLeft(traj))
						{
							_population.erase(_population.begin()+getIndex(_population,traj));
							_leftExit++;	
						}
						else if(hasExitedTop(traj))
						{
							_population.erase(_population.begin()+getIndex(_population,traj));
							_topExit++;	
						}
						else if(hasExitedBottom(traj))
						{
							_population.erase(_population.begin()+getIndex(_population,traj));
							//Program currently recognizes a entrance case if figure leaves frame towards bottom 
							_bottomExit++;	
						}
						if(entera2==1)
						{
							enterCase=_rightExit;
						}
						if(entera2==2)
						{
							enterCase=_leftExit;
						}
						if(entera2==3)
						{
							enterCase=_topExit;
						}
						if(entera2==4)
						{
							enterCase=_bottomExit;
						}
						if(exita3==1)
						{
							exitCase=_rightExit;
						}
						if(exita3==2)
						{
							exitCase=_leftExit;
						}
						if(exita3==3)
						{
							exitCase=_topExit;
						}
						if(exita3==4)
						{
							exitCase=_bottomExit;
						}
						else
						{
							enterCase=_bottomExit;
							exitCase=_topExit;
						}
					}
					//Indication that a cluster has occurred
					if(_population.size()>=contours.size()+1)
					{
						Mat inputMat=createKMat(forKMeans);
						vector<cv::Point> cntrs;
						if(contours.size()!=0)
						{
							cntrs=getClusterCenters(inputMat,_population.size());
							for(cv::Point cntr:cntrs)
							{
								cout<< "Predicted Point: "+to_string(cntr.x)+","+to_string(cntr.y) << endl;
								
							}
					
							if(!checkCentersMatch(_population,cntrs))
							{
								for(Path *traj:_populationcopy)
								{
									_population.erase(_population.begin()+getIndex(_population,traj));
	
								}
		
							}
							else
							{
								for(Path *traj:_populationcopy)
								{
									_population[getIndex(_population,traj)]->addPos(cv::Point(cntrs[getCloseIndex(traj,cntrs)].x,cntrs[getCloseIndex(traj,cntrs)].y));
								}
							}
							cout << "BLOBBING HAS OCCURRED" << endl;
						}
					}
				
				}

			}
			putText(drawing2,"Count = "+to_string(enterCase-exitCase),cv::Point(70,10),FONT_HERSHEY_PLAIN,.75,Scalar(0,0,255));
			putText(drawing,"Count = "+to_string(_population.size())+","+to_string(contours.size()),cv::Point(70,10),FONT_HERSHEY_PLAIN,.75,Scalar(0,0,255));
			//imshow("Binary",_bMat);
			//imshow("Amplitude",_iMat); 
			//imshow("Morph",morphMat);
			imshow("Binary",drawing);
			imshow("Actual",drawing2);
	}
			const char *pathy="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/count.txt";
			ofstream file;
			file.open(pathy);
			string data(to_string(_population.size()));
			file << data;
			file.close();
			const char *pathy3="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/entercase.txt";
			ofstream filea1;
			filea1.open(pathy3);
			string dataa1(to_string(enterCase-exitCase));
			filea1 << dataa1;
			filea1.close();
			if(_analyzeBackground<501)
			{
				cout << "CALIBRATING. DO NOT STAND IN CAMERA VIEW." << endl;
				_analyzeBackground++;
			
			}
			//Creating automatic recalibration data vectors
			if(_analyzeBackground>=501)
			{
				_personsClear++;
				if(_personsClear>501)
				{
					_personsClear=0;
					fill(avgMap2.begin(),avgMap2.end(),0.0);
					fill(stdDev2.begin(),stdDev2.end(),0.0);
				}
				if(_population.size()>0)
				{
					_personsClear=0;
					fill(avgMap2.begin(),avgMap2.end(),0.0);
					fill(stdDev2.begin(),stdDev2.end(),0.0);
				}
			}
			//Recalibration if online settings determine so
			currentTime=chrono::system_clock::now();
			elapsedTime=currentTime-startTime;
			_elapsedSeconds=elapsedTime.count();
			bool ahead=true;
			string s;
			//Next few conditional statements help to update population history and graph
			if(_elapsedSeconds>60.0 && _elapsedSeconds<61.0)
			{
				const char *path1="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/15a.txt";
				ofstream file1;
				file1.open(path1);
				string data1(to_string(enterCase-exitCase));
				file1 << data1;
				file1.close();
				time_t end_time=chrono::system_clock::to_time_t(currentTime);
				if(_go)
				{
					timeData.push_back(ctime(&end_time));
					if(enterCase>exitCase)
					{
						timeData.push_back(to_string(enterCase-exitCase));
					}
					else
					{
						timeData.push_back(to_string(0));
					}
					_go=false;
				}
			}
			if(_elapsedSeconds>120.0 && _elapsedSeconds<121.0)
			{
				const char *path2="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/30a.txt";
				ofstream file2;
				file2.open(path2);
				string data2(to_string(enterCase-exitCase));
				file2 << data2;
				file2.close();
				time_t end_time=chrono::system_clock::to_time_t(currentTime);
				if(!_go)
				{
					timeData.push_back(ctime(&end_time));
					if(enterCase>exitCase)
					{
						timeData.push_back(to_string(enterCase-exitCase));
					}
					else
					{
						timeData.push_back(to_string(0));
					}
					_go=true;
				}
			}
			if(_elapsedSeconds>180.0 && _elapsedSeconds<181.0)
			{
				const char *path3="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/45a.txt";
				ofstream file3;
				file3.open(path3);
				string data3(to_string(enterCase-exitCase));
				file3 << data3;
				file3.close();
				time_t end_time=chrono::system_clock::to_time_t(currentTime);
				if(_go)
				{
					timeData.push_back(ctime(&end_time));
					if(enterCase>exitCase)
					{
						timeData.push_back(to_string(enterCase-exitCase));
					}
					else
					{
						timeData.push_back(to_string(0));
					}
					_go=false;
				}
			}
			if(_elapsedSeconds>240.0 && _elapsedSeconds<241.0)
			{
				const char *path4="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/60a.txt";
				ofstream file4;
				file4.open(path4);
				string data4(to_string(enterCase-exitCase));
				file4 << data4;
				file4.close();
				time_t end_time=chrono::system_clock::to_time_t(currentTime);
				if(!_go)
				{
					timeData.push_back(ctime(&end_time));
					if(enterCase>exitCase)
					{
						timeData.push_back(to_string(enterCase-exitCase));
					}
					else
					{
						timeData.push_back(to_string(0));
					}
					_go=true;
				}
			}
			if(_elapsedSeconds>300.0 && _elapsedSeconds<301.0)
			{
				const char *path5="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/75a.txt";
				ofstream file5;
				file5.open(path5);
				string data5(to_string(enterCase-exitCase));
				file5 << data5;
				file5.close();
				time_t end_time=chrono::system_clock::to_time_t(currentTime);
				if(_go)
				{
					timeData.push_back(ctime(&end_time));
					if(enterCase>exitCase)
					{
						timeData.push_back(to_string(enterCase-exitCase));
					}
					else
					{
						timeData.push_back(to_string(0));
					}
					_go=false;
					
				}
			}
			if(_elapsedSeconds>310.0 && _elapsedSeconds<311.0)
			{
				if(!_go)
				{
					startTime=chrono::system_clock::now();
					clearCounts();
					ahead=false;
					_go=true;
				}
			}
			const char *path6="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/timeData.txt";
			ofstream file6;
			file6.open(path6);
			if(ahead)
			{
				for(int i=0;i<timeData.size();i++)
				{
					if(i%2==0)
					{
						s+=timeData[i]+"Number of people:";
					}
					else
					{
						s+=timeData[i]+"\n\n";
					}
				}
			}
			string data6(s);
			file6 << data6;
			file6.close();
		
		}
}
#undef __PPLCOUNTER_CPP__
