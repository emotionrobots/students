#define __PPLCOUNTER_CPP__
#include "pplCounter.h"
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
	_analyzeBackground=0;
	_toCalibrate=0;
	for(int i=0;i<getDim().height;i++)
	{
		for(int j=0;j<getDim().width;j++)
		{
			maxMap.push_back(0.0);
			avgMap.push_back(0.0);
			minMap.push_back(1000.0);
			stdDev.push_back(0.0);
		}
	}
	
}

void pplCounter::initDisplay()
{
	//namedWindow( "Amplitude", WINDOW_NORMAL );
	//namedWindow( "Binary", WINDOW_NORMAL );
	//namedWindow( "Morph", WINDOW_NORMAL );
	namedWindow( "Draw", WINDOW_NORMAL );
	//namedWindow( "Controls", WINDOW_NORMAL);
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
		//_ampGain=holder[0];
		_ampGain=100;	
		//_depthThresh=holder[1];
		_depthThresh=3;
		//_ampThresh=holder[2];
		_ampThresh=3;
		_minContourArea=holder[3];
		_maxContourArea=holder[4];
		_aspectRatio=holder[5];
		//Don't have a minDist value yet
		_minDist=20;
		
	}
	else
	{
		_ampGain=0;	
		_depthThresh=0;
		_ampThresh=0;
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


Mat pplCounter::clipBackground(float dThr,float iThr)
{
	Mat dMat=Mat::zeros(_iMat.size(),CV_32FC1);
	Mat fMat=_bkgndMat-_dMat;
	//Mat fMat=_dMat;
	
	for(int i=0;i<_dMat.rows;i++) 
	{
		for(int j=0;j<_dMat.cols;j++) 
		{
			if(fMat.at<float>(i,j)<0.0f) 
			{
				fMat.at<float>(i,j)=0.0f;
			}
			dMat.at<float>(i,j)=(_iMat.at<float>(i,j)>iThr && fMat.at<float>(i,j)>dThr) ? 255.0:0.0;
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
/*
void pplCounter::init(Local<Object> exports))
{
	NODE_SET_METHOD(exports,"name", Method);
}

void pplCounter::Method(const FunctionCallbackInfo<Value>& args)
{
	Isolate* isolate=args.GetIsolate();
	string count=to_string(_population.size());
	args.GetReturnValue().Set(String::NewFromUtf8(isolate,count));
}
NODE_MODULE(addon,init)
*/
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
		if(_analyzeBackground<250)
		{
			for (int i=0;i< frm->points.size();i++) 
			{
				float a=avgMap[i];
				avgMap[i]=a+frm->points[i].z;
			}
		}
		if(_analyzeBackground==250)
		{
			for(int i=0;i<avgMap.size();i++)
			{
				float a=avgMap[i];
				avgMap[i]=(float)a/250.0;
			}
		}
		if(_analyzeBackground>250 && _analyzeBackground<=500)
		{
			for(int i=0;i<frm->points.size();i++)
			{
				float a=stdDev[i];
				stdDev[i]=a+(avgMap[i]-frm->points[i].z)*(avgMap[i]-frm->points[i].z);
			}
			
		}
		if(_analyzeBackground==500)
		{
			for(int i=0;i<frm->points.size();i++)
			{
				float a=stdDev[i];
				stdDev[i]=(float)sqrt(a/250.0);
			}
			
		}
		if(_analyzeBackground>=501)
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
			zMapCalib.clear();
		}
		// Apply amplitude gain
		//_iMat=(float)_ampGain*_iMat;
		// Update background as required
		
		// Find foreground by subtraction and convert to binary 
		// image based on amplitude and depth thresholds
		if(_analyzeBackground>=501){
		if (!_setBackground) 
		{
			_dMat.copyTo(_bkgndMat);
			_setBackground = true;
			cout << endl << "Updated background for people counting process." << endl;
		}
		Mat fMat=_dMat.clone();//clipBackground((float)_depthThresh/100.0,(float)_ampThresh/100.0);
		Mat forKMeans=fMat.clone();
		// Apply morphological open to clean up image
		fMat.convertTo(_bMat,CV_8U,255.0);
		Mat morphMat=_bMat.clone();
		Mat element=getStructuringElement(0,Size(5,5),cv::Point(1,1) );
		morphologyEx(_bMat,morphMat,2,element);
	
		// Draw contours that meet a "person" requirement
		Mat drawing=Mat::zeros(_dMat.size(),CV_8UC3 );
		//Mat im_with_keypoints=Mat::zeros(_iMat.size(),CV_8UC3);
		cvtColor(_dMat*0.5,drawing,CV_GRAY2RGB);
		Mat cloneofMorph=morphMat.clone();
		// Find all contours
		findContours(cloneofMorph,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,cv::Point(0,0));
		
		//Ok
		if(_population.size()==0)
		{
			int num;
			for(int i=0;i<contours.size();i++)
			{
				if(isPerson(contours[i]))
				{
					Path *input=new Path(300);
					input->addPos(getCenter(contours[i]));
					//input.addPos(cv::Point(3,3));
					//input.addPos(cv::Point(5,4));
					_population.push_back(input);
					//cout << _population[i].getVel().x << endl;
					cout << "1" << endl;
					//drawContours(drawing,contours,i,Scalar(0,0,255),2,8,vector<Vec4i>(),0,cv::Point());
				}
				//Checking for clustered entry of more than 1 person into frame
				else
				{
					if(!contourArea(contours[i])<_minContourArea)
					{
						if(numPersons(contours[i])>1)
						{
							Mat inputMat=createKMat(forKMeans);
							vector<cv::Point> cntrs=getClusterCenters(inputMat,numPersons(contours[i]));
							for(cv::Point cntr:cntrs)
							{
								Path *newtraj=new Path(300);
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
			Path *min_traj=new Path(300);
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
						cv::circle(drawing,getCenter(contours[x]),1,Scalar(0,0,0),1);
					}
					cout << "1.5" << endl;
					for(Path *traj:_populationcopy)
					{
						cout << "2" << endl;
						if(dist(getCenter(contour),traj->getPos())<minDistTemp)
						{
							//Check for null
							minDistTemp=dist(getCenter(contour),traj->getPos());
							min_traj=traj;
							min_contour=contour;
							cout << "3" << endl;
							passed=true;
						}
					}
				}
			
				//Utilize for loop with integer indexes
				if(passed)
				{
					
					_population[getIndex(_population,min_traj)]->addPos(getCenter(min_contour));
					cout << "Position of person "+to_string(getIndex(_population,min_traj)+1)+": ("+to_string(_population[getIndex(_population,min_traj)]->getPos().x)+","+to_string(_population[getIndex(_population,min_traj)]->getPos().y)+")" << endl;
					cout << "Size of Path _fifo: "+to_string(_population[getIndex(_population,min_traj)]->getSize()) << endl;
					cout << "5" << endl;
	
					_populationcopy.erase(_populationcopy.begin()+getIndex(_populationcopy,min_traj));
					cout << "6" << endl;
					//Create a bag that contains the residual index
					contourscopy.erase(contourscopy.begin()+getIndex(contourscopy,min_contour));
					cout << "7" << endl;
					passed=false;
				}
				if(_populationcopy.size()==0 || contourscopy.size()==0)
				{
					done=true;
					cout << "8" << endl;
				}
				if(iter>=contours.size()*_population.size())
				{
					done=true;
				}
				iter++;
				
			}
		
			//Now checking any residuals					
			//This means new contours have popped into frame				
			if(!contourscopy.size()==0)
			{
				//Don't consider condition of what's left in contours exceeds maxcontoutarea
				cout << "9" << endl;
				for(vector<cv::Point> contour:contourscopy)
				{
					if(isPerson(contour)) /*&& passed)*/
					{
						Path *newtraj=new Path(300);
						newtraj->addPos(getCenter(contour));
						_population.push_back(newtraj);
						cout << "10" << endl;
					}
					else
					{
						if(numPersons(contour)>1)
						{
							Mat inputMat=createKMat(forKMeans);
							vector<cv::Point> cntrs=getClusterCenters(inputMat,numPersons(contour));
							for(cv::Point cntr:cntrs)
							{
								Path *newtraj=new Path(300);
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
				cout << "11" << endl;	
				//First check the case where someone simply exits
				for(Path *traj:_populationcopy)
				{	
					//Testing the case where someone simply exited frame
					if(hasExited(traj))
					{
						_population.erase(_population.begin()+getIndex(_population,traj));
						cout << "12" << endl;	
					}
				}

				if(_population.size()>=contours.size()+1)
				{
					cout << "13" << endl;
					Mat inputMat=createKMat(forKMeans);
					vector<cv::Point> cntrs;
					if(contours.size()!=0)
					{
						cntrs=getClusterCenters(inputMat,_population.size());
						for(cv::Point cntr:cntrs)
						{
							cout<< "Predicted Point: "+to_string(cntr.x)+","+to_string(cntr.y) << endl;
							cv::circle(drawing,cntr,1,Scalar(0,255,0),1);
						}
					
						if(!checkCentersMatch(_population,cntrs))
						{
							cout << "Check" << endl;
							for(Path *traj:_populationcopy)
							{
								_population.erase(_population.begin()+getIndex(_population,traj));
								cout << "14" << endl;

							}
		
						}
						else
						{
							for(Path *traj:_populationcopy)
							{
								_population[getIndex(_population,traj)]->addPos(cv::Point(cntrs[getCloseIndex(traj,cntrs)].x,cntrs[getCloseIndex(traj,cntrs)].y));
								//_population[getIndex(_population,traj)]->blobState();
								cout << "15" << endl;
							}
						}
						cout << "BLOBBING HAS OCCURRED" << endl;
					}
				}
				
			}

		}
		//Catches any logical errors associated with _population collection
		if(contours.size()==0)
		{
			for(Path *traj:_population)
			{
				_population.erase(_population.begin()+getIndex(_population,traj));
			}
		}
		putText(drawing,"Count = "+to_string(_population.size())+","+to_string(contours.size()),cv::Point(70,10),FONT_HERSHEY_PLAIN,.75,Scalar(0,0,255));
		//imshow("Binary",_bMat);
		//imshow("Amplitude",_iMat); 
		imshow("Draw",drawing);}
		//imshow("Morph",morphMat);
		//Notes: can also use initial _fifo.size() difference to determine time of entrance
		const char *pathy="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/count.txt";
		ofstream file;
		file.open(pathy);
		string data(to_string(_population.size()));
		file << data;
		file.close();
		ifstream onlinesettings;
		const char *pathy2="/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/return3.txt";
		onlinesettings.open(pathy2);
		if(onlinesettings.is_open())
		{
			while(getline(onlinesettings,line2))
			{
				int x;
				stringstream convert(line2);
				convert >> x;
				_toCalibrate=x;
				ofstream onlinesettings2;
				onlinesettings2.open(pathy2);
				string data(to_string(0));
				onlinesettings2 << data;
				onlinesettings2.close();
			}
			onlinesettings.close();
		}
		if(_analyzeBackground<501)
		{
			_analyzeBackground++;
		}
		if(_toCalibrate==1)
		{
			_analyzeBackground=0;
			_toCalibrate=0;
		}
	}
}
#undef __PPLCOUNTER_CPP__
