#define PATH
#include "Path.h"
#include <climits>
#include <algorithm>
#include <iostream>
#include <cmath>    
using namespace std;

Path::Path(int w, int h):TOFApp(w,h)
{
	//Creating display through TOFApp interface
	_setBackground=false;
	initDisplay();
}

void Path::initDisplay()
{
	//Creating windows that user can oberve camera scene
	namedWindow("AmplitudeWindow",WINDOW_NORMAL);
	namedWindow("BinaryWindow",WINDOW_NORMAL);
	namedWindow("MorphWindow",WINDOW_NORMAL);
	namedWindow("DrawWindow",WINDOW_NORMAL);
	namedWindow("ControlWindow",WINDOW_NORMAL);
	
	_ampGain=99;
	_ampThresh=2;
	_depthThresh=1;
	_minCountourArea=499;
	_maxContourArea=750;
	_aspectRatio=99;

	//creating user-input variable manipulation
	//utilize &variable_name so that user can directly affect original variable
	
	createTrackbar("Amplitude Gain Control","Controls",&_ampGain,100);
	createTrackbar("Amplitude Thresh Control","Controls",&_ampThresh,100);
	createTrackbar("Depth Threshold Control","Controls",&_depthThresh,500);
	createTrackbar("MinCountour Area Control","Controls",&_minCountourArea,5000);
	createTrackbar("MaxCountour Area (for one person Control","Controls",&_maxCountourArea,7500);
	createTrackbar("Aspect Ratio (Y/X)% Control","Control",&_aspectRatio,500);
}

Mat Path::clipBackground(float dThr,float iThr)
{
	//Background Foreground separator-If dMat and fMat matrix share same addess
	
	//dMat is instantiated with 32 bit float 1 channel data
	Mat dMat=Mat::zeros(_iMat.size(),CV_32FC1);

	//fMat instantiated to share same address
	Mat fMat=_bkgndMat-_dMat;

	//Nested loop to discriminate background from foreground by seeing if Intensity matrix
	//and depth matrix pass certain intensity and depth criteria
	//If criterion pass, then a point in the intensity matrix is indicated to be 
	//A foreground/background with the float value of either 255.0f or 0.0f
	for(int i=0;i<_dMat.rows;i++)
	{
		for(int j=0;j<_dMat.rows;j++)
		{
			if(fMat.at<float>(i,j)<0.0f) fMat.at<float>(i,j)=0.0f;
			dMat.at<float>(i,j)=(_iMat.at<float>(i,j)>iThr && fMat.at<float>(i,j) > dThr) ?255.0:0.0;
		}
	}
	return dMat;
}

void Path::resetBackground()
{
	_setBackground=false;
}


cv::Point Path getCenter(vector<cv::Point> contour)
{
	int sz2=static_cast<int>(contour.size());
	Mat data_pts=Mat(sz,2,CV_32FC1);
	for(int i=0;i<data_pts.rows;++i)
	{
		data_pts.at<float>(i,0)=contour[i].x;
		data_pts.at<float>(i,1)=contour[i].y;
	}
	
	//Perform PCA Analysis
	PCA pca_analysis(data_pts,Mat(),CV_PCA_DATA_AS_ROW);
	
	//Store the center of the object
	cv::Point cntr=cv::Point(static_cast<int>(pca_analysis.mean.at<float>(0,0)),
							static_cast<int>(pca_analysis.mean.at.at<float>(0,1)));

	return cntr;
}

//A contour is ONE person if:
//Aspect ratio is mostly vertical
//All points are within a similar depth interval
//Cross sectional area is within range of reasonability
//Reflectivity is consistent throughout contour
//There are no possible intersections deduced from
//angle of movement from eigenvectors
//Position is wholly unique from another contour

//Appears to be no need for dMat argument as of now
bool Path::isPerson(vector<cv::Point> &contour, Mat dMat)
{
	bool rc=false;
	int area=0;;
	int minX=INT_MAX, minY=INT_MAX;
	int maxX=0, maxY=0;
	int dx, dy;
	float ratio;
	//Calculates overall dimension of contour, ie. circumscribes
	//A rectangle with dimensions of dx, dy around the contour
	//with its vertices:
	//(minX,minY),(minX,maxY),(maxX,minY),(maxX,maxY)
	for(int i=0;i<contour.size();i++)
	{
		minX=std::min(minX,contour[i].x);
		minY=std::min(minY,contour[i].y);
		maxX=std::max(maxX,contour[i].x);
		maxY=std::max(maxY,contour[i].y);
	}

	//Cross sectional area is roughly determined by dx*dy
	//dx*dy value must be in certain arbitrary interval
	//ie. [mininum possible human area,maximum reasonable human area]
	//if area falls under the mininum value, then no person
	//if area falls outside of largest value, then number of persons
	//must be greater than 1-reasonability for 2, 3, etc. grows
	//by determining overall area, dividing by variable x, 
	//with x being the number of possible people given that
	//x>1. If quotient fits within aforementioned range of reasonable
	//human areas, then x must be the number of persons from the contour
	
	dx=maxX-minX;
	dy=maxY-minY;
	area=contourArea(contour)
	//Deteremining if contour is a person from above calculations
	if(area>_minContourArea && area<_maxContourArea)
	{
		if(dx>0)
		{
			ratio=(float)dy/(float)dx;
			if(ratio>(float)_aspectRatio/100.0)
			{
				rc=true;
			}
		}
	}
	
	return rc;
}

float Path::direction(cv::Point p, cv::Point p2)
{
	float dir=atan2(p2.y-p.y,p2.x-p.x);
	return dir;
}

//Finds potential number of people in contour cluster
int Path::numPersons(vector<cv::Point> contour)
{
	float entityarea=contourArea(contour);	
	int correct=1;
	
//Presuming the maximum number of people in a large contour is 20
	if(entityarea<_minContourArea)
	{
		correct=0;
	}
	else
	{
		for(int x=1;x<21;x++)
		{
			if(entityarea/(float)x>_minContourArea && entityarea/(float)x<_maxContourArea)
			{
				correct=x;
			}
		}
	}
	return correct;
}

//Checks if an integer is already in an array of floats (converted to integers)
bool Path::checker(int i,vector<float> r)
{
	bool checker=false;
	for(int x=0;x<r.size();x++)
	{
		if(i==(int)r[x])
			checker=true;
		break;
	}
	return checker;
}
int Path::finalPersons(deque<deque<spatial>> _fifo,int peopleCount,int frame)
{
	int difference=abs(_fifo.size()-peopleCount);
	int _finalPersons=0;
	vector<float> s;
	float angleofintersec;
	int targ;
	vector<float> p;

	//This means that someone has exited the frame
	if(_fifo.size()>peopleCount)
		{
			_finalPersons=moreContoursThanPersons(deque<deque<spatial>> _fifo,peopleCount,frame);
		}

	//This means some persons have clumped
	else if(_fifo.size()<peopleCount && difference>=1)
		{
			int sizep=_fifo.size();
			if(difference==1)
			{
				s=lessContoursThanPersons2(deque<deque<spatial>> _fifo,peopleCount,frame,0,p,true)[0];
				if(s[2]<=1.0f)
				{
					_finalPersons=sizep;
				}
				else
				{
					_finalPersons=sizep++;
				}
			}
			else if(difference>=2)
			{

				//Continuously updates and looks for number of people who could have entered a cluster
				s=lessContoursThanPersons2(deque<deque<spatial>> _fifo,peopleCount,frame,0,p,true)[0];
				for(int i=0;i<difference;i++)
				{
					//Finds a target contour from which to compare other possible intersecting 
					//People to
					targ=(int)s[1];
					if(i==0 && s[2]<=1.0f)
					{
						_finalPersons=sizep;
						break;
					}

					//Increments the total possible people if it discovers there has 
					//indeed been clustering
					else if(i==0 && s[2]==1.0f)
					{
						sizep++;
						//Updating the array of contours that have already been noted
						//To be part of a cluster
						p=s[1];
					}
					s=lessContoursThanPersons2(deque<deque<spatial>> _fifo,peopleCount
						,frame,targ,p,false)[0];		
				
					if(s[2]<=1.0f)
					{
						_finalPersons=sizep;
						break;	
					}
					else if(s[2]==1.0f)
					{
						sizep++;
						p=s[1];
					}
						
				}
			}	

		}
	return _finalPersons;
}
int Path::moreContoursThanPersons(deque<deque<spatial>> _fifo,int peopleCount,int frame)
{
	bool origcorrect=false;
	//The program might have conisdered a stationary object as a human and 
	//Generated a contour around it
	for(int i=0;i<_fifo.size();i++)
	{
		//If the velocity of the object is too close to complete stationary,
		//then it is not human
		//The code also recognizes that someone has left the frame, meaning
		//The deque of spatial elements does not update
		if(_fifo.at(i).size()<frame)
		{
			origcount=true;
		}
		else if(_fifo.at(i).at(frame-1).vel.x<.1)
		{
			origcount=true;
		}
		
	}
	if(origcorrect)
	{
		return peopleCount;
	}
	else
	{
		return _fifo.size();
	}
}

vector<vector<float>> Path::lessContoursThanPersons2(deque<deque<spatial>> _fifo,int peopleCount,int frame,int targ,vector<float> used,bool go)
{
	vector<vector<float>> r;
	r.resize(2);
	int itarg;
	int jtarg;
	//This is where there are more persons calculated than the number of contours
	//represented, indicating that large "blobs" have formed from people
	//clumping together
	bool hasintersect=false;
	//Confirming whether or not blobbing has actually occurred

	//if(go)==true, then the program runs an initial algorithm to determine
	//just 2 people who have intersected
	if(go)
	{
		for(int i=0;i<_fifo.size();i++)
		{
			for(int j=i+1;j<_fifo.size();j++)
			{
				if(dist(_fifo.at(i).at(frame-1).pos,_fifo.at(j).at(frame-1).pos)<.1)
				{
					//Now determining if the angle of travel could cause
					//Intersection of 2 humans
					//If this if statement is passed, then the probability of 
					//an actual intersection grows
					float ivel_y=(float)_fifo.at(i).at(frame-1).vel.x*
						(float)sin(_fifo.at(i).at(frame-1).vel.y);
					float ivel_x=(float)_fifo.at(i).at(frame-1).vel.x*
						(float)cos(_fifo.at(i).at(frame-1).vel.y);
					float jvel_y=(float)_fifo.at(j).at(frame-1).vel.x*
						(float)sin(_fifo.at(j).at(frame-1).vel.y);
					float jvel_x=(float)_fifo.at(j).at(frame-1).vel.x*
						(float)cos(_fifo.at(j).at(frame-1).vel.y);

					//Checking if y velocity of two contours are in opposite
					//Directions
					if(ivel_y+jvel_y<abs(ivel_y)+abs(jvel_y) ||
						ivel_y+jvel_y<-1.0f*(abs(ivel_y)+abs(jvel_y)))
					{
						hasintersect=true;
						itarg=i;
						jtarg=j;
						break;
					}
					
					//If y velocity are in the same direction, then check 
					//if two entities must have intersected through opposite
					//x velocity directions
					else
					{
						if(ivel_x+jvel_x<abs(ivel_x)+abs(jvel_x) ||
						ivel_x+jvel_x<-1.0f*(abs(ivel_x)+abs(jvel_x)))
						{
							hasintersect=true;
							itarg=i;
							jtarg=j;
							break;
						}
					}
				}
			}
		}
	}

	//if(go)==false, then the program assumes 2 already have intersected
	//and finds additional persons who have clumped into a contour
	else
	{
		for(int i=0;i<_fifo.size();i++)
		{
			if(!checker(i,used))
			{
				float ivel_y=(float)_fifo.at(targ).at(frame-1).vel.x*
					(float)sin(_fifo.at(targ).at(frame-1).vel.y);
				float ivel_x=(float)_fifo.at(targ).at(frame-1).vel.x*
					(float)cos(_fifo.at(targ).at(frame-1).vel.y);
				float jvel_y=(float)_fifo.at(i).at(frame-1).vel.x*
					(float)sin(_fifo.at(i).at(frame-1).vel.y);
				float jvel_x=(float)_fifo.at(i).at(frame-1).vel.x*
					(float)cos(_fifo.at(i).at(frame-1).vel.y);
				if(ivel_y+jvel_y<abs(ivel_y)+abs(jvel_y) ||
						ivel_y+jvel_y<-1.0f*(abs(ivel_y)+abs(jvel_y)))
					{
						hasintersect=true;
						itarg=i;
						jtarg=targ;
						break;
					}
					
					//If y velocity are in the same direction, then check 
					//if two entities must have intersected through opposite
					//x velocity directions
					else
					{
						if(ivel_x+jvel_x<abs(ivel_x)+abs(jvel_x) ||
						ivel_x+jvel_x<-1.0f*(abs(ivel_x)+abs(jvel_x)))
						{
							hasintersect=true;
							itarg=i;
							jtarg=targ;
							break;
						}
					}
			}
		}	
	}
	if(hasintersect)
	{
		//If there has been an intersection, the vector r notes the intersection
		//By having r[0][2] equate to 1.0f
		//the used array updates itself as to which contours have been noted
		//to be in the cluster
		used.push_back((float)itarg);
		used.push_back((float)jtarg);
		r[0].push_back((float)peopleCount/1.0f);
		r[0].push_back((float)itarg/1.0f);
		r[0].push_back(1.0f);
		r[1].push_back(used);		
		return r;
	}
	else
	{
		//If there has been no evidence of clustering, then the r array
		//simply displays a 0.0f at r[0][2] and returns the original 
		//_fifo.size() as the number of people
		used.push_back((float)itarg);
		used.push_back((float)jtarg);
		r[0].push_back((float)peopleCount/1.0f);
		r[0].push_back(0.0f);
		r[0].push_back(0.0f);
		r[1].push_back(used);
		return r;
	}
}
	
//Finds distance between 2 points
float dist(cv::Point p,cv::Point p2)
{
	float diffx=(float)p.x-(float)p2.x;
	float diffy=(float)p.y-(float)p2.y;
	float disp=((diffx)^2+(diffy)^2)^.5
	return disp;
}
	
void Path::update(Frame *frame)
{
	vector< vector<cv::Point> > contours;
	vector<Vec4i> hierarchy;
	if(getFrameType()==DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME)
	{
		//Create amplitude and depth Mat
		vector<float> zMap,iMap;
		XYZIPointCloudFrame *frm=dynamic_cast<XYZIPointCloudFrame *>(frame);
		for(int i=0; i< frm->points.size(); i++)
		{
			//Adds more data to the two matrices
			zMap.push_back(frm->points[i].z);
			iMap.push_back(frm->points[i].i);
		}
	
	//_iMat and _dMat storing data in a 2D matrix, like a xy grid 
	_iMat=Mat(getDim().height,getDim().width,CV_32FC1,iMap.data());
	_dMat=Mat(getDim().height,getDim().width,CV_32FC1,zMap.data());
	
	//Apply amplitude gain
	_tMat=(float)_ampGain*_iMat;
	
	//Update background
	if(!_setBackground)
	{
		_dMat.copyTo(_bkgndMat);
		_setbackground=true;
		cout << endl << "Updated background" << endl;
	}
	
	//Find foreround by subtraction and convert to binary 
	//image based on amplitude and depth thresholds
	Mat fMat=clipBackground((float)_depthThresh/100.0, (float)_ampThresh/100.0)
	
	//Apply Morphological open to clean up image
	fMat.convertTo(_bmat,CV_8U,255.0);
	Mat morphMat=_bMat.clone();
	Mat element=getStructuringElement(0,Size(5,5),cv::Point(1,1));
	morphologyEx(_bMat,morphMat,2,element);
	
	//Draw Contours that fit a person criteria
	Mat drawing=Mat::zeros(_dMat,size(),CV_8UC3);	
	
	Mat im_with_keypoints=Mat::zeros(_iMat.size(),CV_8UC3);
	cvtColor(_dMat*.5,drawing,CV_GRAY2RGB);
	 
	peopleCount=0;
	
#if 1
	//Find all contours
	//Hierarchy argument is largely irrelevant in our case b/c there are
	//No contours enclose another "child" countour
	findContours(morphMat,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,cv::Point(0,0));
	struct spatial
	{
		cv::Point pos, vel, acc;
	}
	deque<deque<spatial>> _fifo;
	//Adding elements to the deque<deque<spatial>> structure
	//Determines the basic number of contours (persons) and records
	//A frame by frame history of vel, pos, acc, and deletes itself
	//Following 10 seconds to prevent overwhelming data consumption
	for(int i=0;i<contours.size();i++)
	{
		//_cntr roughly represents position of person
		//Case where a new individual walks in or the camera has just initialized


		//Given: a collection of contours (C), and a collection of path (P),	
		//for(c in contour C)
		//for(p in Path P)
		//{if(distance from little c to 

#if 0
		// CLone P
		// Step 1: first do best match possible
		P = PP.clone()
		while (!done) {
			minDist = 0;
			for (c in C) {
				for (p in P) {
					if (c.distance(p) < minDist)
						minDist = c.distance(p)
						c_min = c;
						p_min = p;
				}
			}
			PP[p_min].add(c_min)

			// Remove them from matching population
			C.remove(c_min);
			P.remove(p_min);

			// Check if done
			if (C.isEmpty() || P.isEmpty())
				done = true;
		}

		// Step 2: Deal with residuals
		if (!C.isEmpty()) {
			for c_min in C
			   if (isQualified(c_min))
					PP.add(c_min)
		}
		else if (!P.IsEmpty()) {
			for (p_min is P)
				if (isQualified2(p_min))
					PP.remove(p_min)
		}
#endif


		if(frameCounter==0 || contours.size()>_fifo.size())
		{
			int pos=0;
			_fifo.resize(contours.size());
			//Finding the center of each contour, presumably a person
			if(frameCounter==0)
			{
				_cntr=getCenter(contours[i]);
				element.pos=_cntr;
				_vel=0.0f;
				_acc=0.0f;
				element.vel=cv::Point(_vel,0);
				element.acc=cv::Point(_acc,0);
				_fifo[i].push_back(element);
			}
			else
			{
				for(int i2=0;i2<contours.size();i2++)
				{
					for(int j=0;j<countours.size();j++)
					{
						if(dist(getCenter(contours[i2]),_fifo.at(j).at(frameCounter-1).pos)>.1)
						{
							pos=i2;
							break;	
						}
					}
				}
				_cntr=getCenter(contours[pos]);
				element.pos=_cntr;
				_vel=0.0f;
				_acc=0.0f;
				element.vel=cv::Point(_vel,0);
				element.acc=cv::Point(_acc,0);
				_fifo[contours.size()-1].resize(frameCounter);
				_fifo[contours.size()-1].insert(frameCounter-1,element);
				
			}																							 
		}

		if(frameCounter>0)
		{
			//If the distance between previous queued centroid and the new centroid of a 				
			//contour is under an arbitrary distance, then it is the same contour/entity,
			//and the pos,vel, and acc are continued to be stored in _fifo
			if(dist(_fifo.at(i).at(frameCounter-1).pos,getCenter(contours[i]))<.1 && 
				dist(_fifo.at(i).at(frameCounter-1).pos,getCenter(contours[i]))>=0)
			{	
				_cntr=getCenter(contours[i]);
				element.pos=_cntr;
				_vel=dist(_cntr,_fifo.at(i).at(frameCounter-1).pos)/(float)(1/30);
				_acc=(_vel.x-_fifo.at(i).at(frameCounter-1).vel.x)/(float)(1/30);
				element.vel=cv::Point(_vel,direction(_fifo.at(i).at(frameCounter-1).pos,_cntr));
				element.acc=cv::Point(_acc, direction(_fifo.at(i).at(frameCounter-1).pos,_cntr));
				_fifo[i].push_back(element);
			}

			//If the contour does not match the data contained in the deque
			//ie. the contour is a different entity from what is contained in
			//the deque, then cycle through available deque elements to determine
			//a match to contour[i]
			else if(dist(_fifo.at(i).at(frameCounter-1).pos,getCenter(contours[i]))>=.1)
			{
				for(int j=i+1;j<_fifo.size();j++)
				{
					if(dist(_fifo.at(j).at(frameCounter-1).pos,
						getCenter(contours[i]))<.1)
					{
						_cntr=getCenter(contours[i]);
						element.pos=_cntr;
						_vel=dist(_cntr,_fifo.at(j).at(frameCounter-1).pos)/(float)(1/30);
						_acc=(_vel.x-_fifo.at(j).at(frameCounter-1).vel.x)/(float)(1/30);
						element.vel=cv::Point(_vel,direction(_fifo.at(i).
							at(frameCounter-1).pos,_cntr));
						element.acc=cv::Point(_acc, direction(_fifo.at(i).
							at(frameCounter-1).pos,_cntr));
						fifo[j].push_back(element);
						break;
					}
				}
			}
		}
		//deque _contain holds a history of a contour's position, velocity, acceleration
		//_fifo[i].push_back(element);
		
		//Clear deque if it becomes too large
			
		if(frameCounter>299)
		{
			_fifo.clear();
		}
	}
	//Determines peoplecount through cross-sectional area
	for(int i=0;i<contours.size();i++)
	{
		if(isPerson(contours[i],_dMat))
		{
			peopleCount++;
			drawContours(drawing,contours,i,scalar,(0,0,255),2,8,vector<Vec4i>(),0,
				cv::Point());
		}
		
		//If the contour cluster is too large or too small and presents a conflict 
		else
		{
			_numPersons=numPersons(contours[i]);
			peopleCount+=_numPersons;
			drawContours(drawing,contours,i,scalar,(0,0,255),2,8,vector<Vec4i>(),0,
				cv::Point());
		}
		
	}
	//If there is a discrepancy between the room population generated from purely
	//contours and that generated from cross-sectional area, then the algorithm
	//executes to another method which determines and weighs which factor
	//more accurately determines room population
	if(peopleCount!=_fifo.size())
	{
		int pc=finalPersons(_fifo,peopleCount,frameCounter);	
		peopleCount=pc;
	}	

//Will not compile anything below here b/c #if is set to 1
#else

	//Find blobs
	std::vector<KeyPoint> keypoints;
	SimpleBlobDetector::Params params;	
	
	//Filter by color
	params.filterByColor=true;
	params.blobColor=255;
	
	//Change thresholds-depth
	params.minThreshold=0;
	params.maxThreshold=1000;
	
	//Filter by Area
	params.filterByArea=true;
	params.minArea=100;	
	params.maxArea=100000;
	
	//Filter by Circularity
	params.filterByCircularity=false;
	params.minCircularity=.1;
	
	//Filter by Convexity
	params.filterByConvexity=false;
	params.minConvexity=.87;
	h
	//Filter by Inertia
	params.filterByInertia=false;
	params.minInertiaRatio=.01;
	
	cv::Ptr<cv::SimpleBlobDetecor> detector=cv::SimpleBlobDetector::create(params);	
	detector->detect(morphMat,keypoints);
	cout<<"Keypoints # "<<keypoints.size()<<endl;
	
	for(int i=0;i<keypoints.size();i++)
	{
		cv::circle(drawing,cv::Point(keypoints[i].pt.x,keypoints[i].pt.y),10,Scalar(0,0,255),4);
	}
	peopleCount=keypoints.size();
#endif
	
	putText(drawing,"Count = "+to_string(peopleCount),cv::Point(200,50),FONT_HERSHEY_PLAIN,1,Scalar(255,255,255));
	
	imshow("Binary draw",_bMat);
	imshow("Amplitude draw",_iMat);
	imshow("Draw draw",drawing)
	imshow("Morph draw",morphMat);
	}
	frameCounter++;
	if(frameCounter>300)
	{
		frameCounter=0;	
	}
	

}
#undef PATH






