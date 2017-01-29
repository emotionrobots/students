#include "TOFApp.h"
#include <math.h>
#include "Path.h"

using namespace std;

#ifndef __PPLCOUNTER_H__
#define __PPLCOUNTER_H__

class pplCounter : public TOFApp
{
public:
	pplCounter(int w, int h);
	void update(Frame *frame);
	void resetBackground();
	void initDisplay();
	

private:
	Mat _dMat, _iMat, _bMat, _bkgndMat,_avgMat,_maxMat,_minMat;
	bool _setBackground;
	int _depthThresh;
	int _ampGain;
	int _ampThresh;
	int _minContourArea;
	int _maxContourArea;
	int _aspectRatio;  	
	int _minDist;
	vector<int> holder;
	vector<Path*> _population;
	vector<float> zMapCalib, iMapCalib, maxMap,minMap,avgMap,stdDev;
	int _analyzeBackground;
	float avgDepth;

private:
	bool isPerson(vector<cv::Point> p);
	Mat clipBackground(float dThr, float iThr);
	cv::Point getCenter(vector <cv::Point> v);
	float dist(cv::Point p, cv::Point p2);
	int getIndex(vector<Path*> t,Path *p);
	int getIndex(vector<vector<cv::Point>> t,vector<cv::Point> p);
	bool hasExited(Path *p);
	Mat createKMat(Mat morphMat2);
	vector<cv::Point> getClusterCenters(Mat m,int n);
	bool checkCentersMatch(vector<Path*> pop,vector<cv::Point> cntrs);
	int getIndex(vector<cv::Point>,cv::Point p);
	bool hasCollided(Path *traj,Path *traj2);
	int getCloseIndex(Path *traj,vector<cv::Point> cntrs);
	int numPersons(vector<cv::Point> v);
};

#endif 

