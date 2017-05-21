#include "TOFApp.h"
#include <math.h>
#include "Path.h"
#include <chrono>
#include <ctime>
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
	int _rightExit;
	int _leftExit;
	int _topExit;
	int _bottomExit;
	bool _go;
	int _personsClear;
	vector<int> holder;
	vector<string> timeData;
	vector<Path*> _population;
	vector<float> zMapCalib, iMapCalib, maxMap,minMap,avgMap,stdDev,stdDev2,avgMap2;
	int _analyzeBackground;
	float avgDepth;
	float _elapsedSeconds;
	std::chrono::time_point<std::chrono::system_clock> startTime,currentTime;
	std::chrono::duration<float> elapsedTime;
	bool startOnce;
	int enterCase;
	int exitCase;
private:
	bool isPerson(vector<cv::Point> p);
	cv::Point getCenter(vector <cv::Point> v);
	float dist(cv::Point p, cv::Point p2);
	int getIndex(vector<Path*> t,Path *p);
	int getIndex(vector<vector<cv::Point>> t,vector<cv::Point> p);
	bool hasExitedRight(Path *p);
	bool hasExitedLeft(Path *p);
	bool hasExitedTop(Path *p);
	bool hasExitedBottom(Path *p);
	Mat createKMat(Mat morphMat2);
	vector<cv::Point> getClusterCenters(Mat m,int n);
	bool checkCentersMatch(vector<Path*> pop,vector<cv::Point> cntrs);
	int getIndex(vector<cv::Point>,cv::Point p);
	int getCloseIndex(Path *traj,vector<cv::Point> cntrs);
	int numPersons(vector<cv::Point> v);
	//void init(Local<Object> exports));
	void clearCounts();
};
#endif 

