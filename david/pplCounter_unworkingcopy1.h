#include "TOFApp.h"
#include <math.h>
#include "Path.cpp"

#ifndef __PPLCOUNTER_H__
#define __PPLCOUNTER_H__

class pplCounter : public TOFApp
{
public:
	pplCounter(int w, int h);
	void update(Frame *frm);
	Mat clipBackground(float dThr, float iThr);
	void resetBackground();
	void initDisplay();
	cv::Point getCenter(vector<cv::Point> v);
	float dist(cv::Point p, cv::Point p2);
	int pointOfInsertion(vector<Path> p,Path p2);
	int cpointOfInsertion(vector<vector<cv::Point>> p,vector<cv::Point> p2);
	bool isPerson(vector<cv::Point> p);
	bool checkCentersMatch(vector<Path> fifo,Mat cntrs);
	bool hasExited(Path p);
	Mat hasClustered(Mat m,int n);
	Mat createKMat(Mat morphMat);
	Mat matErase(int i,Mat cntrs);
	

private:
	Mat _dMat, _iMat, _bMat, _bkgndMat;
	bool _setBackground;
	int _depthThresh;
	int _ampGain;
	int _ampThresh;
	int _minContourArea;
	int _maxContourArea;
	int _aspectRatio;
	vector<Path> _fifo2;
  	
	
};

#endif 
