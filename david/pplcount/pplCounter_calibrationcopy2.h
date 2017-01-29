#include "TOFApp.h"
#include <math.h>
//#include "Path.h"
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
	Mat _dMat, _iMat, _bMat, _bkgndMat;
	bool _setBackground;
	int _depthThresh;
	int _ampGain;
	int _ampThresh;
	int _minContourArea;
	int _maxContourArea;
	int _aspectRatio;  	
	int _minDist;
	bool _willCalibrate;
	bool _calibrateminDist;

private:
	bool isPerson(vector<cv::Point> &p);
	Mat clipBackground(float dThr, float iThr);
	float getminDist(vector<cv::Point> v1,vector<cv::Point> v2);
};

#endif 

