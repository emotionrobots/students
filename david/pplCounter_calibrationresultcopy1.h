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
	vector<int> holder;

private:
	bool isPerson(vector<cv::Point> &p);
	Mat clipBackground(float dThr, float iThr);
};

#endif 

