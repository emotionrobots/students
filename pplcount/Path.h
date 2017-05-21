#include <deque>
#include <math.h>
#include <opencv2/opencv.hpp>

#ifndef __PATH_H__
#define __PATH_H__

using namespace std;

class Path
{
private:
	//Likely need to use std::
	//deque<deque<spatial>> _fifo;
	deque<cv::Point> _fifo;
	int _sz;
	
public:
	Path(int sz);
	~Path();
	cv::Point getPos(); //return lastetst position
	cv::Point getPos(int a);
	void addPos(cv::Point pos);
	typedef cv::Point Vel;
	Vel getVel();
	Vel getVel(int a);
	int getSize();
	void fifoResize(int n);
};

#endif
