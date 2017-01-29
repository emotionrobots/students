#include "TOFApp.h"
#include <math.h>
#include <string>
#include <deque>
using namespace std;


class Path
{
private:
	//Likely need to use std::
	string _name;
	//deque<deque<spatial>> _fifo;
	deque<cv::Point> _fifo;
	int _sz;
	
		

public:
	Path(int sz);
	~Path();
	cv::Point getPos(); //return lastetst position
	cv::Point getPosAt(int a)
	void addPos(cv::Point pos);
};
Path::Path(int sz)
{
	_sz=sz;
}
Path::~Path()
{
	_fifo.clear();
}
cv::Point Path::getPos()
{
	cv::Point pos=_fifo[_fifo.size()-1];
	return pos;
}
cv::Point Path::getPosAt(int a)
{
	if(a<_fifo.size())
	{
		cv::Point pos=_fifo[a];
	}
	return pos;
}
void Path::addPos(cv::Point pos)
{
	
	if(_fifo.size()>=sz)
	{
		_fifo.pop_Front();
	}
	_fifo.push_back(pos);
}
class Path:public TOFApp
{
private:
//Likely need to use std::
	string name;
	deque<deque<spatial>> _fifo;
	deque<spatial> _contain;

public:
	Path(int w, int h);
	void update(Frame *frm);
	void resetBackground();
	void initDisplay();
	//Path(std::string name, int n);
	bool vel(cv::Point &vel);
	bool pos(cv::Point &pos);
	bool acc(cv::Point &acc);
	int size(){return _fifo.size()};
	void clear(){_fifo.clear()};
	float dist(cv::Point p);

private:
   Mat _dMat, _iMat, _bMat, _bkgndMat;
	int peopleCount;
	int frameCounter;
   bool _setBackground;
   int _depthThresh;
   int _ampGain;
   int _ampThresh;
   int _minContourArea;
	//int _maxContourArea;
   int _aspectRatio;
	float _vel;
	cv::Point _cntr;
	float _disp;	
	float _acc;
	spatial element;
	int _numPersons;
	
	
private:
   bool isPerson(vector<cv::Point> &contour, Mat dMat);
   Mat clipBackground(float dThr, float iThr);
   void getPCA(const vector<cv::Point> &contour, float &center, float &angle);
	int numPersons(vector<cv::Point> contour);
	int finalPersons(deque<deque<spatial>> _fifo,int peopleCount,int frame);
	float direction(cv::Point p,cv::Point p2);
	bool checker(int i,vector<float> r)
	//cv::Point getCenter(vector<cv::Point> contour)
	vector<vector<float>> lessContoursThanPersons2(deque<deque<spatial>> _fifo,int peopleCount,int frame,int targ,vector<float> used,bool go);
	int moreContoursThanPersons(deque<deque<spatial>> _fifo,int peopleCount,int frame);
};
