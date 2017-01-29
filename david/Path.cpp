#define __PATH_CPP__
#include "Path.h"
Path::Path(int sz)
{
	_sz=sz;
	blobbed=false;
}	
Path::~Path()
{

}
cv::Point Path::getPos()
{
	cv::Point pos=_fifo[_fifo.size()-1];
	return pos;
}
cv::Point Path::getPos(int a)
{
	cv::Point pos=cv::Point(0,0);
	if(a<_fifo.size())
	{
		pos=_fifo[a];
	}
	return pos;
}
void Path::addPos(cv::Point pos)
{
	
	if(_fifo.size()>=_sz)
	{
		_fifo.pop_front();
	}
	_fifo.push_back(pos);
}
Path::Vel Path::getVel()
{
	int sz=_fifo.size();
	if(sz>=2)
	{	
		cv::Point p2=_fifo[sz-1];
		cv::Point p1=_fifo[sz-2];
		return p2-p1;
	}
	else
	{
		return Vel(0,0);
	}
		
}
Path::Vel Path::getVel(int a)
{
	int sz=_fifo.size();
	if(sz>=a && sz>=2)
	{	cv::Point p2=_fifo[a];
		cv::Point p1=_fifo[a-1];
		return p2-p1;
	}
	else
	{
		return Vel(0,0);
	}
		
}
int Path::getSize()
{
	return _fifo.size();
}
void Path::fifoResize(int n)
{
	_sz=n;
}
void Path::blobState()
{
	if(blobbed)
	{
		blobbed=false;
	}
	else
	{
		blobbed=true;
	}
}
bool Path::getBlobState()
{
	return blobbed;
}
#undef __PATH_CPP__



