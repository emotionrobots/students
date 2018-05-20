#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <fstream>

#define mp make_pair
#define pb push_back

using namespace std;
using namespace pcl;
using namespace sensor_msgs;
using namespace cv;

const int HEIGHT = 480, WIDTH = 640;

Point3d xyz[480][640];
Mat src(HEIGHT, WIDTH, CV_8UC3);
bool hasRead = false;
vector<Point> corners;

bool cmp(const Point& p1, const Point& p2) {
	return p1.x == p2.x ? p1.y < p2.y : p1.x < p2.x;
}

void call_back(const sensor_msgs::PointCloud2ConstPtr& cloud) {
	pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
	pcl::fromROSMsg(*cloud, PointCloudXYZRGB);

	int max_i = cloud->height * cloud->width;
	for (int i = 0; i < max_i; i++) {
		int r = i / cloud->width, c = i % cloud->width;
		PointXYZRGB p = PointCloudXYZRGB.points[i];
		float z = p.z;
		xyz[r][c] = Point3d(p.x, -p.y, p.z);
		src.at<Vec3b>(Point(c, r)) = Vec3b(p.b, p.g, p.r);
	}
	hasRead = true;
}

void read() {
	hasRead = false;
	while (!hasRead) {
		ros::spinOnce();
	}
}

void show() {
	for (int i = 0; i < corners.size(); i++) {
		// cout << '(' << corners[i].first << ", " << corners[i].second << ')' << endl;
		circle(src, Point(corners[i].x, corners[i].y), 5, Scalar(0, 0, 255), -1);
	}
	imshow("show1", src);
}

void compress_lines(vector<Vec2f>& lines) {
	vector<double> weights(lines.size(), 1);
	for (int i = 0; i < lines.size(); i++) {
		double total_r = lines[i][0], total_a = lines[i][1];
		for (int j = lines.size() - 1; j > i; j--) {
			double diff_a = abs(lines[i][1] - lines[j][1]);
			if (abs(lines[i][0] - lines[j][0]) < 50) {
				if (diff_a < .2) {
					total_r += lines[j][0];
					total_a += lines[j][1];
					weights[i] += weights[j];
					lines.erase(lines.begin() + j);
				} else if (diff_a > CV_PI - .2) {
					total_r += lines[j][0];
					total_a += lines[j][1] > CV_PI / 2 ? lines[j][1] - CV_PI : lines[j][1];
					weights[i] += weights[j];
					lines.erase(lines.begin() + j);
				}
			}
		}

		lines[i][0] = total_r / weights[i];
		lines[i][1] = total_a / weights[i];
		if (lines[i][1] < 0) lines[i][1] += CV_PI;
	}
}

vector<Point> inter(const vector<Vec2f>& lines) {
	vector<Point> points;
	for (int i = 0; i < lines.size(); i++) {
		double r1 = lines[i][0], a1 = lines[i][1];
		// cout << r1 << ' ' << a1 << endl;
		for (int j = i + 1; j < lines.size(); j++) {
			double r2 = lines[j][0], a2 = lines[j][1];
			double x = (r1 / sin(a1) - r2 / sin(a2)) / (cos(a1) / sin(a1) - cos(a2) / sin(a2));
			double y = (-cos(a1) / sin(a1)) * x + r1 / sin(a1);
			if (-1 < x && x < 640 && -1 < y && y < 480 && abs(a1 - a2) > .2 && abs(a1 - a2) < CV_PI - .2) {
				points.pb(Point(x, y));
			}
		}
	}
	return points;
}

Point3d trans(const Point3d& p, double h, double a, double d) {
	double x1 = p.x, y1 = p.y, z1 = p.z;
	double x2 = x1, y2 = y1*sin(a) - z1*cos(a) + h, z2 = y1*cos(a) + z1*sin(a);
	double x3 = d - z2, y3 = x2, z3 = y2;
	return Point3d(x3, y3, z3);
}

vector<Point> compress_points(const vector<Point>& points) {
	vector<pair<pair<double, double>, int>> cluster;
	for (int i = 0; i < points.size(); i++) {
		cluster.pb(mp(mp(points[i].x, points[i].y), 1));
	}
	while (cluster.size() > 4) {
		int i1 = 0, i2 = 1;
		pair<double, double> c1 = cluster[0].first, c2 = cluster[1].first;
		double dist = hypot(c1.first - c2.first, c1.second - c2.second);
		for (int i = 0; i < cluster.size(); i++) {
			pair<double, double> c3 = cluster[i].first;
			for (int j = i + 1; j < cluster.size(); j++) {
				pair<double, double> c4 = cluster[j].first;
				double dist2 = hypot(c3.first - c4.first, c3.second - c4.second);
				if (dist2 < dist) {
					c1 = c3;
					c2 = c4;
					dist = dist2;
					i1 = i;
					i2 = j;
				}
			}
		}

		double xsum1 = cluster[i1].first.first * cluster[i1].second;
		double ysum1 = cluster[i1].first.second * cluster[i1].second;
		double xsum2 = cluster[i2].first.first * cluster[i2].second;
		double ysum2 = cluster[i2].first.second * cluster[i2].second;
		int total = cluster[i1].second + cluster[i2].second;
		cluster.erase(cluster.begin() + i2);
		cluster.erase(cluster.begin() + i1);
		cluster.pb(mp(mp((xsum1 + xsum2) / total, (ysum1 + ysum2) / total), total));
	}

	vector<Point> points2;
	for (int i = 0; i < cluster.size(); i++) {
		points2.pb(Point(cvRound(cluster[i].first.first), cluster[i].first.second));
	}
	return points2;
}

bool get_corners() {
	Mat dst, cdst;
	Canny(src, dst, 150, 450, 3);
	cvtColor(dst, cdst, CV_GRAY2BGR);

	// cout << "d1" << endl;

	vector<Vec2f> lines;
	HoughLines(dst, lines, 1, CV_PI/180, 65, 0, 0 );
	// cout << "lines " << lines.size() << endl;
	compress_lines(lines);
	cout << "lines " << lines.size() << endl;
	for (int i = 0; i < lines.size(); i++) {
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));
		line(cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
	}

	if (lines.size() > 100) return false;

	/*
	23
	14
	



	*/


	vector<Point> points = inter(lines);
	corners = compress_points(points);
	sort(corners.begin(), corners.end(), cmp);

	for (int i = 0; i < corners.size(); i++) {
		Point p = corners[i];
		cout << p.x << ' ' << p.y << endl;
		cout << xyz[p.y][p.x].x << ' ' << xyz[p.y][p.x].y << ' ' << xyz[p.y][p.x].z << endl;
	}
	cout << "----------------------------" << endl;

	imshow("show1", dst);
	if (waitKey(700) != -1) exit(0);
	imshow("show1", cdst);
	if (waitKey(700) != -1) exit(0);

	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);
	ros::Subscriber sub;
	sub = n.subscribe("/camera/depth_registered/points", 1, call_back);

	namedWindow("show1", CV_WINDOW_AUTOSIZE);
	// 

	while (true) {
		// cout << 'a' << endl;
		// src = imread(argv[1], 0);
		// cout << 'b' << endl;
		do {
			read();
		} while (!get_corners());
		// cout << 'c' << endl;
		show();
		// cout << 'd' << endl;
		if (waitKey(700) != -1) break;
	}

	return 0;
}m