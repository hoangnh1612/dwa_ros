#ifndef ULTIS_H
#define ULTIS_H
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random> 
#include <iostream>
using namespace Eigen;
struct Point2D
{
	double x, y;
};
struct Point2DPixel
{
	int x, y;
};
struct Velocity
{
	double x, y;

	Velocity() = default;
};
struct Pose2D
{
	Point2D position;
	double theta;
};
struct BGR
{
	int b, g, r;
};
struct DynamicWindow
{
	double linear_min_vel , linear_max_vel;
	double angular_min_vel, angular_max_vel;
};

struct Node
{
	std::vector<Node*> children;
	Node* parent;
	Pose2D pose;
	double cost;

};

struct Twist
{
    double linear= 0.0;
    double angular= 0.0;
};

typedef std::vector<cv::Point> Path;
typedef std::vector<Pose2D> PredictPath;
typedef std::vector<Twist> ReachableVelocity;
typedef std::vector<Point2D> ReferencePath;
typedef std::vector<Point2D> Waypoints;

// Constant
const int meter_to_pixel = 10; // 1m ~ 10 pixels
const double pixel_to_meter = 0.1; // 1 pixel ~ 0.1 m
const double BOT_RADIUS = 1.0;
const double NODE_RADIUS = 0.1;
const double END_DIST_THRESHOLD = 0.1;
const double BOT_CLEARANCE = 1.5 * BOT_RADIUS;
const double BOT_TURN_RADIUS = 1.0;
const double RRTSTAR_NEIGHBOR_FACTOR = 0;
const bool BOT_FOLLOW_DUBIN = false;
Point2DPixel convertMeterToPixel(const Point2D, const double, const double, const int);
BGR getColorInPoint(const cv::Mat, const Point2DPixel);
double normalize_angle(double);
std::vector<Point2D> extracRectangle(Pose2D, double, double);
cv::Mat computeCostMap(cv::Mat, double, BGR);
#endif