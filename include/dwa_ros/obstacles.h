#ifndef OBSTACLES_H
#define OBSTACLES_H
#include "utils.h"

class Obstacle
{
public:
	Point2D position;
    Obstacle(Point2D pos)
    {
        position = pos;
    }
};
typedef std::vector <Obstacle> obstacles;
#endif
