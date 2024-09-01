#pragma once

#include <vector>

#include "Interface.h"

class Astar
{
public:
	Astar();
	~Astar();
	bool Update(const Vector2i& grid_start, const Vector2i& grid_goal, unsigned char** costmap_, std::vector<Vector2i>& path);

private:
	bool** visit_;
	int** path_cost_;
};