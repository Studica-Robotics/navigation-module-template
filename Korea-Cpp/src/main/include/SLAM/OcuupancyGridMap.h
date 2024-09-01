#pragma once

#include <opencv2/core/core.hpp>

#include "../Interface.h"

class OcuupancyGridMap
{
public:
	OcuupancyGridMap(unsigned char **map);

	void ResetMap();
	void Update();

	void Show(cv::Mat &mat);

private:
	unsigned char **map_;

	void Bresenham(const Vector2i pt0, const Vector2i pt1);
};