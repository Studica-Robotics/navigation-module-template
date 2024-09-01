#pragma once

#include <cmath>
#include <iostream>

#include "Interface.h"

using std::cout;
using std::endl;

int GetTime();
void delay(int ms, int past_time = -1);

template <typename T>
inline int sign(const T n) { return n < 0 ? -1 : 1; }
template <typename T>
inline bool IsSafe(const T value, const int max) { return value[0] >=0 && value[0] < max && value[1] >=0 && value[1] < max; }
template <typename T1, typename T2>
inline double GetDistance(const T1 a, const T2 b) { return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1])); }

double Trans(double Min, double Max, double min, double max, double value, bool limit_value = true);
template <typename T, int N, int M>
double Trans(const T(&arr)[N][M], double value)
{
	int dir = sign(value);
	value = std::abs(value);

	if (value < arr[0][0])
		value = arr[0][0];
	else if (value > arr[0][M - 1])
		value = arr[0][M - 1];

	for (int i = 0; i < M - 1; i++)
		if (value <= arr[0][i + 1])
			return Trans(arr[0][i], arr[0][i + 1], arr[1][i], arr[1][i + 1], value) * dir;
	return 0;
}

inline double ToRadian(const double degree) { return degree * M_PI / 180; }
inline double ToDegree(const double theta) { return theta * 180 / M_PI; }
inline double NormalizeRadian(const double theta) { return theta - std::floor((theta + M_PI) / (M_PI * 2)) * M_PI * 2; }
inline double NormalizeDegree(const double degree) { return ToDegree(NormalizeRadian(ToRadian(degree))); }

void GetRobotVertices(const Vector3d robot_pose, std::array<std::array<double, 2>, 4>& vertices, int offset = 0);
void GetRobotCenterPosition(const Vector3d robot_pose, Vector3d &output);
Vector2i ConvertToGridCoordinate(const Vector3d input);
Vector3d ConvertToWorldCoordinate(const Vector2i input);