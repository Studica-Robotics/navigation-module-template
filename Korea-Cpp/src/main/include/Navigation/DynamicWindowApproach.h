#pragma once

#include <array>
#include <opencv2/core/core.hpp>

#include "Interface.h"
#include "Util.h"
#include "KDTree.h"
#include "Constants.h"

class DynamicWindowApproach
{
public:
	DynamicWindowApproach();
	~DynamicWindowApproach();

	void Init();
	void Update(const std::vector<Vector2d>& obstacle, const Vector2d& goal, Vector5d& control, Traj& traj);
	Vector5d Motion(const Vector5d& control, double dt);
	void Show(cv::Mat &mat);

private:
	const double max_speed = Constants::MAX_VELOCITY[0];
	const double min_speed = 0;
	const double max_yawrate = ToRadian(Constants::MAX_VELOCITY[1]);
	const double max_accel = (DRIVE_TYPE == Mecanum_Drive ? 70 : 150);
	const double max_dyawrate = ToRadian(90);
	const double robot_radius = Constants::ROBOT_RADIUS * Constants::MM_PER_PIXEL;
	const double robot_width = Constants::ROBOT_WIDTH;
	const double robot_height = Constants::ROBOT_HEIGHT;
	const double robot_center_offset = Constants::ROBOT_CENTER_OFFSET;
	
	const double v_resolution = 5;
	const double yawrate_resolution = ToRadian(1);
	
	const double dt = 0.1;
	const double predict_time = (DRIVE_TYPE == Mecanum_Drive ? 0.8 : 2); // 0.8
	const double goal_cost_gain = 0.033;
	const double obstacle_cost_gain = 25;
	const double speed_cost_gain = 0.01;

	bool error_flg_;
	int stuck_cnt_;

	KDTree<Vector2d> *kdtree_;
	std::vector<Traj> trajectories_;

	double GetTriangleArea(const std::array<std::array<double, 2>, 3>& pt);
	bool IsPointInsideRectangle(const Vector2d& pt, const Vector3d& robot_pose, int offset = 0);
	double SegmentToPointDistance(const Vector2d &pt0, const Vector2d &pt1, const Vector2d &pt);
	double ComputeTrajectoryAndObstacleCost(Vector5d x, const std::vector<Vector2d>& obstacle, Traj& traj);
};