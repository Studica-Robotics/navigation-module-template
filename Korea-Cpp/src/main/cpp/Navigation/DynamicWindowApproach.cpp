#include <algorithm>
#include <limits>
#include <cmath>

#include <opencv2/imgproc/imgproc.hpp>

#include "Navigation/DynamicWindowApproach.h"

DynamicWindowApproach::DynamicWindowApproach()
{
	kdtree_ = new KDTree<Vector2d>();
}
DynamicWindowApproach::~DynamicWindowApproach()
{
	delete kdtree_;
}

void DynamicWindowApproach::Init()
{
	error_flg_ = false;
	stuck_cnt_ = 0;
}
void DynamicWindowApproach::Update(const std::vector<Vector2d>& obstacle, const Vector2d& goal, Vector5d& control, Traj& traj)
{
	double dynamic_window[4];

	Vector5d x;
	Vector2d min_u;

	min_u[0] = 0;
	min_u[1] = 0;

	
	Traj best_traj, current_traj;
	if(!obstacle.empty()) kdtree_->build(obstacle);

	if(DRIVE_TYPE == Mecanum_Drive)
	{
		dynamic_window[0] = 0;
		dynamic_window[1] = Constants::MAX_VELOCITY[0];
		dynamic_window[2] = ToRadian(-120);
		dynamic_window[3] = ToRadian(120);
	}
	// else if(DRIVE_TYPE == Mecanum_Drive)
	// {
	// 	dynamic_window[0] = 0;
	// 	dynamic_window[1] = Constants::MAX_VELOCITY[0];
	// 	dynamic_window[2] = ToRadian(-90);
	// 	dynamic_window[3] = ToRadian(90);
	// }
	else
	{
		dynamic_window[0] = std::max((control[3] - max_accel * dt), min_speed);
		dynamic_window[1] = std::min((control[3] + max_accel * dt), max_speed);

		double tmp;
		if(dynamic_window[1] < dynamic_window[0])
		{
			tmp = dynamic_window[0];
			dynamic_window[0] = dynamic_window[1];
			dynamic_window[1] = tmp;
		}

		dynamic_window[2] = ToRadian(-Constants::MAX_VELOCITY[1]);
		dynamic_window[3] = ToRadian(Constants::MAX_VELOCITY[1]);
	}
	
	trajectories_.clear();
	double min_cost = std::numeric_limits<double>::max();
	for (double v = dynamic_window[0]; v <= dynamic_window[1]; v += v_resolution)
	{
		if(DRIVE_TYPE == Mecanum_Drive && v > 0 && v < Constants::MIN_VELOCITY[0]) continue;

		for (double w = dynamic_window[2]; w <= dynamic_window[3]; w += yawrate_resolution)
		{
			if(DRIVE_TYPE == Mecanum_Drive && std::abs(w) > 0 && std::abs(w) < ToRadian(20)) continue;
			
			x[0] = control[0], x[1] = control[1], x[2] = control[2];
			x[3] = v, x[4] = w;
			
			double obstacle_cost = ComputeTrajectoryAndObstacleCost(x, obstacle, current_traj);
			double goal_cost = GetDistance(Vector2d{ current_traj.back()[0], current_traj.back()[1] }, goal) * goal_cost_gain;
			double speed_cost = std::abs(speed_cost_gain * (max_speed - current_traj.back()[3]));
			double final_cost = goal_cost + speed_cost + obstacle_cost;

			if (min_cost > final_cost)
			{
				min_cost = final_cost;
				min_u[0] = v;
				min_u[1] = w;
				best_traj = current_traj;
			}
			trajectories_.push_back(current_traj);
		}
	}
	if(DRIVE_TYPE != Mecanum_Drive && min_u[0] == 0 && min_u[1] == 0)
	{
		min_cost = std::numeric_limits<double>::max();
		for (double w = dynamic_window[2]; w <= dynamic_window[3]; w += yawrate_resolution)
		{
			x[0] = control[0], x[1] = control[1], x[2] = control[2];
			x[3] = 0, x[4] = w;
			
			double obstacle_cost = ComputeTrajectoryAndObstacleCost(x, obstacle, current_traj);
			double goal_cost = GetDistance(Vector2d{ current_traj.back()[0], current_traj.back()[1] }, goal) * goal_cost_gain;
			double speed_cost = std::abs(speed_cost_gain * (max_speed - current_traj.back()[3]));
			double final_cost = goal_cost + speed_cost + obstacle_cost;

			if (min_cost > final_cost)
			{
				min_cost = final_cost;
				min_u[0] = 0;
				min_u[1] = w;
				best_traj = current_traj;
			}
			trajectories_.push_back(current_traj);
		}
	}

	if(DRIVE_TYPE == SixWheel_Drive && min_u[0] == 0 && min_u[1] == 0)
	{
		min_cost = std::numeric_limits<double>::max();
		for (double v = -100; v <= 0; v += v_resolution)
		{
			for (double w = dynamic_window[2]; w <= dynamic_window[3]; w += yawrate_resolution)
			{
				x[0] = control[0], x[1] = control[1], x[2] = control[2];
				x[3] = v, x[4] = w;
				
				double obstacle_cost = ComputeTrajectoryAndObstacleCost(x, obstacle, current_traj);
				double goal_cost = GetDistance(Vector2d{ current_traj.back()[0], current_traj.back()[1] }, goal) * goal_cost_gain;
				double speed_cost = std::abs(speed_cost_gain * (max_speed - current_traj.back()[3]));
				double final_cost = goal_cost + speed_cost + obstacle_cost;

				if (min_cost > final_cost)
				{
					min_cost = final_cost;
					min_u[0] = v;
					min_u[1] = w;
					best_traj = current_traj;
				}
				trajectories_.push_back(current_traj);
			}
		}
	}

	traj = best_traj;

	control[3] = min_u[0];
	control[4] = min_u[1];

	if(DRIVE_TYPE == Mecanum_Drive)
	{
		control[4] = min_u[1] * 1.5;

		if(std::abs(control[4]) > ToRadian(80))
		{
			double curvature = control[3] / control[4];
			control[4] = ToRadian(80) * sign(control[4]);
			control[3] = std::abs(control[4] * curvature) * sign(control[3]);

			if(std::abs(control[3]) < 270) control[3] = 0;
		}
	}
	
	if(std::abs(control[3]) < 50 && std::abs(control[4]) < ToRadian(30)) stuck_cnt_++;
	else stuck_cnt_ = 0;

	// if(stuck_cnt_)
	// {
	// 	cout << control[3] << ", " << ToDegree(control[4]) << endl;
	// 	// cout << stuck_cnt_ << ", " << error_flg_ << endl;
	// }

	if(!error_flg_ && stuck_cnt_ >= 5) error_flg_ = true;
	if(error_flg_)
	{
		double theta = NormalizeRadian(std::atan2(goal[1] - Interface::RobotPose[1], goal[0] - Interface::RobotPose[0]) - Interface::RobotPose[2]);
		control[3] = 0;
		control[4] = ToRadian(Constants::MIN_VELOCITY[1]) * sign(theta);

		if(std::abs(theta) < ToRadian(3))
		{
			control[4] = 0;
			stuck_cnt_ = 0, error_flg_ = false;
		}
	}
}
Vector5d DynamicWindowApproach::Motion(const Vector5d& control, double dt)
{
	Vector5d x = control;
	x[2] = NormalizeRadian(x[2] + control[4] * dt);
	x[0] += control[3] * std::cos(x[2]) * dt;
	x[1] += control[3] * std::sin(x[2]) * dt;
	return x;
}
void DynamicWindowApproach::Show(cv::Mat &mat)
{
	Vector2i pose;
	for(Traj traj : trajectories_)
	{
		for (unsigned int i = 0; i < traj.size(); i++)
		{
			pose = ConvertToGridCoordinate(Vector3d{traj[i][0], traj[i][1], 0});
			// cv::circle(mat, cv::Point(pose[1], pose[0]), 1, cv::Scalar(0, 255, 0), -1);
			mat.data[pose[0] * mat.cols * 3 + pose[1] * 3 + 0] = 0;
			mat.data[pose[0] * mat.cols * 3 + pose[1] * 3 + 1] = 255;
			mat.data[pose[0] * mat.cols * 3 + pose[1] * 3 + 2] = 0;
		}
	}

}
double DynamicWindowApproach::GetTriangleArea(const std::array<std::array<double, 2>, 3>& pt)
{
	double calc1 = pt[0][0] * pt[1][1] + pt[1][0] * pt[2][1] + pt[2][0] * pt[0][1];
	double calc2 = pt[1][0] * pt[0][1] + pt[2][0] * pt[1][1] + pt[0][0] * pt[2][1];

	return std::abs(calc1 - calc2) * 0.5;
}
bool DynamicWindowApproach::IsPointInsideRectangle(const Vector2d& pt, const Vector3d& robot_pose, int offset)
{
	double area1 = 0, area2 = 0;
	std::array<std::array<double, 2>, 3> triangle;
	std::array<std::array<double, 2>, 4> vertices;

	GetRobotVertices(robot_pose, vertices, offset);

	triangle[0] = pt;

	triangle[1] = vertices[0];
	triangle[2] = vertices[1];
	area1 += GetTriangleArea(triangle);

	triangle[1] = vertices[1];
	triangle[2] = vertices[2];
	area1 += GetTriangleArea(triangle);

	triangle[1] = vertices[2];
	triangle[2] = vertices[3];
	area1 += GetTriangleArea(triangle);

	triangle[1] = vertices[3];
	triangle[2] = vertices[0];
	area1 += GetTriangleArea(triangle);

	triangle[0] = vertices[0];
	triangle[1] = vertices[1];
	triangle[2] = vertices[2];
	area2 += GetTriangleArea(triangle);

	triangle[0] = vertices[0];
	triangle[1] = vertices[3];
	triangle[2] = vertices[2];
	area2 += GetTriangleArea(triangle);

	return area1 - area2 < 1;
}
double DynamicWindowApproach::SegmentToPointDistance(const Vector2d &pt0, const Vector2d &pt1, const Vector2d &pt)
{
	double A = pt[0] - pt0[0];
	double B = pt[1] - pt0[1];
	double C = pt1[0] - pt0[0];
	double D = pt1[1] - pt0[1];

	double dot = A * C + B * D;
	double len_sq = C * C + D * D;
	double param = -1;
	if (len_sq != 0) param = dot / len_sq;

	double x, y;
	if (param < 0)
	{
		x = pt0[0];
		y = pt0[1];
	}
	else if (param > 1)
	{
		x = pt1[0];
		y = pt1[1];
	}
	else
	{
		x = pt0[0] + param * C;
		y = pt0[1] + param * D;
	}

	double dx = pt[0] - x;
	double dy = pt[1] - y;

	return std::sqrt(dx * dx + dy * dy);
}
double DynamicWindowApproach::ComputeTrajectoryAndObstacleCost(Vector5d x, const std::vector<Vector2d>& obstacle, Traj& traj)
{
	traj.clear();
	traj.push_back(x);
	double min_r = std::numeric_limits<double>::max();
	bool obstacle_exist = !obstacle.empty();

	for (double time = 0; time <= predict_time; time += dt)
	{
		x = Motion(x, dt);

		if(obstacle_exist)
		{
			Vector3d pose;
			GetRobotCenterPosition({x[0], x[1], x[2]}, pose);
			double r;
			int obstacle_idx = kdtree_->nnSearch({pose[0], pose[1]}, &r);

			if (min_r >= r) min_r = r;

			// if(DRIVE_TYPE == SixWheel_Drive)
			// {
			// 	if(obstacle_idx >= 0 && obstacle_idx < int(obstacle.size()) && GetDistance(obstacle[obstacle_idx], Vector2d{pose[0], pose[1]}) < Constants::ROBOT_WIDTH)
			// 		return std::numeric_limits<double>::max();
			// }
			// else
			// {
				if (obstacle_idx >= 0 && obstacle_idx < int(obstacle.size()) && IsPointInsideRectangle(obstacle[obstacle_idx], Vector3d{x[0], x[1], x[2]}))
					return std::numeric_limits<double>::max();
			// }
		}

		traj.push_back(x);
	}
	if(!obstacle_exist) return 0;

	return (robot_radius / min_r) * obstacle_cost_gain;
}