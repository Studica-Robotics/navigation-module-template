#include <thread>

#include "Util.h"
#include "Interface.h"

int GetTime()
{
    static std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
}
void delay(int ms, int past_time)
{
    if (ms < 0)
        return;
    if (past_time == -1)
    {
        for (int i = 0; i < ms / 10; i++)
        {
            if (Interface::EMS)
                return;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (ms % 10)
            std::this_thread::sleep_for(std::chrono::milliseconds(ms % 10));
    }
    else if (past_time == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    else
    {
        ms -= GetTime() - past_time;
        if (ms > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
}
double Trans(double Min, double Max, double min, double max, double value, bool limit_value)
{
    double rv;
    int dir = sign(value);
    value = std::abs(value);
    
    if(limit_value)
    {
        if(value < Min) value = Min;
        else if(value > Max) value = Max;
    }

    rv = (value - Min) / (Max - Min);
    rv *= max - min;

    return (rv + min) * dir;
}
void GetRobotVertices(const Vector3d robot_pose, std::array<std::array<double, 2>, 4> &vertices, int offset)
{
    vertices[0][0] = Constants::ROBOT_HEIGHT / 2.0 + offset;
    vertices[0][1] = -Constants::ROBOT_WIDTH / 2.0 - offset;

    vertices[1][0] = Constants::ROBOT_HEIGHT / 2.0 + offset;
    vertices[1][1] = Constants::ROBOT_WIDTH / 2.0 + offset;

    vertices[2][0] = -Constants::ROBOT_HEIGHT / 2.0 - offset;
    vertices[2][1] = Constants::ROBOT_WIDTH / 2.0 + offset;

    vertices[3][0] = -Constants::ROBOT_HEIGHT / 2.0 - offset;
    vertices[3][1] = -Constants::ROBOT_WIDTH / 2.0 - offset;

    double x, y, th;
    for (int i = 0; i < 4; i++)
    {
        x = vertices[i][0] + Constants::ROBOT_CENTER_OFFSET;
        y = vertices[i][1];

        th = robot_pose[2];

        vertices[i][0] = x * std::cos(th) - y * std::sin(th) + robot_pose[0];
        vertices[i][1] = x * std::sin(th) + y * std::cos(th) + robot_pose[1];
    }
}
void GetRobotCenterPosition(const Vector3d robot_pose, Vector3d &output)
{
    output[0] = robot_pose[0] + Constants::ROBOT_CENTER_OFFSET * std::cos(robot_pose[2]);
    output[1] = robot_pose[1] + Constants::ROBOT_CENTER_OFFSET * std::sin(robot_pose[2]);
    output[2] = robot_pose[2];
}
Vector2i ConvertToGridCoordinate(const Vector3d input)
{
	Vector2i output;
	
	output[0] = std::round(Constants::ROBOT_X_OFFSET - input[0] / Constants::MM_PER_PIXEL);
	output[1] = std::round(Constants::ROBOT_Y_OFFSET + input[1] / Constants::MM_PER_PIXEL);

	return output;
}
Vector3d ConvertToWorldCoordinate(const Vector2i input)
{
	Vector3d output;

	output[0] = -(input[0] - Constants::ROBOT_X_OFFSET) * Constants::MM_PER_PIXEL;
	output[1] = (input[1] - Constants::ROBOT_Y_OFFSET) * Constants::MM_PER_PIXEL;
    output[2] = 0;

	return output;
}