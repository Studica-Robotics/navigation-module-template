#pragma once

#include <array>

class Move
{
public:
    static void Stop();
    static void filter(int x, int y, int w, bool acc_flg = true);

private:
    static int prev_filter_time_;
};