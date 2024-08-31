#pragma once

#include <atomic>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cameraserver/CameraServer.h>

#include "functions/DataContainer.h"

namespace WSHK
{
namespace _2024
{
namespace Func
{
#if defined(__linux__)
void VisionThread();
#endif
} // namespace Func
} // namespace _2024
} // namespace WSHK