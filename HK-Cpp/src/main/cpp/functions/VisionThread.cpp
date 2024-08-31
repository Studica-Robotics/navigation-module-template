#include "functions/VisionThread.h"

using namespace WSHK::_2024;

void Func::VisionThread()
{
  // Setup a CvSource. This will send images back to the Dashboard
  cs::CvSource outputStream =
      frc::CameraServer::GetInstance()->PutVideo("LiDAR_raw", 950, 950);
  // Mats are very memory expensive. Lets reuse this Mat.
  cv::Mat mat, plot, gray, edges, res, cntFiltered, roiL;

  // Create a kernel
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  while (true)
  {
    mat.release();
    plot.release();
    gray.release();
    edges.release();
    res.release();
    cntFiltered.release();
    roiL.release();

    // Logger::log("1");

    mat = cv::Mat(900, 900, CV_8UC3, cv::Scalar(0, 0, 0));
    plot = mat.clone();
    res = mat.clone();
    cntFiltered = cv::Mat::zeros(mat.size(), CV_8UC1);

    std::vector<double> LiDARData = DataContainer::get_LiDAR_data();
    if (LiDARData.size() > 0)
    {
      cv::Scalar color(0, 0, 255); // Black text
      // Limit the value of each element to 2000
      for (size_t i = 0; i < LiDARData.size(); i++)
      {
        // std::vector<size_t> vec;
        // // for (size_t i = 120; i < 240; ++i)
        // for (size_t i = constant::filterAngleRange[0]; i < constant::filterAngleRange[1]; ++i)
        // {
        //   vec.push_back(i);
        // }

        // if ((std::find(vec.begin(), vec.end(), i) != vec.end()) || LiDARData[i] < 30)
        // {
        //   continue;
        // }

        // LiDARData[i] = std::min(LiDARData[i], 5000.0) * 950 / 5000;
        LiDARData[i] = LiDARData[i] * 450 / 2250;
        // LiDARData[i] = std::min(LiDARData[i], 489.0);
        std::array<double, 2> pt = DataContainer::polarToCartesian2(LiDARData[i], i);
        // cv::circle(plot, cv::Point(pt[0] + 450, pt[1]+ 750), 2, cv::Scalar(255,255,255), -1);
        cv::circle(plot, cv::Point(pt[0] + 450, pt[1] + 450), 5, cv::Scalar(255, 255, 255), -1);
      }

      // cv::circle(plot, cv::Point(450, 450), 30, cv::Scalar(255, 255, 255), 2);
      outputStream.PutFrame(plot);

    }

  }
}
