#ifndef ARUCO_FINDER_H
#define ARUCO_FINDER_H

#include <boost/optional.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <rct_optimizations/types.h>
#include <map>
#include <Eigen/Core>
#include <iostream>

namespace rct_image_tools
{

/**
 * @brief
 */
class ArucoGridBoardObservationFinder
{
public:
  ArucoGridBoardObservationFinder(const cv::Ptr<cv::aruco::GridBoard>& board);

  boost::optional<std::map<int, std::vector<Eigen::Vector2d>>> findObservations(const cv::Mat& image) const;

  /**
   * @brief A debugging utility that will draw an observation set onto a copy of a cv::Mat for
   * display purposes. Usually you want to call findObservations() above then this with the result.
   */
  cv::Mat drawObservations(const cv::Mat& image, const std::map<int, std::vector<Eigen::Vector2d>>& observations) const;

  const cv::Ptr<cv::aruco::GridBoard>& target() const { return board_; }

private:
  cv::Ptr<cv::aruco::GridBoard> board_;
};

std::map<int, std::vector<Eigen::Vector3d>>
mapArucoIdsToObjPts(const cv::Ptr<cv::aruco::GridBoard>& board);

}
#endif // ARUCO_FINDER_H
