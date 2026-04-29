// SIGFM algorithm for libfprint drivers.

#include "sigfm.hpp"

#include "binary.hpp"
#include "img-info.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

namespace bin {
template<>
struct serializer<SigfmImgInfo> : public std::true_type {
  static void serialize (const SigfmImgInfo &info, stream &out)
  {
    out << info.keypoints << info.descriptors;
  }
};

template<>
struct deserializer<SigfmImgInfo> : public std::true_type {
  static SigfmImgInfo deserialize (stream &in)
  {
    SigfmImgInfo info;
    in >> info.keypoints >> info.descriptors;
    return info;
  }
};
} // namespace bin

namespace {
constexpr double distance_match = 0.70;
constexpr int min_match = 8;
constexpr double min_inlier_ratio = 0.35;
constexpr double ransac_reproj_threshold = 3.0;
constexpr double max_rotation_degrees = 35.0;
constexpr double min_scale = 0.65;
constexpr double max_scale = 1.55;
constexpr double pi = 3.14159265358979323846;
} // namespace

SigfmImgInfo *
sigfm_copy_info (SigfmImgInfo *info)
{
  return info ? new SigfmImgInfo { *info } : nullptr;
}

int
sigfm_keypoints_count (SigfmImgInfo *info)
{
  return info ? static_cast<int> (info->keypoints.size ()) : 0;
}

unsigned char *
sigfm_serialize_binary (SigfmImgInfo *info, int *outlen)
{
  if (!info || !outlen)
    return nullptr;

  bin::stream stream;
  stream << *info;
  *outlen = static_cast<int> (stream.size ());
  return stream.copy_buffer ();
}

SigfmImgInfo *
sigfm_deserialize_binary (const unsigned char *bytes, int len)
{
  if (!bytes || len <= 0)
    return nullptr;

  try
    {
      bin::stream stream { bytes, bytes + len };
      auto info = std::make_unique<SigfmImgInfo> ();
      stream >> *info;
      return info.release ();
    }
  catch (...)
    {
      return nullptr;
    }
}

SigfmImgInfo *
sigfm_extract (const SigfmPix *pix, int width, int height)
{
  if (!pix || width <= 0 || height <= 0)
    return nullptr;

  cv::Mat image (height, width, CV_8UC1);
  std::memcpy (image.data, pix, static_cast<std::size_t> (width * height));

  cv::Mat enhanced;
  cv::createCLAHE (4.0, cv::Size (4, 4))->apply (image, enhanced);

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  cv::SIFT::create ()->detectAndCompute (enhanced, cv::noArray (), keypoints, descriptors);

  return new SigfmImgInfo { keypoints, descriptors };
}

int
sigfm_match_score (SigfmImgInfo *frame, SigfmImgInfo *enrolled)
{
  if (!frame || !enrolled || frame->descriptors.empty () || enrolled->descriptors.empty ())
    return 0;

  try
    {
      auto matcher = cv::BFMatcher::create ();
      std::vector<std::vector<cv::DMatch>> points;
      std::vector<std::vector<cv::DMatch>> reverse_points;
      matcher->knnMatch (frame->descriptors, enrolled->descriptors, points, 2);
      matcher->knnMatch (enrolled->descriptors, frame->descriptors, reverse_points, 2);

      std::vector<int> reverse_best (enrolled->descriptors.rows, -1);
      for (const auto &pair : reverse_points)
        {
          if (pair.size () < 2)
            continue;

          const cv::DMatch &first = pair[0];
          const cv::DMatch &second = pair[1];
          if (first.distance < distance_match * second.distance)
            reverse_best[first.queryIdx] = first.trainIdx;
        }

      std::vector<cv::DMatch> ratio_matches;
      for (const auto &pair : points)
        {
          if (pair.size () < 2)
            continue;

          const cv::DMatch &first = pair[0];
          const cv::DMatch &second = pair[1];
          if (first.distance < distance_match * second.distance &&
              reverse_best[first.trainIdx] == first.queryIdx)
            ratio_matches.push_back (first);
        }

      if (ratio_matches.size () < min_match)
        return 0;

      std::sort (ratio_matches.begin (), ratio_matches.end (),
                 [] (const cv::DMatch &a, const cv::DMatch &b) {
                   return a.distance < b.distance;
                 });

      std::vector<cv::DMatch> unique_matches;
      std::vector<bool> used_query (frame->keypoints.size (), false);
      std::vector<bool> used_train (enrolled->keypoints.size (), false);

      for (const auto &match : ratio_matches)
        {
          if (used_query[match.queryIdx] || used_train[match.trainIdx])
            continue;

          used_query[match.queryIdx] = true;
          used_train[match.trainIdx] = true;
          unique_matches.push_back (match);
        }

      if (unique_matches.size () < min_match)
        return 0;

      std::vector<cv::Point2f> frame_points;
      std::vector<cv::Point2f> enrolled_points;

      for (const auto &match : unique_matches)
        {
          frame_points.push_back (frame->keypoints[match.queryIdx].pt);
          enrolled_points.push_back (enrolled->keypoints[match.trainIdx].pt);
        }

      cv::Mat inliers;
      cv::Mat affine = cv::estimateAffinePartial2D (frame_points,
                                                    enrolled_points,
                                                    inliers,
                                                    cv::RANSAC,
                                                     ransac_reproj_threshold,
                                                    2000,
                                                    0.995,
                                                    10);
      if (affine.empty () || inliers.empty ())
        return 0;

      cv::Mat affine64;
      affine.convertTo (affine64, CV_64F);

      double a = affine64.at<double> (0, 0);
      double b = affine64.at<double> (1, 0);
      double scale = std::sqrt (a * a + b * b);
      double rotation = std::abs (std::atan2 (b, a) * 180.0 / pi);

      if (scale < min_scale || scale > max_scale || rotation > max_rotation_degrees)
        return 0;

      int count = cv::countNonZero (inliers);
      if (count < min_match)
        return 0;

      if (static_cast<double> (count) / static_cast<double> (unique_matches.size ()) < min_inlier_ratio)
        return 0;

      return count;
    }
  catch (...)
    {
      return -1;
    }
}

void
sigfm_free_info (SigfmImgInfo *info)
{
  delete info;
}
