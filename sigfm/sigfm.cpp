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
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

namespace {
bool sigfm_debug_enabled ()
{
  static const bool enabled = std::getenv ("SIGFM_DEBUG") != nullptr;
  return enabled;
}

#define SIGFM_LOG(...)                                            \
  do {                                                            \
    if (sigfm_debug_enabled ())                                   \
      {                                                           \
        std::fprintf (stderr, "sigfm: " __VA_ARGS__);             \
        std::fputc ('\n', stderr);                                \
      }                                                           \
  } while (0)
} // namespace

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
/* Lowe's ratio is famously unreliable on fingerprints — every ridge keypoint
 * looks similar to its neighbors so the 2nd-best match is nearly as close as
 * the best, killing the ratio test.  Loosened from 0.70 to 0.90 and the
 * mutual-best filter dropped; RANSAC + scale/rotation gates downstream still
 * reject geometrically inconsistent matches. */
constexpr double distance_match = 0.90;
constexpr int min_match = 6;
constexpr double min_inlier_ratio = 0.20;
constexpr double ransac_reproj_threshold = 5.0;
constexpr double max_rotation_degrees = 45.0;
constexpr double min_scale = 0.60;
constexpr double max_scale = 1.70;
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
      matcher->knnMatch (frame->descriptors, enrolled->descriptors, points, 2);

      std::vector<cv::DMatch> ratio_matches;
      for (const auto &pair : points)
        {
          if (pair.empty ())
            continue;

          if (pair.size () < 2)
            {
              ratio_matches.push_back (pair[0]);
              continue;
            }

          const cv::DMatch &first = pair[0];
          const cv::DMatch &second = pair[1];
          if (first.distance < distance_match * second.distance)
            ratio_matches.push_back (first);
        }

      SIGFM_LOG ("descriptors probe=%d enrolled=%d ratio_matches=%zu",
                 frame->descriptors.rows, enrolled->descriptors.rows,
                 ratio_matches.size ());

      if (ratio_matches.size () < min_match)
        {
          SIGFM_LOG ("reject: ratio_matches=%zu < min_match=%d",
                     ratio_matches.size (), min_match);
          return 0;
        }

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

      SIGFM_LOG ("unique_matches=%zu", unique_matches.size ());

      if (unique_matches.size () < min_match)
        {
          SIGFM_LOG ("reject: unique_matches=%zu < min_match=%d",
                     unique_matches.size (), min_match);
          return 0;
        }

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
        {
          SIGFM_LOG ("reject: RANSAC failed (affine.empty=%d inliers.empty=%d)",
                     affine.empty (), inliers.empty ());
          return 0;
        }

      cv::Mat affine64;
      affine.convertTo (affine64, CV_64F);

      double a = affine64.at<double> (0, 0);
      double b = affine64.at<double> (1, 0);
      double scale = std::sqrt (a * a + b * b);
      double rotation = std::abs (std::atan2 (b, a) * 180.0 / pi);
      int count = cv::countNonZero (inliers);
      double inlier_ratio =
        static_cast<double> (count) / static_cast<double> (unique_matches.size ());

      SIGFM_LOG ("RANSAC scale=%.3f rotation=%.1f deg inliers=%d/%zu (ratio=%.3f)",
                 scale, rotation, count, unique_matches.size (), inlier_ratio);

      if (scale < min_scale || scale > max_scale || rotation > max_rotation_degrees)
        {
          SIGFM_LOG ("reject: scale=%.3f (need %.2f..%.2f) rotation=%.1f (max %.1f)",
                     scale, min_scale, max_scale, rotation, max_rotation_degrees);
          return 0;
        }

      if (count < min_match)
        {
          SIGFM_LOG ("reject: inlier count=%d < min_match=%d", count, min_match);
          return 0;
        }

      if (inlier_ratio < min_inlier_ratio)
        {
          SIGFM_LOG ("reject: inlier_ratio=%.3f < min=%.2f",
                     inlier_ratio, min_inlier_ratio);
          return 0;
        }

      SIGFM_LOG ("accept: score=%d", count);
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
