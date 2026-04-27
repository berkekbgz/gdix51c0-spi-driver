// SIGFM algorithm for libfprint drivers.

#include "sigfm.hpp"

#include "binary.hpp"
#include "img-info.hpp"

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <set>
#include <tuple>
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
constexpr double distance_match = 0.85;
constexpr double length_match = 0.05;
constexpr double angle_match = 0.05;
constexpr int min_match = 5;
constexpr double pi = 3.14159265358979323846;

struct match {
  cv::Point2i p1;
  cv::Point2i p2;

  match (cv::Point2i ip1, cv::Point2i ip2) : p1 { ip1 }, p2 { ip2 } {}

  bool operator< (const match &right) const
  {
    return std::tie (p1.y, p1.x, p2.y, p2.x) <
           std::tie (right.p1.y, right.p1.x, right.p2.y, right.p2.x);
  }
};

struct angle_pair {
  double cos;
  double sin;
};
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
      std::vector<std::vector<cv::DMatch>> points;
      cv::BFMatcher::create ()->knnMatch (frame->descriptors, enrolled->descriptors, points, 2);

      std::set<match> unique_matches;
      for (const auto &pair : points)
        {
          if (pair.size () < 2)
            continue;

          const cv::DMatch &first = pair[0];
          const cv::DMatch &second = pair[1];
          if (first.distance < distance_match * second.distance)
            unique_matches.emplace (frame->keypoints[first.queryIdx].pt,
                                    enrolled->keypoints[first.trainIdx].pt);
        }

      if (unique_matches.size () < min_match)
        return 0;

      std::vector<match> matches { unique_matches.begin (), unique_matches.end () };
      std::vector<angle_pair> angles;

      for (std::size_t j = 0; j < matches.size (); j++)
        {
          for (std::size_t k = j + 1; k < matches.size (); k++)
            {
              const match &a = matches[j];
              const match &b = matches[k];
              double v1x = a.p1.x - b.p1.x;
              double v1y = a.p1.y - b.p1.y;
              double v2x = a.p2.x - b.p2.x;
              double v2y = a.p2.y - b.p2.y;
              double len1 = std::sqrt (v1x * v1x + v1y * v1y);
              double len2 = std::sqrt (v2x * v2x + v2y * v2y);

              if (len1 <= 0 || len2 <= 0)
                continue;
              if (1.0 - std::min (len1, len2) / std::max (len1, len2) > length_match)
                continue;

              double product = len1 * len2;
              double dot = std::clamp ((v1x * v2x + v1y * v2y) / product, -1.0, 1.0);
              double cross = std::clamp ((v1x * v2y - v1y * v2x) / product, -1.0, 1.0);
              angles.push_back ({ std::acos (cross), pi / 2.0 + std::asin (dot) });
            }
        }

      if (angles.size () < min_match)
        return 0;

      int count = 0;
      for (std::size_t j = 0; j < angles.size (); j++)
        {
          for (std::size_t k = j + 1; k < angles.size (); k++)
            {
              const angle_pair &a = angles[j];
              const angle_pair &b = angles[k];
              if (1.0 - std::min (a.sin, b.sin) / std::max (a.sin, b.sin) <= angle_match &&
                  1.0 - std::min (a.cos, b.cos) / std::max (a.cos, b.cos) <= angle_match)
                count++;
            }
        }

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
