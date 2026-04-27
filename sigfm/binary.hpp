#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace bin {
using byte = unsigned char;

class stream;

template<typename T>
struct serializer : public std::false_type {};

template<typename T>
struct deserializer : public std::false_type {};

class stream {
public:
  stream () = default;

  template<typename Iter,
           std::enable_if_t<std::is_same_v<typename std::iterator_traits<std::decay_t<Iter>>::value_type,
                                           byte>, bool> = true>
  stream (Iter begin, Iter end) : store_ { begin, end }
  {
  }

  template<typename T, std::enable_if_t<serializer<T>::value, bool> = true>
  stream &operator<< (const T &v)
  {
    serializer<T>::serialize (v, *this);
    return *this;
  }

  template<typename T, std::enable_if_t<deserializer<T>::value, bool> = true>
  stream &operator>> (T &v)
  {
    v = deserializer<T>::deserialize (*this);
    return *this;
  }

  template<typename T, std::enable_if_t<std::is_trivial_v<T>, bool> = true>
  stream &operator<< (const T &v)
  {
    std::array<byte, sizeof (T)> bytes = {};
    std::memcpy (bytes.data (), &v, sizeof (T));
    return write (bytes.begin (), bytes.end ());
  }

  template<typename T, std::enable_if_t<std::is_trivial_v<T>, bool> = true>
  stream &operator>> (T &v)
  {
    std::array<byte, sizeof (T)> bytes = {};
    read (bytes.begin (), bytes.end ());
    std::memcpy (&v, bytes.data (), sizeof (T));
    return *this;
  }

  template<typename Iter,
           std::enable_if_t<std::is_same_v<typename std::iterator_traits<std::decay_t<Iter>>::value_type,
                                           byte>, bool> = true>
  stream &write (Iter begin, Iter end)
  {
    std::copy (begin, end, std::back_inserter (store_));
    return *this;
  }

  template<typename Iter,
           std::enable_if_t<std::is_same_v<typename std::iterator_traits<std::decay_t<Iter>>::value_type,
                                           byte>, bool> = true>
  stream &read (Iter begin, Iter end)
  {
    auto len = static_cast<std::size_t> (std::distance (begin, end));
    if (store_.size () < len)
      throw std::runtime_error { "tried to read from too small stream" };

    std::copy (store_.begin (), store_.begin () + len, begin);
    store_.erase (store_.begin (), store_.begin () + len);
    return *this;
  }

  byte *copy_buffer () const
  {
    byte *raw = static_cast<byte *> (std::malloc (store_.size ()));
    if (raw)
      std::copy (store_.begin (), store_.end (), raw);
    return raw;
  }

  std::size_t size () const { return store_.size (); }

private:
  std::vector<byte> store_;
};

template<>
struct serializer<cv::Mat> : public std::true_type {
  static void serialize (const cv::Mat &m, stream &out)
  {
    cv::Mat continuous = m.isContinuous () ? m : m.clone ();
    int type = continuous.type ();
    int rows = continuous.rows;
    int cols = continuous.cols;
    auto len = static_cast<std::size_t> (continuous.total () * continuous.elemSize ());

    out << type << rows << cols << len;
    out.write (continuous.datastart, continuous.datastart + len);
  }
};

template<>
struct deserializer<cv::Mat> : public std::true_type {
  static cv::Mat deserialize (stream &in)
  {
    int type = 0;
    int rows = 0;
    int cols = 0;
    std::size_t len = 0;
    in >> type >> rows >> cols >> len;

    cv::Mat m;
    m.create (rows, cols, type);
    auto expected = static_cast<std::size_t> (m.total () * m.elemSize ());
    if (expected != len)
      throw std::runtime_error { "serialized matrix size mismatch" };

    in.read (m.data, m.data + expected);
    return m;
  }
};

template<typename T>
struct serializer<cv::Point_<T>> : public std::true_type {
  static void serialize (const cv::Point_<T> &pt, stream &out)
  {
    out << pt.x << pt.y;
  }
};

template<typename T>
struct deserializer<cv::Point_<T>> : public std::true_type {
  static cv::Point_<T> deserialize (stream &in)
  {
    cv::Point_<T> pt;
    in >> pt.x >> pt.y;
    return pt;
  }
};

template<>
struct serializer<cv::KeyPoint> : public std::true_type {
  static void serialize (const cv::KeyPoint &pt, stream &out)
  {
    out << pt.class_id << pt.angle << pt.octave << pt.response << pt.size << pt.pt;
  }
};

template<>
struct deserializer<cv::KeyPoint> : public std::true_type {
  static cv::KeyPoint deserialize (stream &in)
  {
    cv::KeyPoint pt;
    in >> pt.class_id >> pt.angle >> pt.octave >> pt.response >> pt.size >> pt.pt;
    return pt;
  }
};

template<typename T>
struct serializer<std::vector<T>> : public std::true_type {
  static void serialize (const std::vector<T> &values, stream &out)
  {
    out << values.size ();
    for (const auto &value : values)
      out << value;
  }
};

template<typename T>
struct deserializer<std::vector<T>> : public std::true_type {
  static std::vector<T> deserialize (stream &in)
  {
    std::size_t size = 0;
    in >> size;

    std::vector<T> values;
    values.reserve (size);
    for (std::size_t i = 0; i < size; i++)
      {
        T value;
        in >> value;
        values.emplace_back (std::move (value));
      }

    return values;
  }
};
} // namespace bin
