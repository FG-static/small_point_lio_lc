#ifndef SMALL_DLIO__SMALL_GICP_COMPAT_HPP
#define SMALL_DLIO__SMALL_GICP_COMPAT_HPP

#include <unordered_map>

// FLANN (used by small_gicp) expects a serialization implementation for
// std::unordered_map when saving/loading LSH indices. The system FLANN
// version shipped with Ubuntu does not provide this specialization, so we
// provide it before the full FLANN serialization headers are included.
namespace flann {
namespace serialization {

template<typename T>
struct Serializer;

template<typename K, typename V>
struct Serializer<std::unordered_map<K, V>>
{
  template<typename InputArchive>
  static inline void load(InputArchive &ar, std::unordered_map<K, V> &map_val)
  {
    size_t size;
    ar & size;
    for (size_t i = 0; i < size; ++i) {
      K key;
      V value;
      ar & key;
      ar & value;
      map_val.emplace(std::move(key), std::move(value));
    }
  }

  template<typename OutputArchive>
  static inline void save(OutputArchive &ar, const std::unordered_map<K, V> &map_val)
  {
    ar & map_val.size();
    for (const auto &kv : map_val) {
      ar & kv.first;
      ar & kv.second;
    }
  }
};

}  // namespace serialization
}  // namespace flann

#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/registration/registration.hpp>

#endif  // SMALL_DLIO__SMALL_GICP_COMPAT_HPP
