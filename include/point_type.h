#ifndef POINT_TYPE_H__
#define POINT_TYPE_H__

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_base.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

template <class ...T>
constexpr bool always_false = false;

constexpr float min_reflectance_full = 50.;
constexpr float min_reflectance = min_reflectance_full / 65535.;

#ifdef DEBUG
  #define DEBUG_OUT(s) \
    std::cout << "DEBUG: " << s << std::endl;
#else
  #define DEBUG_OUT(s) {};
#endif // DEBUG

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      std::uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

namespace hesai_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      double timestamp;
      std::uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint32_t t;
      std::uint16_t reflectivity;
      std::uint8_t  ring;
      //std::uint16_t noise;
      std::uint16_t ambient;
      std::uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

POINT_CLOUD_REGISTER_POINT_STRUCT( ouster_ros::Point,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint32_t, t, t)
  (std::uint16_t, reflectivity, reflectivity)
  (std::uint8_t, ring, ring)
  //(std::uint16_t, noise, noise)
  (std::uint16_t, ambient, ambient)
  (std::uint32_t, range, range)
)

struct EIGEN_ALIGN16 PointType
{
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  union
  {
    struct
    {
      float intensity;
      float curvature;
      float reflectance;
      float gloss;
      float gradient_mag;
      float intensity_count;
      float intensity_variance;
    };
    float data_c[8];
  };
  PCL_MAKE_ALIGNED_OPERATOR_NEW;
};

POINT_CLOUD_REGISTER_POINT_STRUCT( PointType,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, normal_x, normal_x)
  (float, normal_y, normal_y)
  (float, normal_z, normal_z)
  (float, intensity, intensity)
  (float, curvature, curvature)
  (float, reflectance, reflectance)
  (float, gloss, gloss)
  (float, gradient_mag, gradient_mag)
  (float, intensity_count, intensity_count)
  (float, intensity_variance, intensity_variance)
)


template<class _PtTp>
static bool isValidPoint( const _PtTp& pt_in )
{
  return ( !std::isnan(pt_in.x) && !std::isnan(pt_in.y) && !std::isnan(pt_in.z) );
}
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef pcl::PointCloud<ouster_ros::Point> PointCloudOuster;
typedef pcl::PointCloud<hesai_ros::Point> PointCloudHesai;

#endif // POINT_TYPE_H__

