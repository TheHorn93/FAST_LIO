#ifndef POINT_TYPE_H__
#define POINT_TYPE_H__

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_base.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

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
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint8_t  ring;
      //uint16_t noise;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros


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
    };
    float data_c[4];
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
)


template<class _PtTp>
static bool isValidPoint( const _PtTp& pt_in )
{
  return ( !std::isnan(pt_in.x) && !std::isnan(pt_in.y) && !std::isnan(pt_in.z) );
}

#endif // POINT_TYPE_H__

