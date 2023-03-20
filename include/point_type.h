#ifndef POINT_TYPE_H__
#define POINT_TYPE_H__

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_base.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

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


#endif // POINT_TYPE_H__

