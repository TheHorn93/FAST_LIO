#pragma once
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include "point_type.h"

class PCLFilterBase
{
public:
  struct Params
  {
    double max_var_mult;
    int h_filter_size;
    int w_filter_size;
    int width;
    int height;
  };

  Params& getParams();
  const Params& getParams() const;

private:
  Params m_params;
};

template<class _PtTp>
class PCLFilter
  : public PCLFilterBase
{
public:
  typedef pcl::PointCloud<_PtTp> PointCloudTp;

  void filterOutlier( const PointCloudTp& pc_in, std::vector<float>& new_int ) const;
  void filterOutlierCloud( const PointCloudTp& pc_in, std::vector<float>& new_int ) const;
  void normalizeIntensity( const PointCloudTp& pc_in, std::vector<float>& new_int ) const;

private:
  bool filterOutlierPoint( const PointCloudTp& pc_in, std::vector<float>& new_int, size_t h_it, size_t w_it ) const;
};
