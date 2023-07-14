#pragma once
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include "point_type.h"
#include "lidar_intensity_correction/pc_intensity_correction.h"
#include "lidar_intensity_correction/intensity_compensation.h"

enum PclFilterChannel
{
  PCLFILTER_INTENSITY = 0,
  PCLFILTER_REFLECTIVITY = 1,
  PCLFILTER_AMBIENCE = 2
};


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

  void initCompensationModel( const std::string& type, const std::string& param_str );

protected:
  Params m_params;
  std::unique_ptr<compensation::CompensationModelBase> m_model;
};


template<class _PtTp>
class PCLFilterModelBase
  : public PCLFilterBase
{
public:
  typedef pcl::PointCloud<_PtTp> PointCloudTp;

  virtual void applyFilter( const PointCloudTp& pc_in, std::vector<float>& ints_out ) const = 0;
};


template<class _PtTp, PclFilterChannel _data_channel>
class PCLFilter
  : public PCLFilterModelBase<_PtTp>
{
public:
  typedef pcl::PointCloud<_PtTp> PointCloudTp;

  void applyFilter( const PointCloudTp& pc_in, std::vector<float>& ints_out ) const override;
  void filterOutlier( const PointCloudTp& pc_in, std::vector<float>& new_int ) const;
  void filterOutlierCloud( const PointCloudTp& pc_in, std::vector<float>& new_int ) const;
  void normalizeIntensity( const PointCloudTp& pc_in, std::vector<float>& new_int ) const;
  void applyModel( const PCIntensityComputation& pc_in, std::vector<float>& ints_out ) const;

private:
  bool filterOutlierPoint( const PointCloudTp& pc_in, std::vector<float>& new_int, size_t h_it, size_t w_it ) const;
};

template<class _PointTp>
PCLFilterBase* constructInputFilter( PclFilterChannel channel )
{
  if( channel == PCLFILTER_AMBIENCE )
    return reinterpret_cast<PCLFilterBase*>( new PCLFilter<_PointTp, PCLFILTER_AMBIENCE>() );
  else if( channel == PCLFILTER_INTENSITY )
    return reinterpret_cast<PCLFilterBase*>( new PCLFilter<_PointTp, PCLFILTER_INTENSITY>() );
  else if( channel == PCLFILTER_REFLECTIVITY )
    return reinterpret_cast<PCLFilterBase*>( new PCLFilter<_PointTp, PCLFILTER_REFLECTIVITY>() );
}