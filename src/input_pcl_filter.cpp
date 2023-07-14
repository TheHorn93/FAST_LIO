#include "input_pcl_filter.h"
#include "lidar_intensity_correction/load_files.h"


template<class _Tp, class _ITp>
struct TypeIsEqual
  { static constexpr bool value=false; };
template<class _Tp>
struct TypeIsEqual<_Tp, _Tp>
  { static constexpr bool value=true; };
template<class _Tp, class _ITp>
static constexpr bool type_is_equal = TypeIsEqual<_Tp, _ITp>::value;


template<class _PtTp, PclFilterChannel _data_channel>
struct PointIntensity
{};

template<>
struct PointIntensity<ouster_ros::Point, PclFilterChannel::PCLFILTER_INTENSITY>
{
  using _PtTp = ouster_ros::Point;

  static float get( const typename PCLFilter<_PtTp, PCLFILTER_INTENSITY>::PointCloudTp& pc_in, size_t pt_it )
  {
    return pc_in[pt_it].intensity;
  }
  static void set( typename PCLFilter<_PtTp, PCLFILTER_INTENSITY>::PointCloudTp& pc_in, size_t pt_it, float new_val )
  {
    pc_in[pt_it].intensity = new_val;
  }

  static constexpr float max_val = std::pow(2,16)-1;
};
template<>
struct PointIntensity<ouster_ros::Point, PclFilterChannel::PCLFILTER_REFLECTIVITY>
{
  using _PtTp = ouster_ros::Point;

  static float get( const typename PCLFilter<_PtTp, PCLFILTER_REFLECTIVITY>::PointCloudTp& pc_in, size_t pt_it )
  {
    return pc_in[pt_it].reflectivity;
  }
  static void set( typename PCLFilter<_PtTp, PCLFILTER_REFLECTIVITY>::PointCloudTp& pc_in, size_t pt_it, float new_val )
  {
    pc_in[pt_it].reflectivity = new_val;
  }

  static constexpr float max_val = std::pow(2,16)-1;
};
template<>
struct PointIntensity<ouster_ros::Point, PclFilterChannel::PCLFILTER_AMBIENCE>
{
  using _PtTp = ouster_ros::Point;

  static float get( const typename PCLFilter<_PtTp, PCLFILTER_AMBIENCE>::PointCloudTp& pc_in, size_t pt_it )
  {
    return pc_in[pt_it].ambient;
  }
  static void set( typename PCLFilter<_PtTp, PCLFILTER_AMBIENCE>::PointCloudTp& pc_in, size_t pt_it, float new_val )
  {
    pc_in[pt_it].ambient = new_val;
  }

  static constexpr float max_val = std::pow(2,16)-1;
};



template<class _PtTp, PclFilterChannel _data_channel>
float getIntensity( const typename PCLFilter<_PtTp, _data_channel>::PointCloudTp& pc_in, size_t pt_it )
{
  return PointIntensity<_PtTp, _data_channel>::get( pc_in, pt_it );
}
template<class _PtTp, PclFilterChannel _data_channel>
void setIntensity( typename PCLFilter<_PtTp, _data_channel>::PointCloudTp& pc_in, size_t pt_it, float new_val )
{
  PointIntensity<_PtTp, _data_channel>::set( pc_in, pt_it, new_val );
}



PCLFilterBase::Params& PCLFilterBase::getParams()
{
  return m_params;
}

const PCLFilterBase::Params& PCLFilterBase::getParams() const
{
  return m_params;
}


void PCLFilterBase::initCompensationModel( const std::string& type, const std::string& param_str )
{
  configuru::Config config = loadConfigFromString( param_str );
  ConfiguruLoader cfg_loader( config );
  m_model = std::unique_ptr<compensation::CompensationModelBase>( compensation::getModelFromType( type, cfg_loader ) ); 
  //return nullptr;
}



template<class _PtTp, PclFilterChannel _data_channel>
void PCLFilter<_PtTp, _data_channel>::applyFilter( const PointCloudTp& pc_in, std::vector<float>& ints_out ) const
{
  //std::cout << "Start ";
  normalizeIntensity( pc_in, ints_out );
  filterOutlierCloud( pc_in, ints_out );
  PCIntensityComputation pc_int_cor;
  pc_int_cor.addPC(pc_in, ints_out);
  Eigen::MatrixXd pose = Eigen::MatrixXd::Zero(7,1);
  pose(3,0) = 1;
  pc_int_cor.poses() = pose;
  pc_int_cor.callOrdered( 128, 1024 );
  applyModel( pc_int_cor, ints_out );
  //std::cout << "end " << std::endl;
}

template<class _PtTp, PclFilterChannel _data_channel>
void PCLFilter<_PtTp, _data_channel>::applyModel( const PCIntensityComputation& pc_in, std::vector<float>& ints_out ) const
{
  //pc_out = PointCloudTp();
  const PointCloud<double>& pc = pc_in.getPCs()[0];
  for( size_t pt_it=0; pt_it < pc.numPoints(); ++pt_it )
  {
    double intensity = pc.intensity(pt_it);
    double dist_sqrd = pc_in.distances()[pt_it];
    double angle_cos = std::cos(pc_in.refAngles()[pt_it]);
    if( (std::abs(pc.data()(4, pt_it)) > 0.0 || std::abs(pc.data()(5, pt_it)) > 0.0 || std::abs(pc.data()(6, pt_it)) > 0.0) && angle_cos < 1.5 && intensity > 0.0 )
    {
      ints_out[pt_it] = pc.data()(3, pt_it);
      /*_PtTp new_point;
      new_point.x = pc.data()(0, pt_it);
      new_point.y = pc.data()(1, pt_it);
      new_point.z = pc.data()(2, pt_it);
      new_point.intensity = pc.data()(3, pt_it);
      new_point.ambient = pc.data()(7, pt_it);
      new_point.reflectivity = pc.data()(8, pt_it);
      new_point.ring = pc.pointPoss()(0, pt_it);
      pc_out.push_back(new_point);*/
    }
    else
    {
      ints_out[pt_it] = 0.f;
    }
  }
  //std::cout << pc_out.points.size() << "/" << pc.numPoints() << " inserted" << std::endl;
}



template<class _PtTp, PclFilterChannel _data_channel>
bool PCLFilter<_PtTp, _data_channel>::filterOutlierPoint( const PointCloudTp& pc_in, std::vector<float>& new_int, size_t h_it, size_t w_it ) const
{
  const int32_t& h_filter_size = this->getParams().h_filter_size;
  const int32_t& w_filter_size = this->getParams().w_filter_size;
  int32_t h_min = std::max( int32_t(h_it -h_filter_size), int32_t(0) );
  int32_t h_max = std::min( int32_t(h_it +h_filter_size)+1, int32_t(this->getParams().height) );
  int32_t h_range = h_max -h_min;
  int32_t w_min = std::max( int32_t(w_it -w_filter_size), int32_t(0) );
  int32_t w_max = std::min( int32_t(w_it +w_filter_size)+1, int32_t(this->getParams().width) );
  int32_t w_range = w_max -w_min;
  double num_pts = static_cast<double>(h_range*w_range);

  double mean = 0.0;
  double variance = 0.0;
  size_t cur_pt_it = h_it *this->getParams().width +w_it;
  uint32_t w_offset = this->getParams().width -w_range;

  {
    size_t pt_it = h_min *this->getParams().width +w_min;
    uint32_t num_valid_pts = 0;
    //std::cout << "h_range=[" << h_min << ", " << h_max << "], w_range=[" << w_min << ", " << w_max << "]" << std::endl;
    for( size_t h_it=h_min ; h_it < h_max ; ++h_it )
    {
      for( size_t w_it=w_min ; w_it < w_max ; ++w_it )
      {
        //if( new_int[pt_it] > 1.0 )
          //std::cout << "  @" << pt_it << "=" << new_int[pt_it] << std::endl;
        if( new_int[pt_it] > 0.0 )
        {
          ++num_valid_pts;
          mean += new_int[pt_it];
        }
        ++pt_it;
      }
      pt_it += w_offset;
    }
    if( num_valid_pts == 0 ) return false;
    mean -= new_int[cur_pt_it];
    mean /= num_valid_pts;
    //std::cout << "mean=" << mean << ", " << num_valid_pts << std::endl;
  }

  double diff_cpt;
  {
    size_t pt_it = h_min *this->getParams().width +w_min;
    uint32_t num_valid_pts = 0;
    for( size_t h_it=h_min ; h_it < h_max ; ++h_it )
    {
      for( size_t w_it=w_min ; w_it < w_max ; ++w_it )
      {
        if( new_int[pt_it] > 0.0 )
        {
          ++num_valid_pts;
          double mean_diff = mean -new_int[pt_it];
          variance += mean_diff *mean_diff;
          //std::cout << "var" << pt_it << " = " << variance << std::endl;
        }
        ++pt_it;
        //std::cout << pt_it << ", ";
      }
      pt_it += w_offset;
    }
    diff_cpt = mean -new_int[cur_pt_it];
    //std::cout << "cur_pt_diff=" << diff_cpt << ", var=" << variance << std::endl;
    variance -= diff_cpt*diff_cpt;
    variance /= (num_valid_pts >= 2 ? num_valid_pts -2 : 1); // Bessel correction;
  }

  bool filter_pt = (std::abs(diff_cpt) > (variance *this->getParams().max_var_mult));
  if( filter_pt )
  {
    //ROS_INFO_STREAM( "Filter: @pt.r=" << new_int[cur_pt_it] << ", mean=" << mean << ", var=" << variance );
    new_int[cur_pt_it] = mean;
    return true;
  }
  return false;
}


template<class _PtTp, PclFilterChannel _data_channel>
void PCLFilter<_PtTp, _data_channel>::filterOutlierCloud( const PointCloudTp& pc_in, std::vector<float>& new_int ) const
{
  std::vector<size_t> valid_pt_its;
  valid_pt_its.reserve( pc_in.size() );
  for( size_t pt_it=0 ; pt_it < pc_in.size(); ++pt_it )
  {
    if( isValidPoint( pc_in[pt_it] ) && new_int[pt_it] > 0.0 )
    {
      valid_pt_its.push_back( pt_it );
    }
  }
  int num_valid_pts = valid_pt_its.size() > 1 ? valid_pt_its.size() -1 : 1;

  float mean = 0.0;
  for( const size_t& pt_it : valid_pt_its )
  {
    mean += new_int[pt_it];
    //std::cout << new_int[pt_it] << " => " << mean << std::endl;
  }
  mean /= num_valid_pts;

  float variance = 0.0;
  for( const size_t& pt_it : valid_pt_its )
  {
    float mean_diff = mean -new_int[pt_it];
    variance += mean_diff *mean_diff;
  }
  variance /= num_valid_pts;
  variance = std::sqrt( variance );
  float max_diff = variance *this->getParams().max_var_mult;
  //std::cout << "mean=" << mean << ", std_div=" << variance << std::endl;

  int ct_int_rep = 0;
  for( const size_t& pt_it : valid_pt_its )
  {
    if( std::abs( mean -new_int[pt_it] ) > max_diff )
    {
      new_int[pt_it] = 0.f;
      ++ct_int_rep;
    }
  }
  ROS_INFO_STREAM( "Filtered Points: " << ct_int_rep << "/" << pc_in.size() );
}


template<class _PtTp, PclFilterChannel _data_channel>
void PCLFilter<_PtTp, _data_channel>::filterOutlier( const PointCloudTp& pc_in, std::vector<float>& new_int ) const
{
  int ct_int_rep = 0;
  for( size_t h_it=0 ; h_it < this->getParams().height ; ++h_it )
  {
    for( size_t w_it=0 ; w_it < this->getParams().width ; ++w_it )
    {
      if( isValidPoint( pc_in[h_it *this->getParams().width +w_it] ) 
          && getIntensity<_PtTp, _data_channel>( pc_in, h_it *this->getParams().width +w_it ) > 0.0 )
        ct_int_rep += filterOutlierPoint( pc_in, new_int, h_it, w_it );
    }
  }
  ROS_INFO_STREAM( "Filtered Points: " << ct_int_rep << "/" << pc_in.size() );
}


template<class _PtTp, PclFilterChannel _data_channel>
void PCLFilter<_PtTp, _data_channel>::normalizeIntensity( const PointCloudTp& pc_in, std::vector<float>& new_int ) const
{
  new_int.reserve( pc_in.size() );
  static constexpr float max_val = PointIntensity<_PtTp, _data_channel>::max_val;
  for( size_t pt_it=0 ; pt_it < pc_in.size() ; ++pt_it )
  {
    float cur_int = getIntensity<_PtTp, _data_channel>( pc_in, pt_it );
    //std::cout << cur_int << "->";
    new_int.push_back( static_cast<float>(cur_int) /max_val );
    //std::cout << getIntensity<_PtTp>( pc_in, pt_it ) << ", " << std::endl;
  }
}




template class PCLFilter<ouster_ros::Point, PclFilterChannel::PCLFILTER_AMBIENCE>;
template class PCLFilter<ouster_ros::Point, PclFilterChannel::PCLFILTER_INTENSITY>;
template class PCLFilter<ouster_ros::Point, PclFilterChannel::PCLFILTER_REFLECTIVITY>;
//template class PCLFilter<velodyne_ros::Point>;
