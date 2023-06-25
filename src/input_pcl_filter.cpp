#include "input_pcl_filter.h"


template<class _Tp, class _ITp>
struct TypeIsEqual
  { static constexpr bool value=false; };
template<class _Tp>
struct TypeIsEqual<_Tp, _Tp>
  { static constexpr bool value=true; };
template<class _Tp, class _ITp>
static constexpr bool type_is_equal = TypeIsEqual<_Tp, _ITp>::value;


template<class _PtTp, bool _use_ambient>
struct PointIntensity
{};

template<>
struct PointIntensity<ouster_ros::Point, false>
{
  using _PtTp = ouster_ros::Point;

  static float get( const typename PCLFilter<_PtTp, false>::PointCloudTp& pc_in, size_t pt_it )
  {
    return pc_in[pt_it].reflectivity;
  }
  static void set( typename PCLFilter<_PtTp, false>::PointCloudTp& pc_in, size_t pt_it, float new_val )
  {
    pc_in[pt_it].reflectivity = new_val;
  }

  static constexpr float max_val = std::pow(2,16)-1;
};
template<>
struct PointIntensity<ouster_ros::Point, true>
{
  using _PtTp = ouster_ros::Point;

  static float get( const typename PCLFilter<_PtTp, true>::PointCloudTp& pc_in, size_t pt_it )
  {
    return pc_in[pt_it].ambient;
  }
  static void set( typename PCLFilter<_PtTp, true>::PointCloudTp& pc_in, size_t pt_it, float new_val )
  {
    pc_in[pt_it].ambient = new_val;
  }

  static constexpr float max_val = std::pow(2,16)-1;
};



template<class _PtTp, bool _use_ambient>
float getIntensity( const typename PCLFilter<_PtTp, _use_ambient>::PointCloudTp& pc_in, size_t pt_it )
{
  return PointIntensity<_PtTp, _use_ambient>::get( pc_in, pt_it );
}
template<class _PtTp, bool _use_ambient>
void setIntensity( typename PCLFilter<_PtTp, _use_ambient>::PointCloudTp& pc_in, size_t pt_it, float new_val )
{
  PointIntensity<_PtTp, _use_ambient>::set( pc_in, pt_it, new_val );
}



PCLFilterBase::Params& PCLFilterBase::getParams()
{
  return m_params;
}

const PCLFilterBase::Params& PCLFilterBase::getParams() const
{
  return m_params;
}



template<class _PtTp, bool _use_ambient>
bool PCLFilter<_PtTp, _use_ambient>::filterOutlierPoint( const PointCloudTp& pc_in, std::vector<float>& new_int, size_t h_it, size_t w_it ) const
{
  const int32_t& h_filter_size = getParams().h_filter_size;
  const int32_t& w_filter_size = getParams().w_filter_size;
  int32_t h_min = std::max( int32_t(h_it -h_filter_size), int32_t(0) );
  int32_t h_max = std::min( int32_t(h_it +h_filter_size)+1, int32_t(getParams().height) );
  int32_t h_range = h_max -h_min;
  int32_t w_min = std::max( int32_t(w_it -w_filter_size), int32_t(0) );
  int32_t w_max = std::min( int32_t(w_it +w_filter_size)+1, int32_t(getParams().width) );
  int32_t w_range = w_max -w_min;
  double num_pts = static_cast<double>(h_range*w_range);

  double mean = 0.0;
  double variance = 0.0;
  size_t cur_pt_it = h_it *getParams().width +w_it;
  uint32_t w_offset = getParams().width -w_range;

  {
    size_t pt_it = h_min *getParams().width +w_min;
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
    size_t pt_it = h_min *getParams().width +w_min;
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

  bool filter_pt = (std::abs(diff_cpt) > (variance *getParams().max_var_mult));
  if( filter_pt )
  {
    //ROS_INFO_STREAM( "Filter: @pt.r=" << new_int[cur_pt_it] << ", mean=" << mean << ", var=" << variance );
    new_int[cur_pt_it] = mean;
    return true;
  }
  return false;
}


template<class _PtTp, bool _use_ambient>
void PCLFilter<_PtTp, _use_ambient>::filterOutlierCloud( const PointCloudTp& pc_in, std::vector<float>& new_int ) const
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
  float max_diff = variance *getParams().max_var_mult;
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


template<class _PtTp, bool _use_ambient>
void PCLFilter<_PtTp, _use_ambient>::filterOutlier( const PointCloudTp& pc_in, std::vector<float>& new_int ) const
{
  int ct_int_rep = 0;
  for( size_t h_it=0 ; h_it < getParams().height ; ++h_it )
  {
    for( size_t w_it=0 ; w_it < getParams().width ; ++w_it )
    {
      if( isValidPoint( pc_in[h_it *getParams().width +w_it] ) && getIntensity<_PtTp, _use_ambient>( pc_in, h_it *getParams().width +w_it ) > 0.0 )
        ct_int_rep += filterOutlierPoint( pc_in, new_int, h_it, w_it );
    }
  }
  ROS_INFO_STREAM( "Filtered Points: " << ct_int_rep << "/" << pc_in.size() );
}


template<class _PtTp, bool _use_ambient>
void PCLFilter<_PtTp, _use_ambient>::normalizeIntensity( const PointCloudTp& pc_in, std::vector<float>& new_int ) const
{
  new_int.reserve( pc_in.size() );
  static constexpr float max_val = PointIntensity<_PtTp, _use_ambient>::max_val;
  for( size_t pt_it=0 ; pt_it < pc_in.size() ; ++pt_it )
  {
    float cur_int = getIntensity<_PtTp, _use_ambient>( pc_in, pt_it );
    //std::cout << cur_int << "->";
    new_int.push_back( static_cast<float>(cur_int) /max_val );
    //std::cout << getIntensity<_PtTp>( pc_in, pt_it ) << ", " << std::endl;
  }
}

template class PCLFilter<ouster_ros::Point, false>;
template class PCLFilter<ouster_ros::Point, true>;
//template class PCLFilter<velodyne_ros::Point>;
