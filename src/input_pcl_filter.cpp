#define LOGURU_IMPLEMENTATION 1
#include <loguru.hpp>
#define CONFIGURU_IMPLEMENTATION 1
#include <configuru.hpp>

#include "input_pcl_filter.h"
//#include "lidar_intensity_correction/load_files.h"
#include "ConfiguruLoader.h"
#ifndef COMP_ONLY
#include "load_files.h"
#endif

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

  static constexpr float max_val = float((1<<16)-1);//std::pow(2,16)-1;
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

  static constexpr float max_val = float((1<<16)-1);//std::pow(2,16)-1;
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

  static constexpr float max_val = float((1<<16)-1); //std::pow(2,16)-1;
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
  ROS_WARN_STREAM( "Init: Type: " << type <<", Params: " << param_str );
  configuru::Config config = loadConfigFromString( param_str );
  ConfiguruLoader cfg_loader( config );
  try
  {
    compensation::CompensationModelBase* new_model = compensation::getModelFromType( type, cfg_loader );
    m_model = std::unique_ptr<compensation::CompensationModelBase>( new_model ); 
  }
  catch( const std::runtime_error& err )
  {
    ROS_ERROR_STREAM( err.what() );
    throw err;
  }
  //return nullptr; 
}



template<class _PtTp, PclFilterChannel _data_channel>
void PCLFilter<_PtTp, _data_channel>::applyFilter( const PointCloudTp& pc_in, std::vector<float>& ints_out, Eigen::Matrix<float,3,Eigen::Dynamic> * normals ) const
{
  constexpr bool print_info = false;

  //std::cout << "Start ";
  normalizeIntensity( pc_in, ints_out );
  filterOutlierCloud( pc_in, ints_out );
  PCIntensityComputation pc_int_cor;
  [[maybe_unused]] std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  pc_int_cor.addPC(pc_in, ints_out, PcInfoIntensity(), this->getParams().requires_os_shift);
  [[maybe_unused]] const double t1 = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();

  Eigen::MatrixXd pose = Eigen::MatrixXd::Zero(7,1);
  pose(3,0) = 1;
  pc_int_cor.poses() = pose;
  pc_int_cor.callOrdered( this->getParams().height, this->getParams().width, this->getParams().h_filter_size, this->getParams().w_filter_size );
  [[maybe_unused]] const double t2 = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
  //pc_int_cor.call(21, 15);
  //for(size_t it;it<ints_out.size();++it){if(it%1000==0)std::cout << it << ": " << ints_out[it] << std::endl; }
  applyModel( pc_int_cor, ints_out );
  [[maybe_unused]] const double t3 = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();

  if ( normals != nullptr )
  {
    *normals = pc_int_cor.getPCs()[0].getNormals().template cast<float>();
  }

  //for(size_t it;it<ints_out.size();++it){if(it%1000==0)std::cout << it << ": " << ints_out[it] << std::endl; }
  //std::cout << "end " << std::endl;
  if constexpr ( print_info ) ROS_INFO_STREAM("times... " << t1 << " n: " << (t2-t1) << " cor: " << (t3-t2) );
}

template<class _PtTp, PclFilterChannel _data_channel>
void PCLFilter<_PtTp, _data_channel>::applyModel( const PCIntensityComputation& pc_in, std::vector<float>& ints_out ) const
{
  //pc_out = PointCloudTp();
  const PointCloud<ScalarType>& pc = pc_in.getPCs()[0];
  this->m_model->compensateCloud( pc, pc_in.distances(), pc_in.refAngles(), pc.pointPoss().row(0), ints_out );
  /*for( size_t pt_it=0; pt_it < pc.numPoints(); ++pt_it )
  {
    double intensity = pc.intensity(pt_it);
    double dist_sqrd = pc_in.distances()[pt_it];
    double angle = pc_in.refAngles()[pt_it];
    int ring = pc.pointPoss()(0, pt_it);
    //std::cout << intensity << ", " << dist_sqrd << ", " << angle << ", " << ring <<std::endl;
    if( (std::abs(pc.data()(4, pt_it)) > 0.0 || std::abs(pc.data()(5, pt_it)) > 0.0 || std::abs(pc.data()(6, pt_it)) > 0.0) && angle < 1.5 && intensity > 0.0 )
    {
      float new_int = this->m_model->compensatePoint(intensity, dist_sqrd, angle, ring);
      ints_out[pt_it] = new_int;
      /*_PtTp new_point;
      new_point.x = pc.data()(0, pt_it);
      new_point.y = pc.data()(1, pt_it);
      new_point.z = pc.data()(2, pt_it);
      new_point.intensity = pc.data()(3, pt_it);
      new_point.ambient = pc.data()(7, pt_it);
      new_point.reflectivity = pc.data()(8, pt_it);
      new_point.ring = pc.pointPoss()(0, pt_it);
      pc_out.push_back(new_point);
    }
    else
    {
      ints_out[pt_it] = 0.f;
    }
  }*/
  //std::cout << pc_out.points.size() << "/" << pc.numPoints() << " inserted" << std::endl;
}



template<class _PtTp, PclFilterChannel _data_channel>
bool PCLFilter<_PtTp, _data_channel>::filterOutlierPoint( const PointCloudTp& pc_in, std::vector<float>& new_int, size_t h_it, size_t w_it ) const
{
  throw std::runtime_error("deprecated?");
  using Scalar = float;
  const int32_t& h_filter_size = this->getParams().h_filter_size;
  const int32_t& w_filter_size = this->getParams().w_filter_size;
  int32_t h_min = std::max( int32_t(h_it -h_filter_size), int32_t(0) );
  int32_t h_max = std::min( int32_t(h_it +h_filter_size)+1, int32_t(this->getParams().height) );
  int32_t h_range = h_max -h_min;
  int32_t w_min = std::max( int32_t(w_it -w_filter_size), int32_t(0) );
  int32_t w_max = std::min( int32_t(w_it +w_filter_size)+1, int32_t(this->getParams().width) );
  int32_t w_range = w_max -w_min;
  Scalar num_pts = static_cast<Scalar>(h_range*w_range);

  Scalar mean = Scalar(0.0);
  Scalar variance = Scalar(0.0);
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
        if( new_int[pt_it] > Scalar(0.0) )
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

  Scalar diff_cpt;
  {
    size_t pt_it = h_min *this->getParams().width +w_min;
    uint32_t num_valid_pts = 0;
    for( size_t h_it=h_min ; h_it < h_max ; ++h_it )
    {
      for( size_t w_it=w_min ; w_it < w_max ; ++w_it )
      {
        if( new_int[pt_it] > Scalar(0.0) )
        {
          ++num_valid_pts;
          Scalar mean_diff = mean - new_int[pt_it];
          variance += mean_diff * mean_diff;
          //std::cout << "var" << pt_it << " = " << variance << std::endl;
        }
        ++pt_it;
        //std::cout << pt_it << ", ";
      }
      pt_it += w_offset;
    }
    diff_cpt = mean - new_int[cur_pt_it];
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
    constexpr bool print_info = false;
    [[maybe_unused]]  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    [[maybe_unused]] int ct_int_rep = 0;
    [[maybe_unused]] size_t num_valid_pts = 0;
    constexpr bool dont_use_running = false;
    if constexpr ( dont_use_running )
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

        // TODO: replace with running variance estimator

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
            float mean_diff = mean - new_int[pt_it];
            variance += mean_diff *mean_diff;
        }
        variance /= num_valid_pts;
        variance = std::sqrt( variance );
        float max_diff = variance *this->getParams().max_var_mult;
        //std::cout << "mean=" << mean << ", std_div=" << variance << std::endl;

        for( const size_t& pt_it : valid_pt_its )
        {
            if( std::abs( mean - new_int[pt_it] ) < max_diff ) continue;
            new_int[pt_it] = 0.f;
            if constexpr ( print_info )
                ++ct_int_rep;

        }
        ROS_INFO_STREAM( "FoC: mean: " << mean << " sig " << variance << " mth: " << (max_diff + mean));
    } else {

        if constexpr ( print_info )
        {
            float new_int_min = 1<<16, new_int_max = -1;
            for ( const float & f : new_int )
            {
                if ( f < new_int_min ) new_int_min = f;
                if ( f > new_int_max ) new_int_max = f;
            }
            ROS_INFO_STREAM("ints: " << new_int_min << " " << new_int_max);
        }

        const size_t pt_size = pc_in.size();

        float sum = 0.f;
        float sum_sqrs = 0.f;
        for( size_t pt_it=0 ; pt_it < pt_size; ++pt_it )
        {
            // if ( !isValidPoint( pc_in[pt_it] ) continue; // should not be necessary anymore.
            if ( new_int[pt_it] <= 1e-6f )
                continue;

            if constexpr ( print_info )
            if ( (pt_it & 1023) == 0 )
                ROS_INFO_STREAM("pt: " << pt_it << " i: " << new_int[pt_it] << " p: " << pc_in.points[pt_it].getVector3fMap().transpose());

            const float & new_val = new_int[pt_it];
            if ( num_valid_pts == 0 )
            {
                sum = new_val;
                num_valid_pts = 1;
                continue;
            }

            const float deltaS = sum - num_valid_pts * new_val;
            const float wn = (1.f) / float(num_valid_pts);
            const float wn1 = (1.f) / float(num_valid_pts + 1);
            sum_sqrs += (deltaS * wn) * (deltaS * wn1);
            sum += new_val;
            ++num_valid_pts;
        }
        if ( num_valid_pts <= 1 ) return;
        const float mean = sum / num_valid_pts;
        const float variance = std::sqrt(sum_sqrs / (num_valid_pts-1));
        const float max_diff = variance * this->getParams().max_var_mult;
        //std::cout << "mean=" << mean << ", std_div=" << variance << std::endl;

        const float max_threshold = max_diff + mean;
        for( size_t pt_it = 0 ; pt_it < pt_size; ++pt_it )
        {
            if ( new_int[pt_it] < max_threshold ) continue; // true for invalid ones, which are skipped already
            //if( std::abs( mean - new_int[pt_it] ) < max_diff ) continue;
            new_int[pt_it] = 0.f;
            ++ct_int_rep;
        }
        if constexpr ( print_info )
        ROS_INFO_STREAM( "FoC: mean: " << mean << " sig " << variance << " mth: " << max_threshold << " for: " << num_valid_pts << " 0ofThose: " << ct_int_rep);
    }
    if constexpr ( print_info )
    ROS_INFO_STREAM( "FoC: Filtered Points: " << ct_int_rep << "/" << num_valid_pts << " of " << pc_in.size()
     << " dt: " << std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count() );
}


template<class _PtTp, PclFilterChannel _data_channel>
void PCLFilter<_PtTp, _data_channel>::filterOutlier( const PointCloudTp& pc_in, std::vector<float>& new_int ) const
{
  throw std::runtime_error("deprecated?");//static_assert (always_false<int>);
  int ct_int_rep = 0;
  for( size_t h_it=0 ; h_it < this->getParams().height ; ++h_it )
  {
    for( size_t w_it=0 ; w_it < this->getParams().width ; ++w_it )
    {
      if( isValidPoint( pc_in[h_it *this->getParams().width +w_it] ) 
          && (getIntensity<_PtTp, _data_channel>( pc_in, h_it *this->getParams().width +w_it ) > 0.0) )
        ct_int_rep += filterOutlierPoint( pc_in, new_int, h_it, w_it );
    }
  }
  ROS_INFO_STREAM( "Fo: Filtered Points: " << ct_int_rep << "/" << pc_in.size() );
}

template<class _PtTp, PclFilterChannel _data_channel>
void PCLFilter<_PtTp, _data_channel>::normalizeIntensity( const PointCloudTp& pc_in, std::vector<float>& new_int ) const
{
  constexpr bool print_info = false;
  size_t num_pts = pc_in.size();
  new_int = std::vector<float>( pc_in.size(), 0.f );
  [[maybe_unused]] float new_max_val = 0, new_min_val = 1e5;
  static constexpr float inv_max_val = 1.f/PointIntensity<_PtTp, _data_channel>::max_val;
  for( size_t pt_it=0 ; pt_it < num_pts; ++pt_it )
  {
    if ( !isValidPoint( pc_in[pt_it] ) || pc_in[pt_it].getVector3fMap().squaredNorm() < 0.1f )
        continue; // already set to 0.
    float cur_int = getIntensity<_PtTp, _data_channel>( pc_in, pt_it );
    new_int[pt_it] = cur_int * inv_max_val;
    if constexpr ( print_info ) if ( new_max_val < cur_int ) new_max_val = cur_int;
    if constexpr ( print_info ) if ( new_min_val > cur_int ) new_min_val = cur_int;
    //new_int[pt_it] = float(cur_int) / float(PointIntensity<_PtTp, _data_channel>::max_val);
  }
  if constexpr ( print_info )
  ROS_INFO_STREAM("" << (PointIntensity<_PtTp, _data_channel>::max_val)<< " " << inv_max_val  << " actual_max_val: " << new_max_val << " " << new_min_val);
}




template class PCLFilter<ouster_ros::Point, PclFilterChannel::PCLFILTER_AMBIENCE>;
template class PCLFilter<ouster_ros::Point, PclFilterChannel::PCLFILTER_INTENSITY>;
template class PCLFilter<ouster_ros::Point, PclFilterChannel::PCLFILTER_REFLECTIVITY>;
//template class PCLFilter<velodyne_ros::Point>;
