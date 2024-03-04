#define LOGURU_IMPLEMENTATION 1
#include <loguru.hpp>
#define CONFIGURU_IMPLEMENTATION 1
#include <configuru.hpp>

#include "input_pcl_filter.h"
#include "ConfiguruLoader.h"

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
}


template<class _PtTp, PclFilterChannel _data_channel>
void PCLFilter<_PtTp, _data_channel>::applyFilter( const PointCloudTp& pc_in, Eigen::VectorXf & ints_out, Eigen::Matrix<float,3,Eigen::Dynamic> * normals ) const
{
  constexpr bool print_info = false;

  normalizeIntensity( pc_in, ints_out );

  // TODO: reenable the stuff afterwards...
  return;

  filterOutlierCloud( pc_in, ints_out );

  //ROS_INFO_STREAM( "pts: " << pc_in.points.size() << " " << pc_in.width << " " << pc_in.height );

  PCIntensityComputation pc_int_cor;
  [[maybe_unused]] std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  pc_int_cor.addPC(pc_in, ints_out, PcInfoIntensity(), this->getParams().requires_os_shift);
  [[maybe_unused]] const double t1 = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();

  Eigen::MatrixXd pose = Eigen::MatrixXd::Zero(7,1);
  pose(3,0) = 1;
  pc_int_cor.poses() = pose;
  pc_int_cor.precomputeUsingOrdered( this->getParams().height, this->getParams().width, this->getParams().h_filter_size, this->getParams().w_filter_size, this->getParams().num_filter_points );
  [[maybe_unused]] const double t2 = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();

  const PointCloud<ScalarType>& pc = pc_int_cor.getPCs()[0];
  this->m_model->compensateCloud( pc, pc.sqrd_dists(), pc.cos_angles(), pc.pointPoss(), ints_out );

  [[maybe_unused]] const double t3 = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();

  if ( normals != nullptr )
  {
    *normals = pc.getNormals().template cast<float>();
  }

  if constexpr ( print_info ) ROS_INFO_STREAM_THROTTLE(1,"times... " << t1 << " n: " << (t2-t1) << " cor: " << (t3-t2) << " n: " << this->getParams().num_filter_points);
}

template<class _PtTp, PclFilterChannel _data_channel>
void PCLFilter<_PtTp, _data_channel>::filterOutlierCloud( const PointCloudTp& pc_in, Eigen::VectorXf & new_int ) const
{
    constexpr bool print_info = false;
    [[maybe_unused]]  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    [[maybe_unused]] int ct_int_rep = 0;
    [[maybe_unused]] size_t num_valid_pts = 0;

    if constexpr ( print_info )
    {
        float new_int_min = 1<<16, new_int_max = -1;
        for ( int i = 0; i < new_int.rows(); ++i )
        {
            const float & f = new_int[i];
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

    if constexpr ( print_info )
        ROS_INFO_STREAM( "FoC: Filtered Points: " << ct_int_rep << "/" << num_valid_pts << " of " << pc_in.size()
         << " dt: " << std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count() );
}

template<class _PtTp, PclFilterChannel _data_channel>
void PCLFilter<_PtTp, _data_channel>::normalizeIntensity( const PointCloudTp& pc_in, Eigen::VectorXf & new_int ) const
{
  constexpr bool print_info = false;
  const size_t num_pts = pc_in.size();
  new_int = Eigen::VectorXf::Zero(num_pts,1);
  [[maybe_unused]] float new_max_val = 0, new_min_val = 1e5;
  static constexpr float inv_max_val = 1.f/PointIntensity<_PtTp, _data_channel>::max_val;
  for( size_t pt_it=0 ; pt_it < num_pts; ++pt_it )
  {
    if ( !isValidPoint( pc_in[pt_it] ) || pc_in[pt_it].getVector3fMap().squaredNorm() < 0.1f )
        continue; // already set to 0.
    const float cur_int = getIntensity<_PtTp, _data_channel>( pc_in, pt_it );
    new_int[pt_it] = cur_int;
    if constexpr ( print_info ) if ( new_max_val < cur_int ) new_max_val = cur_int;
    if constexpr ( print_info ) if ( new_min_val > cur_int ) new_min_val = cur_int;
    //new_int[pt_it] = float(cur_int) / float(PointIntensity<_PtTp, _data_channel>::max_val);
  }
  new_int *= inv_max_val;
  if constexpr ( print_info )
  ROS_INFO_STREAM("" << (PointIntensity<_PtTp, _data_channel>::max_val)<< " " << inv_max_val  << " actual_max_val: " << new_max_val << " " << new_min_val);
}


template class PCLFilter<ouster_ros::Point, PclFilterChannel::PCLFILTER_AMBIENCE>;
template class PCLFilter<ouster_ros::Point, PclFilterChannel::PCLFILTER_INTENSITY>;
template class PCLFilter<ouster_ros::Point, PclFilterChannel::PCLFILTER_REFLECTIVITY>;
//template class PCLFilter<velodyne_ros::Point>;
