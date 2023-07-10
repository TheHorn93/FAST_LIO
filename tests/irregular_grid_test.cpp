#define DEBUG

#include "common_lib.h"
#include "../src/reflectance_grad.h"
#include <cstdlib>
#include <random>


class NormalDistribution
{
  NormalDistribution( std::mt19937& gen, double mean, double std_div )
    : dist( mean, std_div )
    , generator( gen )
    {}

  double sample()
  {
    return dist( generator );
  }

  std::normal_distribution<> dist;
  std::mt19937& generator;
};


Eigen::Vector3d getNormal( const PointVector& points_near )
{
  VF(4) pabcd;
  esti_plane(pabcd, points_near, 0.1f);
  return Eigen::Vector3d( pabcd(0), pabcd(1), pabcd(2) );
}


int main( int argc, char** argv )
{
  std::random_device rd{};
  std::mt19937 generator{rd()};

  Eigen::Matrix<double,4,5> points;
    points << 1, 0.5, 0.5, -0.5, -0.5,
              0, 0.5, -0.5, 0.5, -0.5,
              0, 0.1, 0, 0, 0,
              1, 0.3, 0.3, 0.5, 0.5;

  Eigen::Vector3d plane_normal_base( 1.0,0.0,1.0 );
  Eigen::Vector3d plane_normal = Eigen::Vector3d( 0.0,1.0,0.0 ).normalized();
  Eigen::Quaterniond quad = Eigen::Quaterniond().setFromTwoVectors( plane_normal_base, plane_normal );

  PointVector vec_pts;
  for( size_t p_it=0 ; p_it < points.cols() ; ++p_it )
  {
    Eigen::Vector3d point = quad*points.block<3,1>( 0, p_it );

    PointType point_new;
    point_new.x = point[0];
    point_new.y = point[1];
    point_new.z = point[2];
    point_new.reflectance = points( 3, p_it );
    std::cout << point_new.x << ", " << point_new.y << ", " << point_new.z << std::endl;
    vec_pts.push_back( point_new );
  }

  Eigen::Vector3d normal = getNormal( vec_pts );

  std::cout << points << std::endl;
  std::cout << "Normal " << normal << std::endl;

  PointVector vec_querys;

  {
    PointType point_query;
    point_query.x = 0.7;
    point_query.y = 0.0;
    point_query.z = 0.4;
    point_query.reflectance = 1.0;
    vec_querys.push_back( point_query );
  }
  {
    PointType point_query;
    point_query.x = 0.4;
    point_query.y = 0.0;
    point_query.z = 0.4;
    point_query.reflectance = 1.0;
    vec_querys.push_back( point_query );
  }
  {
    PointType point_query;
    point_query.x = 0.1;
    point_query.y = 0.0;
    point_query.z = 0.4;
    point_query.reflectance = 1.0;
    vec_querys.push_back( point_query );
  }
  for( const PointType& pt_query : vec_querys )
  {
    Eigen::Vector3d ref_grad = reflectance::IrregularGrid::call( pt_query, vec_pts, normal );
    std::cout << std::endl;
    std::cout << ref_grad << std::endl << " ========" << std::endl;
  }

  return 0;
}
