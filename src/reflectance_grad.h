#pragma once

#include "common_lib.h"
#include "point_type.h"

namespace reflectance
{

using MatchVector = Eigen::Matrix<double, 1, NUM_MATCH_POINTS>;

class IrregularGrid
{
public:
  static constexpr int mat_p_size = 6+NUM_MATCH_POINTS;

  static Eigen::Vector3d transformTo2DPlane( const PointType& pt, const PointVector& pts_near, const Eigen::Vector3d& normal, const double & dist,
                                             const Eigen::Quaterniond& rot_quad, Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj,
                                             Eigen::Vector3d & anchor_pos );

  static Eigen::Matrix<double, 6, 1> getPolynomial( const PointVector& pts_near, const Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj );

  static double getIntensityFromPosOnPlane( const Eigen::Vector2d& pt_query, const Eigen::Matrix<double, 6, 1>& lambda );

  static Eigen::Vector2d getIntensityGradOnPlane( const Eigen::Vector2d& pt_query, const Eigen::Matrix<double, 6, 1>& lambda );

  static bool computeErrorAndGradient( const PointType& pt, const PointVector& pts_near, const PointType & norm_p, double & value, Eigen::Vector3d& grad_out );
  static bool computeErrorAndGradientPlane2DTPS( const PointType& pt, const PointVector& pts_near, const PointType & norm_p, double & value, Eigen::Vector3d& grad_out );
  static bool computeErrorAndGradientPlane3DTPS( const PointType& pt, const PointVector& pts_near, const PointType & norm_p, double & value, Eigen::Vector3d& grad_out );



  static Eigen::Matrix<double, 3+NUM_MATCH_POINTS, 1> getTPS( const PointVector& pts_near, const Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj );

  static double getIntensityFromPosOnPlaneTPS( const Eigen::Vector2d& pt_query, const Eigen::Matrix<double,1,NUM_MATCH_POINTS+3> & lambda, const Eigen::Matrix<double,1,NUM_MATCH_POINTS> & dists2, Eigen::Matrix<double,1,NUM_MATCH_POINTS> & c_logr );

  static Eigen::Vector2d getIntensityGradOnPlaneTPS( const Eigen::Vector2d& pt_query, const Eigen::Matrix<double,1,NUM_MATCH_POINTS> & c, const Eigen::Vector3d & v,
                                                     const Eigen::Matrix<double,2,NUM_MATCH_POINTS> & diff, const Eigen::Matrix<double,1,NUM_MATCH_POINTS> & c_logr );


  static Eigen::Vector3d transformTo3DPlane( const PointType& pt, const PointVector& pts_near, const Eigen::Vector3d& normal, const double & dist,
                                             Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj,
                                             Eigen::Vector3d & anchor_pos );


  static Eigen::Matrix<double, 4+NUM_MATCH_POINTS, 1> getTPS3D( const PointVector& pts_near, const Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj );

  static double getIntensityFromPosTPS3D( const Eigen::Vector3d& pt_query, const Eigen::Matrix<double,1,NUM_MATCH_POINTS+4> & lambda, const Eigen::Matrix<double,1,NUM_MATCH_POINTS> & dists2, Eigen::Matrix<double,1,NUM_MATCH_POINTS> & c_logr );

  static Eigen::Vector3d getIntensityGradTPS3D( const Eigen::Vector3d& pt_query, const Eigen::Matrix<double,1,NUM_MATCH_POINTS> & c, const Eigen::Vector4d & v,
                                                       const Eigen::Matrix<double,3,NUM_MATCH_POINTS> & diff, const Eigen::Matrix<double,1,NUM_MATCH_POINTS> & c_logr );


  static bool computeErrorAndGradient3D( const PointType& pt, const PointVector& pts_near, double & value, Eigen::Vector3d& grad_out );


};

}
