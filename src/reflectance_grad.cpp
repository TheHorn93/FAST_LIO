#include "reflectance_grad.h"
#include "common_lib.h"

namespace reflectance
{

Eigen::Vector3d IrregularGrid::transformTo2DPlane( const PointType& pt, const PointVector& pts_near, const Eigen::Vector3d& normal,
                                                          const Eigen::Quaterniond& rot_quad, Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj )
{
  const PointType& anchor = pts_near[0];
  const Eigen::Vector3d pt_pos( pt.x, pt.y, pt.z );
  const Eigen::Vector3d anchor_pos( anchor.x, anchor.y, anchor.z );

  for( size_t p_it=0 ;  p_it < NUM_MATCH_POINTS ; ++p_it )
  {
    Eigen::Vector3d pt_near_pos( pts_near[p_it].x, pts_near[p_it].y, pts_near[p_it].z );
    pts_near_proj.col(p_it) = rot_quad *projectPtToPlane( pt_near_pos, anchor_pos, normal );
    //DEBUG_OUT( "point_rot=" << pts_near_proj.col(p_it) << std::endl << std::endl );
  }
  return rot_quad *projectPtToPlane( pt_pos, anchor_pos, normal );
}


Eigen::Matrix<double, IrregularGrid::mat_p_size, 1> IrregularGrid::getPolynomial( const PointVector& pts_near, const Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj )
{
  Eigen::Matrix<double,NUM_MATCH_POINTS,6> mat_a;
  mat_a.rightCols<1>().setOnes();
  for ( int p_it = 0; p_it < NUM_MATCH_POINTS; ++p_it )
  {
    mat_a.block<1,2>(p_it,3) = pts_near_proj.block<2,1>(0,p_it).transpose();
  }
  mat_a.col(0) = mat_a.col(3).array() * mat_a.col(3).array();
  mat_a.col(1) = mat_a.col(3).array() * mat_a.col(4).array();
  mat_a.col(2) = mat_a.col(4).array() * mat_a.col(4).array();
  DEBUG_OUT( pts_near_proj << std::endl );

  Eigen::Matrix<double,mat_p_size,mat_p_size> mat_p = Eigen::Matrix<double,mat_p_size,mat_p_size>::Zero();
  mat_p.topLeftCorner<3,3>().diagonal().setOnes();
  mat_p.bottomLeftCorner<NUM_MATCH_POINTS,6>() = mat_a;
  mat_p.topRightCorner<6,NUM_MATCH_POINTS>() = mat_a.transpose();

  Eigen::Matrix<double,mat_p_size,1> vec_z = Eigen::Matrix<double,mat_p_size,1>::Zero();
  for( size_t p_it=0 ; p_it < NUM_MATCH_POINTS ; ++p_it )
  {
    vec_z[3+p_it] = pts_near[p_it].reflectance;
  }

  return mat_p.colPivHouseholderQr().solve( vec_z );
}


Eigen::Matrix<double, 3, 2> IrregularGrid::getGradientMat( const Eigen::Matrix<double, 6, 1>& lambda )
{
  Eigen::Matrix<double, 3, 2> mat_out;
  mat_out(0, 0) = lambda[0] *2; // ax^2 -> 2ax
  mat_out(0, 1) = lambda[2] *2; // ay^2 -> 2ay
  mat_out(1, 0) = lambda[1]; // axy -> ay
  mat_out(1, 1) = lambda[1]; // axy -> ax
  mat_out(2, 0) = lambda[3]; // bx -> b
  mat_out(2, 1) = lambda[4]; // by -> b
  return mat_out;
}

double IrregularGrid::getIntensityFromPosOnPlane( const Eigen::Vector3d& pt_query, const Eigen::Matrix<double, 6, 1>& lambda )
{
  //DEBUG_OUT( "lambda=" << lambda << std::endl );
  Eigen::Matrix<double, 6, 1> pt_params;
  pt_params[0] = pt_query[0] *pt_query[0];
  pt_params[1] = pt_query[0] *pt_query[1];
  pt_params[2] = pt_query[1] *pt_query[1];
  pt_params[3] = pt_query[0];
  pt_params[4] = pt_query[1];
  pt_params[5] = 1;
  //DEBUG_OUT( "ref=" << lambda.dot(pt_params) << std::endl );
  return lambda.dot(pt_params);
}


Eigen::Vector3d IrregularGrid::getIntensityGradOnPlane( const Eigen::Vector3d& pt_query, const Eigen::Matrix<double, 3, 2>& grads )
{
  Eigen::Vector3d grad( 0,0,0 );

  Eigen::Vector3d pt_params_x;
  pt_params_x[0] = pt_query[0]; // ax^2 -> 2ax
  pt_params_x[1] = pt_query[1]; // axy -> ay
  pt_params_x[2] = 1; // bx -> b

  Eigen::Vector3d pt_params_y;
  pt_params_y[0] = pt_query[1]; // ay^2 -> 2ay
  pt_params_y[1] = pt_query[0]; // axy -> ax
  pt_params_y[2] = 1; // by -> b

  //DEBUG_OUT( "grads=" << grads << std::endl );
  //DEBUG_OUT( "x=" << pt_params_x << ", y=" << pt_params_y << std::endl );
  grad[0] = grads.col(0).dot( pt_params_x );
  grad[1] = grads.col(1).dot( pt_params_y );
  //DEBUG_OUT( "grad=" << grad << std::endl );

  return grad;
}


double IrregularGrid::call( const PointType& pt, const PointVector& pts_near, const Eigen::Vector3d& normal, Eigen::Vector3d& grad_out )
{
  //std::cout << pt.reflectance << ": " << pt.x << ", " << pt.y << ", " << pt.z << std::endl; 
  Eigen::Matrix<double, 3, NUM_MATCH_POINTS> pts_near_proj;
  Eigen::Quaterniond rot_quad = getRotationToPlane( normal );
  Eigen::Vector3d pt_pos_rot = transformTo2DPlane( pt, pts_near, normal, rot_quad, pts_near_proj );
  //DEBUG_OUT( "qpt=" << pt_pos_rot << std::endl << std::endl );

  Eigen::Matrix<double, mat_p_size, 1> lambda_full = getPolynomial( pts_near, pts_near_proj );
  Eigen::Matrix<double, 6, 1> lambda = lambda_full.head<6>();

  double int_at_pos = getIntensityFromPosOnPlane( pt_pos_rot, lambda );
  Eigen::Matrix<double, 3, 2> mat_grad = getGradientMat( lambda );
  Eigen::Vector3d grad_on_plane = getIntensityGradOnPlane( pt_pos_rot, mat_grad );

  //double grad_norm = grad_on_plane.norm();
  //std::cout << "grad_n" << grad_norm << " <- " << grad_on_plane.transpose()<< std::endl;
  grad_out = (rot_quad.conjugate() *grad_on_plane).transpose(); //* (Eigen::Matrix<double, 3,3>::Identity() - normal * normal.transpose());
  double grad_norm = grad_out.squaredNorm();
  if( grad_norm > 0.0 )
    grad_out /= grad_norm;

  DEBUG_OUT( std::endl << "Grad_in_Plane:" << grad_on_plane *(-pt.reflectance +int_at_pos) << std::endl << "norm=" << grad_on_plane.norm() );
  //std::cout << "    " << int_at_pos << " - " << pt.reflectance << std::endl;
  /*if( grad_out != Eigen::Vector3d(0.0,0.0,0.0) )
  {
    std::cout << std::endl;
    std::cout << "ref_delta=r_pt(" << -pt.reflectance << ") -r_int(" << int_at_pos << ") = " <<-pt.reflectance +int_at_pos << std::endl;
    std::cout << "Grad: " << rot_quad.conjugate() *grad_on_plane *(-pt.reflectance +int_at_pos) << std::endl;
  }*/

  return (-pt.reflectance +int_at_pos);
}

}
