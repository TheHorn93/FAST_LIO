#pragma once

#include "point_type.h"

namespace reflectance
{

Eigen::Vector3d projectPtToPlane( const Eigen::Vector3d& pt_pos, const Eigen::Vector3d& plane_anchor, const Eigen::Vector3d& normal )
{
  Eigen::Vector3d pt_offset = pt_pos -plane_anchor;
  double dist = pt_offset.dot( normal );
  DEBUG_OUT( "pt=" << pt_pos << std::endl << "dist=" << dist << std::endl << "pt_out=" << pt_offset -dist *normal );
  return pt_pos -dist *normal;
}

Eigen::Quaterniond getRotationToPlane( const Eigen::Vector3d& plane_normal )
{
  static Eigen::Vector3d z_axis(0,0,1);
  return Eigen::Quaterniond().setFromTwoVectors( plane_normal, z_axis );
}

using MatchVector = Eigen::Matrix<double, 1, NUM_MATCH_POINTS>;


class AttractionCenter
{
  public:
  // Project point onto plane
  // This subtracts the normal difference from direction vec
  // => Resulting intensity correction vector can be added to normal vec


  // Given five point plane: get interpolated reflectance value
  static double getReflectance( const Eigen::Vector3d& pt_pos, const PointVector& pts )
  {
    MatchVector pt_w = MatchVector::Zero();
    double w_sum = 0.0, int_out = 0.0;
    for( size_t o_it=0 ; o_it < NUM_MATCH_POINTS ; ++o_it )
    {
      double dist_to_pt = ( reinterpret_cast<const Eigen::Vector3d&>( pts[o_it].x ) -pt_pos ).norm();
      if( dist_to_pt < 1e-7 )
      {
        pt_w[o_it] = 1 /dist_to_pt;
      }
      else
      {
        pt_w[o_it] = 1e7;
      }
      int_out += pt_w[o_it] *pts[o_it].reflectance;
      w_sum += pt_w[o_it];
    }
    return int_out /w_sum;
  }

  // Given five point plane: compute reflectance gradient from plane points
  static Eigen::Vector3d getReflectanceGrad( const Eigen::Vector3d& pt_pos, const PointVector& pts )
  {
    Eigen::Vector3d ref_grad = Eigen::Vector3d::Zero();
    MatchVector pt_w = MatchVector::Zero();

    double w_sum = 0.0;
    for( size_t o_it=0 ; o_it < NUM_MATCH_POINTS ; ++o_it )
    {
      double norm = ( reinterpret_cast<const Eigen::Vector3d&>( pts[o_it].x ) -pt_pos ).squaredNorm();
      double divi = 1/(std::pow( norm, 1.5 ) +1e-45);
      std::cout << "  " << o_it << ": " << divi << std::endl;

      ref_grad[0] += divi *(pt_pos[0] -pts[o_it].x);
      ref_grad[1] += divi *(pt_pos[1] -pts[o_it].y);
      ref_grad[2] += divi *(pt_pos[2] -pts[o_it].z);
      w_sum += divi;
    }

    if( ref_grad != Eigen::Vector3d(0.0,0.0,0.0) )
      std::cout << "  ref grad=" << ref_grad/w_sum <<std::endl;
    return ref_grad /w_sum;
  }


  // Extend FastLIO assumption of point-to-plane association
  // Assume Intensity is evenly disributed from keypoints
  // Ideal point position should be guided by diff( pt, key_pt )
  // => get differences -> position weight for key_pt
  //    point pos should be weighted center of keypoints
  // New point pos is independent of input pos
  static Eigen::Vector3d getAttractionCenter( const PointType& pt, const PointVector& pts_near, const Eigen::Vector3d& normal )
  {
    assert( pts_near.size() >= NUM_MATCH_POINTS );

    // Compute point attraction as |pt.refl -pt_near.refl|
    MatchVector pt_attr = MatchVector::Zero();
    double w_sums = 0.0;
    for( size_t o_it=0 ; o_it < NUM_MATCH_POINTS ; ++o_it )
    {
      double inv_attr = std::abs(pt.reflectance -pts_near[o_it].reflectance);
      // Check for near-zero dist
      if( inv_attr > 1e-7 )
      {
        pt_attr[o_it] = 1/inv_attr;
      }
      // Max attraction
      else
      {
        static constexpr double const_attr = 1e7;
        pt_attr[o_it] = const_attr;
      }
      w_sums += pt_attr[o_it];
    }

    // Compute weighted center of nearest pts
    Eigen::Vector3d center_ref = Eigen::Vector3d::Zero();
    for( size_t o_it=0 ; o_it < 4 ; ++o_it )
    {
      const Eigen::Vector3d& pt_near = reinterpret_cast<const Eigen::Vector3d&>(pts_near[o_it].x);
      center_ref += pt_near *(pt_attr[o_it] /w_sums);
    }
    return center_ref;
  }


  // Scale diff vector to attraction by local gradient
  // Estimate gradient by local delta
  static Eigen::Vector3d call( const PointType& pt, const PointVector& pts_near, const Eigen::Vector3d& normal )
  {
    /*const PointType& anchor = pts_near[0];
    const Eigen::Vector3d pt_offset( pt.x -anchor.x, pt.y -anchor.y, pt.z -anchor.z );
    const Eigen::Vector3d pt_on_plane = projectPtToPlane( pt_offset, normal );
    const Eigen::Vector3d attr_center = getAttractionCenter( pt, pts_near, normal );
    Eigen::Vector3d diff_vec = attr_center -pt_on_plane;

    const double ref_on_plane = getIntensityFromPlane( pt_on_plane, pts_near );
    const double ref_center = getIntensityFromPlane( attr_center, pts_near );
    double grad_val = (ref_center -ref_on_plane)*(ref_center -ref_on_plane);
    return diff_vec;*/

    //std::cout << "Compute grad from " << pt.x << "," << pt.y << "," << pt.z << " | " << pts_near.size() << std::endl;
    const PointType& anchor = pts_near[0];
    const Eigen::Vector3d pt_pos( pt.x, pt.y, pt.z );
    const Eigen::Vector3d anchor_pos( anchor.x, anchor.y, anchor.z );
    const Eigen::Vector3d pt_on_plane = projectPtToPlane( pt_pos, anchor_pos, normal );

    return getReflectanceGrad( pt_on_plane, pts_near );
  }
};



class IrregularGrid
{
public:
  static constexpr int mat_p_size = 6+NUM_MATCH_POINTS;

  static Eigen::Vector3d transformTo2DPlane( const PointType& pt, const PointVector& pts_near, const Eigen::Vector3d& normal,
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


  static Eigen::Matrix<double, mat_p_size, 1> getPolynomial( const PointVector& pts_near, const Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj )
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


  static Eigen::Matrix<double, 3, 2> getGradientMat( const Eigen::Matrix<double, 6, 1>& lambda )
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

  static double getIntensityFromPosOnPlane( const Eigen::Vector3d& pt_query, const Eigen::Matrix<double, 6, 1>& lambda )
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


  static Eigen::Vector3d getIntensityGradOnPlane( const Eigen::Vector3d& pt_query, const Eigen::Matrix<double, 3, 2>& grads )
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


  static Eigen::Vector3d call( const PointType& pt, const PointVector& pts_near, const Eigen::Vector3d& normal )
  {
    Eigen::Matrix<double, 3, NUM_MATCH_POINTS> pts_near_proj;
    Eigen::Quaterniond rot_quad = getRotationToPlane( normal );
    Eigen::Vector3d pt_pos_rot = transformTo2DPlane( pt, pts_near, normal, rot_quad, pts_near_proj );
    //DEBUG_OUT( "qpt=" << pt_pos_rot << std::endl << std::endl );

    Eigen::Matrix<double, mat_p_size, 1> lambda_full = getPolynomial( pts_near, pts_near_proj );
    Eigen::Matrix<double, 6, 1> lambda = lambda_full.head<6>();

    double int_at_pos = getIntensityFromPosOnPlane( pt_pos_rot, lambda );
    Eigen::Matrix<double, 3, 2> mat_grad = getGradientMat( lambda );
    Eigen::Vector3d grad_on_plane = getIntensityGradOnPlane( pt_pos_rot, mat_grad );

    Eigen::Vector3d grad_out = rot_quad.conjugate() *(grad_on_plane *(-pt.reflectance +int_at_pos));
    DEBUG_OUT( std::endl << "Grad_in_Plane:" << grad_on_plane *(-pt.reflectance +int_at_pos) << std::endl << "norm=" << grad_on_plane.norm() );
    if( grad_out != Eigen::Vector3d(0.0,0.0,0.0) )
    {
      std::cout << std::endl;
      std::cout << "ref_delta=r_pt(" << -pt.reflectance << ") -r_int(" << int_at_pos << ") = " <<-pt.reflectance +int_at_pos << std::endl;
      std::cout << "Grad: " << rot_quad.conjugate() *grad_on_plane *(-pt.reflectance +int_at_pos) << std::endl;
    }

    return grad_out;
  }
};

}
