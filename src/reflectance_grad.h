#pragma once

#include "point_type.h"

namespace reflectance
{

using MatchVector = Eigen::Matrix<double, 1, NUM_MATCH_POINTS>;

class AttractionCenter
{
  public:
  // Project point onto plane
  // This subtracts the normal difference from direction vec
  // => Resulting intensity correction vector can be added to normal vec
  static Eigen::Vector3d projectPtToPlane( const Eigen::Vector3d& pt_pos, const Eigen::Vector3d& normal )
  {
    double dist = pt_pos.dot( normal );
    return pt_pos -dist *normal;
  }


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
      double divi = 1/std::pow( norm, 1.5 );
      //std::cout << "  " << o_it << ": " << divi << std::endl;
      if( divi < 1e-40 )
      {
        divi = 1e-40;
      }

      ref_grad[0] += divi *(pt_pos[0] -pts[o_it].x);
      ref_grad[1] += divi *(pt_pos[1] -pts[o_it].y);
      ref_grad[2] += divi *(pt_pos[2] -pts[o_it].z);
      w_sum += divi;
    }

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
    const Eigen::Vector3d pt_offset( pt.x -anchor.x, pt.y -anchor.y, pt.z -anchor.z );
    const Eigen::Vector3d pt_on_plane = projectPtToPlane( pt_offset, normal );

    return getReflectanceGrad( pt_on_plane, pts_near );
  }
};

}
