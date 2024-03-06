#include "reflectance_grad.h"
#include "common_lib.h"

namespace reflectance
{

Eigen::Vector3d projectPtToPlane( const Eigen::Vector3d& pt_pos, const Eigen::Vector3d& plane_anchor, const Eigen::Vector3d& normal )
{
    const Eigen::Vector3d pt_offset = pt_pos - plane_anchor;
    const double dist = pt_offset.dot( normal );
    //DEBUG_OUT( "pt=" << pt_pos << std::endl << "dist=" << dist << std::endl << "pt_out=" << (pt_offset - dist * normal ));
    return pt_pos - dist * normal;
}

inline
Eigen::Quaterniond getRotationToPlane( const Eigen::Vector3d& plane_normal )
{
    return Eigen::Quaterniond().setFromTwoVectors( plane_normal, Eigen::Vector3d::UnitZ() ); // rotates plane_normal to UnitZ
}

Eigen::Vector3d IrregularGrid::transformTo2DPlane( const PointType& pt, const PointVector& pts_near, const Eigen::Vector3d& normal, const double & dist,
                                                   const Eigen::Quaterniond& rot_quad, Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj, Eigen::Vector3d & anchor_pos )
{
    Eigen::Vector3d pt_sum = Eigen::Vector3d::Zero();
    for( size_t p_it=0 ;  p_it < NUM_MATCH_POINTS ; ++p_it )
        pt_sum += Eigen::Vector3d( pts_near[p_it].x, pts_near[p_it].y, pts_near[p_it].z );
    anchor_pos.noalias() = pt_sum / NUM_MATCH_POINTS; // mean value is on the plane

    // project points
    for( size_t p_it=0 ;  p_it < NUM_MATCH_POINTS ; ++p_it )
        pts_near_proj.col(p_it).noalias() = rot_quad * projectPtToPlane( Eigen::Vector3d ( pts_near[p_it].x, pts_near[p_it].y, pts_near[p_it].z ), anchor_pos, normal ); // get point on plane, rotate to UnitZ

    const Eigen::Vector3d pt_pos( pt.x, pt.y, pt.z );
    return rot_quad * projectPtToPlane( pt_pos, anchor_pos, normal ); // get point on plane, rotate to UnitZ ( Eq. \pi(p) = R_n (p-d_o(p)n_o) )
}

Eigen::Matrix<double, 6, 1> IrregularGrid::getPolynomial( const PointVector& pts_near, const Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj )
{
    Eigen::Matrix<double,NUM_MATCH_POINTS,6> mat_a; // x^2, y^2, x*y, x, y, 1
    mat_a.template rightCols<1>().setOnes();
    for ( int p_it = 0; p_it < NUM_MATCH_POINTS; ++p_it )
        mat_a.template block<1,2>(p_it,3) = pts_near_proj.template block<2,1>(0,p_it).transpose(); // x, y

    mat_a.col(0) = mat_a.col(3).array() * mat_a.col(3).array(); // x*x
    mat_a.col(1) = mat_a.col(4).array() * mat_a.col(4).array(); // y*y
    mat_a.col(2) = mat_a.col(3).array() * mat_a.col(4).array(); // x*y

    // Augmented System of Equations
    Eigen::Matrix<double,mat_p_size,mat_p_size> mat_p = Eigen::Matrix<double,mat_p_size,mat_p_size>::Zero();
    mat_p.template topLeftCorner<3,3>().diagonal().setOnes();
    mat_p.template bottomLeftCorner<NUM_MATCH_POINTS,6>() = mat_a;
    mat_p.template topRightCorner<6,NUM_MATCH_POINTS>() = mat_a.transpose();

    Eigen::Matrix<double,mat_p_size,1> vec_z = Eigen::Matrix<double,mat_p_size,1>::Zero();
    for( size_t p_it=0 ; p_it < NUM_MATCH_POINTS ; ++p_it )
    {
        vec_z[6+p_it] = pts_near[p_it].intensity; // reflectance channel is dropped by the voxelgrid filter..., but we write it into intensity either way...
    }
    return mat_p.colPivHouseholderQr().solve( vec_z ).template head<6>();
}

double IrregularGrid::getIntensityFromPosOnPlane( const Eigen::Vector2d& pt_query, const Eigen::Matrix<double, 6, 1>& lambda )
{
    Eigen::Matrix<double, 6, 1> pt_params;
    pt_params[0] = pt_query[0] *pt_query[0]; // x^2
    pt_params[1] = pt_query[1] *pt_query[1]; // y^2
    pt_params[2] = pt_query[0] *pt_query[1]; // x*y
    pt_params[3] = pt_query[0]; // x
    pt_params[4] = pt_query[1]; // y
    pt_params[5] = 1;
    return lambda.dot(pt_params);
}

// f = l0 * x^2 + l1 * y^2 + l2 * x*y + l3 * x + l4*y + l5*1
// df = [dx, dy] in R^1x2

Eigen::Vector2d IrregularGrid::getIntensityGradOnPlane( const Eigen::Vector2d& pt_query, const Eigen::Matrix<double, 6, 1>& lambda )
{
    // dx: 2 * l0 * x + l2 * y + l3
    // dy: 2 * l1 * y + l2 * x + l4
    const double dx = 2 * lambda[0] * pt_query[0] +     lambda[2] * pt_query[1] + lambda[3];
    const double dy =     lambda[2] * pt_query[0] + 2 * lambda[1] * pt_query[1] + lambda[4];
    return Eigen::Vector2d(dx,dy);
}

Eigen::Matrix<double, 3+NUM_MATCH_POINTS, 1> IrregularGrid::getTPS( const PointVector& pts_near, const Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj )
{
    constexpr bool print_info = false; //true;
    constexpr int num_params = 3+NUM_MATCH_POINTS;

    // Augmented System of Equations
    Eigen::Matrix<double,num_params,num_params> mat_p = Eigen::Matrix<double,num_params,num_params>::Zero();
    //mat_p.template topLeftCorner<3,3>().diagonal().setOnes();
    for ( int i = 0; i < NUM_MATCH_POINTS; ++i)
    {
        const Eigen::Vector2d p = pts_near_proj.template block<2,1>(0,i);
        for ( int j = i+1; j < NUM_MATCH_POINTS; ++j )
        {
            const double r2 = (p-pts_near_proj.template block<2,1>(0,j)).squaredNorm();
            mat_p(i,j) = r2 * 0.5 * log(r2);
            if ( !std::isfinite(mat_p(i,j)) ) mat_p(i,j) = 0; // case r2=0
            mat_p(j,i) = mat_p(i,j);
        }
    }
    Eigen::Matrix<double,3,NUM_MATCH_POINTS> pts_a = pts_near_proj;
    pts_a.template bottomRows<1>().setOnes();
    mat_p.template bottomLeftCorner<3,NUM_MATCH_POINTS>() = pts_a;
    mat_p.template topRightCorner<NUM_MATCH_POINTS,3>() = pts_a.transpose();

    Eigen::Matrix<double,num_params,1> vec_z = Eigen::Matrix<double,num_params,1>::Zero();
    for( size_t p_it=0 ; p_it < NUM_MATCH_POINTS ; ++p_it )
    {
        vec_z[p_it] = pts_near[p_it].intensity; // reflectance channel is dropped by the voxelgrid filter..., but we write it into intensity either way...
    }

    if constexpr ( print_info )
    {
        static int cnt = 0;
        ++cnt;
        if ( (cnt & 1023) == 0 )
        {
            cnt = 0;
            ROS_INFO_STREAM("p:\n" << pts_near_proj.template topRows<2>());
            ROS_INFO_STREAM("A:\n" << mat_p << "\nv:" << vec_z.transpose());
            ROS_INFO_STREAM("cv:" << mat_p.colPivHouseholderQr().solve( vec_z ).transpose());
        }
    }
    return mat_p.colPivHouseholderQr().solve( vec_z );
}

double IrregularGrid::getIntensityFromPosOnPlaneTPS( const Eigen::Vector2d& pt_query, const Eigen::Matrix<double,1,NUM_MATCH_POINTS+3> & lambda, const Eigen::Matrix<double,1,NUM_MATCH_POINTS> & dists2, Eigen::Matrix<double,1,NUM_MATCH_POINTS> & c_logr )
{
    //c'*(r.^2.*log(r)) + v'*[1;u]
    //const Eigen::Vector3d u (1, pt_query(0), pt_query(1));
    c_logr = lambda.template head<NUM_MATCH_POINTS>().array() * (0.5*dists2.array().log()); // log(sqrt(dist2)) == 0.5 * log(dist2)
    return c_logr.dot(dists2) + lambda.template tail<3>().dot(pt_query.homogeneous());
}

Eigen::Vector2d IrregularGrid::getIntensityGradOnPlaneTPS( const Eigen::Vector2d& pt_query, const Eigen::Matrix<double,1,NUM_MATCH_POINTS> & c, const Eigen::Vector3d & v,
                                                           const Eigen::Matrix<double,2,NUM_MATCH_POINTS> & diff, const Eigen::Matrix<double,1,NUM_MATCH_POINTS> & c_logr )
{

    //r0 = ((d0x - ux)^2 + (d0y - uy)^2)^(1/2)
    //r1 = ((d1x - ux)^2 + (d1y - uy)^2)^(1/2)
    //r2 = ((d2x - ux)^2 + (d2y - uy)^2)^(1/2)
    //r3 = ((d3x - ux)^2 + (d3y - uy)^2)^(1/2)
    //r4 = ((d4x - ux)^2 + (d4y - uy)^2)^(1/2)

//    const double dx = v1 - (c0*(2*d0x - 2*ux))/2 - (c1*(2*d1x - 2*ux))/2 - (c2*(2*d2x - 2*ux))/2 - (c3*(2*d3x - 2*ux))/2 - (c4*(2*d4x - 2*ux))/2
//            - c0*log(r0)*(2*d0x - 2*ux) - c1*log(r1)*(2*d1x - 2*ux) - c2*log(r2)*(2*d2x - 2*ux) - c3*log(r3)*(2*d3x - 2*ux) - c4*log(r4)*(2*d4x - 2*ux);
//    const double dy = v2 - (c0*(2*d0y - 2*uy))/2 - (c1*(2*d1y - 2*uy))/2 - (c2*(2*d2y - 2*uy))/2 - (c3*(2*d3y - 2*uy))/2 - (c4*(2*d4y - 2*uy))/2
//            - c0*log(r0)*(2*d0y - 2*uy) - c1*log(r1)*(2*d1y - 2*uy) - c2*log(r2)*(2*d2y - 2*uy) - c3*log(r3)*(2*d3y - 2*uy) - c4*log(r4)*(2*d4y - 2*uy);

    const double dx = v(0) - c.dot(diff.row(0)) - 2 * c_logr.dot(diff.row(0));
    const double dy = v(1) - c.dot(diff.row(1)) - 2 * c_logr.dot(diff.row(1));
    return Eigen::Vector2d(dx,dy);
}




Eigen::Matrix<double, 4+NUM_MATCH_POINTS, 1> IrregularGrid::getTPS3D( const PointVector& pts_near, const Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj )
{
    constexpr bool print_info = false; //true;
    constexpr int num_params = 4+NUM_MATCH_POINTS;

    // Augmented System of Equations
    Eigen::Matrix<double,num_params,num_params> mat_p = Eigen::Matrix<double,num_params,num_params>::Zero();
    //mat_p.template topLeftCorner<3,3>().diagonal().setOnes();
    for ( int i = 0; i < NUM_MATCH_POINTS; ++i)
    {
        const Eigen::Vector3d p = pts_near_proj.template block<3,1>(0,i);
        for ( int j = i+1; j < NUM_MATCH_POINTS; ++j )
        {
            const double r2 = (p-pts_near_proj.template block<3,1>(0,j)).squaredNorm();
            mat_p(i,j) = r2 * 0.5 * log(r2);
            if ( !std::isfinite(mat_p(i,j)) ) mat_p(i,j) = 0; // case r2=0
            mat_p(j,i) = mat_p(i,j);
        }
    }
    Eigen::Matrix<double,4,NUM_MATCH_POINTS> pts_a;
    pts_a.template topRows<3>() = pts_near_proj;
    pts_a.template bottomRows<1>().setOnes();
    mat_p.template bottomLeftCorner<4,NUM_MATCH_POINTS>() = pts_a;
    mat_p.template topRightCorner<NUM_MATCH_POINTS,4>() = pts_a.transpose();

    Eigen::Matrix<double,num_params,1> vec_z = Eigen::Matrix<double,num_params,1>::Zero();
    for( size_t p_it=0 ; p_it < NUM_MATCH_POINTS ; ++p_it )
    {
        vec_z[p_it] = pts_near[p_it].intensity; // reflectance channel is dropped by the voxelgrid filter..., but we write it into intensity either way...
    }

    if constexpr ( print_info )
    {
        static int cnt = 0;
        ++cnt;
        if ( (cnt & 1023) == 0 )
        {
            cnt = 0;
            ROS_INFO_STREAM("p:\n" << pts_near_proj.template topRows<2>());
            ROS_INFO_STREAM("A:\n" << mat_p << "\nv:" << vec_z.transpose());
            ROS_INFO_STREAM("cv:" << mat_p.colPivHouseholderQr().solve( vec_z ).transpose());
        }
    }
    return mat_p.colPivHouseholderQr().solve( vec_z );
}

double IrregularGrid::getIntensityFromPosOnPlaneTPS3D( const Eigen::Vector3d& pt_query, const Eigen::Matrix<double,1,NUM_MATCH_POINTS+4> & lambda, const Eigen::Matrix<double,1,NUM_MATCH_POINTS> & dists2, Eigen::Matrix<double,1,NUM_MATCH_POINTS> & c_logr )
{
    //c'*(r.^2.*log(r)) + v'*[1;u]
    //const Eigen::Vector3d u (1, pt_query(0), pt_query(1));
    c_logr = lambda.template head<NUM_MATCH_POINTS>().array() * (0.5*dists2.array().log()); // log(sqrt(dist2)) == 0.5 * log(dist2)
    return c_logr.dot(dists2) + lambda.template tail<4>().dot(pt_query.homogeneous());
}

Eigen::Vector3d IrregularGrid::getIntensityGradOnPlaneTPS3D( const Eigen::Vector3d& pt_query, const Eigen::Matrix<double,1,NUM_MATCH_POINTS> & c, const Eigen::Vector4d & v,
                                                             const Eigen::Matrix<double,3,NUM_MATCH_POINTS> & diff, const Eigen::Matrix<double,1,NUM_MATCH_POINTS> & c_logr )
{

    //r0 = ((d0x - ux)^2 + (d0y - uy)^2)^(1/2)
    //r1 = ((d1x - ux)^2 + (d1y - uy)^2)^(1/2)
    //r2 = ((d2x - ux)^2 + (d2y - uy)^2)^(1/2)
    //r3 = ((d3x - ux)^2 + (d3y - uy)^2)^(1/2)
    //r4 = ((d4x - ux)^2 + (d4y - uy)^2)^(1/2)

//    const double dx = v1 - (c0*(2*d0x - 2*ux))/2 - (c1*(2*d1x - 2*ux))/2 - (c2*(2*d2x - 2*ux))/2 - (c3*(2*d3x - 2*ux))/2 - (c4*(2*d4x - 2*ux))/2
//            - c0*log(r0)*(2*d0x - 2*ux) - c1*log(r1)*(2*d1x - 2*ux) - c2*log(r2)*(2*d2x - 2*ux) - c3*log(r3)*(2*d3x - 2*ux) - c4*log(r4)*(2*d4x - 2*ux);
//    const double dy = v2 - (c0*(2*d0y - 2*uy))/2 - (c1*(2*d1y - 2*uy))/2 - (c2*(2*d2y - 2*uy))/2 - (c3*(2*d3y - 2*uy))/2 - (c4*(2*d4y - 2*uy))/2
//            - c0*log(r0)*(2*d0y - 2*uy) - c1*log(r1)*(2*d1y - 2*uy) - c2*log(r2)*(2*d2y - 2*uy) - c3*log(r3)*(2*d3y - 2*uy) - c4*log(r4)*(2*d4y - 2*uy);

    const double dx = v(0) - c.dot(diff.row(0)) - 2 * c_logr.dot(diff.row(0));
    const double dy = v(1) - c.dot(diff.row(1)) - 2 * c_logr.dot(diff.row(1));
    const double dz = v(2) - c.dot(diff.row(2)) - 2 * c_logr.dot(diff.row(2));
    return Eigen::Vector3d(dx,dy,dz);
}

Eigen::Vector3d IrregularGrid::transformTo3DPlane( const PointType& pt, const PointVector& pts_near, const Eigen::Vector3d& normal, const double & dist,
                                                   Eigen::Matrix<double, 3, NUM_MATCH_POINTS>& pts_near_proj, Eigen::Vector3d & anchor_pos )
{
    Eigen::Vector3d pt_sum = Eigen::Vector3d::Zero();
    for( size_t p_it=0 ;  p_it < NUM_MATCH_POINTS ; ++p_it )
        pt_sum += Eigen::Vector3d( pts_near[p_it].x, pts_near[p_it].y, pts_near[p_it].z );
    anchor_pos.noalias() = pt_sum / NUM_MATCH_POINTS; // mean value is on the plane

    // project points
    for( size_t p_it=0 ;  p_it < NUM_MATCH_POINTS ; ++p_it )
        pts_near_proj.col(p_it).noalias() = projectPtToPlane( Eigen::Vector3d ( pts_near[p_it].x, pts_near[p_it].y, pts_near[p_it].z ), anchor_pos, normal ); // get point on plane, rotate to UnitZ

    const Eigen::Vector3d pt_pos( pt.x, pt.y, pt.z );
    return projectPtToPlane( pt_pos, anchor_pos, normal ); // get point on plane, rotate to UnitZ ( Eq. \pi(p) = R_n (p-d_o(p)n_o) )
}

bool IrregularGrid::computeErrorAndGradient( const PointType& pt, const PointVector& pts_near, const PointType & norm_p, double & value, Eigen::Vector3d& grad_out )
{
#define USE_3D_TPS
#define USE_TPS

#ifdef USE_3D_TPS
    constexpr bool print_info = false;
    const Eigen::Vector3d normal ( norm_p.x, norm_p.y, norm_p.z );
    //std::cout << pt.reflectance << ": " << pt.x << ", " << pt.y << ", " << pt.z << std::endl;
    Eigen::Matrix<double, 3, NUM_MATCH_POINTS> pts_near_proj;
    Eigen::Vector3d pt_mean;
    const Eigen::Vector3d pt_pos_rot = transformTo3DPlane( pt, pts_near, normal, norm_p.intensity, pts_near_proj, pt_mean ); // should be on plane ( of UnitZ ), norm_p.intensity is the distance of plane equation
    //DEBUG_OUT( "qpt=" << pt_pos_rot << std::endl << std::endl );

    double min_intensity = pts_near[0].intensity;
    double max_intensity = pts_near[0].intensity;
    bool further_away = true;
    const double dist = (pt_pos_rot-pt_mean).squaredNorm();
    for ( int i = 0; i < pts_near.size(); ++i)
    {
        further_away &= (pts_near_proj.col(i) - pt_mean).squaredNorm() > dist;
        if ( pts_near[i].intensity < min_intensity ) min_intensity = pts_near[i].intensity;
        if ( pts_near[i].intensity > max_intensity ) max_intensity = pts_near[i].intensity;
    }

    if ( further_away ) return false;

    const Eigen::Matrix<double,4+NUM_MATCH_POINTS, 1> lambda = getTPS3D( pts_near, pts_near_proj );

    const Eigen::Matrix<double,3,NUM_MATCH_POINTS> diffs = pts_near_proj.colwise() - pt_pos_rot;
    const Eigen::Matrix<double,1,NUM_MATCH_POINTS> dists2 = diffs.colwise().squaredNorm();

    Eigen::Matrix<double,1,NUM_MATCH_POINTS> c_logr;
    const double int_at_pos = getIntensityFromPosOnPlaneTPS3D( pt_pos_rot, lambda, dists2, c_logr );

    if ( min_intensity > int_at_pos || int_at_pos > max_intensity )
        return false;

    const Eigen::Vector3d grad_on_plane = getIntensityGradOnPlaneTPS3D( pt_pos_rot, lambda.template head<NUM_MATCH_POINTS>(), lambda.template tail<4>(), diffs, c_logr ); // [dx,dy] of plane...

    if constexpr ( print_info )
    {
        static int cnt = 0;
        ++cnt;
        if ( (cnt & 1023) == 0 )
        {
            const Eigen::Vector3d grad_on_plane3D = grad_on_plane;
            Eigen::Matrix<double, 3, NUM_MATCH_POINTS> pts_near_proj;
            const Eigen::Quaterniond rot_quad = getRotationToPlane( normal ); // normal in world coords, rotate normal to UnitZ
            Eigen::Vector3d pt_mean;
            const Eigen::Vector2d pt_pos_rot = transformTo2DPlane( pt, pts_near, normal, norm_p.intensity, rot_quad, pts_near_proj, pt_mean ).template head<2>(); // should be on plane ( of UnitZ ), norm_p.intensity is the distance of plane equation

            const Eigen::Matrix<double,3+NUM_MATCH_POINTS, 1> lambda = getTPS( pts_near, pts_near_proj );

            const Eigen::Matrix<double,2,NUM_MATCH_POINTS> diffs = pts_near_proj.template topRows<2>().colwise() - pt_pos_rot;
            const Eigen::Matrix<double,1,NUM_MATCH_POINTS> dists2 = diffs.colwise().squaredNorm();

            Eigen::Matrix<double,1,NUM_MATCH_POINTS> c_logr;
            const double int_at_pos = getIntensityFromPosOnPlaneTPS( pt_pos_rot, lambda, dists2, c_logr );
            const Eigen::Vector2d grad_on_plane = getIntensityGradOnPlaneTPS( pt_pos_rot, lambda.template head<NUM_MATCH_POINTS>(), lambda.template tail<3>(), diffs, c_logr );

            const Eigen::Matrix3d rot_to_plane = rot_quad.matrix();
            Eigen::Vector3d grad_on_plane_2D = (grad_on_plane.transpose() * rot_to_plane.template topRows<2>()).transpose();


            ROS_INFO_STREAM("2d: " << grad_on_plane_2D.transpose() << " 3D: " << grad_on_plane3D.transpose() << " d: " << (grad_on_plane_2D - grad_on_plane3D).transpose() );
        }
    }


    if ( !grad_on_plane.allFinite() )
    {
        ROS_ERROR_STREAM("a: " << grad_on_plane.transpose() << " l: " << lambda.transpose());
    }

    grad_out = (grad_on_plane.transpose() * (Eigen::Matrix3d::Identity() - normal * normal.transpose())).transpose();

    if constexpr ( print_info )
    {
        static int cnt = 0;
        ++cnt;
        if ( (cnt & 1023) == 0 )
        {
            cnt = 0;
            Eigen::Matrix<double, NUM_MATCH_POINTS, 1> intensities;
            for ( int i = 0; i < pts_near.size(); ++i )
                intensities(i) = pts_near[i].intensity;
            ROS_INFO_STREAM("A: " << int_at_pos << " int: " << pt.reflectance << " i: " << intensities.transpose() << " l: " << lambda.transpose() << " p: " << pt_pos_rot.transpose() << " g: " << grad_out.transpose());
            //          const Eigen::Vector3d closer_pt = pt_pos_rot - pts_near_proj.col(0);
            //          const Eigen::Matrix<double, 3, NUM_MATCH_POINTS> closer_pts = (pts_near_proj.colwise() - pts_near_proj.col(0));
            //          const Eigen::Matrix<double, mat_p_size, 1> other_lambda = getPolynomial( pts_near, closer_pts );
            //          Eigen::Matrix<double, 3, NUM_MATCH_POINTS> pts_near_xyz, pts_near_xyz_rot;
            //          Eigen::Matrix<double, NUM_MATCH_POINTS, 1> intensities;
            //          Eigen::Matrix<double, NUM_MATCH_POINTS, 1> dists;//reflectances,
            //          for ( int i = 0; i < pts_near.size(); ++i )
            //          {
            //              pts_near_xyz.col(i) << pts_near[i].x, pts_near[i].y, pts_near[i].z;
            //              pts_near_xyz_rot.col(i) = rot_quad * pts_near_xyz.col(i);
            //reflectances(i) = pts_near[i].reflectance;
            //              intensities(i) = pts_near[i].intensity;
            //              dists(i) = pts_near_xyz.col(i).dot(normal) + norm_p.gloss; // gloss now has D of plane inside, while intensity contains the pt's distance to plane...
            //          }
            //          const Eigen::Vector3d pts_near_mean = pts_near_xyz.rowwise().mean();
            //          const double d = pts_near_mean.dot(normal);
            //          ROS_INFO_STREAM("pts:\n" << pts_near_xyz << "\nrotated:\n" << pts_near_xyz_rot << "\n pm: " << pts_near_mean.transpose() << " d: " << d << " diff: " << (d+norm_p.gloss) << " D: " << norm_p.intensity << "\ndn:" << dists.transpose() );
            //          ROS_INFO_STREAM("pts_proj: " << pt_pos_rot.transpose() << " near: \n" << pts_near_proj << " \n " << closer_pts);
            //          ROS_INFO_STREAM("A: " << int_at_pos << " ( "  << getIntensityFromPosOnPlane( closer_pt, other_lambda.head<6>() ) << " ) int: " << pt.reflectance //<< " m: " << reflectances.transpose()
            //                          << " i: " << intensities.transpose() << " g: " << grad_out.transpose() << " gn: "<< grad_norm
            //                          << "\nL: " << lambda.transpose() << " o: " << (other_lambda.template head<6>()).transpose()  );
        }
    }

    // why is it with another minus in front on the pc?
    value = (int_at_pos - pt.reflectance);

    return true;



#else
#ifdef USE_TPS
    constexpr bool print_info = false;
#else
    constexpr bool print_info = false;
#endif
    const Eigen::Vector3d normal ( norm_p.x, norm_p.y, norm_p.z );
    //std::cout << pt.reflectance << ": " << pt.x << ", " << pt.y << ", " << pt.z << std::endl;
    Eigen::Matrix<double, 3, NUM_MATCH_POINTS> pts_near_proj;
    const Eigen::Quaterniond rot_quad = getRotationToPlane( normal ); // normal in world coords, rotate normal to UnitZ
    Eigen::Vector3d pt_mean;
    const Eigen::Vector2d pt_pos_rot = transformTo2DPlane( pt, pts_near, normal, norm_p.intensity, rot_quad, pts_near_proj, pt_mean ).template head<2>(); // should be on plane ( of UnitZ ), norm_p.intensity is the distance of plane equation
    //DEBUG_OUT( "qpt=" << pt_pos_rot << std::endl << std::endl );

    double min_intensity = pts_near[0].intensity;
    double max_intensity = pts_near[0].intensity;
    bool further_away = true;
    const double dist = (pt_pos_rot-pt_mean.template head<2>()).squaredNorm();
    for ( int i = 0; i < pts_near.size(); ++i)
    {
        further_away &= (pts_near_proj.col(i).template head<2>() - pt_mean.template head<2>()).squaredNorm() > dist;
        if ( pts_near[i].intensity < min_intensity ) min_intensity = pts_near[i].intensity;
        if ( pts_near[i].intensity > max_intensity ) max_intensity = pts_near[i].intensity;
    }

#ifdef USE_TPS

    const Eigen::Matrix<double,3+NUM_MATCH_POINTS, 1> lambda = getTPS( pts_near, pts_near_proj );

    const Eigen::Matrix<double,2,NUM_MATCH_POINTS> diffs = pts_near_proj.template topRows<2>().colwise() - pt_pos_rot;
    const Eigen::Matrix<double,1,NUM_MATCH_POINTS> dists2 = diffs.colwise().squaredNorm();

    Eigen::Matrix<double,1,NUM_MATCH_POINTS> c_logr;
    const double int_at_pos = getIntensityFromPosOnPlaneTPS( pt_pos_rot, lambda, dists2, c_logr );



#else
    const Eigen::Matrix<double, 6, 1> lambda = getPolynomial( pts_near, pts_near_proj );
    const double int_at_pos = getIntensityFromPosOnPlane( pt_pos_rot, lambda );
#endif

    if ( further_away || min_intensity > int_at_pos || int_at_pos > max_intensity )
        return false;

    if constexpr ( false )
    if ( int_at_pos < 0 || (std::abs(int_at_pos - pt.reflectance) < 1e-7 ) ) // projection is likely further away than the rest of the points
    {
        Eigen::Matrix<double, NUM_MATCH_POINTS, 1> intensities;
        for ( int i = 0; i < pts_near.size(); ++i )
            intensities(i) = pts_near[i].intensity;
        Eigen::Vector2d pt_query = pt_pos_rot;
        Eigen::Matrix<double, 6, 1> pt_params;
        pt_params[0] = pt_query[0] *pt_query[0]; // x^2
        pt_params[1] = pt_query[1] *pt_query[1]; // y^2
        pt_params[2] = pt_query[0] *pt_query[1]; // x*y
        pt_params[3] = pt_query[0]; // x
        pt_params[4] = pt_query[1]; // y
        pt_params[5] = 1;
        ROS_ERROR_STREAM("lambda: " << lambda.transpose() << " p: " << pt_params.transpose() << " i: " << int_at_pos << " /  " << (lambda.dot(pt_params))  << " v: " << intensities.transpose()<< " kp: " << pt_pos_rot.transpose() << " kp:\n" << pts_near_proj<< "\nd:\n" <<
                         (pts_near_proj.template topRows<2>().colwise()-pt_pos_rot));
        return false;
    }
#ifdef USE_TPS
    const Eigen::Vector2d grad_on_plane = getIntensityGradOnPlaneTPS( pt_pos_rot, lambda.template head<NUM_MATCH_POINTS>(), lambda.template tail<3>(), diffs, c_logr ); // [dx,dy] of plane...
#else
    const Eigen::Vector2d grad_on_plane = getIntensityGradOnPlane( pt_pos_rot, lambda ); // [dx,dy] of plane...
#endif
    const Eigen::Matrix3d rot_to_plane = rot_quad.matrix();
    //const Eigen::Vector3d grad_in_world = rot_quad.conjugate() * grad_on_plane;
    //grad_out = grad_in_world.transpose() * (Eigen::Matrix3d::Identity() - normal * normal.transpose());
    grad_out = (grad_on_plane.transpose() * rot_to_plane.template topRows<2>() * (Eigen::Matrix3d::Identity() - normal * normal.transpose())).transpose();
    //  const double grad_norm = grad_out.norm();
    //  if( grad_norm > 0.0 )
    //    grad_out /= grad_norm;

    //DEBUG_OUT( std::endl << "Grad_in_Plane:" << (grad_on_plane * (int_at_pos-pt.reflectance)) << std::endl << "norm=" << grad_on_plane.norm() );

    if constexpr ( print_info )
    {
        static int cnt = 0;
        ++cnt;
        if ( (cnt & 1023) == 0 )
        {
            cnt = 0;
            Eigen::Matrix<double, NUM_MATCH_POINTS, 1> intensities;
            for ( int i = 0; i < pts_near.size(); ++i )
                intensities(i) = pts_near[i].intensity;
            ROS_INFO_STREAM("A: " << int_at_pos << " int: " << pt.reflectance << " i: " << intensities.transpose() << " l: " << lambda.transpose() << " p: " << pt_pos_rot.transpose() << " g: " << grad_out.transpose());
            //          const Eigen::Vector3d closer_pt = pt_pos_rot - pts_near_proj.col(0);
            //          const Eigen::Matrix<double, 3, NUM_MATCH_POINTS> closer_pts = (pts_near_proj.colwise() - pts_near_proj.col(0));
            //          const Eigen::Matrix<double, mat_p_size, 1> other_lambda = getPolynomial( pts_near, closer_pts );
            //          Eigen::Matrix<double, 3, NUM_MATCH_POINTS> pts_near_xyz, pts_near_xyz_rot;
            //          Eigen::Matrix<double, NUM_MATCH_POINTS, 1> intensities;
            //          Eigen::Matrix<double, NUM_MATCH_POINTS, 1> dists;//reflectances,
            //          for ( int i = 0; i < pts_near.size(); ++i )
            //          {
            //              pts_near_xyz.col(i) << pts_near[i].x, pts_near[i].y, pts_near[i].z;
            //              pts_near_xyz_rot.col(i) = rot_quad * pts_near_xyz.col(i);
            //reflectances(i) = pts_near[i].reflectance;
            //              intensities(i) = pts_near[i].intensity;
            //              dists(i) = pts_near_xyz.col(i).dot(normal) + norm_p.gloss; // gloss now has D of plane inside, while intensity contains the pt's distance to plane...
            //          }
            //          const Eigen::Vector3d pts_near_mean = pts_near_xyz.rowwise().mean();
            //          const double d = pts_near_mean.dot(normal);
            //          ROS_INFO_STREAM("pts:\n" << pts_near_xyz << "\nrotated:\n" << pts_near_xyz_rot << "\n pm: " << pts_near_mean.transpose() << " d: " << d << " diff: " << (d+norm_p.gloss) << " D: " << norm_p.intensity << "\ndn:" << dists.transpose() );
            //          ROS_INFO_STREAM("pts_proj: " << pt_pos_rot.transpose() << " near: \n" << pts_near_proj << " \n " << closer_pts);
            //          ROS_INFO_STREAM("A: " << int_at_pos << " ( "  << getIntensityFromPosOnPlane( closer_pt, other_lambda.head<6>() ) << " ) int: " << pt.reflectance //<< " m: " << reflectances.transpose()
            //                          << " i: " << intensities.transpose() << " g: " << grad_out.transpose() << " gn: "<< grad_norm
            //                          << "\nL: " << lambda.transpose() << " o: " << (other_lambda.template head<6>()).transpose()  );
        }
    }

    // why is it with another minus in front on the pc?
    value = (int_at_pos - pt.reflectance);

    return true;
#endif
}

}
