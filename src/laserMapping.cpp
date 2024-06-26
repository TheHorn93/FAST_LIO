// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "IMU_Processing.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd-Tree/ikd_Tree.h>
#include "reflectance_grad.h"
#include "voxel_grid.h"
#include <cv_bridge/cv_bridge.h>
#include "pcl/filters/crop_box.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <mutex>
#include <so3_math.h>
#include <condition_variable>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = true, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

int64_t lidar_ts_ns = 0;
double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0;
double cube_len = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, effct_int_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0}, point_selected_int[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited, has_lidar_end_time_ = false;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false, store_compensated_ = false;
std::shared_ptr<std::ofstream> posesFile = nullptr;
std::shared_ptr<std::ofstream> pcdPosesFile = nullptr;
bool use_reflec_enabled = false;
double ref_grad_w = 0.1;
double thrsd2 = 0.1, thrsd = 0.1;
double min_gradient_mag = 0.1;
std::string tum_out_fname, tum_comp_fname;
int use_channel;

vector<vector<int>>  pointSearchInd_surf;
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points;
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<int64_t>                    time_buffer_ns;
deque<double>                     time_buffer;
deque<float>                      max_curvature_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<PointCloudOuster::Ptr>      lidar_buffer_raw_os;
deque<PointCloudHesai::Ptr>       lidar_buffer_raw_hs;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
PointCloudOuster::Ptr meas_lidar_raw_os = nullptr;
PointCloudOuster::Ptr meas_lidar_comp_os = nullptr;
PointCloudHesai::Ptr meas_lidar_raw_hs = nullptr;
PointCloudHesai::Ptr meas_lidar_comp_hs = nullptr;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr intvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOriInt(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_intvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;
std::shared_ptr<std::vector<bool>> corr_int_selected = std::make_shared<std::vector<bool>>();

std::string comp_type, comp_params;

pcl::VoxelGrid1<PointType> downSizeFilterSurf;
pcl::VoxelGrid1<PointType> downSizeFilterSurfFine;
pcl::VoxelGrid1<PointType> downSizeFilterSurfCoarse;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped, odom_cor_msg;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<Preprocess> p_pre_raw(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

template <typename Scalar>
inline
Scalar project_row ( const Scalar & z, const Scalar & range )
{
    static constexpr Scalar fov_up = 45;
    static constexpr Scalar fov_down = -45;
    static constexpr int num_scan_lines = 128;
    static constexpr Scalar inv_vert_sensor_res = num_scan_lines / (std::abs(fov_up) + std::abs(fov_down));
    static constexpr Scalar invPIx180 = Scalar(180.f)/Scalar(M_PI);
    return  ((Scalar(M_PI_2)-std::acos(z/range)) * invPIx180 - fov_down) * inv_vert_sensor_res;
}

template <typename Scalar>
inline
Scalar project_col ( const Scalar & y, const Scalar & x )
{
    static constexpr Scalar fov_up = 45;
    static constexpr Scalar fov_down = 45;
    static constexpr int num_scan_columns = 1024;

    static constexpr Scalar inv_hori_sensor_res = num_scan_columns / Scalar(360.f);
    static constexpr Scalar to_rel_angle = Scalar(180.f)/Scalar(M_PI) * inv_hori_sensor_res;
    static constexpr Scalar scan_column_offset = num_scan_columns / 2;

    static constexpr bool ccw = false;

    const Scalar col = std::atan2(y,x) * to_rel_angle + scan_column_offset;
    const Scalar col_rel = col + (( col < 0 ) ? +num_scan_columns : ( col >= num_scan_columns ? -num_scan_columns : 0 ));
    return ccw ? col_rel : num_scan_columns - col_rel;


    // point2d[0] = 0.5 * width * (1 - PI_INV * atan2f(point3d[1], point3d[0]));

}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
    po->reflectance = pi->reflectance;
    po->intensity_count = pi->intensity_count;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
    po->reflectance = pi->reflectance;
    po->intensity_count = pi->intensity_count;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

template <typename PointType>
void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    //std::cout << pi->intensity << ", ";
    po->intensity = pi->intensity;
    po->reflectance = pi->reflectance;
    po->intensity_count = pi->intensity_count;
    po->gradient_mag = pi->gradient_mag;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
    po->reflectance = pi->reflectance;
    po->intensity_count = pi->intensity_count;
    po->gradient_mag = pi->gradient_mag;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    //std::cout << "Got pcl" << std::endl;

    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
        max_curvature_buffer.clear();
        lidar_buffer_raw_os.clear();
        lidar_buffer_raw_hs.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    const float min_ref_threshold = use_reflec_enabled ? min_reflectance : min_reflectance_full;
    p_pre->process<PointCloudXYZI>(msg, ptr, min_ref_threshold);
    lidar_buffer.push_back(ptr);
    max_curvature_buffer.emplace_back(p_pre->max_curvature);
    if ( store_compensated_ && p_pre_raw )
    {
        if  ( p_pre->lidar_type == OUSTER )
        {
            PointCloudOuster::Ptr ptr_raw(new PointCloudOuster());
            p_pre_raw->process<PointCloudOuster>(msg, ptr_raw, min_ref_threshold);
            lidar_buffer_raw_os.push_back(ptr_raw);
        }
        if ( p_pre->lidar_type == HESAI32 )
        {
            PointCloudHesai::Ptr ptr_raw(new PointCloudHesai());
            p_pre_raw->process<PointCloudHesai>(msg, ptr_raw, min_ref_threshold);
            lidar_buffer_raw_hs.push_back(ptr_raw);
        }
    }
    time_buffer_ns.push_back(msg->header.stamp.toNSec());
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
        max_curvature_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    const float min_ref_threshold = use_reflec_enabled ? min_reflectance : min_reflectance_full;
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr, min_ref_threshold);
    lidar_buffer.push_back(ptr);
    max_curvature_buffer.emplace_back(p_pre->max_curvature);
    time_buffer_ns.push_back(msg->header.stamp.toNSec());
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    constexpr bool print_info = false;
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        float max_curvature = max_curvature_buffer.front();
        double  lidar_beg_time = time_buffer.front();
        lidar_ts_ns = time_buffer_ns.front();
        if ( has_lidar_end_time_ )
        {
            lidar_end_time = lidar_beg_time;
            if (meas.lidar->points.size() <= 1) // time too little
            {
                lidar_beg_time = lidar_end_time - lidar_mean_scantime;
                ROS_WARN("Too few input point cloud!\n");
            }
            else if (max_curvature / double(1000) < 0.5 * lidar_mean_scantime)
            {
                lidar_beg_time = lidar_end_time - lidar_mean_scantime;
            }
            else
            {
                scan_num ++;
                lidar_beg_time = lidar_end_time - max_curvature / double(1000);
                lidar_mean_scantime += (max_curvature / double(1000) - lidar_mean_scantime) / scan_num;
            }
        }
        else
        {
            if (meas.lidar->points.size() <= 1) // time too little
            {
                lidar_end_time = lidar_beg_time + lidar_mean_scantime;
                ROS_WARN("Too few input point cloud!\n");
            }
            else if (max_curvature / double(1000) < 0.5 * lidar_mean_scantime)
            {
                lidar_end_time = lidar_beg_time + lidar_mean_scantime;
                if constexpr ( print_info )
                ROS_INFO_STREAM("Nope. LastPt: " << max_curvature << " mean: " << lidar_mean_scantime );
            }
            else
            {
                scan_num ++;
                lidar_end_time = lidar_beg_time + max_curvature/ double(1000);
                lidar_mean_scantime += (max_curvature / double(1000) - lidar_mean_scantime) / scan_num;

                if constexpr ( print_info )
                ROS_INFO_STREAM("Matched. LastPt: " << max_curvature << " mean: " << lidar_mean_scantime );
            }
        }

        meas_lidar_raw_hs = nullptr;
        if ( ! lidar_buffer_raw_hs.empty() )
        {
            meas_lidar_raw_hs = lidar_buffer_raw_hs.front();
        }
        meas_lidar_comp_hs = nullptr;

        meas_lidar_raw_os = nullptr;
        if ( ! lidar_buffer_raw_os.empty() )
        {
            meas_lidar_raw_os = lidar_buffer_raw_os.front();
        }
        meas_lidar_comp_os = nullptr;

        meas.lidar_beg_time = lidar_beg_time;
        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    max_curvature_buffer.pop_front();
    if ( ! lidar_buffer_raw_os.empty() ) lidar_buffer_raw_os.pop_front();
    if ( ! lidar_buffer_raw_hs.empty() ) lidar_buffer_raw_hs.pop_front();
    time_buffer_ns.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));

        if ( feats_down_body->points[i].getVector3fMap().squaredNorm() < 5 ) // prevent close ones from fusion
        {
            feats_down_world->points[i].intensity = 0;
            feats_down_world->points[i].reflectance = 0;
            feats_down_world->points[i].intensity_count = 0;
            feats_down_world->points[i].gradient_mag = 0;
        }
        else
        {
            if ( feats_down_world->points[i].intensity > min_reflectance && feats_down_world->points[i].intensity_count < 1 )
                feats_down_world->points[i].intensity_count = 1;
        }

        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }

            PointToAdd.push_back(feats_down_world->points[i]);
            continue;

            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}
#include <rosbag/bag.h>
template <typename PointCloudT, bool toWorld>
void store_compensated_cloud ( typename PointCloudT::Ptr cloud, const uint64_t & prev_pcd_end_time, const std::string & topic  )
{
    const size_t size = cloud->points.size();
    typename PointCloudT::Ptr laserCloudWorld;
    if constexpr ( toWorld )
    {
        laserCloudWorld = typename PointCloudT::Ptr( new PointCloudT(size, 1));
        for (int i = 0; i < size; ++i)
        {
            RGBpointBodyToWorld( &cloud->points[i], &laserCloudWorld->points[i]);
        }
    }
    else
    {
        laserCloudWorld = cloud;
    }

    constexpr bool store_to_bag = true;
    if constexpr ( store_to_bag )
    {
        const string all_points_dir(string(string(ROOT_DIR) + "/PCD_COMP/stored.bag"));
        static std::shared_ptr<rosbag::Bag> bagFile = nullptr;
        if ( ! bagFile )
            bagFile = std::make_shared<rosbag::Bag>( all_points_dir, rosbag::bagmode::Write );

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromNSec(lidar_ts_ns);
        laserCloudmsg.header.frame_id = "camera_init";
        bagFile->write(topic, laserCloudmsg.header.stamp,laserCloudmsg);
        cout << "current scan saved to PCD_COMP: " << all_points_dir << " " << to_string(prev_pcd_end_time) << " t: " << std::to_string(lidar_ts_ns)<< endl;
    }
    else
    {
        const string all_points_dir(string(string(ROOT_DIR) + "/PCD_COMP/") + to_string(prev_pcd_end_time) + string(".pcd"));
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to PCD_COMP: " << all_points_dir << " "<< endl;
        pcd_writer.writeBinary(all_points_dir, *laserCloudWorld);
    }
}


bool dist_passed = false;
PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    //std::cout << "Pub registered" << std::endl;
    if(scan_pub_en && pubLaserCloudFull.getNumSubscribers() > 0)
    {
        constexpr bool print_info = false;
        float maxVal = 0, maxValI = 0, maxValG = 0;
        if constexpr ( print_info )
        {
            for (int i = 0; i < feats_undistort->size(); i++)
            {
                if ( maxValI < feats_undistort->points[i].intensity ) maxValI = feats_undistort->points[i].intensity;
                if ( maxVal < feats_undistort->points[i].reflectance ) maxVal = feats_undistort->points[i].reflectance;
                if ( maxValG < feats_undistort->points[i].gradient_mag ) maxValG = feats_undistort->points[i].gradient_mag;
            }
            std::cout << "maxPubVal: " << maxVal << " I: " << maxValI << " G: " << maxValG << std::endl;
            maxVal = 0; maxValI = 0; maxValG = 0;
            for (int i = 0; i < feats_down_body->size(); i++)
            {
                if ( maxValI < feats_down_body->points[i].intensity ) maxValI = feats_down_body->points[i].intensity;
                if ( maxVal < feats_down_body->points[i].reflectance ) maxVal = feats_down_body->points[i].reflectance;
                if ( maxValG < feats_down_body->points[i].gradient_mag ) maxValG = feats_down_body->points[i].gradient_mag;
            }
            std::cout << "maxPubVal: " << maxVal << " I: " << maxValI << " G: " << maxValG << std::endl;
            maxVal = 0; maxValI = 0; maxValG = 0;
        }

        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        maxVal = 0; maxValI = 0; maxValG = 0;
        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
            if ( maxValI < laserCloudWorld->points[i].intensity ) maxValI = laserCloudWorld->points[i].intensity;
            if ( maxVal < laserCloudWorld->points[i].reflectance ) maxVal = laserCloudWorld->points[i].reflectance;
            if ( maxValG < laserCloudWorld->points[i].gradient_mag ) maxValG = laserCloudWorld->points[i].gradient_mag;
        }
        if constexpr ( print_info )
        ROS_INFO_STREAM_THROTTLE(1, "maxPubVal: " << maxVal << " I: " << maxValI << " G: " << maxValG );
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories  */
    /* 2. noted that pcd save will influence the real-time performences */
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        static uint64_t prev_pcd_end_time = first_lidar_time * 1e9;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/") + to_string(prev_pcd_end_time) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
            prev_pcd_end_time = lidar_end_time * 1e9;
        }
    }

    if ( dist_passed && store_compensated_ && (meas_lidar_comp_os || meas_lidar_comp_hs) )
    {
        static int scan_wait_num = 0;
        static uint64_t prev_pcd_end_time = first_lidar_time * 1e9;
        scan_wait_num ++;

        constexpr bool use_full = true;
        constexpr bool toWorld = false;

        //PointCloudOuster::Ptr cloud = meas_lidar_comp; //( use_full ? meas_lidar_comp : feats_undistort );

        const int size = meas_lidar_comp_os ? meas_lidar_comp_os->points.size() :
                                              ( meas_lidar_comp_hs ? meas_lidar_comp_hs->points.size() : 0 );
        if ( //scan_wait_num >= pcd_save_interval &&
             size > 0 )
        {
            if ( meas_lidar_comp_hs )
                store_compensated_cloud<PointCloudHesai,toWorld>( meas_lidar_comp_hs, prev_pcd_end_time,  "/hs_cloud/comp" );
            if ( meas_lidar_comp_os )
                store_compensated_cloud<PointCloudOuster,toWorld>( meas_lidar_comp_os, prev_pcd_end_time, "/os_cloud/comp" );
            //if ( meas_lidar_comp_os )
            //    store_compensated_cloud<PointCloudXYZI,toWorld>( feats_undistort, prev_pcd_end_time, "/os_cloud_dist/comp" );
            scan_wait_num = 0;
            prev_pcd_end_time = lidar_end_time * 1e9;

            if ( ! pcdPosesFile ) pcdPosesFile = std::make_shared<std::ofstream>(tum_comp_fname);
            if( pcdPosesFile && pcdPosesFile->is_open() )
            {
                const Eigen::Quaterniond q((state_point.rot * state_point.offset_R_L_I).coeffs());
                if constexpr ( toWorld )
                    (*pcdPosesFile) << (lidar_ts_ns) << " 0 0 0 0 0 0 1"<< std::endl;
                else
                    (*pcdPosesFile) << (lidar_ts_ns) << " " << pos_lid(0) << " " << pos_lid(1) << " " << pos_lid(2)
                                    << " " << (q.x()) << " " << (q.y()) << " " << (q.z()) << " " << q.w() << std::endl;
            }
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    if ( pubLaserCloudFull_body.getNumSubscribers() <= 0 ) return;

    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    if ( pubLaserCloudEffect.getNumSubscribers() <= 0 ) return;

    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    if ( pubLaserCloudMap.getNumSubscribers() <= 0 ) return;

    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;

}


void publish_odometry_correction( const ros::Publisher& pub_odom_cor )
{
    odom_cor_msg.header.frame_id = "camera_init";
    odom_cor_msg.child_frame_id = "body";
    odom_cor_msg.header.stamp = ros::Time().fromSec(lidar_end_time);

    odom_cor_msg.pose.pose.position.x = 0;
    odom_cor_msg.pose.pose.position.y = 0;
    odom_cor_msg.pose.pose.position.z = 0;
    odom_cor_msg.pose.pose.orientation.x = 0;
    odom_cor_msg.pose.pose.orientation.y = 0;
    odom_cor_msg.pose.pose.orientation.z = 0;
    odom_cor_msg.pose.pose.orientation.w = 1;

    pub_odom_cor.publish(odom_cor_msg);
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
    {
        // write to file:
        if ( ! posesFile ) posesFile = std::make_shared<std::ofstream>(tum_out_fname);
        if( posesFile && posesFile->is_open() )
        {
             (*posesFile) << (lidar_ts_ns) << " " << transform.getOrigin().x() << " " << transform.getOrigin().y() << " " << transform.getOrigin().z()
                          << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
    }
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        if ( pubPath.getNumSubscribers() <= 0 ) return;
        pubPath.publish(path);
    }
}

constexpr bool use_reflec_on_plane = true;
constexpr bool only_high_grad = false;

void h_share_model_with_reflec_on_plane(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) // TODO measurement prediction from state prediction
{
    constexpr bool print_info = false;
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    corr_intvect->clear();
    corr_int_selected->clear();
    total_residual = 0.0;

    //std::atomic<int> num_larger = 0, num_int = 0;
    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for schedule (dynamic, 16)
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /// Transform pt laser -> pt global frame
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        // Forward all necessary information
        point_world.intensity = point_body.intensity;
        //std::cout << "pt_W_i=" << point_body.intensity << ", pt_W_r=" << point_body.reflectance << std::endl;
        point_world.reflectance = point_body.intensity;
        point_world.gradient_mag = point_body.gradient_mag;


        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        PointVector& points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /// Search ikd-tree for closest points
            // returns nearest point in points_near ańd distances in pointSearchSqDis
            // This list should be sorted -> ikd-Tree MANUAL_HEAP.FloatUp() on insert should do so...?
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis); // Find nearest surface -> find nearest surface and identit
            // If at least 5 points can be found current: point can be used further.
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : (pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true);
        }

        if (!point_selected_surf[i]) continue;

        /// PCA of 5 nearest points
        VF(4) pabcd;
        point_selected_surf[i] = false;
        point_selected_int[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f)) // Estimate plane by pca: see here: "https://stats.stackexchange.com/questions/163356/fitting-a-plane-to-a-set-of-points-in-3d-using-pca"
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
            /// If plane is valid point can be computed further
            if (s > 0.9) // TODO: Maybe checks difference point to plane along normal?
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2; // will be used for distance to plane
                normvec->points[i].reflectance = point_body.intensity; // contains either intensity or real reflectivity
                normvec->points[i].gradient_mag = point_body.gradient_mag;
                normvec->points[i].gloss = pabcd(3); // store D of plane
                normvec->points[i].curvature = 
                res_last[i] = abs(pd2);

                if constexpr ( only_high_grad ) if ( point_body.reflectance < min_reflectance ) continue; //else ++num_larger;
                if ( use_reflec_enabled && point_world.reflectance > min_reflectance && point_body.gradient_mag > min_gradient_mag )
                {
                    // get reflectance gradient to surface
                    double error = 0, value = 0;
                    V3D ref_grad; // should be in world frame
                    point_selected_int[i] = reflectance::IrregularGrid::computeErrorAndGradientPlane( point_world, points_near, normvec->points[i], error, ref_grad, value );
                    //point_selected_int[i] = reflectance::IrregularGrid::computeErrorAndGradientPlane2DTPS( point_world, points_near, normvec->points[i], error, ref_grad, value );
                    //point_selected_int[i] = reflectance::IrregularGrid::computeErrorAndGradientPlane3DTPS( point_world, points_near, normvec->points[i], error, ref_grad, value );
                    intvec->points[i].intensity = error;
                    intvec->points[i].reflectance = value;
                    intvec->points[i].x = ref_grad(0);
                    intvec->points[i].y = ref_grad(1);
                    intvec->points[i].z = ref_grad(2);
                    //if ( point_selected_int[i] ) ++num_int;
                }
            }
        }
    }
    //ROS_INFO_STREAM("high grad: " << num_larger << " " << num_int);

    /// Accumulate points in laser rep and associated normvecs for all valid points
    // Result are two point clouds: 1st containing points in laser coor-rep, 2nd containing estimated plane normal vecs as points
    effct_feat_num = 0, effct_int_num = 0;
    laserCloudOri->reserve(feats_down_size);
    corr_normvect->reserve(feats_down_size);
    corr_intvect->reserve(feats_down_size);
    corr_int_selected->reserve(feats_down_size); // not threadsafe for writing.
    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points.emplace_back(feats_down_body->points[i]);
            corr_normvect->points.emplace_back(normvec->points[i]);
            corr_intvect->points.emplace_back(intvec->points[i]);
            corr_int_selected->emplace_back(point_selected_int[i]);
            total_residual += res_last[i];
            ++effct_feat_num;
        }
    }
    effct_int_num = effct_feat_num;
    // If no valid points are found return function without changing state
    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();

    /*** Computation of Measurment Jacobian matrix H and measurents vector ***/
    size_t num_residuals = effct_feat_num;
    ekfom_data.h_x = MatrixXd::Zero(num_residuals, 12); //23
    ekfom_data.h = MatrixXd::Zero(num_residuals,1);
    ekfom_data.h_x_int = MatrixXd::Zero(effct_int_num, 12); //23
    ekfom_data.h_int = MatrixXd::Zero(effct_int_num,1);

    /** Create one error point for point_to_plane and one error point for intensity gradient point */
    //num_int = 0;
    int num_from_intensity = 0;
    int res_it = 0;
    for (int i = 0; i < effct_feat_num; ++i, ++res_it)
    {
        /// Use point in laser coords
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be); // SKEW_SYM_MATRX returns ((0,-z,y), (z,0,-x), (-y,x,0))

        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I; // Transform current point: laser -> imu
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this); // Skew sym matrix for imu coord point == [p_imu]_x ( deriv of rotation )

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measurement Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec); // Conjugates quaternion s.rot -> rotate world frame norm vec to imu frame
        V3D A(point_crossmat * C);
        // If Extrinsic laser -> IMU should be estimated
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.template block<1, 12>(res_it,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        // If Extrinsic sufficiently known
        else
        {
            /// h_x is mat elem R^(num_pts x measurement DOFs)
            // measurement dofs = world->body rot +trans => 6-dims + body->IMU rot+trans => 6-dims => 12-dims
            // to h_x<1,6>(i,0) holds estimated difference point[i] measured pos -> point[i] esti pos
            // 1-3 contains translation component
            // 4-6 contains rotation component
            ekfom_data.h_x.template block<1, 12>(res_it,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            //     n_w', ([p_i]_x * R_wi' * n_w)'      // since VEC_FROM_ARRAY inherently is a transpose
            // <=> n_w', n_w' * (R_wi')' * [p_i]_x' =  // [p_i]_x' = -[p_i]_x
            // <=> n_w', n_w' * R_wi * [-p_i]_x
            // <=> n_w' * ( I, R_wi * [-p_i]_x ) <= all in world now
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        // Sum of all errors/resiudals
        ekfom_data.h(res_it) = -norm_p.intensity; // minus should come from "-Kz" in paper equation?

        //if ( point_selected_int[i] ) ++num_int;

        /** Add Intensity gradient to kalman state. */
        if ( use_reflec_enabled && (*corr_int_selected)[i] )
        {
            const PointType &int_p = corr_intvect->points[i];
            // get reflectance gradient to surface
            V3D ref_grad ( int_p.x, int_p.y, int_p.z ); // should be in world frame

//            const double res2 = int_p.intensity*int_p.intensity;
//            const double th_res2 = thrsd + res2;
//            const double weight = ref_grad_w * thrsd2 / ( th_res2 * th_res2 );
            const double weight = ref_grad_w;

            // all points in world, normal too.
            // ref_g' * ( I, R_wi * [-p_i]_x )

            // Transform intensity grad: World -> IMU coords
            Eigen::Matrix<double,6,1> J_ref;
            J_ref.template head<3>() = ref_grad;
            J_ref.template tail<3>() = point_crossmat * (s.rot.conjugate() * ref_grad); // Conjugates quaternion s.rot -> rotate world frame grad vec to imu frame

            ekfom_data.h_x_int.template block<1, 6>(num_from_intensity,0) = weight * J_ref.transpose(); // 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            ekfom_data.h_int(num_from_intensity) = - weight * int_p.intensity;

            if constexpr ( print_info )
            {
                if ( (res_it & 1023) == 0 )
                {
                    ROS_INFO_STREAM("ri: " << res_it << " w: " << ref_grad_w << " ed: " << ekfom_data.h(res_it) << " ei: " << ekfom_data.h_int(num_from_intensity) << "\n"
                                    << (ekfom_data.h_x.template block<1, 6>(res_it,0))  << "\n" << (ekfom_data.h_x_int.template block<1, 6>(num_from_intensity,0)));
                }
            }
            ++num_from_intensity;
        }
    }

    if ( effct_feat_num != res_it )
    {
        ekfom_data.h_x.conservativeResize(res_it,12);
        ekfom_data.h.conservativeResize(res_it,1);
    }
    if ( effct_int_num != num_from_intensity )
    {
        ekfom_data.h_x_int.conservativeResize(num_from_intensity,12);
        ekfom_data.h_int.conservativeResize(num_from_intensity,1);
    }
    //if constexpr ( print_info )
    ROS_INFO_STREAM_THROTTLE(1,"int: " << num_from_intensity << " d: " << effct_feat_num << " H: " << ekfom_data.h_x.rows() << " x " << ekfom_data.h_x.cols() << " h: " << ekfom_data.h.rows() << " " << ekfom_data.h.cols() << " ref_en: " << use_reflec_enabled );
    solve_time += omp_get_wtime() - solve_start_;
}

void h_share_model_without_reflec_on_plane(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) // TODO measurement prediction from state prediction
{
    constexpr bool print_info = false;
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    laserCloudOriInt->clear();
    corr_normvect->clear();
    corr_intvect->clear();
    total_residual = 0.0;

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for schedule (dynamic, 16)
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /// Transform pt laser -> pt global frame
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        // Forward all necessary information
        point_world.intensity = point_body.intensity;
        //std::cout << "pt_W_i=" << point_body.intensity << ", pt_W_r=" << point_body.reflectance << std::endl;
        point_world.reflectance = point_body.intensity;
        point_world.gradient_mag = point_body.gradient_mag;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        PointVector& points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /// Search ikd-tree for closest points
            // returns nearest point in points_near ańd distances in pointSearchSqDis
            // This list should be sorted -> ikd-Tree MANUAL_HEAP.FloatUp() on insert should do so...?
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis); // Find nearest surface -> find nearest surface and identit
            // If at least 5 points can be found current: point can be used further.
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : (pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true);
        }

        if (!point_selected_surf[i]) continue;

        /// PCA of 5 nearest points
        VF(4) pabcd;
        point_selected_surf[i] = false;
        point_selected_int[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f)) // Estimate plane by pca: see here: "https://stats.stackexchange.com/questions/163356/fitting-a-plane-to-a-set-of-points-in-3d-using-pca"
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());


//            if ( (i & 511) == 0 )
//            {
//            V3D mean ( points_near[0].x,points_near[0].y,points_near[0].z );
//            for ( int i = 1; i < NUM_MATCH_POINTS; ++i)
//            {
//                mean += V3D(points_near[i].x,points_near[i].y,points_near[i].z);
//            }
//            mean /= NUM_MATCH_POINTS;
//            double e = pabcd.template head<3>().template cast<double>().dot( V3D(point_world.x,point_world.y,point_world.z)-mean );
//            ROS_INFO_STREAM("e: " << e << " " << pd2);
//            }



            /// If plane is valid point can be computed further
            if (s > 0.9) // TODO: Maybe checks difference point to plane along normal?
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2; // will be used for distance to plane
                normvec->points[i].reflectance = point_body.intensity; // contains either intensity or real reflectivity
                normvec->points[i].gradient_mag = point_body.gradient_mag;
                normvec->points[i].gloss = pabcd(3); // store D of plane
                normvec->points[i].curvature =
                res_last[i] = abs(pd2);
            }
        }

        if ( use_reflec_enabled && point_world.reflectance > min_reflectance && point_body.gradient_mag > min_gradient_mag )
        {
            if ( ! point_selected_surf[i] ) continue;
            // get reflectance gradient to surface
            double error = 0, value = 0;
            V3D ref_grad; // should be in world frame
            point_selected_int[i] = reflectance::IrregularGrid::computeErrorAndGradient3D( point_world, points_near, error, ref_grad, value );
            intvec->points[i].intensity = error;
            intvec->points[i].reflectance = value;
            intvec->points[i].x = ref_grad(0);
            intvec->points[i].y = ref_grad(1);
            intvec->points[i].z = ref_grad(2);
        }
    }

    /// Accumulate points in laser rep and associated normvecs for all valid points
    // Result are two point clouds: 1st containing points in laser coor-rep, 2nd containing estimated plane normal vecs as points
    effct_feat_num = 0, effct_int_num = 0;
    laserCloudOri->reserve(feats_down_size);
    corr_normvect->reserve(feats_down_size);
    laserCloudOriInt->reserve(feats_down_size);
    corr_intvect->reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points.emplace_back(feats_down_body->points[i]);
            corr_normvect->points.emplace_back(normvec->points[i]);
            total_residual += res_last[i];
            ++effct_feat_num;
        }
        if ( point_selected_int[i] )
        {
            laserCloudOriInt->points.emplace_back(feats_down_body->points[i]);
            corr_intvect->points.emplace_back(intvec->points[i]);
            ++effct_int_num;
        }
    }
    // If no valid points are found return function without changing state
    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();

    /*** Computation of Measurment Jacobian matrix H and measurents vector ***/
    size_t num_residuals = effct_feat_num; //*2;
    ekfom_data.h_x = MatrixXd::Zero(num_residuals, 12); //23
    ekfom_data.h = MatrixXd::Zero(num_residuals,1);
    ekfom_data.h_x_int = MatrixXd::Zero(effct_int_num, 12); //23
    ekfom_data.h_int = MatrixXd::Zero(effct_int_num,1);

    /** Create one error point for point_to_plane and one error point for intensity gradient point */
    int num_from_intensity = 0;
    int res_it = 0;
    for (int i = 0; i < effct_feat_num; ++i, ++res_it)
    {
        //const int res_it = 2*i;
        /// Use point in laser coords
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be); // SKEW_SYM_MATRX returns ((0,-z,y), (z,0,-x), (-y,x,0))

        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I; // Transform current point: laser -> imu
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this); // Skew sym matrix for imu coord point == [p_imu]_x ( deriv of rotation )

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measurement Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec); // Conjugates quaternion s.rot -> rotate world frame norm vec to imu frame
        V3D A(point_crossmat * C);
        // If Extrinsic laser -> IMU should be estimated
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.template block<1, 12>(res_it,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        // If Extrinsic sufficiently known
        else
        {
            /// h_x is mat elem R^(num_pts x measurement DOFs)
            // measurement dofs = world->body rot +trans => 6-dims + body->IMU rot+trans => 6-dims => 12-dims
            // to h_x<1,6>(i,0) holds estimated difference point[i] measured pos -> point[i] esti pos
            // 1-3 contains translation component
            // 4-6 contains rotation component
            ekfom_data.h_x.template block<1, 12>(res_it,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            //     n_w', ([p_i]_x * R_wi' * n_w)'      // since VEC_FROM_ARRAY inherently is a transpose
            // <=> n_w', n_w' * (R_wi')' * [p_i]_x' =  // [p_i]_x' = -[p_i]_x
            // <=> n_w', n_w' * R_wi * [-p_i]_x
            // <=> n_w' * ( I, R_wi * [-p_i]_x ) <= all in world now
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        // Sum of all errors/resiudals
        ekfom_data.h(res_it) = -norm_p.intensity; // minus should come from "-Kz" in paper equation?
    }
    if ( use_reflec_enabled )
    for (int i = 0; i < effct_int_num; ++i)
    {
        //const int res_it = 2*i;
        /// Use point in laser coords
        const PointType &laser_p  = laserCloudOriInt->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I; // Transform current point: laser -> imu
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this); // Skew sym matrix for imu coord point == [p_imu]_x ( deriv of rotation )

        const PointType &int_p = corr_intvect->points[i];
        // get reflectance gradient to surface
        V3D ref_grad ( int_p.x, int_p.y, int_p.z ); // should be in world frame

//        const double res2 = int_p.intensity*int_p.intensity;
//        const double th_res2 = thrsd + res2;
//        const double weight = ref_grad_w * thrsd2 / ( th_res2 * th_res2 );
        const double weight = ref_grad_w;

        // weight them
        ref_grad *= weight;

        // ([p_i]_x * R_wi' * n_w)'  = ([p_i]_x * R_iw * n_w)'

        // all points in world, normal too.
        // ref_g' * ( I, R_wi * [-p_i]_x )

        // Transform intensity grad: World -> IMU coords
        V3D C_ref(s.rot.conjugate() * ref_grad); // Conjugates quaternion s.rot -> rotate world frame grad vec to imu frame
        V3D A_ref(point_crossmat * C_ref);

        /** Add vector to state. */
        ekfom_data.h_x_int.template block<1, 12>(num_from_intensity,0) << VEC_FROM_ARRAY(ref_grad), VEC_FROM_ARRAY(A_ref), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        ekfom_data.h_int(num_from_intensity) = - weight * int_p.intensity;
        ++num_from_intensity;
    }

    if ( effct_feat_num != res_it )
    {
        ekfom_data.h_x.conservativeResize(res_it,12);
        ekfom_data.h.conservativeResize(res_it,1);
    }
    if ( effct_int_num != num_from_intensity )
    {
        ekfom_data.h_x_int.conservativeResize(num_from_intensity,12);
        ekfom_data.h_int.conservativeResize(num_from_intensity,1);
    }
    if constexpr ( print_info )
    ROS_INFO_STREAM_THROTTLE(1,"int: " << num_from_intensity << " d: " << effct_feat_num << " H: " << ekfom_data.h_x.rows() << " x " << ekfom_data.h_x.cols() << " h: " << ekfom_data.h.rows() << " " << ekfom_data.h.cols()  );
    solve_time += omp_get_wtime() - solve_start_;
}

Eigen::Vector3f colorFromNormal( const Eigen::Vector3f & normal )
{
    const float ngfx = (normal.x()+1)*.5;
    const float ngfy = (normal.y()+1)*.5;
    const float ngfz = (normal.z()+1)*.5;
    return Eigen::Vector3f(ngfx,ngfy,ngfz);
}

std::string generatePathFileName( double ref_grad_w, double filter_size_surf, double filter_size_map, int point_filter_num )
{
    std::stringstream sstr;
    sstr << "w" << ref_grad_w << "_";
    sstr << "fs" << filter_size_surf << "_";
    sstr << "fm" << filter_size_map << "_";
    sstr << "np" << point_filter_num << ".tum";
    return sstr.str();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;
    bool dont_compensate = false;
    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<string>("common/comp_lid_topic",lid_topic,"/cloud_compensated");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("filter_use_channel", use_channel, 0);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("dont_compensate", dont_compensate, false);
    nh.param<bool>("store_compensated", store_compensated_, false);
    nh.param<bool>("has_lidar_end_time", has_lidar_end_time_, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, true);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    nh.param<double>("ref_grad_w", ref_grad_w, 0.1);
    nh.param<double>("min_gradient_mag", min_gradient_mag, 0.1);
    ROS_INFO_STREAM("p_pre->lidar_type "<<p_pre->lidar_type);

    std::stringstream sstr;
    sstr << "./fast_lio_after_map_poses_";
    sstr << generatePathFileName( ref_grad_w, filter_size_surf_min, filter_size_map_min, p_pre->point_filter_num );
    tum_out_fname = sstr.str();
    sstr <<"_pcd.txt";
    tum_comp_fname = sstr.str();

    use_reflec_enabled = false;
    const double ref_grad_w_orig = ref_grad_w;
    if ( ref_grad_w > 0. )
    {
        use_reflec_enabled = true;
        p_pre->use_compensated = true; // raw ouster nce
        //p_pre->point_filter_num = 1; // filtering in compensate node already!
    }
    ROS_WARN_STREAM( "ref_grad_w = " << ref_grad_w << " ( " << ref_grad_w_orig << " ) " << " grad th: " << min_gradient_mag << " f: " << tum_out_fname);
    ROS_WARN_STREAM( "runtime_pos_log = " << (runtime_pos_log ? "true" : "false") );
    ROS_WARN_STREAM( "size_surf = " << filter_size_surf_min << ", size_map = " << filter_size_map_min );
    ROS_WARN_STREAM( "use_channel = " << use_channel );
    if ( store_compensated_ )
    {
        p_pre_raw->blind = p_pre->blind;
        p_pre_raw->lidar_type = p_pre->lidar_type;
        p_pre_raw->N_SCANS = p_pre->N_SCANS;
        p_pre_raw->time_unit = p_pre->time_unit;
        p_pre_raw->SCAN_RATE = p_pre->SCAN_RATE;
        p_pre_raw->point_filter_num = 1;
        p_pre_raw->pass_through = true;

        {
            float time_unit_scale = 1.f;
            switch (p_pre_raw->time_unit)
            {
              case SEC:
                time_unit_scale = 1.e3f;
                break;
              case MS:
                time_unit_scale = 1.f;
                break;
              case US:
                time_unit_scale = 1.e-3f;
                break;
              case NS:
                time_unit_scale = 1.e-6f;
                break;
              default:
                time_unit_scale = 1.f;
                break;
            }
            p_imu->time_unit_scale = time_unit_scale;
        }
    }

    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterSurfFine.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterSurfCoarse.setLeafSize(filter_size_surf_min*2, filter_size_surf_min*2, filter_size_surf_min*2);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(point_selected_int, false, sizeof(point_selected_int));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->set_dont_compensate(dont_compensate);

    std::array<double, 23> epsi{}; // stricter values from ethz_asl: fast lio
    epsi.fill(1e-5);
    if constexpr ( use_reflec_on_plane )
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model_with_reflec_on_plane, NUM_MAX_ITERATIONS, &epsi.front()); // TODO init function
    else
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model_without_reflec_on_plane, NUM_MAX_ITERATIONS, &epsi.front()); // TODO init function

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    if (runtime_pos_log)
    {
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;
    }

    /*** ROS subscribe initialization ***/
    ROS_INFO_STREAM( "Subscribing:\n" << "  lidar_topic=" << lid_topic << "\n  imu_topic=" << imu_topic << std::endl );
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 10, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 2, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 50, imu_cbk);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>
            ("/Odometry", 100000);
    ros::Publisher pubOdomCorMapped = nh.advertise<nav_msgs::Odometry>
            ("/odometry_correction", 100000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path>
            ("/path", 100000);

    ros::Publisher pubRefImg = nh.advertise<sensor_msgs::Image>("/refimg", 100);
    ros::Publisher pubRefCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ref_cloud", 100000);
    ros::Publisher pubRefCorCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ref_cor_cloud", 100000);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);


            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            if ( store_compensated_ )
            {
                if ( meas_lidar_raw_os )
                {
                    meas_lidar_comp_os = PointCloudOuster::Ptr(new PointCloudOuster());
                    p_imu->UndistortRawPcl<PointCloudOuster>( meas_lidar_raw_os, kf, meas_lidar_comp_os );
                }
                if ( meas_lidar_raw_hs )
                {
                    meas_lidar_comp_hs = PointCloudHesai::Ptr(new PointCloudHesai());
                    p_imu->UndistortRawPcl<PointCloudHesai>( meas_lidar_raw_hs, kf, meas_lidar_comp_hs );
                }
            }


            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            constexpr bool close_with_higher_res = true;
            if constexpr ( close_with_higher_res )
            {
                pcl::CropBox<PointType> crop;
                PointCloudXYZI::Ptr feats_undistort_crop_in( new PointCloudXYZI);
                PointCloudXYZI::Ptr feats_undistort_crop_out( new PointCloudXYZI );
                crop.setMin(Eigen::Vector4f::Constant(-10));
                crop.setMax(Eigen::Vector4f::Constant(10));
                crop.setInputCloud(feats_undistort);
                crop.filter(*feats_undistort_crop_in);
                crop.setNegative(true);
                crop.filter(*feats_undistort_crop_out);

                PointCloudXYZI feats_down_fine;
                downSizeFilterSurfFine.setInputCloud(feats_undistort_crop_in); // builds pcl::Voxelgrid<PointXYZINormal>, builds equally spaced 3D Grid and filters for centerpoints
                downSizeFilterSurfFine.filter(feats_down_fine); // Apply grid filter and write to pcl::PointCloud &feats_down_body

                PointCloudXYZI feats_down_coarse;
                downSizeFilterSurfCoarse.setInputCloud(feats_undistort_crop_out); // builds pcl::Voxelgrid<PointXYZINormal>, builds equally spaced 3D Grid and filters for centerpoints
                downSizeFilterSurfCoarse.filter(feats_down_coarse); // Apply grid filter and write to pcl::PointCloud &feats_down_body


                *feats_down_body = feats_down_coarse + feats_down_fine;
            }
            else
            {
                downSizeFilterSurf.setInputCloud(feats_undistort); // builds pcl::Voxelgrid<PointXYZINormal>, builds equally spaced 3D Grid and filters for centerpoints
                downSizeFilterSurf.filter(*feats_down_body); // Apply grid filter and write to pcl::PointCloud &feats_down_body
            }
            //feats_down_body = feats_undistort;

            //if constexpr ( false )
            if constexpr ( only_high_grad )
            if ( use_reflec_enabled )
            {
                // disable others...
                for ( int i = 0; i < feats_down_body->points.size(); ++i )
                {
                    (*feats_down_body)[i].reflectance = 0;
                    (*feats_down_body)[i].gradient_mag = 0;
                }

//                int num_counts_grt_one = 0;
//                for ( int i = 0; i < feats_down_body->points.size(); ++i )
//                {
//                    num_counts_grt_one += (*feats_down_body)[i].intensity_count > 1.1f;
//                }

                const int prev_size = feats_down_body->points.size();
                int num_high_grad = 0;
                for ( int i = 0; i < feats_undistort->points.size(); ++i )
                {
                    if ( (*feats_undistort)[i].intensity > min_reflectance && (*feats_undistort)[i].reflectance > 0.1f) // should contain grad mag
                    {
                        ++num_high_grad;
                        feats_down_body->points.emplace_back((*feats_undistort)[i]);
                        //if  ( (*feats_undistort)[i].reflectance != feats_down_body->points.back().reflectance )
                        //    ROS_ERROR_STREAM("er: "<<  (*feats_undistort)[i].reflectance << " " << feats_down_body->points.back().reflectance );
                    }
                }
                ROS_INFO_STREAM_THROTTLE(1,"e " << num_high_grad //<< " ng: " << num_counts_grt_one
                                         << " mr: " << min_reflectance << " prev: " << prev_size << " now: " << feats_down_body->points.size());
            }

            if constexpr ( !false )
            {
                // voxelgrid filter drops reflectance channel.
                float maxValR = 0, maxValI = 0, maxValG = 0;
                for (int i = 0; i < feats_undistort->size(); i++)
                {
                    if ( maxValI < feats_undistort->points[i].intensity ) maxValI = feats_undistort->points[i].intensity;
                    if ( maxValR < feats_undistort->points[i].reflectance ) maxValR = feats_undistort->points[i].reflectance;
                    if ( maxValG < feats_undistort->points[i].gradient_mag ) maxValG = feats_undistort->points[i].gradient_mag;
                }
                ROS_INFO_STREAM_THROTTLE(1,"dSmaxPubVal: " << maxValR << " I: " << maxValI << " G: " << maxValG << " #: " << feats_undistort->size() << " pf: " <<  p_pre->point_filter_num);
                maxValR = 0; maxValI = 0; maxValG = 0;
                for (int i = 0; i < feats_down_body->size(); i++)
                {
                    if ( maxValI < feats_down_body->points[i].intensity ) maxValI = feats_down_body->points[i].intensity;
                    if ( maxValR < feats_down_body->points[i].reflectance ) maxValR = feats_down_body->points[i].reflectance;
                    if ( maxValG < feats_down_body->points[i].gradient_mag ) maxValG = feats_down_body->points[i].gradient_mag;
                }
                ROS_INFO_STREAM_THROTTLE(1,"dSmaxPubVal: " << maxValR << " I: " << maxValI << " G: " << maxValG << " #: " << feats_down_body->size() << " pf: " <<  p_pre->point_filter_num);
            }


            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            normvec->resize(feats_down_size);
            intvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            if (runtime_pos_log)
                fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if ( pubLaserCloudMap.getNumSubscribers() > 0 ) //if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            if ( store_compensated_ )
            {
                static Eigen::Vector3d last_pos = Eigen::Vector3d(state_point.pos[0]-2.,state_point.pos[1]-2.,state_point.pos[2]-2.); // take first one too
                dist_passed = (last_pos - state_point.pos).norm() > 0.5;
                if ( dist_passed )
                {
                    //std::cout << "dist: " << dist_passed << " cur: " << state_point.pos(0) << " " << state_point.pos(1) << " " << state_point.pos(2) << " last: " << last_pos.transpose() << std::endl;
                    last_pos = state_point.pos;
                }
            }
            /******* Publish odometry *******/
            publish_odometry_correction(pubOdomCorMapped);
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();

            /******* Publish points *******/
            //std::cout << "publish?: " << ( scan_pub_en || pcd_save_en || store_compensated_ ) << std::endl;
            if (path_en) publish_path(pubPath);
            if (scan_pub_en || pcd_save_en || (store_compensated_ && dist_passed) )  publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            // publish_effect_world(pubLaserCloudEffect);
            if ( pubLaserCloudMap.getNumSubscribers() > 0 ) publish_map(pubLaserCloudMap);


            if  ( pubRefCloud.getNumSubscribers() > 0 )
            {
                int num_ints = 0, num_neighs = 0;
                for ( int i = 0; i < effct_feat_num; ++i )
                {
                    if ( !(*corr_int_selected)[i] ) continue;
                    ++num_ints;
                }
                for ( int i = 0, m = 0; i < feats_down_size; ++i )
                {
                    if ( ! point_selected_int[i] ) continue;
                    const PointVector &points_near = Nearest_Points[i];
                    for ( int j = 0; j < NUM_MATCH_POINTS; ++j)
                        if ( points_near[j].intensity > min_reflectance )
                            ++num_neighs;
                }

                PointCloudXYZI::Ptr refCloudWorld( new PointCloudXYZI(num_ints, 1));
                PointCloudXYZI::Ptr refCorCloudWorld( new PointCloudXYZI(num_neighs, 1));
                for ( int i = 0, k = 0; i < effct_feat_num; ++i )
                {
                    if ( !(*corr_int_selected)[i] ) continue;
                    //const PointType &int_p = corr_intvect->points[i];
                    RGBpointBodyToWorld(&laserCloudOri->points[i], &refCloudWorld->points[k]);
                    ++k;
                }
                for ( int i = 0, m = 0; i < feats_down_size; ++i )
                {
                    if ( ! point_selected_int[i] ) continue;
                    const PointVector &points_near = Nearest_Points[i];
                    for ( int j = 0; j < NUM_MATCH_POINTS; ++j)
                        if ( points_near[j].intensity > min_reflectance )
                        {
                            refCorCloudWorld->points[m] = points_near[j];
                            ++m;
                        }
                }
                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*refCloudWorld, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
                laserCloudFullRes3.header.frame_id = "camera_init";
                pubRefCloud.publish(laserCloudFullRes3);

                sensor_msgs::PointCloud2 laserCloudFullRes4;
                pcl::toROSMsg(*refCorCloudWorld, laserCloudFullRes4);
                laserCloudFullRes4.header.stamp = ros::Time().fromSec(lidar_end_time);
                laserCloudFullRes4.header.frame_id = "camera_init";
                pubRefCorCloud.publish(laserCloudFullRes4);

//                for ( int i = 0; i < feats_down_size; ++i )
//                {
//                    if ( ! point_selected_int[i] ) continue;
//                    const PointType & pt_lidar = feats_down_body->points[i];
//                    const V3D p_lidar_(pt_lidar.x, pt_lidar.y, pt_lidar.z);
//                    const V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_lidar_ + state_point.offset_T_L_I) + state_point.pos);

//                    const float crow = (height-1) - project_row<float> ( p_lidar_.z(), p_lidar_.norm() );
//                    const float ccol = project_col<float> ( p_lidar_.y(), p_lidar_.x() );

//                    const PointVector &points_near = Nearest_Points[i];
//                    for ( int j = 0; j < NUM_MATCH_POINTS; ++j)
//                    {
//                        const PointType &pt_p = points_near[j];
//                        const V3D p_world(pt_p.x, pt_p.y, pt_p.z);
//                        const V3D p_lidar = state_point.offset_R_L_I.conjugate() * ((state_point.rot.conjugate() * (p_world - state_point.pos)) - state_point.offset_T_L_I); // world -> lidar

//                        const float row = (height-1) - project_row<float> ( p_lidar.z(), p_lidar.norm() );
//                        const float col = project_col<float> ( p_lidar.y(), p_lidar.x() );
//                        if ( row < 0 || row >= height || col < 0 || col >= width ) continue;

//                        const bool too_far ( std::abs(ccol - col) > 10 || std::abs(crow - row) > 10 );
//                        if ( too_far && (i&1023)==0 )
//                        {
//                            //ROS_ERROR_STREAM("i: " << i << " r: " << row << " " << crow << " c: " << col<< " " <<ccol << " p: " << p_lidar_.transpose() << " p: " << p_lidar.transpose() << " pw: "<< p_global.transpose() << " pw: " << p_world.transpose() );
//                            continue;
//                        }

//                        const Eigen::Vector3f color(255* pt_p.intensity,0,0);
//                        //const Eigen::Vector3f color = 255.f * colorFromNormal(Eigen::Vector3f(int_p.x,int_p.y,int_p.z).normalized());
//                        //ROS_INFO_STREAM("r: " << row << " c: " << col << " pt: " << pt_p.x << " " << pt_p.y << " " << pt_p.z << " c: " << color.transpose() << " i: " << i << " n: " << effct_feat_num  << " c: " << corr_intvect->points.size() << " " << laserCloudOri->points.size());
//                        int cr = max(0,min(height-1,int(round(row))));
//                        int cc = max(0,min(width-1,int(round(col))));
//                        ref_img.at<cv::Vec3b>(cr,cc) = cv::Vec3b(color(2),color(1),color(0));
//                    }
//                }
//                pubRefImg.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", ref_img).toImageMsg());
            }

            if  ( pubRefImg.getNumSubscribers() > 0 )
            {
                constexpr int height = 128;
                constexpr int width = 1024;
                cv::Mat ref_img = cv::Mat::zeros(height, width, CV_8UC3);
                // add
                for ( int i = 0; i < effct_feat_num; ++i )
                {
                    if ( !(*corr_int_selected)[i] ) continue;
                    const PointType &int_p = corr_intvect->points[i];
                    const PointType &pt_p = laserCloudOri->points[i];

                    const float row = (height-1) - project_row<float> ( pt_p.z, sqrt(pt_p.x*pt_p.x+pt_p.y*pt_p.y+pt_p.z*pt_p.z) );
                    const float col = project_col<float> ( pt_p.y, pt_p.x );
                    if ( row < 0 || row >= height || col < 0 || col >= width ) continue;
                    const Eigen::Vector3f color(0, 255* int_p.reflectance, 0);
                    //const Eigen::Vector3f color = 255.f * colorFromNormal(Eigen::Vector3f(int_p.x,int_p.y,int_p.z).normalized());
                    //ROS_INFO_STREAM("r: " << row << " c: " << col << " pt: " << pt_p.x << " " << pt_p.y << " " << pt_p.z << " c: " << color.transpose() << " i: " << i << " n: " << effct_feat_num  << " c: " << corr_intvect->points.size() << " " << laserCloudOri->points.size());
                    int cr = max(0,min(height-1,int(round(row))));
                    int cc = max(0,min(width-1,int(round(col))));
                    ref_img.at<cv::Vec3b>(cr,cc) = cv::Vec3b(color(2),color(1),color(0));
                }

                for ( int i = 0; i < feats_down_size; ++i )
                {
                    if ( ! point_selected_int[i] ) continue;
                    const PointType & pt_lidar = feats_down_body->points[i];
                    const V3D p_lidar_(pt_lidar.x, pt_lidar.y, pt_lidar.z);
                    const V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_lidar_ + state_point.offset_T_L_I) + state_point.pos);

                    const float crow = (height-1) - project_row<float> ( p_lidar_.z(), p_lidar_.norm() );
                    const float ccol = project_col<float> ( p_lidar_.y(), p_lidar_.x() );

                    const PointVector &points_near = Nearest_Points[i];
                    for ( int j = 0; j < NUM_MATCH_POINTS; ++j)
                    {
                        const PointType &pt_p = points_near[j];
                        const V3D p_world(pt_p.x, pt_p.y, pt_p.z);
                        const V3D p_lidar = state_point.offset_R_L_I.conjugate() * ((state_point.rot.conjugate() * (p_world - state_point.pos)) - state_point.offset_T_L_I); // world -> lidar

                        const float row = (height-1) - project_row<float> ( p_lidar.z(), p_lidar.norm() );
                        const float col = project_col<float> ( p_lidar.y(), p_lidar.x() );
                        if ( row < 0 || row >= height || col < 0 || col >= width ) continue;

                        const bool too_far ( std::abs(ccol - col) > 10 || std::abs(crow - row) > 10 );
                        if ( too_far && (i&1023)==0 )
                        {
                            //ROS_ERROR_STREAM("i: " << i << " r: " << row << " " << crow << " c: " << col<< " " <<ccol << " p: " << p_lidar_.transpose() << " p: " << p_lidar.transpose() << " pw: "<< p_global.transpose() << " pw: " << p_world.transpose() );
                            continue;
                        }

                        const Eigen::Vector3f color(255* pt_p.intensity,0,0);
                        //const Eigen::Vector3f color = 255.f * colorFromNormal(Eigen::Vector3f(int_p.x,int_p.y,int_p.z).normalized());
                        //ROS_INFO_STREAM("r: " << row << " c: " << col << " pt: " << pt_p.x << " " << pt_p.y << " " << pt_p.z << " c: " << color.transpose() << " i: " << i << " n: " << effct_feat_num  << " c: " << corr_intvect->points.size() << " " << laserCloudOri->points.size());
                        int cr = max(0,min(height-1,int(round(row))));
                        int cc = max(0,min(width-1,int(round(col))));
                        ref_img.at<cv::Vec3b>(cr,cc) = cv::Vec3b(color(2),color(1),color(0));
                    }
                }
                pubRefImg.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", ref_img).toImageMsg());
            }

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " [" << ext_euler.transpose() << "/ " << state_point.offset_R_L_I.coeffs().transpose() << "; "<<state_point.offset_T_L_I.transpose()<<"] "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories  */
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log)
    {
        fout_out.close();
        fout_pre.close();
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }
    if ( posesFile ) posesFile->close();

    return 0;
}
