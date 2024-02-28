#pragma once
#include "input_pcl_filter.h"
using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

//typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef pcl::PointCloud<ouster_ros::Point> PointCloudOuster;
typedef pcl::PointCloud<hesai_ros::Point> PointCloudHesai;

enum LID_TYPE{AVIA = 1, VELO16, OUST64, HESAI32}; //{1, 2, 3, 4}
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
//enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
//enum Surround{Prev, Next};
//enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

//struct orgtype
//{
//  double range;
//  double dista;
//  double angle[2];
//  double intersect;
//  E_jump edj[2];
//  Feature ftype;
//  orgtype()
//  {
//    range = 0;
//    edj[Prev] = Nr_nor;
//    edj[Next] = Nr_nor;
//    ftype = Nor;
//    intersect = 2;
//  }
//};

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    //(std::uint16_t, noise, noise)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

class Preprocess
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  template <typename Cloud>
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, typename Cloud::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);
  void setUseAmbient( bool ambient );

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudOuster pl_raw_os;
  PointCloudHesai pl_raw_hs;
  PointCloudXYZI pl_full, pl_surf;
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;
  bool given_offset_time, pass_through;
  ros::Publisher pub_full, pub_surf;

  private:
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void hesai32_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);

//  int group_size;
//  double disA, disB, inf_bound;
//  double limit_maxmid, limit_midmin, limit_maxmin;
//  double p2l_ratio;
//  double jump_up_limit, jump_down_limit;
//  double cos160;
//  double edgea, edgeb;
//  double smallp_intersect, smallp_ratio;
//  double vx, vy, vz;
  bool use_ambient;
};
