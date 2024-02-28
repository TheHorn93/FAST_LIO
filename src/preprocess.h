#pragma once
#include "input_pcl_filter.h"
using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef pcl::PointCloud<ouster_ros::Point> PointCloudOuster;
typedef pcl::PointCloud<hesai_ros::Point> PointCloudHesai;

enum LID_TYPE{AVIA = 1, VELO16, OUST64, HESAI32}; //{1, 2, 3, 4}
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};

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

  bool use_ambient;
};
