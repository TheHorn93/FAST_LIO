#include "preprocess.h"
#include "os_shift.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
    : lidar_type(AVIA), blind(0.01), point_filter_num(1), pass_through ( false )
{
    N_SCANS   = 6;
    SCAN_RATE = 10;
    given_offset_time = false;
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool pass_through_, int lid_type, double bld, int pfilt_num)
{
    pass_through = pass_through_;
    lidar_type = lid_type;
    blind = bld;
    point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out, const float & threshold)
{
    min_intensity = threshold;
    avia_handler(msg);
    *pcl_out = pl_surf;
}

template <typename Cloud>
void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, typename Cloud::Ptr &pcl_out, const float & threshold)
{
    switch (time_unit)
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

    min_intensity = threshold;

    switch (lidar_type)
    {
    case OUST64:
        oust64_handler( msg);
        break;

    case HESAI32:
        hesai32_handler( msg);
        break;

    case VELO16:
        velodyne_handler(msg);
        break;

    default:
        printf("Error LiDAR Type");
        break;
    }
    if constexpr ( std::is_same_v<Cloud,PointCloudOuster> )
        *pcl_out = pl_raw_os;
    else
    {
        if constexpr ( std::is_same_v<Cloud,PointCloudHesai> )
            *pcl_out = pl_raw_hs;
        else
            *pcl_out = pl_surf;
    }
}

template void Preprocess::process<PointCloudOuster>(const sensor_msgs::PointCloud2::ConstPtr &msg, typename PointCloudOuster::Ptr &pcl_out, const float & threshold);
template void Preprocess::process<PointCloudHesai>(const sensor_msgs::PointCloud2::ConstPtr &msg, typename PointCloudHesai::Ptr &pcl_out, const float & threshold);
template void Preprocess::process<PointCloudXYZI>(const sensor_msgs::PointCloud2::ConstPtr &msg, typename PointCloudXYZI::Ptr &pcl_out, const float & threshold);

void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    max_curvature = 0;
    pl_surf.clear();
    pl_full.clear();
    double t1 = omp_get_wtime();
    const int plsize = msg->point_num;

    pl_surf.reserve(plsize);
    pl_full.resize(plsize);

    uint valid_num = 0;
    if ( pass_through )
    {
        pl_surf.resize(plsize);
        for (int i = 0; i < plsize; ++i)
        {
            if(!((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)))
            {
                continue;
            }
            PointType & added_pt = pl_surf[i];
            added_pt.x = msg->points[i].x;
            added_pt.y = msg->points[i].y;
            added_pt.z = msg->points[i].z;
            added_pt.intensity = msg->points[i].reflectivity;
            added_pt.reflectance = msg->points[i].reflectivity; // intensity
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms
            if ( max_curvature < added_pt.curvature ) max_curvature = added_pt.curvature;
        }
    }
    else
    {
        const float blind2 = blind * blind;
        for(uint i=1; i<plsize; i++)
        {
            if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {
                valid_num ++;
                if (valid_num % point_filter_num == 0)
                {
                    pl_full[i].x = msg->points[i].x;
                    pl_full[i].y = msg->points[i].y;
                    pl_full[i].z = msg->points[i].z;
                    pl_full[i].intensity = msg->points[i].reflectivity;
                    pl_full[i].reflectance = msg->points[i].reflectivity;
                    pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms
                    pl_full[i].normal_x = 0;
                    pl_full[i].normal_y = 0;
                    pl_full[i].normal_z = 0;
                    if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
                            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
                            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)
                            && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > blind2 ))
                    {
                        if ( max_curvature < pl_full[i].curvature ) max_curvature = pl_full[i].curvature;
                        pl_surf.push_back(pl_full[i]);
                    }
                }
            }
        }
    }
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

//#ifndef COMP_ONLY
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
//#endif
void Preprocess::oust64_handler( const sensor_msgs::PointCloud2::ConstPtr &msg )
{
    //static const double max_ref = std::pow( 2, 16 )-1;
    max_curvature = 0;
    pl_raw_os.clear();
    pl_surf.clear();
    pl_full.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    pl_raw_os = pl_orig;

    const int plsize = pl_orig.size();
    const float blind2 = (blind * blind);
    const float blind_null_2 = 4*4;
    pl_surf.reserve(plsize);

    if ( pass_through )
    {
        pl_surf.resize(plsize);
        for (int i = 0; i < plsize; ++i)
        {
            PointType & added_pt = pl_surf[i];
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.reflectance = pl_orig.points[i].reflectivity; // intensity
            added_pt.gloss = 0;
            added_pt.intensity_count = 1;
            added_pt.intensity_variance = 0;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms
            if ( max_curvature < added_pt.curvature ) max_curvature = added_pt.curvature;
        }
    }
    else
    {
        float max_int = -1, max_refl = -1;
        constexpr int height = 128;
        constexpr int width = 1024;
        constexpr bool printcomp = false, print_info = false;
        cv::Mat img, imgc, imgi;
        if constexpr ( printcomp )
        {
            img = cv::Mat::zeros(height, width, CV_8UC1);
            imgc = cv::Mat::zeros(height, width, CV_8UC1);
            imgi = cv::Mat::zeros(height, width, CV_8UC1);
        }
        pl_full.resize(plsize);
        int offset = 0, row = 0, col = 0, last_i = 0;
        for (int i = 0; i < plsize; ++i, ++col)
        {
            if ( row >= height ) { row = 0; offset = row * width; }
            if ( col >= width ) { col = 0; ++row; offset = row * width; }
            const int comp_col = os_shift::inverseCompensateColPos1024( row, col );
            const int comp_idx = offset + comp_col;

            PointType & added_pt = pl_full[comp_idx];
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.reflectance = pl_orig.points[i].reflectivity; // intensity
            added_pt.gloss = 0;
            added_pt.intensity_count = 1;
            added_pt.intensity_variance = 0;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms
            if ( added_pt.curvature > max_curvature ) max_curvature = added_pt.curvature;

            if constexpr ( printcomp )
            {
                const float draw_range = std::min<float>(255.f,25.f* std::sqrt(added_pt.x * added_pt.x +added_pt.y * added_pt.y + added_pt.z * added_pt.z));
                img.at<uchar>(row,col) = uchar(draw_range);
                imgc.at<uchar>(row,comp_col) = uchar(draw_range);

                const float draw_int = std::min<float>(255.f, 255.f * added_pt.intensity);
                imgi.at<uchar>(row,comp_col) = uchar(draw_int);

                if ( (i & 511) == 0 )
                {
                    ROS_INFO_STREAM("c: " << col << " r: " << row << " cc: " << comp_col << " o: "<< offset << " ci: " << comp_idx << " d: " << draw_range );
                }
            }

            if (i % point_filter_num != 0) continue;
            if ( !isValidPoint( pl_orig.points[i] ) ) continue;

            const float range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
            if (range < blind2) continue;

            if ( use_compensated )
            {
                if (range < blind_null_2 && added_pt.intensity < min_intensity ) continue;
            }
            else
            {
                if (range < blind_null_2 && added_pt.reflectance < min_intensity ) continue;
            }

            if ( added_pt.reflectance > max_refl ) max_refl = added_pt.reflectance;
            if ( added_pt.intensity > max_int ) max_int = added_pt.intensity;

            last_i = i;
            pl_surf.points.emplace_back(added_pt);
        }
        if constexpr ( print_info )
        {
            ROS_INFO_STREAM("max_curv: " << max_curvature << " last: " << pl_surf.points.back().curvature << " ( " << last_i << " ) ti: " << pl_orig.points[last_i].t << " te: " << pl_orig.points.back().t);
            ROS_INFO_STREAM("maxRefl: " << max_refl << " " << max_int << " th: " << min_intensity);
        }

        if constexpr ( printcomp )
        {
            cv::imwrite("./range.png",img);
            cv::imwrite("./rangec.png",imgc);
            cv::imwrite("./rangei.png",imgi);

            cv::Mat imgd = cv::Mat::zeros(height, width, CV_8UC1);
            int offset = 0, row = 0, col = 0;
            for (int i = 0; i < plsize; ++i, ++col)
            {
                if ( row >= height ) { row = 0; offset = row * width; }
                if ( col >= width ) { col = 0; ++row; offset = row * width; }
                const PointType & added_pt = pl_full[offset + col];
                const float range = std::sqrt(added_pt.x * added_pt.x +added_pt.y * added_pt.y + added_pt.z * added_pt.z);
                const float draw_range = std::min<float>(255.f,25.f* range);


                if ( (i & 511) == 0 && range > 0.1 )
                {
                    const float prow = (height-1) - project_row<float> ( added_pt.z, range );
                    const float pcol = project_col<float> ( added_pt.y, added_pt.x );
                    ROS_INFO_STREAM("r: " << row << " " << prow << " c: " << col << " " << pcol );
                }

                imgd.at<uchar>(row,col) = uchar(draw_range);
            }
            cv::imwrite("./ranged.png",imgd);
            // pl_full is now correctly ordered!
        }

    }
    // pub_func(pl_surf, pub_full, msg->header.stamp);
    // pub_func(pl_surf, pub_corn, msg->header.stamp);
    //ROS_INFO_STREAM( "Got " << pl_surf.points.size()  << " Points. max: " << maxVal << " " << oMaxVal);
}

void Preprocess::hesai32_handler( const sensor_msgs::PointCloud2::ConstPtr &msg )
{
    max_curvature = 0;
    pl_raw_hs.clear();
    pl_surf.clear();
    pl_full.clear();
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    double start_time = msg->header.stamp.toSec();

    float maxVal = 0, oMaxVal = 0, ring = -1;
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
        if ( maxVal < pl_orig.points[i].intensity ) maxVal = pl_orig.points[i].intensity;
        if ( ring < pl_orig.points[i].ring ) ring = pl_orig.points[i].ring;
    }
    pl_raw_hs = pl_orig;

    const int plsize = pl_orig.size();
    pl_surf.reserve(plsize);

    if ( pass_through )
    {
        pl_surf.resize(plsize);
        for (int i = 0; i < plsize; ++i)
        {
            PointType & added_pt = pl_surf[i];
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.reflectance = 0;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.curvature = (pl_orig.points[i].timestamp-start_time) * time_unit_scale; // curvature unit: ms
            if ( max_curvature < added_pt.curvature ) max_curvature = added_pt.curvature;
        }
    }
    else
    {
        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            if (i % point_filter_num != 0) continue;
            if ( !isValidPoint( pl_orig.points[i] ) ) continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

            if (range < (blind * blind)) continue;

            PointType added_pt;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.reflectance = 0;

            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.curvature = (pl_orig.points[i].timestamp-start_time) * time_unit_scale; // curvature unit: ms
            if ( max_curvature < added_pt.curvature ) max_curvature = added_pt.curvature;

            if ( oMaxVal < added_pt.intensity ) oMaxVal = added_pt.intensity;
            pl_surf.points.push_back(added_pt);
        }
        //ROS_INFO_STREAM("maxRefl: " << maxRefl );
    }
    // pub_func(pl_surf, pub_full, msg->header.stamp);
    // pub_func(pl_surf, pub_corn, msg->header.stamp);
    ROS_INFO_STREAM( "Got " << pl_surf.points.size()  << " Points. max: " << maxVal << " " << oMaxVal << " n: " << ring);
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    //pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)
    {
        given_offset_time = true;
    }
    else
    {
        given_offset_time = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end  = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--)
        {
            if (pl_orig.points[i].ring == layer_first)
            {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    {
        for (int i = 0; i < plsize; i++)
        {
            PointType added_pt;
            // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;

            if (!given_offset_time)
            {
                int layer = pl_orig.points[i].ring;
                double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

                if (is_first[layer])
                {
                    // printf("layer: %d; is first: %d", layer, is_first[layer]);
                    yaw_fp[layer]=yaw_angle;
                    is_first[layer]=false;
                    added_pt.curvature = 0.0;
                    yaw_last[layer]=yaw_angle;
                    time_last[layer]=added_pt.curvature;
                    continue;
                }

                // compute offset time
                if (yaw_angle <= yaw_fp[layer])
                {
                    added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
                }
                else
                {
                    added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
                }

                if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

                yaw_last[layer] = yaw_angle;
                time_last[layer]=added_pt.curvature;
            }

            if (i % point_filter_num == 0)
            {
                if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
                {
                    if ( max_curvature < added_pt.curvature ) max_curvature = added_pt.curvature;
                    pl_surf.points.push_back(added_pt);
                }
            }
        }
    }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}
