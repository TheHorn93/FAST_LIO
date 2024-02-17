#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
//#include "reflectance_grad.h"
#include "input_pcl_filter.h"

using PointCloudType = pcl::PointCloud<ouster_ros::Point>;
using PointCloudNormalType = pcl::PointCloud<pcl::PointNormal>;

std::unique_ptr<PCLFilterModelBase<ouster_ros::Point>> input_filter;
ros::Publisher pub_out, pub_normals;

inline
bool ptIsValid( const ouster_ros::Point& pt )
{
    return (std::abs(pt.x) > 0.0f) || (std::abs(pt.y) > 0.0f) || (std::abs(pt.z) > 0.0f);
}

//void getValidPointsIdxs( const PointCloudType& pc_in, std::vector<uint32_t>& idxs )
//{
//    idxs.reserve(pc_in.points.size());
//    // TODO: possible parallel
//    for( size_t pt_it=0; pt_it < pc_in.points.size(); ++pt_it )
//    {
//        if( !ptIsValid(pc_in.points[pt_it]) ) continue;
//        idxs.push_back( pt_it );
//    }
//}

ouster_ros::Point getInvalidPoint ( )
{
    ouster_ros::Point invalid_point;
    invalid_point.getVector4fMap().setZero();
    invalid_point.intensity = 0;
    invalid_point.t = 0;
    invalid_point.reflectivity = 0;
    invalid_point.ring = 0;
    //invalid_point.noise = 0;
    invalid_point.ambient = 0;
    invalid_point.range = 0;
    return invalid_point;
}

void transferPoints( PointCloudType& pc_in, const std::vector<float>& ints, PointCloudType& pc_out )
{
    //std::vector<uint32_t> idxs;
    //getValidPointsIdxs( pc_in, idxs );
    const size_t num_pts = pc_in.points.size();
    size_t num_valid = 0;
    pc_out.points.resize( num_pts, getInvalidPoint() );
    float maxVal = 0, maxRefl = 0;
    // TODO possible parallel
    const ouster_ros::Point inv_pt = getInvalidPoint();
    for( size_t pt_it=0; pt_it < num_pts; ++pt_it )
    {
        float cur_int = ints[pt_it];
        if( cur_int <= 1e-6f ) continue; // invalid ones are set to 0 before hand
        //if( !ptIsValid(pc_in.points[pt_it]) ) continue; // unnecessary, because checked multiple times before...
        if ( cur_int > 1.f ) cur_int = 1.f;
        ouster_ros::Point& cur_pt = pc_out.points[pt_it];
        cur_pt = pc_in.points[pt_it];
        cur_pt.intensity = cur_int;
        ++num_valid;
        //cur_pt.reflectivity = static_cast<uint16_t>(cur_int*65535);
        //pc_out.points.emplace_back( cur_pt );
        //pc_out.points[pt_it].intensity = cur_int;
        //pc_out.points[pt_it].reflectivity = static_cast<uint16_t>(cur_int*65535);
        if ( maxVal < cur_int ) maxVal = cur_int;
        if ( maxRefl < cur_pt.reflectivity ) maxRefl = cur_pt.reflectivity;
        //std::cout << pc_out.points.back().x << ", " << pc_out.points.back().y << ", " << pc_out.points.back().z << ", " << pc_out.points.back().intensity << ", " << pc_out.points.back().reflectivity << std::endl;
        //std::cout << "  " <<  pc_out.points[pt_it].x << ", " << pc_out.points[pt_it].y << ", " << pc_out.points[pt_it].z << ", " << pc_out.points[pt_it].intensity << std::endl;
    }
    ROS_INFO_STREAM_THROTTLE(1,"VALID Ints: " << num_valid << " of " << pc_out.points.size() << " max: " << maxVal << " ( " << maxRefl << " ) of: " << num_pts );
}

void publishNormals(const pcl::PointCloud<ouster_ros::Point> & pc_in, const Eigen::Matrix3Xf & normals, const std_msgs::Header & header )
{
    const size_t num_pts = pc_in.points.size();
    PointCloudNormalType pc_out;
    pc_out.points.reserve( num_pts );
    pcl::PointNormal pt_n;
    for( size_t pt_it=0; pt_it < num_pts; ++pt_it )
    {
        const ouster_ros::Point& cur_pt = pc_in.points[pt_it];
        pt_n.getVector3fMap() = cur_pt.getVector3fMap();
        pt_n.getNormalVector3fMap() = normals.col(pt_it);
        pc_out.points.emplace_back( pt_n );
    }
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg( pc_out, msg_out );
    msg_out.header = header;
    pub_normals.publish(msg_out);
}

// TODO: move all the processing out of the callback itself
void pcCallback( const sensor_msgs::PointCloud2::ConstPtr &msg )
{
    pcl::PointCloud<ouster_ros::Point> pc_in;
    pcl::fromROSMsg(*msg, pc_in);

    const bool should_publish_normals = pub_normals.getNumSubscribers() > 0;
    Eigen::Matrix3Xf normals;
    Eigen::Matrix3Xf * normals_ptr = should_publish_normals ? & normals : nullptr;
    std::vector<float> new_ints;
    input_filter->applyFilter( pc_in, new_ints, normals_ptr );
    PointCloudType pc_out;
    transferPoints( pc_in, new_ints, pc_out );
    //for( size_t pt_it=0; pt_it < pc_out.size(); ++pt_it )
    //{
    //    if(pt_it%1000 == 0)
    //        std::cout << pc_out.points[pt_it].x << ", " << pc_out.points[pt_it].y << ", " << pc_out.points[pt_it].z << ", " << pc_out.points[pt_it].intensity << ", " << pc_out.points[pt_it].reflectivity << std::endl;
    //}

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg( pc_out, msg_out );
    msg_out.header = msg->header;
    //for( size_t pt_it=0; pt_it < pc_out.size(); ++pt_it )
    //{
    //    if(pt_it%1000 == 0)
    //        std::cout <<  pc_out.points[pt_it].intensity << ", ";
    //}
    pub_out.publish( msg_out );
    if ( should_publish_normals )
        publishNormals( pc_in, normals, msg->header );
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "compensate_intensity");
    ros::NodeHandle nh;

    int use_channel;
    std::string comp_type, comp_params, lidar_topic;
    nh.param<std::string>("common/lid_topic", lidar_topic, "/livox/lidar");
    nh.param<std::string>("comp_model", comp_type, "None");
    nh.param<std::string>("comp_params", comp_params, "{}");
    nh.param<int>("filter_use_channel", use_channel, 0);
    std::cout << "CHANNEL" << use_channel << std::endl;
    if ( use_channel == PCLFILTER_INTENSITY )
    {
        ROS_WARN_STREAM( "Using intensity channel" );
        input_filter = std::make_unique<PCLFilter<ouster_ros::Point, PCLFILTER_INTENSITY>>();
    }
    else if( use_channel == PCLFILTER_REFLECTIVITY )
    {
        ROS_WARN_STREAM( "Using reflectivity channel" );
        input_filter = std::make_unique<PCLFilter<ouster_ros::Point, PCLFILTER_REFLECTIVITY>>();
    }
    else if( use_channel == PCLFILTER_AMBIENCE )
    {
        ROS_WARN_STREAM( "Using ambience channel" );
        input_filter = std::make_unique<PCLFilter<ouster_ros::Point, PCLFILTER_AMBIENCE>>();
    }
    else
    {
        ROS_ERROR_STREAM( "Invalid intensity channel: " << use_channel );
        return -1;
    }
    input_filter->initCompensationModel( comp_type, comp_params );
    nh.param<int>("point_filter/width", input_filter->getParams().width, 1024);
    nh.param<int>("point_filter/height", input_filter->getParams().height, 128);
    nh.param<bool>("requires_os_shift",input_filter->getParams().requires_os_shift, false);
    nh.param<int >("w_filter_size", input_filter->getParams().w_filter_size, 4);
    nh.param<int>("h_filter_size", input_filter->getParams().h_filter_size, 4);
    nh.param<double>("point_filter/max_var_mult", input_filter->getParams().max_var_mult, 1.0);

    ROS_WARN_STREAM( "Filter: w=" << input_filter->getParams().w_filter_size << ", h=" << input_filter->getParams().h_filter_size );

    pub_out = nh.advertise<sensor_msgs::PointCloud2>("/cloud_compensated", 1000);
    pub_normals = nh.advertise<sensor_msgs::PointCloud2>("/cloud_normals", 1000);
    ros::Subscriber pc_sub = nh.subscribe( lidar_topic, 2, pcCallback );
    
    ros::spin();

    return 0;
}
