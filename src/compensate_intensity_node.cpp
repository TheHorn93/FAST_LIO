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
#include "reflectance_grad.h"
#include "input_pcl_filter.h"

using PointCloudType = pcl::PointCloud<ouster_ros::Point>;

std::unique_ptr<PCLFilterModelBase<ouster_ros::Point>> input_filter;
ros::Publisher pub_out;

bool ptIsValid( const ouster_ros::Point& pt )
{
    return (std::abs(pt.x) > 0.0 || std::abs(pt.y) > 0.0 || std::abs(pt.z) > 0.0);
}

void getValidPointsIdxs( const PointCloudType& pc_in, std::vector<uint32_t>& idxs )
{
    for( size_t pt_it=0; pt_it < pc_in.points.size(); ++pt_it )
    {
        if( ptIsValid(pc_in.points[pt_it]) )
            idxs.push_back( pt_it );
    }
}

void transferPoints( PointCloudType& pc_in, const std::vector<float>& ints, PointCloudType& pc_out )
{
    std::vector<uint32_t> idxs;
    getValidPointsIdxs( pc_in, idxs );
    pc_out.points.reserve( idxs.size() );
    for( size_t pt_it=0; pt_it < idxs.size(); ++pt_it )
    {
        float cur_int = ints[idxs[pt_it]];
        if( cur_int > 0.f )
        {
            ouster_ros::Point& cur_pt( pc_in.points[idxs[pt_it]] );
            cur_pt.intensity = cur_int;
            cur_pt.reflectivity = static_cast<uint16_t>(cur_int*65535);
            pc_out.points.push_back( cur_pt );
            //pc_out.points[pt_it].intensity = cur_int;
            //pc_out.points[pt_it].reflectivity = static_cast<uint16_t>(cur_int*65535);
            //std::cout << pc_out.points[pt_it].x << ", " << pc_out.points[pt_it].y << ", " << pc_out.points[pt_it].z << ", " << pc_out.points[pt_it].intensity << ", " << pc_out.points[pt_it].reflectivity << std::endl;
        }
        //std::cout << "  " <<  pc_out.points[pt_it].x << ", " << pc_out.points[pt_it].y << ", " << pc_out.points[pt_it].z << ", " << pc_out.points[pt_it].intensity << std::endl;
    }
    ROS_INFO_STREAM( "VALID Ints: " << pc_out.points.size() );
}

void pcCallback( const sensor_msgs::PointCloud2::ConstPtr &msg )
{
    pcl::PointCloud<ouster_ros::Point> pc_in;
    pcl::fromROSMsg(*msg, pc_in);

    std::vector<float> new_ints;
    input_filter->applyFilter( pc_in, new_ints );
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
    //        std::cout << pc_out.points[pt_it].x << ", " << pc_out.points[pt_it].y << ", " << pc_out.points[pt_it].z << ", " << pc_out.points[pt_it].intensity << ", " << pc_out.points[pt_it].reflectivity << std::endl;
    //}
    pub_out.publish( msg_out );
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
    if( use_channel == PCLFILTER_INTENSITY )
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
    nh.param<int >("w_filter_size", input_filter->getParams().w_filter_size, 4);
    nh.param<int>("h_filter_size", input_filter->getParams().h_filter_size, 4);
    nh.param<double>("point_filter/max_var_mult", input_filter->getParams().max_var_mult, 1.0);

    ROS_WARN_STREAM( "Filter: w=" << input_filter->getParams().w_filter_size << ", h=" << input_filter->getParams().h_filter_size );

    pub_out = nh.advertise<sensor_msgs::PointCloud2>("/cloud_compensated", 1000);
    ros::Subscriber pc_sub = nh.subscribe( lidar_topic, 2, pcCallback );
    
    ros::spin();

    return 0;
}