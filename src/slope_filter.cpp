#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include "livox_ros_driver2/CustomMsg.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
//Include for compute normals
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>

//parameters given by the launch file
double first_RADIUS, second_RADIUS, max_height,start_height;

double slope_1,slp_first_RADIUS, height_1;
double slope_2, slp_second_RADIUS, height_2;
double slope_3,slp_third_RADIUS, height_3;

//法向量那一路的前置处理参数，要调就提成 launch 参数
const double VOXEL_LEAF_SIZE = 0.05; //体素下采样边长 (m)
const int    SOR_MEAN_K      = 20;   //统计滤波近邻数
const double SOR_STDDEV_MUL  = 1.0;  //统计滤波标准差倍数

bool get_msg_left = 0;
bool get_msg_right = 0;
std::string base_frame;
std::string laser_frame;
std::string scan_topic_left;
std::string scan_topic_right;
std::string new_scan_topic;
std::string filted_topic_3d;

livox_ros_driver2::CustomMsg scan_copy_left;//receiving the message from the livox
livox_ros_driver2::CustomMsg scan_copy_right;

//callback function for the livox
void scanCallback_left(const livox_ros_driver2::CustomMsg &scan)
{
    scan_copy_left = scan;
    get_msg_left = 1;
}
void scanCallback_right(const livox_ros_driver2::CustomMsg &scan)
{
    scan_copy_right = scan;
    get_msg_right = 1;
}

//坡度滤波：按离车体中心的距离分成四段，每段给一条高度上限直线，超过上限的点判为坡面/地面
bool ispoint (double nx, double ny, double z, double nI)
{
    (void)nI; //暂时不用反射强度
    // nx += 0.011;  偏心量？ 暂时弃用
    // ny -= -0.19495+0.02329;

    double r2 = nx*nx + ny*ny;
    if (r2 <= first_RADIUS*first_RADIUS){
        return false; //车体自身附近直接丢
    }
    if (r2 <= second_RADIUS*second_RADIUS){
        return z<= start_height;
    }
    if(r2 <=slp_first_RADIUS * slp_first_RADIUS){
        double dis=sqrt(r2)-second_RADIUS;
        return z<=std::min(max_height,dis*slope_1+start_height);//max_height for security
    }

    if(r2 <=slp_second_RADIUS * slp_second_RADIUS){
        double dis=sqrt(r2)-slp_first_RADIUS;
        return z<=std::min(height_1,dis*slope_2+max_height);
    }

    double dis=sqrt(r2)-slp_second_RADIUS;
    return z<=std::min(height_2,dis*slope_3+height_1);
    //add judgement for the indensity
}

//把一帧 CustomMsg 做安装角补偿后拆成两路：origin 收全部有效点，filtered 只收通过坡度滤波的点
void accumulate_scan(const livox_ros_driver2::CustomMsg &scan,
                     pcl::PointCloud<pcl::PointXYZI> &origin,
                     pcl::PointCloud<pcl::PointXYZI> &filtered)
{
    for (const auto &pt : scan.points)
    {
        double x = pt.x - 0.011;
        double y = pt.y + 0.02329;
        double z = pt.z - 0.04412;
        double intensity = pt.reflectivity;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(intensity))
        {
            continue;
        }
        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = intensity;
        origin.points.push_back(point);

        //注意这里传的是 -z，和存进点云的 z 不是一个值
        if (ispoint(x, y, -z, intensity))
        {
            filtered.points.push_back(point);
        }
    }
}

//体素下采样
void down_sampling(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr &output)
{
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
    voxel.filter(*output);
}

//统计离群点移除
void statistical_removal(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr &output)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(input);
    sor.setMeanK(SOR_MEAN_K);
    sor.setStddevMulThresh(SOR_STDDEV_MUL);
    sor.filter(*output);
}


int main (int argc, char **argv)
{
    std::string node_name = "threeD_lidar_filter_pointcloud";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    if (!nh.getParam("/" + node_name + "/base_frame", base_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'base_frame'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/laser_frame", laser_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'laser_frame'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/scan_topic_left", scan_topic_left))
    {
        ROS_ERROR("Failed to retrieve parameter 'scan_topic_left'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/scan_topic_right", scan_topic_right))
    {
        ROS_ERROR("Failed to retrieve parameter 'scan_topic_right'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/new_scan_topic", new_scan_topic))
    {
        ROS_ERROR("Failed to retrieve parameter 'new_scan_topic'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/first_RADIUS", first_RADIUS))
    {
        ROS_ERROR("Failed to retrieve parameter 'first_RADIUS'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/second_RADIUS", second_RADIUS))
    {
        ROS_ERROR("Failed to retrieve parameter 'second_RADIUS'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/max_height", max_height))
    {
        ROS_ERROR("Failed to retrieve parameter 'max_height'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/start_height", start_height))
    {
        ROS_ERROR("Failed to retrieve parameter 'start_height'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/slope_1", slope_1))
    {
        ROS_ERROR("Failed to retrieve parameter 'slope_1'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/slope_2", slope_2))
    {
        ROS_ERROR("Failed to retrieve parameter 'slope_2'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/slope_3", slope_3))
    {
        ROS_ERROR("Failed to retrieve parameter 'slope_3'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/slp_first_RADIUS", slp_first_RADIUS))
    {
        ROS_ERROR("Failed to retrieve parameter 'slp_first_RADIUS'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/slp_second_RADIUS", slp_second_RADIUS))
    {
        ROS_ERROR("Failed to retrieve parameter 'slp_second_RADIUS'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/slp_third_RADIUS", slp_third_RADIUS))
    {
        ROS_ERROR("Failed to retrieve parameter 'slp_third_RADIUS'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/filted_topic_3d", filted_topic_3d))
    {
        ROS_ERROR("Failed to retrieve parameter 'filted_topic_3d'");
        return -1;
    }

    ros::Subscriber sub_left = nh.subscribe(scan_topic_left, 1, scanCallback_left);
    ros::Subscriber sub_right = nh.subscribe(scan_topic_right, 1, scanCallback_right);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>(new_scan_topic, 1);//original pointcloud
    ros::Publisher pub4 = nh.advertise<sensor_msgs::PointCloud2>(filted_topic_3d, 1);//filtered pointcloud
    ros::Rate rate(50.0);

    while (ros::ok())
    {
        if (!get_msg_left || !get_msg_right)
        {ros::spinOnce();
            // ROS_INFO("waiting for the message");
            continue;
        }

        auto scan_record_left = scan_copy_left;
        auto scan_record_right = scan_copy_right;

        pcl::PointCloud<pcl::PointXYZI> origin_pcl_cloud;//只做安装角补偿，给调试看
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;//过了坡度滤波的点
        origin_pcl_cloud.points.reserve(scan_record_left.points.size() + scan_record_right.points.size());
        pcl_cloud.points.reserve(scan_record_left.points.size() + scan_record_right.points.size());

        accumulate_scan(scan_record_left, origin_pcl_cloud, pcl_cloud);
        accumulate_scan(scan_record_right, origin_pcl_cloud, pcl_cloud);

        // print the sizes of the two clouds
        // ROS_INFO("Origin points: %zu, after slope filter: %zu", origin_pcl_cloud.points.size(), pcl_cloud.points.size());

        ros::Time start_time = ros::Time::now();//TimeTest Start

        //Down Sampling + Statistical Removal
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_cloud));
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr denoised_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        down_sampling(pcl_cloud_ptr, temp_cloud);
        statistical_removal(temp_cloud, denoised_cloud);
        // ROS_INFO("After down sampling + SOR: %zu", denoised_cloud->points.size());

        //Find Normals
        pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        n.setInputCloud(denoised_cloud);
        n.setSearchMethod(tree);
        n.setKSearch(20);
        n.setNumberOfThreads(8);
        n.compute(*normals);

        //Filter the points whose normal is (nearly) horizontal, i.e. the angle to the z axis is near 90 deg
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
        filtered_cloud_xyzi->points.reserve(denoised_cloud->points.size());
        for (size_t i = 0; i < denoised_cloud->points.size(); ++i) {
            const auto& normal = normals->points[i];
            // 计算法向量与 z 轴的夹角
            float norm = std::sqrt(normal.normal_x * normal.normal_x + normal.normal_y * normal.normal_y + normal.normal_z * normal.normal_z);
            if (norm <= 0.0f) {
                continue;//法向量没算出来的点直接丢
            }
            float angle = std::acos(normal.normal_z / norm);
            
            if (d <= M_PI/11.5 || d >= M_PI/10) { //滤除17度左右的坡面
                filtered_cloud_xyzi->points.push_back(denoised_cloud->points[i]);
            }
        }
        // ROS_INFO("After normal filter: %zu", filtered_cloud_xyzi->points.size());

        ros::Time end_time = ros::Time::now();//TimeTest End

        ros::Duration filter_duration = end_time - start_time;
        // ROS_INFO("Filtering took %f seconds", filter_duration.toSec());

        // Publish original cloud
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(origin_pcl_cloud, output);
        output.header.frame_id = laser_frame; // replace with your frame id
        output.header.stamp = ros::Time::now();
        pub2.publish(output);

        // Publish filtered cloud
        sensor_msgs::PointCloud2 output_filtered;
        pcl::toROSMsg(*filtered_cloud_xyzi, output_filtered);
        output_filtered.header.frame_id = laser_frame;
        output_filtered.header.stamp = ros::Time::now();
        pub4.publish(output_filtered);

        get_msg_left = 0;
        get_msg_right = 0;
        rate.sleep();


    }
    return 0;
}
