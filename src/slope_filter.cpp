#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
//Include for compute normals
#include <pcl/features/normal_3d_omp.h>

#include "livox_ros_driver2/msg/custom_msg.hpp"

//parameters given by the launch file
std::string laser_frame;
std::string scan_topic_left;
std::string scan_topic_right;
std::string new_scan_topic;
std::string filted_topic_3d;
double first_radius = 0.35;//车体自身附近直接丢的半径 (m)

//法向量那一路的前置处理参数
double voxel_leaf_size = 0.05; //体素下采样边长 (m)
int    sor_mean_k      = 20;   //统计滤波近邻数
double sor_stddev_mul  = 1.0;  //统计滤波标准差倍数

const int NORMAL_K_SEARCH = 20;//法向量估计的近邻数

bool get_msg_left = false;
bool get_msg_right = false;

livox_ros_driver2::msg::CustomMsg scan_copy_left;//receiving the message from the livox
livox_ros_driver2::msg::CustomMsg scan_copy_right;

//callback function for the livox
void scanCallback_left(const livox_ros_driver2::msg::CustomMsg &scan)
{
    scan_copy_left = scan;
    get_msg_left = true;
}
void scanCallback_right(const livox_ros_driver2::msg::CustomMsg &scan)
{
    scan_copy_right = scan;
    get_msg_right = true;
}

//坡度滤波：当前只保留了"丢掉车体附近"这一条，分段高度上限那套已禁用
bool ispoint (double nx, double ny, double z, double nI)
{
    (void)z;
    (void)nI; //暂时不用反射强度
    // nx += 0.011;  偏心量？ 暂时弃用
    // ny -= -0.19495+0.02329;

    double r2 = nx*nx + ny*ny;
    if (r2 <= first_radius*first_radius){
        return false; //车体自身附近直接丢
    }
    // 分段坡度判据（需要时把参数补回来再启用）：
    // if (r2 <= second_radius*second_radius)          return z <= start_height;
    // if (r2 <= slp_first_radius*slp_first_radius)    return z <= std::min(max_height, (sqrt(r2)-second_radius)*slope_1 + start_height);
    // if (r2 <= slp_second_radius*slp_second_radius)  return z <= std::min(height_1, (sqrt(r2)-slp_first_radius)*slope_2 + max_height);
    // return z <= std::min(height_2, (sqrt(r2)-slp_second_radius)*slope_3 + height_1);
    return true;
}

//把一帧 CustomMsg 拆成两路：origin 收全部有效点，filtered 只收通过坡度滤波的点
void accumulate_scan(const livox_ros_driver2::msg::CustomMsg &scan,
                     pcl::PointCloud<pcl::PointXYZI> &origin,
                     pcl::PointCloud<pcl::PointXYZI> &filtered)
{
    for (const auto &pt : scan.points)
    {
        double x = pt.x ;
        double y = pt.y ;
        double z = pt.z ;
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

        if (ispoint(x, y, z, intensity))
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
    voxel.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel.filter(*output);
}

//统计离群点移除
void statistical_removal(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr &output)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(input);
    sor.setMeanK(sor_mean_k);
    sor.setStddevMulThresh(sor_stddev_mul);
    sor.filter(*output);
}


int main (int argc, char **argv)
{
    const std::string node_name = "threeD_lidar_filter_pointcloud";
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(node_name);

    //参数：yaml / launch 里给了就用给的值，没给就用这里的默认值
    laser_frame      = node->declare_parameter<std::string>("laser_frame", "livox_frame");
    scan_topic_left  = node->declare_parameter<std::string>("scan_topic_left",  "/livox/lidar_192_168_1_135");
    scan_topic_right = node->declare_parameter<std::string>("scan_topic_right", "/livox/lidar_192_168_1_122");
    new_scan_topic   = node->declare_parameter<std::string>("new_scan_topic",   "/cloud_origin");
    filted_topic_3d  = node->declare_parameter<std::string>("filted_topic_3d",  "/cloud_slope_filtered");
    first_radius     = node->declare_parameter<double>("first_radius", first_radius);

    voxel_leaf_size = node->declare_parameter<double>("voxel_leaf_size", voxel_leaf_size);
    sor_mean_k      = static_cast<int>(node->declare_parameter<int>("sor_mean_k", sor_mean_k));
    sor_stddev_mul  = node->declare_parameter<double>("sor_stddev_mul", sor_stddev_mul);

    RCLCPP_INFO(node->get_logger(), "topic: left=%s right=%s | origin=%s filtered=%s",
                scan_topic_left.c_str(), scan_topic_right.c_str(),
                new_scan_topic.c_str(), filted_topic_3d.c_str());
    RCLCPP_INFO(node->get_logger(), "first_radius=%.3f voxel_leaf=%.3f sor_mean_k=%d sor_stddev_mul=%.2f",
                first_radius, voxel_leaf_size, sor_mean_k, sor_stddev_mul);

    auto sub_left  = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        scan_topic_left,  10, scanCallback_left);
    auto sub_right = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        scan_topic_right, 10, scanCallback_right);
    //点云流用 SensorDataQoS(best_effort + keep_last)，和 rog_map 那边的订阅对得上
    auto pub2 = node->create_publisher<sensor_msgs::msg::PointCloud2>(new_scan_topic, rclcpp::SensorDataQoS());//original pointcloud
    auto pub4 = node->create_publisher<sensor_msgs::msg::PointCloud2>(filted_topic_3d, rclcpp::SensorDataQoS());//filtered pointcloud
    rclcpp::WallRate rate(50.0);

    while (rclcpp::ok())
    {
        if (!get_msg_left || !get_msg_right)
        {
            rclcpp::spin_some(node);
            // RCLCPP_INFO(node->get_logger(), "waiting for the message");
            continue;
        }

        auto scan_record_left = scan_copy_left;
        auto scan_record_right = scan_copy_right;

        pcl::PointCloud<pcl::PointXYZI> origin_pcl_cloud;//给调试看的原始点云
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;//过了坡度滤波的点
        origin_pcl_cloud.points.reserve(scan_record_left.points.size() + scan_record_right.points.size());
        pcl_cloud.points.reserve(scan_record_left.points.size() + scan_record_right.points.size());

        accumulate_scan(scan_record_left, origin_pcl_cloud, pcl_cloud);
        accumulate_scan(scan_record_right, origin_pcl_cloud, pcl_cloud);

        // print the sizes of the two clouds
        // RCLCPP_INFO(node->get_logger(), "Origin points: %zu, after slope filter: %zu", origin_pcl_cloud.points.size(), pcl_cloud.points.size());

        auto start_time = node->now();//TimeTest Start

        //Down Sampling + Statistical Removal
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_cloud));
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr denoised_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        down_sampling(pcl_cloud_ptr, temp_cloud);
        statistical_removal(temp_cloud, denoised_cloud);
        // RCLCPP_INFO(node->get_logger(), "After down sampling + SOR: %zu", denoised_cloud->points.size());

        //Find Normals
        pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        n.setInputCloud(denoised_cloud);
        n.setSearchMethod(tree);
        n.setKSearch(NORMAL_K_SEARCH);
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

            if (angle <= M_PI/11.5 || angle >= M_PI/10) { //滤除17度左右的坡面
                filtered_cloud_xyzi->points.push_back(denoised_cloud->points[i]);
            }
        }
        // RCLCPP_INFO(node->get_logger(), "After normal filter: %zu", filtered_cloud_xyzi->points.size());

        auto end_time = node->now();//TimeTest End
        auto filter_duration = end_time - start_time;
        // RCLCPP_INFO(node->get_logger(), "Filtering took %f seconds", filter_duration.seconds());

        // Publish original cloud
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(origin_pcl_cloud, output);
        output.header.frame_id = laser_frame;
        output.header.stamp = node->now();
        pub2->publish(output);

        // Publish filtered cloud
        sensor_msgs::msg::PointCloud2 output_filtered;
        pcl::toROSMsg(*filtered_cloud_xyzi, output_filtered);
        output_filtered.header.frame_id = laser_frame;
        output_filtered.header.stamp = node->now();
        pub4->publish(output_filtered);

        get_msg_left = false;
        get_msg_right = false;

        rclcpp::spin_some(node);//处理完也转一下，免得订阅队列堆积
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
