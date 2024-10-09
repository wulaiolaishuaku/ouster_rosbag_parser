#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// 将点云数据写入TXT文件
void write_points_to_file(const std::string& filename, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return;
    }

    for (const auto& point : *cloud) {
        // 写入x, y, z, 反射强度
        file << point.x << "," << point.y << "," << point.z << "," << point.intensity << "\n";
    }
    file.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ouster_rosbag_parser_node");
    ros::NodeHandle nh("~");

    // 配置参数
    std::string bag_file, output_dir;
    nh.param<std::string>("bag_file", bag_file, "/home/zzh/catkin_ws/cqu_datset.bag");
    nh.param<std::string>("output_dir", output_dir, "/home/zzh/txt/");

    // 创建ROS bag文件的读取对象
    rosbag::Bag bag;
    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (rosbag::BagException& e) {
        ROS_ERROR("Error opening bag file: %s", e.what());
        return -1;
    }

    // 读取bag文件中的所有消息
    rosbag::View view(bag);

    int frame_id = 0;
    for (const rosbag::MessageInstance& m : view) {
        // 检查消息类型是否是PointCloud2
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg != nullptr) {
            // 转换为PCL点云
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg, pcl_pc2);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

            // 生成输出文件名
            std::string filename = output_dir + "frame_" + std::to_string(frame_id) + ".txt";

            // 将点云数据写入文件
            write_points_to_file(filename, cloud);

            // 更新帧ID
            frame_id++;

            // 可以在这里添加一些日志信息
            ROS_INFO("Processed frame %d and saved to %s", frame_id, filename.c_str());
        }
    }

    // 关闭bag文件
    bag.close();

    return 0;
}
