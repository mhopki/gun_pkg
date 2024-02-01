#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudFusion
{
public:
    PointCloudFusion() : nh("~")
    {
        // Initialize parameters
        nh.param<std::string>("input_topic1", inputTopic1, "/pointcloud1");
        nh.param<std::string>("input_topic2", inputTopic2, "/pointcloud2");
        nh.param<std::string>("output_topic", outputTopic, "/fused_pointcloud");

        // Setup subscribers
        sub1 = nh.subscribe(inputTopic1, 1, &PointCloudFusion::pointCloudCallback1, this);
        sub2 = nh.subscribe(inputTopic2, 1, &PointCloudFusion::pointCloudCallback2, this);

        // Setup publisher
        pub = nh.advertise<sensor_msgs::PointCloud2>(outputTopic, 1);
    }

    void pointCloudCallback1(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // Store the first point cloud
        pcl::fromROSMsg(*msg, cloud1);
        cloud1_received = true;
        fusePointClouds();
    }

    void pointCloudCallback2(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // Store the second point cloud
        pcl::fromROSMsg(*msg, cloud2);
        cloud2_received = true;
        fusePointClouds();
    }

    void fusePointClouds()
    {
        // Check if both point clouds are received
        if (cloud1_received && cloud2_received)
        {
            // Fuse the point clouds
            pcl::PointCloud<pcl::PointXYZ> fused_cloud;
            fused_cloud = cloud1;
            fused_cloud += cloud2;

            // Downsample (optional)
            pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setInputCloud(fused_cloud.makeShared());
            sor.setLeafSize(0.01f, 0.01f, 0.01f); // Adjust leaf size as needed
            sor.filter(fused_cloud);

            // Convert to ROS message and publish
            sensor_msgs::PointCloud2 fused_cloud_msg;
            pcl::toROSMsg(fused_cloud, fused_cloud_msg);
            fused_cloud_msg.header = cloud1.header; // Use the header of the first cloud

            pub.publish(fused_cloud_msg);

            // Reset flags
            cloud1_received = false;
            cloud2_received = false;
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub1, sub2;
    ros::Publisher pub;

    std::string inputTopic1, inputTopic2, outputTopic;
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    bool cloud1_received = false;
    bool cloud2_received = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_fusion");
    PointCloudFusion pointCloudFusion;
    ros::spin();
    return 0;
}
