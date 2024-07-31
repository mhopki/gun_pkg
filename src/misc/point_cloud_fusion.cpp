#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//DOES NOT COMPILE UNLESS DOTRANSFORM IS COMMENTED OUT!!!!!!
//NEED TO DO TRANSFORM MANUALLY TO GET IT TO WORK!!!!!

class OctomapFusion {
public:
    void fuseOctomaps(const octomap_msgs::Octomap::ConstPtr& octomap_msg1,
                      const octomap_msgs::Octomap::ConstPtr& octomap_msg2) {
        geometry_msgs::TransformStamped transform;

        try {
            // Replace "common_frame" with your desired common frame
            transform = tf_buffer.lookupTransform("dragonfly26/odom", octomap_msg1->header.frame_id, ros::Time(0));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Transform octomap_msg1 to the common frame
        octomap_msgs::Octomap transformed_octomap_msg1;
        //tf2::doTransform(*octomap_msg1, transformed_octomap_msg1, transform);

        // Convert Octomap messages to Octrees
        octomap::OcTree* octree1 = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*octomap_msg1));
        octomap::OcTree* octree2 = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*octomap_msg2));

        // Iterate through occupied nodes of the second octree and insert into the first
        for (auto it = octree2->begin(); it != octree2->end(); ++it) {
            if (octree2->isNodeOccupied(*it)) {
                octree1->updateNode(it.getCoordinate(), true);
            }
        }

        // Convert the fused Octree back to an Octomap message
        octomap_msgs::Octomap fused_octomap_msg;
        octomap_msgs::binaryMapToMsg(*octree1, fused_octomap_msg);

        // Publish the fused Octomap message
        fused_octomap_pub.publish(fused_octomap_msg);

        delete octree1;
        delete octree2;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber octomap_sub1;
    ros::Subscriber octomap_sub2;
    ros::Publisher fused_octomap_pub;
    tf2_ros::Buffer tf_buffer;

public:
    OctomapFusion() : nh("~") {
        octomap_sub1 = nh.subscribe("/dragonfly26/octomap_full", 1, &OctomapFusion::octomapCallback1, this);
        octomap_sub2 = nh.subscribe("/dragonfly26_sonar/octomap_full", 1, &OctomapFusion::octomapCallback2, this);
        fused_octomap_pub = nh.advertise<octomap_msgs::Octomap>("/fused_octomap_out", 1);
    }

    void octomapCallback1(const octomap_msgs::Octomap::ConstPtr& msg) {
        octomap_msgs::Octomap::ConstPtr octomap_msg2 = ros::topic::waitForMessage<octomap_msgs::Octomap>("/dragonfly26_sonar/octomap_full", nh, ros::Duration(1.0));
        if (octomap_msg2) {
            fuseOctomaps(msg, octomap_msg2);
        }
    }

    void octomapCallback2(const octomap_msgs::Octomap::ConstPtr& msg) {
        octomap_msgs::Octomap::ConstPtr octomap_msg1 = ros::topic::waitForMessage<octomap_msgs::Octomap>("/dragonfly26/octomap_full", nh, ros::Duration(1.0));
        if (octomap_msg1) {
            fuseOctomaps(octomap_msg1, msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_fusion_node");
    OctomapFusion fusion;
    ros::spin();
    return 0;
}
