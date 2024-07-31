import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
import ros_numpy

def callback(data):
    # Convert the ROS point cloud message to a NumPy array
    points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)

    # Create a new PointCloud2 message for the output
    output_msg = PointCloud2()
    output_msg.header = data.header
    output_msg.width = points.shape[0]
    output_msg.height = 1

    # Convert the NumPy array to a byte array
    points_list = points.flatten().tolist()
    #output_msg.data = bytearray(points_list)

    points_list_int = [int(point) for point in points_list]
    output_msg.data = bytearray(points_list_int)

    # Publish the output message
    pub.publish(output_msg)

if __name__ == '__main__':
    # Initialize the node and subscribe to the topic
    rospy.init_node('pointcloud_subscriber', anonymous=True)
    sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback)

    # Create a publisher to publish the output message
    pub = rospy.Publisher("/my_pointcloud", PointCloud2, queue_size=10)

    # Spin the node to receive and publish messages
    rospy.spin()
