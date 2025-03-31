#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, Point
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2 as PC2
import tf2_ros
from std_srvs.srv import SetBool, SetBoolResponse

class BridgeDetector:
    def __init__(self):
        rospy.init_node('bridge_detector')
        
        # Configuration
        self.cone_min_height = 0.8
        self.cone_max_height = 1.2
        self.max_height_variance = 0.01  # Allowable height variation
        self.tf_timeout = rospy.Duration(1.0)  # Wait up to 1 second for TF
        self.start_detection = False  # Default to False

        # ROS setup
        self.cloud_sub = rospy.Subscriber('/mid/points', PointCloud2, self.cloud_callback)
        self.detected_cone_pub = rospy.Publisher('/detected_cone', PointStamped, queue_size=10)
        self.detected_bridge_pub = rospy.Publisher('/detected_bridge', PointStamped, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Add service
        self.service = rospy.Service('~enable_detection', SetBool, self.handle_enable_request)

        rospy.loginfo("Bridge Detector Ready")

    def handle_enable_request(self, req):
        """Service callback to enable/disable detection"""
        self.start_detection = req.data
        message = "ENABLED" if self.start_detection else "DISABLED"
        rospy.loginfo(f"Detection {message}")
        return SetBoolResponse(success=True, message=f"Detection {message}")

    def cloud_callback(self, cloud_msg):
        if not self.start_detection:
            return
        try:
            # Transform entire point cloud to base_link frame
            transformed_cloud = self.transform_cloud_to_base_link(cloud_msg)
            if transformed_cloud is None:
                return

            # Convert transformed PointCloud2 to numpy array
            points = np.array(list(pc2.read_points(
                transformed_cloud, 
                field_names=("x", "y", "z"),
                skip_nans=True
            )))

            if len(points) == 0:
                rospy.loginfo("Empty point cloud received")
                return

            # Step 3: Filter points by height (in base_link frame)
            z_min = self.cone_min_height   
            z_max = self.cone_max_height
            z_filter = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
            filtered_points = points[z_filter]

            if len(filtered_points) == 0:
                rospy.logwarn(f"No points in Z-range [{z_min:.3f}, {z_max:.3f}] meters")
                return

            # Step 4: Find nearest cluster and report Z-range
            distances = np.linalg.norm(filtered_points[:,:2], axis=1)  # XY distance only
            nearest_idx = np.argmin(distances)
            nearest_point = filtered_points[nearest_idx]
            
            # Calculate Z-range of all points within 10m of nearest point (crude cluster)
            cluster_mask = np.linalg.norm(filtered_points[:,:2] - nearest_point[:2], axis=1) <= 10.0
            cluster_points = filtered_points[cluster_mask]
            z_min_cluster = np.min(cluster_points[:, 2])
            z_max_cluster = np.max(cluster_points[:, 2])
            
            rospy.loginfo(
                f"Nearest cone on bridge candidate at (base_link frame):\n"
                f"  Position: X={nearest_point[0]:.3f}m, Y={nearest_point[1]:.3f}m, Z={nearest_point[2]:.3f}m\n"
                f"  Z-range of cluster: {z_min_cluster:.3f}m to {z_max_cluster:.3f}m\n"
                f"  Cluster contains {len(cluster_points)} points"
            )

            # Step 5: Publish detected pole position
            detected_cone = PointStamped()
            detected_cone.header.frame_id = "base_link"
            detected_cone.header.stamp = rospy.Time.now()
            detected_cone.point = Point(*nearest_point)

            detected_bridge = PointStamped()
            detected_bridge.header.frame_id = "base_link"
            detected_bridge.header.stamp = rospy.Time.now()

            detected_bridge.point = Point(
                x=detected_cone.point.x - 3.0,
                y=detected_cone.point.y, # if world frame, should increase in y for 3.0
                z=detected_cone.point.z
            )
            self.detected_cone_pub.publish(detected_cone)
            self.detected_bridge_pub.publish(detected_bridge)

        except Exception as e:
            rospy.logerr(f"Error in cloud_callback: {str(e)}")

    def transform_cloud_to_base_link(self, cloud_msg):
        """Transform PointCloud2 from its frame to base_link"""
        try:
            # Wait for transform
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
                rospy.Duration(1.0))
            
            # Apply transform
            transformed_cloud = do_transform_cloud(cloud_msg, transform)
            return transformed_cloud

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF transform failed: {str(e)}")
            return None

if __name__ == '__main__':
    try:
        detector = BridgeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass