#!/usr/bin/python3

import rospy
import cv2
import numpy as np
import tf2_ros 
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalID, GoalStatusArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from cv_bridge import CvBridge


BOX_SIZE = 0.8
BOXES_AREA_MIN_X_COORD = 11.0
BOXES_AREA_MIN_Y_COORD = -22.0
BOXES_AREA_MAX_X_COORD = 19.0
BOXES_AREA_MAX_Y_COORD = -2.0
ROBOT_WIDTH = 0.34
ROBOT_LENGTH = 0.42
PARTITION_SIZE = 4


class _WayPoint:
    def __init__(self):
        rospy.init_node('nav_point_sender', anonymous=True)
        self._nav_point_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    def send_goal(self, x, y, yaw):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0

        # Convert yaw to quaternion
        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]
        
        self._nav_point_pub.publish(goal_msg)
        rospy.loginfo(f'Sent goal: x={x}, y={y}, yaw={yaw}')


class SimpleCoveragePlanner:
    NAV_STATE_IDLE = 0
    NAV_STATE_GOAL_SENT = 1
    NAV_STATE_NAVIGATING = 2

    def __init__(self, area_bound, resolution=0.02, scan_proximity=1.6):
        rospy.init_node('simple_coverage_planner', anonymous=True)
        self._coverage_map_origin = np.array(area_bound[0])
        self._coverage_map_limit = np.array(area_bound[1])
        self.resolution = resolution
        (min_x, min_y), (max_x, max_y) = area_bound
        col_size = int((max_x - min_x) / resolution)
        row_size = int((max_y - min_y) / resolution)
        self._coverage_map = np.zeros((row_size, col_size))
        self._box_locations = []
        self._unvisited_boxes = []
        self._proximity = scan_proximity
        self._navigate_state = self.NAV_STATE_IDLE
        self._goal_status = -1
        self._current_pose = [0, 0, 0]
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        # self._laserscan_sub = rospy.Subscriber('/front/scan', LaserScan, self._laserscan_cb)
        self._camera_sub = rospy.Subscriber('/camera/image_raw', Image, self._image_cb)
        self._pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._pose_cb)
        # self._pose_sub = rospy.Subscriber('/odometry/filtered', Odometry, self._pose_cb)
        self._mvoe_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self._goal_status_cb)
        self._init_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self._nav_point_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self._nav_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

    def _pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = []
        q.append(msg.pose.pose.orientation.x)
        q.append(msg.pose.pose.orientation.y)
        q.append(msg.pose.pose.orientation.z)
        q.append(msg.pose.pose.orientation.w)
        _, _, yaw = euler_from_quaternion(q)
        self._current_pose = np.array((x, y, np.rad2deg(yaw)))

    def _goal_status_cb(self, msg):
        if msg.status_list:
            self._goal_status = msg.status_list[0].status

    def send_goal(self, x, y, yaw):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0

        # Convert yaw to quaternion
        q = quaternion_from_euler(0, 0, np.deg2rad(yaw))
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]

        self._nav_point_pub.publish(goal_msg)
        self._navigate_state = self.NAV_STATE_GOAL_SENT
        rospy.loginfo(f'Sent goal: x={x}, y={y}, yaw={yaw}')

    def set_init_pose(self, x, y, yaw):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, np.deg2rad(yaw))
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        self._init_pose.publish(pose)
        self._current_pose = [x, y, yaw]
        rospy.loginfo(f"Set estimated pose: {self._current_pose}")

    def _laserscan_cb(self, msg):
        if (self._current_pose[0] > self._coverage_map_origin[0] and
            self._current_pose[1] > self._coverage_map_origin[1] and
            self._current_pose[0] < self._coverage_map_limit[0] and
            self._current_pose[1] < self._coverage_map_limit[1]):
            rospy.loginfo(self._current_pose)
            try:
                points = self._transform_laserscan_to_map_frame(msg)

            except Exception as e:
                print(e)
                return

            if points:
                detected_lines = self._split_and_merge(points, outlier_thres=0.1)
                self._detect_box(detected_lines)
                # print(self._unvisited_boxes)
                # if self._unvisited_boxes:

    def _detect_box(self, lines):
        for p1, p2 in lines:
            vec = p2 - p1
            length = np.linalg.norm(vec)
            rospy.loginfo(f'length = {length}')
            # if length < 0.8 * BOX_SIZE or length > 1.2 * BOX_SIZE:
            #     continue

            row_1, col_1 = self._transform_point_to_coverage_map(p1)
            row_2, col_2 = self._transform_point_to_coverage_map(p2)

            # Check if the line is parallel to x or y axis
            rospy.loginfo(f'row diff = {abs(row_1 - row_2)}')
            rospy.loginfo(f'col diff = {abs(col_1 - col_2)}')
            if abs(row_1 - row_2) < 3:
                avg_row = int((row_1 + row_2) / 2)
                if self._current_pose[1] < p1[1]:
                    box_y = avg_row + int((BOX_SIZE / 2) / self.resolution)
                else:
                    box_y = avg_row - int((BOX_SIZE / 2) / self.resolution)
                box_centre = np.array((int((col_1 + col_2) / 2), box_y))

            elif abs(col_1 - col_2) < 3:
                avg_col = int((col_1 + col_2) / 2)
                if self._current_pose[0] < p1[0]:
                    box_x = avg_col + int((BOX_SIZE / 2) / self.resolution)
                else:
                    box_x = avg_col - int((BOX_SIZE / 2) / self.resolution)
                box_centre = np.array((box_x, int((row_1 + row_2) / 2)))

            else:
                continue

            min_dist = np.sqrt(2 * 0.8**2)
            for box in self._box_locations:
                if np.linalg.norm(box - box_centre) < min_dist:
                    break

            else:
                self._box_locations.append(box_centre)
                self._unvisited_boxes.append(box_centre)

    def _transform_point_to_coverage_map(self, point):
        transformed = (point - self._coverage_map_origin) / self.resolution
        max_row, max_col = self._coverage_map.shape
        row = np.max([np.min([transformed[1], max_row - 1]), 0])
        col = np.max([np.min([transformed[0], max_col - 1]), 0])
        return (row, col)

    def _transform_laserscan_to_map_frame(self, msg):
        try:
            # Get transformation from laser frame to map
            transform = self._tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            ranges = msg.ranges
            min_angle = msg.angle_min
            angle_step = msg.angle_increment
            range_max = msg.range_max
            curr_angle = min_angle
            transformed_points = []
            for distance in ranges:
                if distance < range_max and distance < self._proximity:
                    x = distance * np.cos(curr_angle)
                    y = distance * np.sin(curr_angle)

                    point = PoseStamped()
                    point.header.frame_id = msg.header.frame_id
                    point.header.stamp = msg.header.stamp
                    point.pose.position.x = x
                    point.pose.position.y = y
                    point.pose.position.z = 0.0
                    point.pose.orientation.w = 1.0

                    transformed = do_transform_pose(point, transform)
                    transformed_points.append(np.array((transformed.pose.position.x, transformed.pose.position.y)))

                curr_angle += angle_step

            return transformed_points

        except Exception as e:
            rospy.logerr(f'LaserScan transformation error: {e}')

    def _split_and_merge(self, points, outlier_thres=0.05):
        if len(points) < 2:
            return []

        # Fit line from first to last point
        p1, p2 = points[0], points[-1]
        max_dist = 0
        split_index = 0

        # Find the point with max deviation
        for i in range(1, len(points) - 1):
            dist = np.abs(np.cross(p2 - p1, points[i] - p1)) / np.linalg.norm(p2 - p1)
            if dist > max_dist:
                max_dist = dist
                split_index = i

        if max_dist > outlier_thres:
            left_segments = self._split_and_merge(points[:split_index + 1], outlier_thres)
            right_segments = self._split_and_merge(points[split_index:], outlier_thres)
            return left_segments + right_segments

        else:
            return [(p1, p2)]

    def _image_cb(self, msg):
        if self._navigate_state == self.NAV_STATE_IDLE:
            pass

    def mainloop(self):
        loop_rate = rospy.Rate(30)
        rospy.sleep(2.0)

        self.set_init_pose(0.0, 0.0, 0.0)
        rospy.sleep(2.0)
        goals = [
            (BOXES_AREA_MAX_X_COORD + 3, 0, 0), 
            (BOXES_AREA_MAX_X_COORD - 3, BOXES_AREA_MIN_Y_COORD + 3, 90)
        ]
        current_goal = 0
        self.send_goal(*goals[current_goal])

        while not rospy.is_shutdown():
            if self._navigate_state == self.NAV_STATE_NAVIGATING:
                if self._goal_status in (2, 3, 4, 5, 8, 9):
                    self._navigate_state = self.NAV_STATE_IDLE
                    current_goal += 1
                    if current_goal < len(goals):
                        self.send_goal(*goals[current_goal])

            elif self._navigate_state == self.NAV_STATE_GOAL_SENT:
                if self._goal_status == 1:
                    self._navigate_state = self.NAV_STATE_NAVIGATING

            # else:
            #     self.set_init_pose(*self._current_pose)

            loop_rate.sleep()


if __name__ == '__main__':
    area = [(BOXES_AREA_MIN_X_COORD, BOXES_AREA_MIN_Y_COORD),
            (BOXES_AREA_MAX_X_COORD, BOXES_AREA_MAX_Y_COORD)]
    coverage_planner = SimpleCoveragePlanner(area)
    coverage_planner.mainloop()
