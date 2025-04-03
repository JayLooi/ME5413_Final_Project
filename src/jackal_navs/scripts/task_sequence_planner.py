#!/usr/bin/python3

import rospy
import time
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
IMG_CAPTURE_DIST = 1.2


class SimpleCoveragePlanner:
    NAV_STATE_IDLE = 0
    NAV_STATE_GOAL_SENT = 1
    NAV_STATE_NAVIGATING = 2
    CAPTURE_STATE_IDLE = 0
    CAPTURE_STATE_NAV = 1
    CAPTURE_STATE_CAP = 2

    def __init__(self, area_bound, resolution=0.02, scan_proximity=2.0):
        rospy.init_node('simple_coverage_planner', anonymous=True)
        self._coverage_map_origin = np.array(area_bound[0])
        self._coverage_map_limit = np.array(area_bound[1])
        self.resolution = resolution
        (min_x, min_y), (max_x, max_y) = area_bound
        col_size = int((max_x - min_x) / resolution)
        row_size = int((max_y - min_y) / resolution)
        self._coverage_map = np.zeros((row_size, col_size))
        self._box_locations = []
        self._goal_poses = [
            (BOXES_AREA_MAX_X_COORD + 3, 0, -np.pi / 2), 
            (BOXES_AREA_MAX_X_COORD, BOXES_AREA_MIN_Y_COORD, np.pi / 2)
        ]
        self._goal_xy_tol = 0.45
        self._goal_yaw_tol = 0.35
        self._proximity = scan_proximity
        self._navigate_state = self.NAV_STATE_IDLE
        self._goal_status = -1
        self._capture_state = self.CAPTURE_STATE_IDLE
        self._img_num = 0
        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._laserscan_sub = rospy.Subscriber('/front/scan', LaserScan, self._laserscan_cb)
        self._camera_sub = rospy.Subscriber('/front/image_raw', Image, self._image_cb)
        self._mvoe_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self._goal_status_cb)
        self._init_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self._nav_point_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self._nav_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

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
        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]

        self._nav_point_pub.publish(goal_msg)
        self._navigate_state = self.NAV_STATE_GOAL_SENT
        self._current_goal = [x, y, yaw]
        rospy.loginfo(f'Sent goal: x={x}, y={y}, yaw={yaw}')

    def set_init_pose(self, x, y, yaw):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        self._init_pose.publish(pose)
        rospy.loginfo(f"Set estimated pose: ({x}, {y}, {yaw})")

    def get_current_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            position = transform.transform.translation
            q = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            current_pose = (position.x, position.y, yaw)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f'TF2 Lookup failed: {e}')
            return

        return current_pose

    def _check_bound(self, point):
        return (point[0] > self._coverage_map_origin[0] and
                point[1] > self._coverage_map_origin[1] and
                point[0] < self._coverage_map_limit[0] and
                point[1] < self._coverage_map_limit[1])

    def _laserscan_cb(self, msg):
        current_pose = self.get_current_pose()
        if current_pose is not None and self._check_bound(current_pose):
            try:
                points = self._transform_laserscan_to_map_frame(msg)

            except Exception as e:
                print(e)
                return

            if points:
                detected_lines = self._split_and_merge(points, outlier_thres=0.1)
                self._detect_box(detected_lines, current_pose)

    def _detect_box(self, lines, current_pose):
        for p1, p2 in lines:
            if not self._check_bound(p1) or not self._check_bound(p2):
                continue

            vec = p2 - p1
            length = np.linalg.norm(vec)
            # rospy.loginfo(f'length = {length}')
            if length < 0.75 * BOX_SIZE or length > 1.1 * BOX_SIZE:
                continue

            # Check if the line is parallel to x or y axis
            vec_x_abs = abs(vec[0])
            vec_y_abs = abs(vec[1])
            # rospy.loginfo(f'x component = {vec_x_abs}')
            # rospy.loginfo(f'y component = {vec_y_abs}')
            if vec_y_abs < 0.035:
                box_x = (p1[0] + p2[0]) / 2
                avg_y = (p1[1] + p2[1]) / 2
                if current_pose[1] < avg_y:
                    box_y = avg_y + BOX_SIZE / 2
                    potential_goal = (box_x, avg_y - IMG_CAPTURE_DIST, np.pi / 2)
                else:
                    box_y = avg_y - BOX_SIZE / 2
                    potential_goal = (box_x, avg_y + IMG_CAPTURE_DIST, -np.pi / 2)

                box_centre = np.array((box_x, box_y))

            elif vec_x_abs < 0.035:
                box_y = (p1[1] + p2[1]) / 2
                avg_x = (p1[0] + p2[0]) / 2
                if current_pose[0] < avg_x:
                    box_x = avg_x + BOX_SIZE / 2
                    potential_goal = (avg_x - IMG_CAPTURE_DIST, box_y, 0.0)
                else:
                    box_x = avg_x - BOX_SIZE / 2
                    potential_goal = (avg_x + IMG_CAPTURE_DIST, box_y, np.pi)

                box_centre = (box_x, box_y)

            else:
                continue

            min_dist = np.sqrt(2 * 0.8**2)
            for box in self._box_locations:
                if np.linalg.norm(np.array(box) - np.array(box_centre)) < min_dist:
                    break

            else:
                self._box_locations.append(box_centre)
                self._goal_poses.append(potential_goal)
                rospy.loginfo(f'box={box_centre}, potential_goal={potential_goal}, detected_line length={length}')

    def _transform_point_to_coverage_map(self, point):
        transformed = (np.array(point) - self._coverage_map_origin) / self.resolution
        max_row, max_col = self._coverage_map.shape
        row = np.max([np.min([int(transformed[1]), max_row - 1]), 0])
        col = np.max([np.min([int(transformed[0]), max_col - 1]), 0])
        return (row, col)

    def _transform_point_from_coverage_map(self, row, col):
        point = np.array([col, row])
        transformed = point * self.resolution + self._coverage_map_origin
        return transformed

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
        if self._navigate_state == self.NAV_STATE_IDLE and self._capture_state == self.CAPTURE_STATE_CAP:
            rospy.loginfo(f'Capturing {self._img_num}')
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(f'./src/jackal_navs/temp/{self._img_num}.png', cv_image)
            self._img_num += 1
            self._capture_state = self.CAPTURE_STATE_IDLE

    def _update_coverage_map(self):
        current_pose = self.get_current_pose()
        if current_pose is not None and self._check_bound(current_pose):
            min_coord = [current_pose[0] - self._proximity, current_pose[1] - self._proximity]
            max_coord = [current_pose[0] + self._proximity, current_pose[1] + self._proximity]
            row_min, col_min = self._transform_point_to_coverage_map(min_coord)
            row_max, col_max = self._transform_point_to_coverage_map(max_coord)
            self._coverage_map[row_min:row_max+1, col_min:col_max+1] = 1.0

    def _check_goal_reached(self, goal_x, goal_y, goal_yaw):
        pose = self.get_current_pose()
        if pose:
            dist_to_goal = np.linalg.norm(np.array(pose[0:2]) - np.array([goal_x, goal_y]))
            if dist_to_goal < self._goal_xy_tol and abs(pose[2] - goal_yaw) < self._goal_yaw_tol:
                return True

        return False

    def _explore(self):
        current_pose = self.get_current_pose()
        if current_pose is not None:
            min_distance = np.Inf
            for i in range(self._coverage_map.shape[0]):
                for j in range(self._coverage_map.shape[1]):
                    if self._coverage_map[i, j] == 0:  # Not covered yet
                        position = self._transform_point_from_coverage_map(i, j)
                        distance = np.linalg.norm(position - np.array(current_pose[0:2]))
                        if distance > self._proximity and distance < min_distance:
                            min_distance = distance
                            nearest = position

            if min_distance < np.Inf:
                return [nearest[0], nearest[1], np.pi / 2]

    def mainloop(self):
        rate = 30
        loop_rate = rospy.Rate(rate)
        rospy.sleep(2.0)

        self.set_init_pose(0.0, 0.0, 0.0)

        curr_goal_idx = 0
        pose_print_rate = 0.2
        pose_print_timer = 0
        cov_map_update_rate = 10
        cov_map_update_timer = 0

        while not rospy.is_shutdown():
            if self._navigate_state == self.NAV_STATE_IDLE:
                if curr_goal_idx < len(self._goal_poses):
                    if curr_goal_idx < 2:
                        self.send_goal(*self._goal_poses[curr_goal_idx])
                        curr_goal_idx += 1

                    else:
                        if self._capture_state == self.CAPTURE_STATE_NAV:
                            self._capture_state = self.CAPTURE_STATE_CAP

                        elif self._capture_state == self.CAPTURE_STATE_IDLE:
                            self.send_goal(*self._goal_poses[curr_goal_idx])
                            curr_goal_idx += 1
                            self._capture_state = self.CAPTURE_STATE_NAV

                else:
                    if np.sum(self._coverage_map) / np.multiply(*self._coverage_map.shape) < 0.99:
                        next_goal = self._explore()
                        if next_goal is not None:
                            self.send_goal(*next_goal)

            elif self._navigate_state == self.NAV_STATE_NAVIGATING:
                if self._goal_status in (2, 3, 4, 5, 8, 9):
                    self._navigate_state = self.NAV_STATE_IDLE

                elif self._check_goal_reached(*self._current_goal):
                    self._nav_cancel.publish(GoalID())
                    self._navigate_state = self.NAV_STATE_IDLE

            elif self._navigate_state == self.NAV_STATE_GOAL_SENT:
                if self._goal_status == 1:
                    self._navigate_state = self.NAV_STATE_NAVIGATING

            pose_print_timer += 1
            if pose_print_timer >= (rate / pose_print_rate):
                current_pose = self.get_current_pose()
                if current_pose is not None:
                    rospy.loginfo(f'Current Pose (tf2): x={current_pose[0]}, y={current_pose[1]}, yaw={current_pose[2]}')
                    rospy.loginfo(f'Coverage rate: {np.sum(self._coverage_map) / np.multiply(*self._coverage_map.shape)} nav_state={self._navigate_state} cap_state={self._capture_state} curr_goal_idx={curr_goal_idx}, {self._goal_poses}')
                pose_print_timer = 0

            cov_map_update_timer += 1
            if cov_map_update_rate >= (rate / cov_map_update_rate):
                self._update_coverage_map()
                cov_map_update_timer = 0

            loop_rate.sleep()


if __name__ == '__main__':
    area = [(BOXES_AREA_MIN_X_COORD, BOXES_AREA_MIN_Y_COORD),
            (BOXES_AREA_MAX_X_COORD, BOXES_AREA_MAX_Y_COORD)]
    coverage_planner = SimpleCoveragePlanner(area)
    coverage_planner.mainloop()
