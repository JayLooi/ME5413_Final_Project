#!/usr/bin/python3

import rospy
import time
import cv2
import os
import numpy as np
import tf2_ros
from threading import Lock
from tf2_geometry_msgs import do_transform_pose, do_transform_point
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import Int16, Int32, Bool
from actionlib_msgs.msg import GoalID, GoalStatusArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker


BOX_SIZE = 0.8
BOXES_AREA_MIN_X_COORD = 11.0
BOXES_AREA_MIN_Y_COORD = -22.0
BOXES_AREA_MAX_X_COORD = 19.0
BOXES_AREA_MAX_Y_COORD = -2.0
ROBOT_WIDTH = 0.34
ROBOT_LENGTH = 0.42
IMG_CAPTURE_DIST = 0.8
BRIDGE_DETECT_AREA_MIN_X = BOXES_AREA_MIN_X_COORD - 2.0
BRIDGE_DETECT_AREA_MIN_Y = BOXES_AREA_MIN_Y_COORD
BRIDGE_DETECT_AREA_MAX_X = BOXES_AREA_MIN_X_COORD + 1.0
BRIDGE_DETECT_AREA_MAX_Y = BOXES_AREA_MAX_Y_COORD


class SimpleCoveragePlanner:
    NAV_STATE_IDLE = 0
    NAV_STATE_GOAL_SENT = 1
    NAV_STATE_NAVIGATING = 2
    NAV_STATE_CAPTURING = 3
    NAV_STATE_WAITING_RESULT = 4
    NAV_STATE_DONE = 0xFF
    GOAL_TYPE_EXPLORE = 0
    GOAL_TYPE_CAPTURE = 1
    GOAL_TYPE_CAPTURE_TARGET = 2
    GOAL_TYPE_DETECT_BRIDGE = 3
    GOAL_TYPE_CROSS_BRIDGE = 4
    GOAL_TYPE_RESULT = 5

    def __init__(self, area_bound, resolution=0.02, scan_proximity=5.0):
        rospy.init_node('simple_coverage_planner', anonymous=True)
        self._coverage_map_origin = np.array(area_bound[0])
        self._coverage_map_limit = np.array(area_bound[1])
        self.resolution = resolution
        (min_x, min_y), (max_x, max_y) = area_bound
        col_size = int((max_x - min_x) / resolution)
        row_size = int((max_y - min_y) / resolution)
        self._coverage_map = np.zeros((row_size, col_size))
        self._coverage_rate = 0.0
        self._box_locations = []
        self._goal_box_poses = []
        self._goal_xy_tol = 0.4
        self._goal_yaw_tol = 0.32
        self._proximity = scan_proximity
        self._navigate_state = self.NAV_STATE_IDLE
        self._curr_goal_type = self.GOAL_TYPE_EXPLORE
        self._are_boxes_localised = False
        self._target_box_poses = []
        self._num_target_box_detected = 0
        self._prepare_for_bridge_detection = False
        self._bridge_detect_start = False
        self._cross_bridge_goal_pose = None
        self._cross_bridge_pose_idx = 0
        self._detect_bridge_poses = [(BRIDGE_DETECT_AREA_MIN_X + BRIDGE_DETECT_AREA_MAX_X) / 2, (BRIDGE_DETECT_AREA_MIN_Y + BRIDGE_DETECT_AREA_MAX_Y) / 2, np.pi]
        self._blockade_goal_pose = None
        self._least_occ = None
        self._goal_status = -1

        self._img_num = 0
        self._cov_map_mutex = Lock()
        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._laserscan_sub = rospy.Subscriber('/front/scan', LaserScan, self._laserscan_cb)
        self._camera_sub = rospy.Subscriber('/front/image_raw', Image, self._image_cb)
        self._move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self._goal_status_cb)
        self._least_occ_box_sub = rospy.Subscriber('/least_occurrence', Int32, self._least_occ_result_cb)
        self._target_box_digit_sub = rospy.Subscriber('/target_box_digit', Int32, self._target_box_digit_cb)
        self._bridge_sub = rospy.Subscriber('/detected_bridge', PointStamped, self._bridge_detect_cb)
        self._blockade_cone_sub = rospy.Subscriber('/detected_cone', PointStamped, self._blockade_cone_detect_cb)
        self._init_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self._nav_point_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self._nav_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self._viz_pub = rospy.Publisher('/viz/box_line', Marker, queue_size=20)
        self._image_pub = rospy.Publisher('/box_image', Image, queue_size=20)
        self._start_bridge_detect_pub = rospy.Publisher('/start_bridge_detect', Bool, queue_size=1)
        self._target_box_pub = rospy.Publisher('/target_box', Image, queue_size=1)
        self._bridge_open_pub = rospy.Publisher('/cmd_open_bridge', Bool, queue_size=1)
        self._sub_respawn_objects_pub = rospy.Publisher("/rviz_panel/respawn_objects", Int16, queue_size=1)

        directory = os.path.dirname(os.path.realpath(__file__))
        self._temp_directory = os.path.abspath(directory + '/../temp')
        if not os.path.isdir(self._temp_directory):
            os.mkdir(self._temp_directory)

    def _goal_status_cb(self, msg):
        if msg.status_list:
            self._goal_status = msg.status_list[0].status
            # Check if current goal is bridge crossing and failed
            if (self._curr_goal_type == self.GOAL_TYPE_CROSS_BRIDGE and 
                self._goal_status in [4, 8]):  # ABORTED, LOST
                rospy.logwarn("Bridge crossing goal failed! Will ignore and retry previous pose...")
                self._navigate_state = self.NAV_STATE_IDLE

    def spawn_objects(self):
        self._sub_respawn_objects_pub.publish(Int16(1))

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
        if self._curr_goal_type == self.GOAL_TYPE_EXPLORE:
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
            if length < 0.8 * BOX_SIZE or length > 1.1 * BOX_SIZE:
                continue

            # Check if the line is parallel to x or y axis
            vec_x_abs = abs(vec[0])
            vec_y_abs = abs(vec[1])
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
                # self._goal_box_poses.append(potential_goal)
                rospy.loginfo(f'box={box_centre}, potential_goal={potential_goal}, detected_line length={length}')
                box_bottomleft = np.array(box_centre) - BOX_SIZE / 2 - 0.05
                box_topright = np.array(box_centre) + BOX_SIZE / 2 + 0.05
                viz = Marker()
                viz.ns = str(len(self._box_locations))
                viz.type = Marker.CUBE
                viz.action = Marker.ADD
                viz.header.frame_id = 'map'
                viz.pose.position.x = box_centre[0]
                viz.pose.position.y = box_centre[1]
                viz.pose.position.z = 0.0
                viz.pose.orientation.x = 0.0
                viz.pose.orientation.y = 0.0
                viz.pose.orientation.z = 0.0
                viz.pose.orientation.w = 1.0
                viz.scale.x = BOX_SIZE + 0.1
                viz.scale.y = BOX_SIZE + 0.1
                viz.scale.z = 0.01
                viz.color.a = 1.0
                self._viz_pub.publish(viz)
                self._update_coverage_map(box_bottomleft, box_topright, -1.0, blocking=True)

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
        if self._navigate_state == self.NAV_STATE_CAPTURING:
            if self._curr_goal_type == self.GOAL_TYPE_CAPTURE:
                rospy.loginfo(f'Capturing {self._img_num}')
                cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv2.imwrite(f'{self._temp_directory}/{self._img_num}.png', cv_image)
                self._image_pub.publish(msg)
                self._img_num += 1
                self._navigate_state = self.NAV_STATE_IDLE

            elif self._curr_goal_type == self.GOAL_TYPE_CAPTURE_TARGET:
                rospy.loginfo(f'Capturing target...')
                self._target_box_pub.publish(msg)
                self._navigate_state = self.NAV_STATE_WAITING_RESULT

    def _target_box_digit_cb(self, msg):
        if self._navigate_state == self.NAV_STATE_WAITING_RESULT:
            self._target_box_poses[self._num_target_box_detected]['digit'] = msg.data
            self._num_target_box_detected += 1
            if self._least_occ is not None and self._least_occ == msg.data:
                rospy.loginfo(f'Found least occurrence digit target box ({msg.data})')
                self._navigate_state = self.NAV_STATE_DONE
            else:
                self._navigate_state = self.NAV_STATE_IDLE

    def _least_occ_result_cb(self, msg):
        # if self._curr_goal_type == self.GOAL_TYPE_RESULT:
        self._least_occ = msg.data

    def _bridge_detect_cb(self, msg):
        if self._cross_bridge_goal_pose is None:
            try:
                # Get transformation from base link to map
                transform = self._tf_buffer.lookup_transform(
                    'map',
                    msg.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )

                transformed = do_transform_point(msg, transform)
                self._cross_bridge_goal_pose = [
                    (transformed.point.x, transformed.point.y, np.pi),
                    (self._blockade_goal_pose[0] + 0.5, self._blockade_goal_pose[1], np.pi),
                    (5.5, self._blockade_goal_pose[1], np.pi),
                    (2.0, self._blockade_goal_pose[1], np.pi)
                ]

            except Exception as e:
                rospy.logerr(f'Bridge position transformation error: {e}')

    def _blockade_cone_detect_cb(self, msg):
        if self._blockade_goal_pose is None:
            try:
                # Get transformation from base link to map
                transform = self._tf_buffer.lookup_transform(
                    'map',
                    msg.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )

                transformed = do_transform_point(msg, transform)
                self._blockade_goal_pose = [
                    transformed.point.x, transformed.point.y, np.pi
                ]
            except Exception as e:
                rospy.logerr(f'Cone position transformation error: {e}')

    def _update_coverage_map(self, bottomleft, topright, value, blocking):
        if self._cov_map_mutex.acquire(blocking=blocking):
            row_min, col_min = self._transform_point_to_coverage_map(bottomleft)
            row_max, col_max = self._transform_point_to_coverage_map(topright)
            self._coverage_map[row_min:row_max+1, col_min:col_max+1] = value
            self._coverage_rate = np.sum(self._coverage_map != 0) / np.multiply(*self._coverage_map.shape)
            self._cov_map_mutex.release()

    def _update_visited_area(self):
        current_pose = self.get_current_pose()
        if current_pose is not None and self._check_bound(current_pose):
            min_coord = [current_pose[0] - self._proximity, current_pose[1] - self._proximity]
            max_coord = [current_pose[0] + self._proximity, current_pose[1] + self._proximity]
            self._update_coverage_map(min_coord, max_coord, 1.0, blocking=False)

    def _check_goal_reached(self, goal_x, goal_y, goal_yaw):
        pose = self.get_current_pose()
        if pose:
            dist_to_goal = np.linalg.norm(np.array(pose[0:2]) - np.array([goal_x, goal_y]))
            yaw_diff = abs(pose[2] - goal_yaw)
            yaw_diff = min(yaw_diff, 2 * np.pi - yaw_diff)
            if self._curr_goal_type == self.GOAL_TYPE_EXPLORE:
                tolerance_increase = 1.7    # More tolerant for exploration goal
                yaw_diff = 0.0              # Don't care exploration goal yaw angle
            else:
                tolerance_increase = 1.0

            minimize_yaw_tolerance_ratio = 1.0
            # stricter for alignment before detect bridge
            if self._prepare_for_bridge_detection is True:
                minimize_yaw_tolerance_ratio = 0.675

            if (dist_to_goal < tolerance_increase * self._goal_xy_tol and
                yaw_diff < tolerance_increase * self._goal_yaw_tol * minimize_yaw_tolerance_ratio):
                self._nav_cancel.publish(GoalID())
                return True

        return False

    def _explore(self):
        current_pose = self.get_current_pose()
        if current_pose is not None:
            min_distance = np.Inf
            t = time.time()
            for i in range(self._coverage_map.shape[0]):
                for j in range(self._coverage_map.shape[1]):
                    if self._coverage_map[i, j] == 0:  # Not covered yet
                        position = self._transform_point_from_coverage_map(i, j)
                        distance = np.linalg.norm(position - np.array(current_pose[0:2]))
                        if distance > self._proximity and distance < min_distance:
                            row_min, col_min = self._transform_point_to_coverage_map(position - ROBOT_LENGTH / 2 - 0.05)
                            row_max, col_max = self._transform_point_to_coverage_map(position + ROBOT_LENGTH / 2 + 0.05)
                            if np.all(self._coverage_map[row_min:row_max+1, col_min:col_max+1] != -1.0):
                                min_distance = distance
                                nearest = position
            t = time.time() - t
            rospy.loginfo(f'Explore loop time={t}')
            if min_distance < np.Inf:
                return [nearest[0], nearest[1], current_pose[2]]#np.pi / 2]

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

        corridor_goal_poses = [
            (BOXES_AREA_MAX_X_COORD + 3, 0, -np.pi / 2), 
            (BOXES_AREA_MAX_X_COORD, BOXES_AREA_MIN_Y_COORD, np.pi)
        ]

        # target_box_detect_goal_idx = 0
        y_step_size = (BRIDGE_DETECT_AREA_MAX_Y - BRIDGE_DETECT_AREA_MIN_Y) / 5
        target_box_detect_poses = [
            (2.2, BRIDGE_DETECT_AREA_MIN_Y + offset * y_step_size, np.pi) for offset in range(1, 5)
        ]
        self._target_box_poses = [
            {'pose': (1.0, pose[1], pose[2]), 'digit': None} for pose in target_box_detect_poses
        ]
        self.spawn_objects()

        while not rospy.is_shutdown():
            if self._navigate_state == self.NAV_STATE_DONE:
                continue

            if self._navigate_state == self.NAV_STATE_IDLE:
                if curr_goal_idx < len(corridor_goal_poses):
                    self.send_goal(*corridor_goal_poses[curr_goal_idx])
                    self._curr_goal_type = self.GOAL_TYPE_EXPLORE
                    curr_goal_idx += 1

                elif self._goal_box_poses:
                    current_pose = self.get_current_pose()
                    if current_pose is not None:
                        min_cost = np.Inf
                        target_goal = None
                        target_goal_idx = None
                        for idx, possible_goals in enumerate(self._goal_box_poses):
                            for goal in possible_goals:
                                yaw_diff = abs(current_pose[2] - goal[2])
                                yaw_diff = min(yaw_diff, 2 * np.pi - yaw_diff)
                                dist = np.linalg.norm(np.array(current_pose[0:2]) - np.array(goal[0:2]))
                                cost = dist + yaw_diff
                                if cost < min_cost:
                                    min_cost = cost
                                    target_goal = goal
                                    target_goal_idx = idx

                        if target_goal is not None:
                            self.send_goal(*target_goal)
                            self._goal_box_poses.pop(target_goal_idx)
                            self._curr_goal_type = self.GOAL_TYPE_CAPTURE

                else:
                    if self._coverage_rate < 0.99:
                        next_goal = self._explore()
                        if next_goal is not None:
                            self.send_goal(*next_goal)
                            self._curr_goal_type = self.GOAL_TYPE_EXPLORE

                    elif not self._are_boxes_localised:
                        self._are_boxes_localised = True
                        boxes_grid_map = np.zeros_like(self._coverage_map)
                        boxes_grid_map[0, :] = 1
                        boxes_grid_map[-1, :] = 1
                        boxes_grid_map[:, 0] = 1
                        boxes_grid_map[:, -1] = 1
                        offset = BOX_SIZE / 2
                        for box in self._box_locations:
                            box = np.array(box)
                            box_bottomleft = box - offset
                            box_topright = box + offset
                            row_min, col_min = self._transform_point_to_coverage_map(box_bottomleft)
                            row_max, col_max = self._transform_point_to_coverage_map(box_topright)
                            boxes_grid_map[row_min:row_max+1, col_min:col_max+1] = 1

                        offset = BOX_SIZE / 2 + ROBOT_LENGTH / 2 + IMG_CAPTURE_DIST
                        offset_robot = np.linalg.norm([ROBOT_LENGTH, ROBOT_WIDTH]) / 2
                        for box in self._box_locations:
                            box_left = [box[0] - offset, box[1], 0.0]
                            box_right = [box[0] + offset, box[1], np.pi]
                            box_bottom = [box[0], box[1] - offset, np.pi / 2]
                            box_top = [box[0], box[1] + offset, -np.pi / 2]
                            possible_poses = []
                            for pose in (box_left, box_right, box_bottom, box_top):
                                position = np.array(pose[0:2])
                                area_to_navigate = [position - offset_robot, position + offset_robot]
                                if pose[2] == 0.0:
                                    area_to_navigate[0][0] - IMG_CAPTURE_DIST
                                elif pose[2] == np.pi:
                                    area_to_navigate[1][0] + IMG_CAPTURE_DIST
                                elif pose[2] == np.pi / 2:
                                    area_to_navigate[0][1] - IMG_CAPTURE_DIST
                                elif pose[2] == -np.pi / 2:
                                    area_to_navigate[1][1] + IMG_CAPTURE_DIST
                                row_min, col_min = self._transform_point_to_coverage_map(area_to_navigate[0])
                                row_max, col_max = self._transform_point_to_coverage_map(area_to_navigate[1])
                                if np.all(boxes_grid_map[row_min:row_max+1, col_min:col_max+1] == 0):
                                    possible_poses.append(pose)

                            if possible_poses:
                                self._goal_box_poses.append(possible_poses)
                            else:
                                rospy.logwarn(f'Could not find feasible pose for capturing box at {box}')

                    elif self._detect_bridge_poses:
                        self.send_goal(*self._detect_bridge_poses)
                        self._detect_bridge_poses.clear()
                        rospy.loginfo('Sending goal for detect bridge...')
                        self._prepare_for_bridge_detection = True
                        self._curr_goal_type = self.GOAL_TYPE_DETECT_BRIDGE

                    elif self._curr_goal_type == self.GOAL_TYPE_DETECT_BRIDGE:
                        if self._cross_bridge_goal_pose is not None and self._bridge_detect_start:
                            trigger = Bool()
                            trigger.data = False
                            self._start_bridge_detect_pub.publish(trigger)
                            self._bridge_detect_start = False
                            self._curr_goal_type = self.GOAL_TYPE_EXPLORE

                    elif (self._cross_bridge_goal_pose is not None and
                        self._cross_bridge_pose_idx < len(self._cross_bridge_goal_pose)):
                        if self._cross_bridge_pose_idx == 2:
                            cmd = Bool()
                            cmd.data = True
                            self._bridge_open_pub.publish(cmd)
                            rospy.sleep(1)

                        self.send_goal(*self._cross_bridge_goal_pose[self._cross_bridge_pose_idx])
                        self._curr_goal_type = self.GOAL_TYPE_CROSS_BRIDGE
                        self._cross_bridge_pose_idx += 1

                    elif self._num_target_box_detected < len(self._target_box_poses):
                        # if target_box_detect_goal_idx < len(target_box_detect_poses):
                        self.send_goal(*target_box_detect_poses[self._num_target_box_detected])
                        self._curr_goal_type = self.GOAL_TYPE_CAPTURE_TARGET
                        # target_box_detect_goal_idx += 1

                    else:
                        if self._least_occ is not None:
                            for target in self._target_box_poses:
                                if self._least_occ == target['digit']:
                                    self.send_goal(*target['pose'])
                        else:
                                self._curr_goal_type = self.GOAL_TYPE_RESULT

            elif self._navigate_state == self.NAV_STATE_NAVIGATING:
                if (self._goal_status in (2, 3, 4, 5, 8, 9) or
                    self._check_goal_reached(*self._current_goal)):
                    if self._curr_goal_type in (self.GOAL_TYPE_EXPLORE, self.GOAL_TYPE_DETECT_BRIDGE, self.GOAL_TYPE_CROSS_BRIDGE):
                        if (self._curr_goal_type == self.GOAL_TYPE_DETECT_BRIDGE and
                            not self._bridge_detect_start):
                            current_pose = self.get_current_pose()
                            if current_pose is not None:
                                if (current_pose[0] >= BRIDGE_DETECT_AREA_MIN_X and
                                    current_pose[1] >= BRIDGE_DETECT_AREA_MIN_Y and
                                    current_pose[0] <= BRIDGE_DETECT_AREA_MAX_X and
                                    current_pose[1] <= BRIDGE_DETECT_AREA_MAX_Y):
                                    if self._prepare_for_bridge_detection:
                                        rospy.loginfo('Starting bridge detection...')
                                        trigger = Bool()
                                        trigger.data = True
                                        self._start_bridge_detect_pub.publish(trigger)
                                        self._bridge_detect_start = True
                                        self._prepare_for_bridge_detection = False
                                    else:
                                        rospy.loginfo('Sending goal2 for detect bridge...')
                                        self._curr_goal_type = self.GOAL_TYPE_DETECT_BRIDGE
                                        self.send_goal(self._detect_bridge_poses)
                                        self._prepare_for_bridge_detection = True

                        self._navigate_state = self.NAV_STATE_IDLE

                    elif self._curr_goal_type in (self.GOAL_TYPE_CAPTURE, self.GOAL_TYPE_CAPTURE_TARGET):
                        self._navigate_state = self.NAV_STATE_CAPTURING

            elif self._navigate_state == self.NAV_STATE_GOAL_SENT:
                if self._goal_status == 1:
                    self._navigate_state = self.NAV_STATE_NAVIGATING

            pose_print_timer += 1
            if pose_print_timer >= (rate / pose_print_rate):
                current_pose = self.get_current_pose()
                if current_pose is not None:
                    rospy.loginfo(f'Current Pose (tf2): x={current_pose[0]}, y={current_pose[1]}, yaw={current_pose[2]}')
                    rospy.loginfo(f'Coverage rate: {self._coverage_rate} nav_state={self._navigate_state}')
                pose_print_timer = 0

            cov_map_update_timer += 1
            if cov_map_update_rate >= (rate / cov_map_update_rate):
                self._update_visited_area()
                cov_map_update_timer = 0

            loop_rate.sleep()


if __name__ == '__main__':
    area = [(BOXES_AREA_MIN_X_COORD, BOXES_AREA_MIN_Y_COORD),
            (BOXES_AREA_MAX_X_COORD, BOXES_AREA_MAX_Y_COORD)]
    coverage_planner = SimpleCoveragePlanner(area)
    coverage_planner.mainloop()
