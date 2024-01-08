#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2023  Alapaca-zip
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import os
import sys

#import mip
import rospy
import tf
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from PIL import Image, ImageDraw
from sensor_msgs.msg import LaserScan


class VgraphPlannerNode:
    def __init__(self):
        self.map_file_path = rospy.get_param("~map_file", "map.yaml")
        self.test_folder_path = rospy.get_param("~test_folder", "test")
        self.down_scale_factor = rospy.get_param("~down_scale_factor", 0.1)
        self.robot_radius = rospy.get_param("~robot_radius", 0.1)
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.vel_linear = rospy.get_param("~vel_linear", 0.1)
        self.vel_theta = rospy.get_param("~vel_theta", 0.1)
        self.angle_tolerance = rospy.get_param("~angle_tolerance", 0.1)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.1)
        self.verbose = rospy.get_param("~verbose", False)
        self.start_point = None
        self.end_point = None

        self.costmap_pub = rospy.Publisher("/cost_map", OccupancyGrid, queue_size=10)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.initial_pose_callback
        )
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.original_image, self.resolution, self.origin = self.load_map_file(
            self.map_file_path
        )

        (
            self.enlarged_image,
            self.down_scaled_image,
            self.up_scaled_image,
        ) = self.change_image_resolution(self.original_image, self.down_scale_factor)

        self.publish_initial_cost_map()

    def __del__(self):
        pass

    def load_map_file(self, yaml_file_path):
        with open(yaml_file_path, "r") as file:
            map_yaml = yaml.safe_load(file)

        directory = os.path.dirname(yaml_file_path)
        image_file_name = map_yaml.get("image", None)

        if image_file_name is not None:
            image_path = os.path.join(directory, image_file_name)
        else:
            image_path = None

        image = Image.open(image_path)
        resolution = map_yaml.get("resolution", None)
        origin = map_yaml.get("origin", None)

        return image, resolution, origin

    def change_image_resolution(self, image, factor):
        width = int(image.width * factor)
        height = int(image.height * factor)
        black_pixels = self.find_black_pixels(image)

        enlarged_image, black_pixels = self.enlarge_black_pixels(
            image, black_pixels, self.robot_radius
        )

        down_scaled_image = enlarged_image.resize(
            (width, height), Image.Resampling.NEAREST
        )

        down_scaled_image = self.apply_black_pixels(
            down_scaled_image, black_pixels, factor
        )

        up_scaled_image = down_scaled_image.resize(image.size, Image.Resampling.NEAREST)

        return enlarged_image, down_scaled_image, up_scaled_image

    def find_black_pixels(self, image):
        black_pixels = []

        for y in range(image.height):
            for x in range(image.width):
                if image.getpixel((x, y)) == 0:
                    black_pixels.append((x, y))

        return black_pixels

    def enlarge_black_pixels(self, image, black_pixels, width):
        new_image = image.copy()
        pixel_width = math.ceil(width / self.resolution)
        pixel_width = pixel_width + 1
        new_black_pixels = set(black_pixels)

        for x, y in black_pixels:
            for dx in range(-pixel_width, pixel_width + 1):
                for dy in range(-pixel_width, pixel_width + 1):
                    selected_x, selected_y = x + dx, y + dy
                    if (
                        0 <= selected_x < new_image.width
                        and 0 <= selected_y < new_image.height
                    ):
                        if (selected_x, selected_y) not in new_black_pixels:
                            new_image.putpixel((selected_x, selected_y), 0)
                            new_black_pixels.add((selected_x, selected_y))

        return new_image, new_black_pixels

    def apply_black_pixels(self, image, black_pixels, scale):
        for x, y in black_pixels:
            scaled_x = int(x * scale)
            scaled_y = int(y * scale)
            if 0 <= scaled_x < image.width and 0 <= scaled_y < image.height:
                image.putpixel((scaled_x, scaled_y), 0)

        return image

    def publish_initial_cost_map(self):
        while True:
            if (
                self.up_scaled_image is not None
                and self.costmap_pub.get_num_connections() > 0
            ):
                self.publish_cost_map(self.up_scaled_image)
                break
            rospy.sleep(5.0)

    def publish_cost_map(self, original_image, edges=None):
        rgb_image = original_image.convert("RGB")
        draw = ImageDraw.Draw(rgb_image)

        if edges is not None:
            for start, end in edges:
                draw.line([start, end], fill=(128, 128, 128), width=1)

        cost_map_msg = OccupancyGrid()
        cost_map_msg.header.frame_id = "map"
        cost_map_msg.header.stamp = rospy.Time.now()
        cost_map_msg.info.resolution = self.resolution
        cost_map_msg.info.width = rgb_image.width
        cost_map_msg.info.height = rgb_image.height
        cost_map_msg.info.origin.position.x = self.origin[0]
        cost_map_msg.info.origin.position.y = self.origin[1]
        cost_map_msg.info.origin.orientation.w = 1.0
        cost_map_msg.data = []

        for y in reversed(range(rgb_image.height)):
            for x in range(rgb_image.width):
                original_pixel = original_image.getpixel((x, y))
                rgb_pixel = rgb_image.getpixel((x, y))
                if original_pixel == 0 and rgb_pixel == (128, 128, 128):
                    cost_map_msg.data.append(50)
                elif original_pixel == 0:
                    cost_map_msg.data.append(100)
                else:
                    cost_map_msg.data.append(0)

        self.costmap_pub.publish(cost_map_msg)

    def initial_pose_callback(self, msg):
        self.start_point = self.pose_to_pixel(
            (msg.pose.pose.position.x, msg.pose.pose.position.y)
        )
        rospy.loginfo("Set initial pose.")

    def goal_callback(self, msg):
        if self.start_point is None:
            current_position = self.get_current_position()
            if current_position is not None:
                self.start_point = self.pose_to_pixel(current_position)
                rospy.loginfo("Automatically set initial pose.")
            else:
                rospy.logwarn(
                    "Cannot set initial pose automatically. Please set initial pose manually."
                )
                return
        elif self.up_scaled_image is None:
            rospy.logwarn("Map is not loaded. Please wait for the map to be loaded.")
            return

        self.end_point = self.pose_to_pixel((msg.pose.position.x, msg.pose.position.y))
        rospy.loginfo("Set goal.")

        path = self.make_plan(self.start_point, self.end_point)

        self.navigate_along_path(path, msg)

        self.start_point = None
        self.end_point = None

    def odom_callback(self, msg):
        self.current_odom = msg

    def scan_callback(self, msg):
        self.scan_data = msg.ranges
        self.range_min = msg.range_min

    def pose_to_pixel(self, pose):
        origin_x = self.origin[0]
        origin_y = self.origin[1]
        pixel_x = int(round((pose[0] - origin_x) / self.resolution))
        pixel_y = self.original_image.height - int(
            round((pose[1] - origin_y) / self.resolution)
        )

        return (pixel_x, pixel_y)

    def pixel_to_pose(self, pixel):
        origin_x = self.origin[0]
        origin_y = self.origin[1]
        pose_x = origin_x + (pixel[0] + 0.5) * self.resolution
        pose_y = (
            origin_y + (self.original_image.height - pixel[1] - 0.5) * self.resolution
        )

        return (pose_x, pose_y)

    def make_plan(self, start, goal):
        rospy.loginfo("[Make Plan] Start planning.")
        self.animation_running = True
        timer = rospy.Timer(rospy.Duration(0.1), self.animate_loading)
        corners = []
        corners.append(start)
        corners = self.find_black_pixel_corners(self.up_scaled_image, corners)
        corners.append(goal)

        path_edges = self.get_path_edges(self.up_scaled_image, corners)

        if path_edges is not None:
            path_info = self.get_path(path_edges)

            self.publish_path(path_info)

            self.publish_cost_map(self.up_scaled_image, path_info)

            path_graph_image = self.draw_path_with_markers(
                self.up_scaled_image, path_info
            )

            self.original_image.save(self.test_folder_path + "/original.png")
            self.enlarged_image.save(self.test_folder_path + "/enlarged.png")
            self.down_scaled_image.save(self.test_folder_path + "/down_scaled.png")
            self.up_scaled_image.save(self.test_folder_path + "/up_scaled.png")
            path_graph_image.save(self.test_folder_path + "/path_graph.png")

            self.animation_running = False
            timer.shutdown()
            sys.stdout.write("\r" + " " * 30 + "\r")
            rospy.loginfo("[Make Plan] Finished planning successfully.")

        return path_info

    def animate_loading(self, event):
        if self.animation_running:
            chars = "/—\\|"
            char = chars[int(event.current_real.to_sec() * 10) % len(chars)]
            sys.stdout.write("\r" + "Planning... " + char)
            sys.stdout.flush()

    def find_black_pixel_corners(self, image, corners):
        for y in range(1, image.height - 1):
            for x in range(1, image.width - 1):
                if image.getpixel((x, y)) == 0:
                    black_neighbors = sum(
                        image.getpixel((j, i)) == 0
                        for i in range(y - 1, y + 2)
                        for j in range(x - 1, x + 2)
                        if (i, j) != (y, x)
                    )
                    if black_neighbors == 3:
                        corners.append((x, y))

        return corners
    
    def get_path_edges(self, image, corners):
        path_edges = []
        path_edges.append(corners[0])
        edges = []

        if not self.path_white(image, corners):
            x0, y0 = corners[0]
            x1, y1 = corners[-1]
            del corners[0]

            dx = x1 - x0
            dy = y1 - y0
            print(x0, y0)
            print(x1, y1)
            print(dx, dy)

            for i in range(len(corners)):
                x, y = corners[i]

                if dx >= 0 and dy >= 0:
                    if dx >= dy and x0 <= x and x <= x1 and y0 <= y and y <= y1 + 10:
                        path_edges.append((x,y))
                        rospy.loginfo("get path edges a.")
                        print(x,y)
                        break
                    elif dx < dy and x0 <= x and x <= x1 + 10 and y0 <= y and y <= y1:
                        path_edges.append((x,y))
                        rospy.loginfo("get path edges b.")
                        print(x,y)
                        break

                elif dx < 0 and dy >= 0:
                    if -dx >= dy and x1 <= x and x <= x0 and y0 <= y and y <= y1 + 10:
                        path_edges.append((x,y))
                        rospy.loginfo("get path edges c.")
                        print(x,y)
                        break
                    elif -dx < dy and x1 - 10 <= x and x <= x0 and y0 <= y and y <= y1:
                        path_edges.append((x,y))
                        rospy.loginfo("get path edges d.")
                        print(x,y)
                        break

                elif dx < 0 and dy < 0:
                    if -dx >= -dy and x1 <= x and x <= x0 and y1 - 10 <= y and y <= y0:
                        path_edges.append((x,y))
                        rospy.loginfo("get path edges e.")
                        print(x,y)
                        break
                    elif -dx < -dy and x1 - 10 <= x and x <= x0 + 10 and y1 <= y and y <= y0:
                        path_edges.append((x,y))
                        rospy.loginfo("get path edges f.")
                        print(x,y)
                        break

                elif dx >= 0 and dy < 0:
                    if dx >= -dy and x0 <= x and x <= x1 and y1 - 10 <= y and y <= y0:
                        path_edges.append((x,y))
                        rospy.loginfo("get path edges g.")
                        print(x,y)
                        break
                    elif dx < -dy and x0 <= x and x <= x1 + 10 and y1 <= y and y <= y0:
                        path_edges.append((x,y))
                        rospy.loginfo("get path edges h.")
                        print(x,y)
                        break
        
        path_edges.append(corners[-1])

        return path_edges
    
    def path_white(self, image, corners):
        x0, y0 = corners[0]
        x1, y1 = corners[-1]
        dx = x1 - x0
        dy = y1 - y0
        black_sum = 0

        for i in range(10):
            x = x0 + round(dx * i / 10)
            y = y0 + round(dy * i / 10)
            if image.getpixel((x,y)) == 0:
                black_sum += 1
                
        if black_sum > 1:
            rospy.loginfo("False.")
            return False
        
        rospy.loginfo("True.")

        return True

# def 6

    def get_path(self, edges):
        path_info = []
        for i in range(len(edges) - 1):
            path_info.append((edges[i], edges[i + 1]))

        return path_info

    def publish_path(self, aligned_edges):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        start_edge = aligned_edges[0]
        start_pose = self.pixel_to_pose((start_edge[0][0], start_edge[0][1]))

        start_pose_stamped = PoseStamped()
        start_pose_stamped.header = path_msg.header
        start_pose_stamped.pose.position.x = start_pose[0]
        start_pose_stamped.pose.position.y = start_pose[1]
        start_pose_stamped.pose.orientation.w = 1.0
        path_msg.poses.append(start_pose_stamped)

        for edge in aligned_edges:
            end_pose = self.pixel_to_pose((edge[1][0], edge[1][1]))

            end_pose_stamped = PoseStamped()
            end_pose_stamped.header = path_msg.header
            end_pose_stamped.pose.position.x = end_pose[0]
            end_pose_stamped.pose.position.y = end_pose[1]
            end_pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(end_pose_stamped)

        self.path_pub.publish(path_msg)

    def draw_path_with_markers(self, image, aligned_edges):
        rgb_image = image.convert("RGB")
        draw = ImageDraw.Draw(rgb_image)

        for start, end in aligned_edges:
            draw.line([start, end], fill=(255, 0, 0), width=1)

        start_point = aligned_edges[0][0]
        end_point = aligned_edges[-1][1]
        radius = 5
        draw.ellipse(
            (
                start_point[0] - radius,
                start_point[1] - radius,
                start_point[0] + radius,
                start_point[1] + radius,
            ),
            fill=(0, 255, 0),
        )
        draw.ellipse(
            (
                end_point[0] - radius,
                end_point[1] - radius,
                end_point[0] + radius,
                end_point[1] + radius,
            ),
            fill=(0, 0, 255),
        )

        return rgb_image

    def navigate_along_path(self, edges, final_pose):
        rospy.loginfo("[Navigate] Start navigation.")
        if edges is None or self.current_odom is None or self.scan_data is None:
            return

        for edge in edges:
            rospy.loginfo(f"[Navigate] Moving to waypoint: {edge[1]}.")
            if not self.move_to_goal(edge):
                rospy.logwarn("Aborting navigation.")
                return
            rospy.loginfo("[Navigate] Reached waypoint.")

        if final_pose is not None:
            if not self.rotate_to_final_pose(final_pose):
                rospy.logwarn("Aborting navigation.")
                return
            rospy.loginfo("[Navigate] Reached final pose.")

    def move_to_goal(self, edge):
        current_position = self.get_current_position()
        if current_position is None:
            return False
        end_position = self.pixel_to_pose((edge[1][0], edge[1][1]))
        goal_distance = math.sqrt(
            (end_position[0] - current_position[0]) ** 2
            + (end_position[1] - current_position[1]) ** 2
        )

        while goal_distance > self.goal_tolerance:
            if not self.rotate_to_goal(edge):
                return False

            current_position = self.get_current_position()
            if current_position is None:
                return False
            goal_distance = math.sqrt(
                (end_position[0] - current_position[0]) ** 2
                + (end_position[1] - current_position[1]) ** 2
            )

            if self.check_obstacle():
                rospy.logwarn_throttle(3.0, "Obstacle detected. Stopping the robot.")
                self.send_stop()
                continue

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.vel_linear
            self.cmd_vel_pub.publish(cmd_vel_msg)

            rospy.sleep(0.1)

        self.send_stop()
        return True

    def rotate_to_goal(self, edge):
        rotate_flag = False
        current_position = self.get_current_position()
        if current_position is None:
            return False
        end_position = self.pixel_to_pose((edge[1][0], edge[1][1]))
        goal_angle = math.atan2(
            end_position[1] - current_position[1], end_position[0] - current_position[0]
        )
        current_angle = self.get_current_angle()
        if current_angle is None:
            return False
        diff_angle = self.normalize_angle(goal_angle - current_angle)
        current_speed = self.vel_theta

        while abs(diff_angle) > self.angle_tolerance:
            rotate_flag = True
            current_angle = self.get_current_angle()
            if current_angle is None:
                return False
            diff_angle = self.normalize_angle(goal_angle - current_angle)
            rotation_time = abs(diff_angle) / current_speed
            rotation_direction = 1 if diff_angle > 0 else -1

            if self.check_obstacle():
                rospy.logwarn_throttle(3.0, "Obstacle detected. Stopping the robot.")
                self.send_stop()
                continue

            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = rotation_direction * current_speed
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(rotation_time)

            current_speed *= 0.8

        if rotate_flag:
            self.send_stop()

        return True

    def rotate_to_final_pose(self, final_pose):
        final_angle = tf.transformations.euler_from_quaternion(
            [
                final_pose.pose.orientation.x,
                final_pose.pose.orientation.y,
                final_pose.pose.orientation.z,
                final_pose.pose.orientation.w,
            ]
        )[2]
        current_angle = self.get_current_angle()
        if current_angle is None:
            return False
        diff_angle = self.normalize_angle(final_angle - current_angle)
        current_speed = self.vel_theta

        while abs(diff_angle) > self.angle_tolerance:
            current_angle = self.get_current_angle()
            if current_angle is None:
                return False
            diff_angle = self.normalize_angle(final_angle - current_angle)
            rotation_time = abs(diff_angle) / current_speed
            rotation_direction = 1 if diff_angle > 0 else -1

            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = rotation_direction * current_speed
            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(rotation_time)

            current_speed *= 0.8

        self.send_stop()
        return True

    def send_stop(self):
        start_time = rospy.get_time()
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0

        while True:
            self.cmd_vel_pub.publish(cmd_vel_msg)

            current_velocity = math.sqrt(
                self.current_odom.twist.twist.linear.x**2
                + self.current_odom.twist.twist.linear.y**2
                + self.current_odom.twist.twist.angular.z**2
            )

            if current_velocity < 0.01:
                break
            elif rospy.get_time() - start_time > 10:
                rospy.logwarn_throttle(
                    3.0,
                    "It took more than 10 seconds to stop. Please check if the odometry is working properly.",
                )

            rospy.sleep(0.1)

    def check_obstacle(self):
        if self.robot_radius < self.range_min:
            rospy.logwarn(
                "robot_radius is smaller than range_min. Please check the parameters."
            )
            return False

        for distance in self.scan_data:
            if self.range_min < distance < self.robot_radius:
                return True

        return False

    def get_current_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                self.current_odom.child_frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except:
            rospy.logwarn("Failed to get current position.")
            return None

    def get_current_angle(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                self.current_odom.child_frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            quaternion = transform.transform.rotation
            euler = tf.transformations.euler_from_quaternion(
                [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            )

            return euler[2]
        except:
            rospy.logwarn("Failed to get current angle.")
            return None

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi

        return angle


def main():
    rospy.init_node("vgraph_planner_node")
    VgraphPlannerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
