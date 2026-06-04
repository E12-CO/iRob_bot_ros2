# Trajectory smoother with save to CSV capabilities
# Thanks god ChatGPT can help me write python

import csv
import math
import sys

import transforms3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from irob_msgs.msg import IrobCmdMsg

import numpy as np
from ccma import CCMA

import os, sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class PathInterpolator(Node):
    def __init__(self):
        super().__init__('path_interpolator', namespace='r1')
        self.linearPreviewPub_  = self.create_publisher(Path, 'interpolated_path', 10)
        self.smoothPreviewPub_  = self.create_publisher(Path, 'smooth_path', 10)
        self.smoothPathPub_     = self.create_publisher(Path, 'path', 10) 
        self.subscription_ = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.pose_callback,
            10
        )
        
        """
        Communication between this python backend node and the RViz frontend panel node
        """
        self.pubToPanelNode     =   self.create_publisher(IrobCmdMsg, '/irob_backend_to_panel', 10)
        self.subFromPanelNode   =   self.create_subscription(
                IrobCmdMsg,
                '/irob_panel_to_backend',
                self.panel_callback,
                10
            )
        
        self.poseArray = Path()
        self.linearPath_msg = Path()
        self.smoothPath_msg = Path()
        self.linearPath_msg.header.frame_id = 'map'
        self.ccma = CCMA(w_ma=300, w_cc=3)

        self.poseNumeber = 0
        self.totalPosePoints = 0

        self.pathCSVName = ''

        self.last_point = None
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Robot Club Engineering KMITL : starting iRob trajectory maker...")

    """
    Support code
    """
    def quat_to_yaw(self, x, y, z, w):
        (row, pitch, yaw) = transforms3d.euler.quat2euler([w, x, y, z], 'sxyz')
        return yaw
        
    def yaw_to_quat(self, yaw):
        (w, x, y, z) = transforms3d.euler.euler2quat(0, 0, yaw,'sxyz')
        return w, x, y, z
        
    """
    CSV Reader/Writer
    """
    def csv_reader(self):
        self.poseArray = Path()
        with open(self.pathCSVName, mode='r', newline='\n') as csvFile:
            reader = csv.DictReader(csvFile)
            for rawPose in reader:
                readPoseStamped = PoseStamped()
                readPoseStamped.pose.position.x = float(rawPose['x'])
                readPoseStamped.pose.position.y = float(rawPose['y'])
                q = self.yaw_to_quat(
                        float(rawPose['yaw'])
                    )
                readPoseStamped.pose.orientation.x = q[1]
                readPoseStamped.pose.orientation.y = q[2]
                readPoseStamped.pose.orientation.z = q[3]
                readPoseStamped.pose.orientation.w = q[0]

                self.poseArray.poses.append(readPoseStamped)
                   
        csvFile.close()
        self.pose_interpolatePoints()
        
    def csv_writer(self):
        with open(self.pathCSVName, mode='w', newline='\n') as csvFile:
            
            writer = csv.DictWriter(csvFile, fieldnames=['x', 'y', 'yaw'])
            writer.writeheader()
            for rawPose in self.poseArray.poses:
                writer.writerow(
                    {'x': round(rawPose.pose.position.x, 3), 
                    'y': round(rawPose.pose.position.y, 3),
                    'yaw': round(self.quat_to_yaw(
                        rawPose.pose.orientation.x,
                        rawPose.pose.orientation.y,
                        rawPose.pose.orientation.z,
                        rawPose.pose.orientation.w
                    ), 4)}
                )
                
        csvFile.close()        
                
    """
    ROS callback handlers
    """
    def pose_callback(self, msg):
        """
        Callback function for processing incoming PoseStamped messages.
        """
        self.get_logger().info('Received Pose')
        self.poseNumeber = self.poseNumeber + 1
        self.poseArray.poses.append(msg)
        self.pose_interpolatePoints()

    def panel_callback(self, panelMsg):
        self.get_logger().debug('Received command from panel')
        
        if panelMsg.irobcmd.find(',') != -1:
            panelCommand, panelData = panelMsg.irobcmd.split(',')
        else:
            panelCommand = panelMsg.irobcmd
            
        if panelCommand == 'undo':
            # Undo the last pose
            if len(self.poseArray.poses) > 0:
                self.poseNumeber = self.poseNumeber - 1
                self.get_logger().info("Undoing last point")
                self.poseArray.poses.pop()
                self.pose_interpolatePoints() # Update the path
            
        elif panelCommand == 'pub':
            # publish path to topic
            if len(self.linearPath_msg.poses) < 2:
                return

            # Smoothing the path
            self.get_logger().info('Publishing smooth Path...')
            self.smoothPathPub_.publish(self.smoothPath_msg)
            
        elif panelCommand =='clr':
            # Clear all pose
            self.get_logger().info("Clear current Path...")
            self.linearPath_msg.poses.clear()
            self.last_point = None
            
        elif panelCommand == 'save':
            # save pose waypoint to CSV
            self.get_logger().info('Saving the waypoint pose...')
            self.pathCSVName = panelData
            self.csv_writer()
            
        elif panelCommand == 'open':
            # open pose waypoint csv file
            self.get_logger().info('Opening the waypoint pose...')
            self.pathCSVName = panelData
            self.csv_reader()
            
        else:
            self.get_logger().error('Unknown command!')

    """
    Path interpolator and smoother
    """
    def interpolate_points(self, start, end, num_points):
        x_values = np.linspace(start[0], end[0], num_points + 2)
        y_values = np.linspace(start[1], end[1], num_points + 2)
        yaw_values = np.linspace(start[2], end[2], num_points + 2)
        return [(x, y, yaw) for x, y, yaw in zip(x_values, y_values, yaw_values)]

    def pose_publishPreview(self):
        self.linearPath_msg.header.stamp = self.get_clock().now().to_msg()
        self.smoothPath_msg.header.stamp = self.linearPath_msg.header.stamp
        self.linearPreviewPub_.publish(self.linearPath_msg)
        self.smoothPreviewPub_.publish(self.smoothPath_msg)
        
    def pose_interpolatePoints(self):
        if len(self.poseArray.poses) < 2:
            self.linearPath_msg.poses.clear()
            self.smoothPath_msg.poses.clear()
            self.linearPath_msg.header.stamp = self.get_clock().now().to_msg()
            self.pose_publishPreview()
            return
    
        # Clear previous poses
        self.linearPath_msg.poses.clear()
        self.smoothPath_msg.poses.clear()
    
        for i in range(len(self.poseArray.poses) - 1):
            current_point = (
                self.poseArray.poses[i].pose.position.x, 
                self.poseArray.poses[i].pose.position.y,
                self.quat_to_yaw(
                    self.poseArray.poses[i].pose.orientation.x,
                    self.poseArray.poses[i].pose.orientation.y,
                    self.poseArray.poses[i].pose.orientation.z,
                    self.poseArray.poses[i].pose.orientation.w
                )
                )
        
            next_point = (
                self.poseArray.poses[i+1].pose.position.x, 
                self.poseArray.poses[i+1].pose.position.y,
                self.quat_to_yaw(
                    self.poseArray.poses[i+1].pose.orientation.x,
                    self.poseArray.poses[i+1].pose.orientation.y,
                    self.poseArray.poses[i+1].pose.orientation.z,
                    self.poseArray.poses[i+1].pose.orientation.w
                )
                )

            interpolated_points = self.interpolate_points(current_point, next_point, num_points=50)

            for lPose in interpolated_points:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = lPose[0]
                pose.pose.position.y = lPose[1]
                pose.pose.position.z = 0.0
                q = transforms3d.euler.euler2quat(0, 0, lPose[2], 'sxyz')
                pose.pose.orientation.x = q[1]
                pose.pose.orientation.y = q[2]
                pose.pose.orientation.z = q[3]
                pose.pose.orientation.w = q[0]
                self.linearPath_msg.poses.append(pose)

            print("Interpolated path length:", len(self.linearPath_msg.poses))

        # Update the smooth path
        self.pose_generateSmoothPath()
    
    def pose_generateSmoothPath(self):
        x_list = []
        y_list = []
        for xy_point in self.linearPath_msg.poses:
            x_list.append(xy_point.pose.position.x)
            y_list.append(xy_point.pose.position.y)

        smooth_path = self.ccma.filter(np.column_stack([x_list,y_list]), cc_mode=False)
        
        self.smoothPath_msg.poses.clear() # Clear all previous path before regenerating them
        self.smoothPath_msg.header.frame_id = 'map'
        
        # Insert initial point
        smoothPose = PoseStamped()
        smoothPose.header.frame_id = 'map'
        smoothPose.header.stamp = self.get_clock().now().to_msg()
        smoothPose.pose.position.x = self.linearPath_msg.poses[0].pose.position.x
        smoothPose.pose.position.y = self.linearPath_msg.poses[0].pose.position.y
        self.smoothPath_msg.poses.append(smoothPose)

        for smooth_pose in smooth_path:
            smoothPose = PoseStamped()
            smoothPose.header.frame_id = 'map'
            smoothPose.header.stamp = self.get_clock().now().to_msg()
            smoothPose.pose.position.x = smooth_pose[0]
            smoothPose.pose.position.y = smooth_pose[1]
            self.smoothPath_msg.poses.append(smoothPose)

        # Insert last point
        smoothPose.header.stamp = self.get_clock().now().to_msg()
        smoothPose.pose.position.x = self.linearPath_msg.poses[-1].pose.position.x
        smoothPose.pose.position.y = self.linearPath_msg.poses[-1].pose.position.y
        self.smoothPath_msg.poses.append(smoothPose)

        # Copy YAW
        self.smoothPath_msg.poses[0].pose.orientation = self.linearPath_msg.poses[0].pose.orientation

        for yaw_pose_idx in range(0, len(self.linearPath_msg.poses)):
            self.smoothPath_msg.poses[yaw_pose_idx+1].pose.orientation = self.linearPath_msg.poses[yaw_pose_idx].pose.orientation

        self.smoothPath_msg.poses[-1].pose.orientation = self.linearPath_msg.poses[-1].pose.orientation

        self.totalPosePoints = len(self.smoothPath_msg.poses)
        print("Smooth path length:", self.totalPosePoints)
        
        # Publish the updated review path
        self.pose_publishPreview()

    def timer_callback(self):
        # Keypress detection
        
        #self.pubToPanelNode.publish()
        return

def main(args=None):
    rclpy.init()

    node = PathInterpolator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

