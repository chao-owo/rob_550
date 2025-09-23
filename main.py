#!/usr/bin/env python3

"""!
Class to represent the camera.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import argparse
import sys
import cv2
import time
import numpy as np
from PyQt5.QtGui import QImage
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError


class Camera():
    """!
    @brief      This class describes a camera.
    """

    def __init__(self):
        """!
        @brief      Construcfalsets a new instance.
        """
        self.VideoFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.GridFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.TagImageFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.DepthFrameRaw = np.zeros((720, 1280)).astype(np.uint16)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.DepthFrameRGB = np.zeros((720, 1280, 3)).astype(np.uint8)

        # mouse clicks & calibration variables
        self.camera_calibrated = False
        self.intrinsic_matrix = np.matrix([[917.426899, 0.0, 609.600747],
                                           [0.0, 913.808287, 390.655705],
                                           [0.0, 0.0, 1.0]])
        # self.extrinsic_matrix = np.eye(4)
        x_rot = np.pi * 190 / 180
        R = np.matrix([[1.0, 0.0, 0.0],
                       [0.0, np.cos(x_rot), -1 * np.sin(x_rot)],
                       [0.0, np.sin(x_rot), np.cos(x_rot)]])
        self.extrinsic_matrix = np.matrix([[R[0, 0], R[0, 1], R[0, 2], 10.0],
                                           [R[1, 0], R[1, 1], R[1, 2], 150.0],
                                           [R[2, 0], R[2, 1], R[2, 2], 1035.0],
                                           [0.0, 0.0, 0.0, 1.0]])

        self.homography_matrix = np.eye(3)
        self.last_click = np.array([0, 0])  # This contains the last clicked position
        self.new_click = False  # This is automatically set to True whenever a click is received. Set it to False yourself after processing a click
        self.rgb_click_points = np.zeros((5, 2), int)
        self.depth_click_points = np.zeros((5, 2), int)
        self.grid_x_points = np.arange(-450, 500, 50)
        self.grid_y_points = np.arange(-175, 525, 50)
        self.grid_points = np.array(np.meshgrid(self.grid_x_points, self.grid_y_points))
        self.tag_detections = np.array([])
        self.tag_locations = [[-250, -25], [250, -25], [250, 275], [-250, 275]]
        """ block info """
        self.block_contours = np.array([])
        self.block_detections = np.array([])

        self.color_range = {
            'red': [([0, 50, 20], [10, 255, 255]), ([160, 50, 20], [180, 255, 255])],
            'green': [([35, 40, 20], [85, 255, 255])],
            'blue': [([90, 50, 20], [150, 255, 255])]
        }

        self.min_depth = 0
        self.max_depth = 1100
        self.color_dict={}

    def depthVisual(self):
        """
        darker as the value of depth is larger
        """
        # range
        depth_range = np.clip(self.DepthFrameRaw, self.min_depth, self.max_depth)

        # 0~255
        depth_normalized = ((depth_range - self.min_depth) /(self.max_depth - self.min_depth) * 255).astype(np.uint8)

        self.DepthFrameRGB = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        # depth in GUI?

        return depth_normalized

    def detectEdgesFromDepthCanny(self, low_threshold=50, high_threshold=150, min_area=50):
        """
        min_area: filter out noise
        """
        depth_normalized = self.depthVisual()
        edges = cv2.Canny(depth_normalized, low_threshold, high_threshold)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        centers = []
        for contour in contours:
            if cv2.contourArea(contour) < min_area:
                continue
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append((cx, cy)) # pixel coordinate

        return centers, edges

    def identify_colors_at_centers(self, centers, box_size=5):
        """
        box_size: range around center
        return list of (cx, cy, color)
        """
        frame = self.VideoFrame
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        results = []
        for cx, cy in centers:
            x1 = max(cx - box_size // 2, 0)
            y1 = max(cy - box_size // 2, 0)
            x2 = min(cx + box_size // 2 + 1, frame.shape[1])
            y2 = min(cy + box_size // 2 + 1, frame.shape[0])

            box = hsv[y1:y2, x1:x2]

            detected_color = None
            for color_name, ranges in self.color_range.items():
                for lower, upper in ranges:
                    lower_np = np.array(lower)
                    upper_np = np.array(upper)
                    mask = cv2.inRange(box, lower_np, upper_np)
                    if np.any(mask > 0):
                        detected_color = color_name
                        break
                if detected_color is not None:
                    break
            self.color_dict[detected_color].append((cx, cy))

        return self.color_dict


    def processVideoFrame(self):
        """!
        @brief      Process a video frame
        """


    def ColorizeDepthFrame(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        self.DepthFrameHSV[..., 0] = self.DepthFrameRaw >> 1
        self.DepthFrameHSV[..., 1] = 0xFF
        self.DepthFrameHSV[..., 2] = 0x9F
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV,
                                          cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread("data/rgb_image.png", cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB)

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread("data/raw_depth.png",
                                        0).astype(np.uint16)

    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """
        try:
            frame = cv2.resize(self.VideoFrame, (1280, 720))
            frame = cv2.warpPerspective(frame, self.homography_matrix, (frame.shape[1], frame.shape[0]))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtGridFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.GridFrame, (1280, 720))
            frame = cv2.warpPerspective(frame, self.homography_matrix, (frame.shape[1], frame.shape[0]))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtDepthFrame(self):
        """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
        try:
            frame = frame = cv2.resize(self.DepthFrameRGB, (1280, 720))
            frame = cv2.warpPerspective(frame, self.homography_matrix, (frame.shape[1], frame.shape[0]))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtTagImageFrame(self):
        """!
        @brief      Converts tag image frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.TagImageFrame, (1280, 720))
            frame = cv2.warpPerspective(frame, self.homography_matrix, (frame.shape[1], frame.shape[0]))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

        @param      coord1  Points in coordinate frame 1
        @param      coord2  Points in coordinate frame 2

        @return     Affine transform between coordinates.
        """
        pts1 = coord1[0:3].astype(np.float32)
        pts2 = coord2[0:3].astype(np.float32)
        print(cv2.getAffineTransform(pts1, pts2))
        return cv2.getAffineTransform(pts1, pts2)

    def loadCameraCalibration(self, file):
        """!
        @brief      Load camera intrinsic matrix from file.

                    TODO: use this to load in any calibration files you need to

        @param      file  The file
        """
        pass

    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """
        pass

    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth

                    TODO: Implement a blob detector to find blocks in the depth image
        """
        pass

    def projectGridInRGBImage(self):
        """!
        @brief      projects

                    TODO: Use the intrinsic and extrinsic matricies to project the gridpoints
                    on the board into pixel coordinates. copy self.VideoFrame to self.GridFrame
                    and draw on self.GridFrame the grid intersection points from self.grid_points
                    (hint: use the cv2.circle function to draw circles on the image)
        """
        modified_image = self.VideoFrame.copy()
        # Write your code here

        for x in range(len(self.grid_points[0])):
            for y in range(len(self.grid_points[0][0])):
                pt = [self.grid_points[0][x][y], self.grid_points[1][x][y]]
                world_pt = np.array([[pt[0]], [pt[1]], [0.0], [1.0]])
                camera_pt = np.matmul(self.extrinsic_matrix, world_pt)
                camera_pt = np.array([[camera_pt[0] / camera_pt[2]], [camera_pt[1] / camera_pt[2]], [1.0]])
                pixel_pt = np.matmul(self.intrinsic_matrix, camera_pt)

                cv2.circle(modified_image, (int(pixel_pt[0]), int(pixel_pt[1])), 3, (0, 255, 0), -1)

        self.GridFrame = modified_image

    def drawTagsInRGBImage(self, msg):
        """
        @brief      Draw tags from the tag detection

                    TODO: Use the tag detections output, to draw the corners/center/tagID of
                    the apriltags on the copy of the RGB image. And output the video to self.TagImageFrame.
                    Message type can be found here: /opt/ros/humble/share/apriltag_msgs/msg

                    center of the tag: (detection.centre.x, detection.centre.y) they are floats
                    id of the tag: detection.id
        """
        modified_image = self.VideoFrame.copy()
        # Write your code here

        for detection in self.tag_detections.detections:
            center = (int(detection.centre.x), int(detection.centre.y))
            cv2.circle(modified_image, center, 3, (0, 255, 0), -1)

            corners = detection.corners
            for i in range(4):
                cv2.line(modified_image, (int(corners[i].x), int(corners[i].y)),
                         (int(corners[(i + 1) % 4].x), int(corners[(i + 1) % 4].y)), (0, 0, 255), 3)

            id = str(detection.id)
            cv2.putText(modified_image, "ID: " + id, (int(detection.centre.x), int(detection.centre.y - 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2)

        self.TagImageFrame = modified_image


class ImageListener(Node):
    def __init__(self, topic, camera):
        super().__init__('image_listener')
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, topic, self.callback, 10)
        self.camera = camera

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)

        self.camera.VideoFrame = cv_image


class TagDetectionListener(Node):
    def __init__(self, topic, camera):
        super().__init__('tag_detection_listener')
        self.topic = topic
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            topic,
            self.callback,
            10
        )
        self.camera = camera

    def callback(self, msg):
        self.camera.tag_detections = msg
        if np.any(self.camera.VideoFrame != 0):
            self.camera.drawTagsInRGBImage(msg)


class CameraInfoListener(Node):
    def __init__(self, topic, camera):
        super().__init__('camera_info_listener')
        self.topic = topic
        self.tag_sub = self.create_subscription(CameraInfo, topic, self.callback, 10)
        self.camera = camera

    def callback(self, data):
        self.camera.intrinsic_matrix = np.reshape(data.k, (3, 3))
        # self.camera.intrinsic_matrix = np.matrix([[917.426899, 0.0, 609.600747],
        #                                [0.0, 913.808287, 390.655705],
        #                                [0.0, 0.0, 1.0]])
        # print(self.camera.intrinsic_matrix)


class DepthListener(Node):
    def __init__(self, topic, camera):
        super().__init__('depth_listener')
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, topic, self.callback, 10)
        self.camera = camera

    def callback(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # cv_depth = cv2.rotate(cv_depth, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)
        self.camera.DepthFrameRaw = cv_depth
        # self.camera.DepthFrameRaw = self.camera.DepthFrameRaw / 2
        self.camera.ColorizeDepthFrame()


class VideoThread(QThread):
    updateFrame = pyqtSignal(QImage, QImage, QImage, QImage)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent)
        self.camera = camera
        image_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
        camera_info_topic = "/camera/color/camera_info"
        tag_detection_topic = "/detections"
        image_listener = ImageListener(image_topic, self.camera)
        depth_listener = DepthListener(depth_topic, self.camera)
        camera_info_listener = CameraInfoListener(camera_info_topic,
                                                  self.camera)
        tag_detection_listener = TagDetectionListener(tag_detection_topic,
                                                      self.camera)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(image_listener)
        self.executor.add_node(depth_listener)
        self.executor.add_node(camera_info_listener)
        self.executor.add_node(tag_detection_listener)

    def run(self):
        if __name__ == '__main__':
            cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Tag window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Grid window", cv2.WINDOW_NORMAL)
            time.sleep(0.5)
        try:
            while rclpy.ok():
                start_time = time.time()
                rgb_frame = self.camera.convertQtVideoFrame()
                depth_frame = self.camera.convertQtDepthFrame()
                tag_frame = self.camera.convertQtTagImageFrame()
                self.camera.projectGridInRGBImage()
                grid_frame = self.camera.convertQtGridFrame()
                if ((rgb_frame != None) & (depth_frame != None)):
                    self.updateFrame.emit(
                        rgb_frame, depth_frame, tag_frame, grid_frame)
                self.executor.spin_once()  # comment this out when run this file alone.
                elapsed_time = time.time() - start_time
                sleep_time = max(0.03 - elapsed_time, 0)
                time.sleep(sleep_time)

                if __name__ == '__main__':
                    cv2.imshow(
                        "Image window",
                        cv2.cvtColor(self.camera.VideoFrame, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Depth window", self.camera.DepthFrameRGB)
                    cv2.imshow(
                        "Tag window",
                        cv2.cvtColor(self.camera.TagImageFrame, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Grid window",
                               cv2.cvtColor(self.camera.GridFrame, cv2.COLOR_RGB2BGR))
                    cv2.waitKey(3)
                    time.sleep(0.03)
        except KeyboardInterrupt:
            pass

        self.executor.shutdown()


def main(args=None):
    rclpy.init(args=args)
    try:
        camera = Camera()
        videoThread = VideoThread(camera)
        videoThread.start()
        try:
            videoThread.executor.spin()
        finally:
            videoThread.executor.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()