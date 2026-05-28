#!/usr/bin/env python3
"""
Phase 3 — Red cube perception node for fr3_delivery_sim.

Subscribes to /camera/image, isolates the red cube using HSV colour
thresholding, and publishes the detected pixel centroid as a geometry_msgs/Point
on /detected_block/pixel so later nodes can consume it without tight coupling.

Run in a separate terminal while the simulation is already running:
    ros2 run fr3_delivery_sim vision_detector.py
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


# ── HSV thresholds for red ────────────────────────────────────────────────────
# Red wraps around the hue wheel, so we need two ranges and OR them together.
# Tune these if Gazebo's material renders at a slightly different hue.
LOWER_RED_1 = np.array([  0, 120,  70])   # H=0–10
UPPER_RED_1 = np.array([ 10, 255, 255])

LOWER_RED_2 = np.array([160, 120,  70])   # H=160–179
UPPER_RED_2 = np.array([179, 255, 255])

# Minimum contour area (px²) — filters out single-pixel noise
MIN_CONTOUR_AREA = 200


class VisionDetector(Node):

    def __init__(self):
        super().__init__('vision_detector')

        # ── QoS must match the camera publisher (Best Effort) ────────────────
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.bridge = CvBridge()

        # ── Subscriber: raw RGB image from overhead camera ───────────────────
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self._image_callback,
            cam_qos,
        )

        # ── Publisher: pixel centroid of detected block ───────────────────────
        # Point.x = column (u), Point.y = row (v), Point.z = 0 (unused here)
        self.centroid_pub = self.create_publisher(Point, '/detected_block/pixel', 10)

        self.get_logger().info('Vision detector ready — waiting for camera frames...')

    # ── Main callback ─────────────────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        # 1. Convert ROS Image → OpenCV BGR matrix
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # 2. Blur slightly to reduce sensor noise before thresholding
        blurred = cv2.GaussianBlur(bgr, (5, 5), 0)

        # 3. Convert BGR → HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 4. Build a binary mask that is white wherever the pixel is red
        mask1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
        mask2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
        mask  = cv2.bitwise_or(mask1, mask2)

        # Optional morphological cleanup: fill holes, remove tiny specks
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 5. Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        debug_frame = bgr.copy()

        if contours:
            # Pick the largest contour (most likely the cube, not noise)
            largest = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest) >= MIN_CONTOUR_AREA:
                # 6. Compute the centroid using image moments
                M  = cv2.moments(largest)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # 7. Publish the pixel centroid
                pt      = Point()
                pt.x    = float(cx)
                pt.y    = float(cy)
                pt.z    = 0.0
                self.centroid_pub.publish(pt)

                self.get_logger().info(
                    f'Block detected — pixel centroid: u={cx}, v={cy}  '
                    f'(area={cv2.contourArea(largest):.0f} px²)'
                )

                # 8. Draw debug overlay on the frame
                cv2.drawContours(debug_frame, [largest], -1, (0, 255, 0), 2)
                cv2.circle(debug_frame, (cx, cy), 6, (0, 0, 255), -1)   # red dot
                cv2.putText(
                    debug_frame,
                    f'u={cx}  v={cy}',
                    (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                )
            else:
                self.get_logger().warn('Red region too small — may be noise, skipping.')
        else:
            self.get_logger().warn('No red object detected in frame.', throttle_duration_sec=2.0)

        # 9. Show the live debug windows
        cv2.imshow('Camera Feed', debug_frame)
        cv2.imshow('Red Mask',    mask)
        cv2.waitKey(1)   # non-blocking — just pumps the OpenCV event loop

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()