#!/usr/bin/env python3
"""
ROS 2 Camera Node for Pi Dog
Captures and publishes camera images and face detection
Uses standard ROS 2 message types (no custom imports required)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, ReliabilityPolicy, HistoryPolicy

# Standard ROS 2 imports instead of custom ones
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np
import time
import threading
import os


class PiDogCameraNode(Node):
    def __init__(self):
        super().__init__('pidog_camera_node')
        
        # Parameters
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('enable_face_detection', True)
        self.declare_parameter('save_photos', False)
        self.declare_parameter('photo_save_path', '/home/ros/Pictures/pidog/')
        
        self.camera_device = self.get_parameter('camera_device').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value
        self.enable_face_detection = self.get_parameter('enable_face_detection').value
        self.save_photos = self.get_parameter('save_photos').value
        self.photo_path = self.get_parameter('photo_save_path').value
        
        # Create save directory if needed
        if self.save_photos and not os.path.exists(self.photo_path):
            os.makedirs(self.photo_path)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # ROS 2 Publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
           # reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Change from RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.image_pub = self.create_publisher(
            Image,
            'camera/image_raw',
            qos_profile
        )

        qos_profile_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Face detection publisher using String (format: "count:x1,y1,w1,h1:x2,y2,w2,h2...")
        self.face_pub = self.create_publisher(
            String,
            'face_detection',
            qos_profile_best_effort  # Change from RELIABLE to BEST_EFFORT
        )
        
        # Initialize camera
        self.cap = None
        self.face_cascade = None
        
        if self.enable_face_detection:
            # Load OpenCV face cascade
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            if self.face_cascade.empty():
                self.get_logger().warning("Failed to load face cascade classifier")
                self.face_cascade = None
            else:
                self.get_logger().info("Face cascade classifier loaded successfully")
        
        self.init_camera()
        
        # Start camera capture thread
        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        
        self.get_logger().info("PiDog Camera Node Ready")
        self.get_logger().info(f"Publishing to: camera/image_raw and face_detection")
    
    def init_camera(self):
        """Initialize camera capture"""
        try:
            # Try Vilib first (from original code)
            try:
                from vilib import Vilib
                self.use_vilib = True
                #Vilib.camera_start(vflip=False, hflip=False)
                Vilib.camera_start(vflip=False, hflip=False, size=(1280, 720))
                Vilib.show_fps()
                Vilib.display(local=False,web=True)
                time.sleep(1)  # give camera time to warm up
                if self.enable_face_detection:
                    Vilib.face_detect_switch(True)
                self.get_logger().info("Camera initialized with Vilib")
                return
            except ImportError:
                self.get_logger().info("Vilib not available, falling back to OpenCV")
                pass
            
            # Fallback to OpenCV
            self.use_vilib = False
            self.cap = cv2.VideoCapture(self.camera_device)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            if not self.cap.isOpened():
                self.get_logger().error("Failed to open camera")
                self.cap = None
            else:
                self.get_logger().info(f"Camera initialized with OpenCV (device {self.camera_device})")
                
        except Exception as e:
            self.get_logger().error(f"Camera initialization error: {e}")
            self.cap = None
    
    def capture_frame_vilib(self):
        """Capture frame using Vilib"""
        try:
            from vilib import Vilib
            
            # Vilib provides frame in memory
            frame = Vilib.get_frame()
            
            if frame is not None:
                # Get face detection data
                if self.enable_face_detection:
                    face_count = Vilib.detect_obj_parameter.get('human_n', 0)
                    faces = []
                    
                    for i in range(face_count):
                        x = Vilib.detect_obj_parameter.get(f'human_{i}_x', 0)
                        y = Vilib.detect_obj_parameter.get(f'human_{i}_y', 0)
                        w = Vilib.detect_obj_parameter.get(f'human_{i}_w', 0)
                        h = Vilib.detect_obj_parameter.get(f'human_{i}_h', 0)
                        faces.append((x, y, w, h))
                    
                    return frame, faces
                
                return frame, []
                
        except Exception as e:
            self.get_logger().debug(f"Vilib capture error: {e}")
            return None, []
    
    def capture_frame_opencv(self):
        """Capture frame using OpenCV"""
        if self.cap is None:
            return None, []
        
        ret, frame = self.cap.read()
        if not ret:
            return None, []
        
        # Detect faces
        faces = []
        if self.face_cascade is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detected = self.face_cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
            )
            faces = [(x, y, w, h) for (x, y, w, h) in detected]
        
        return frame, faces
    
    def format_face_message(self, faces):
        """Format face detection data as String message
        Format: "count:x1,y1,w1,h1:x2,y2,w2,h2:..."
        """
        if not faces:
            return "0"
        
        parts = [str(len(faces))]
        for x, y, w, h in faces:
            parts.append(f"{x},{y},{w},{h}")
        
        return ":".join(parts)
    
    def capture_loop(self):
        """Main capture loop"""
        frame_interval = 1.0 / self.fps
        frame_count = 0
        
        self.get_logger().info("Camera capture loop started")
        
        while self.running and rclpy.ok():
            start_time = time.time()
            
            try:
                # Capture frame
                if hasattr(self, 'use_vilib') and self.use_vilib:
                    frame, faces = self.capture_frame_vilib()
                else:
                    frame, faces = self.capture_frame_opencv()
                
                if frame is not None:
                    # Resize if needed
                    if frame.shape[1] != self.image_width or frame.shape[0] != self.image_height:
                        frame = cv2.resize(frame, (self.image_width, self.image_height))
                    
                    # Publish image
                    ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    ros_image.header.frame_id = 'camera_frame'
                    self.image_pub.publish(ros_image)
                    
                    # Publish face detection as String
                    if self.enable_face_detection:
                        face_msg = String()
                        face_msg.data = self.format_face_message(faces)
                        self.face_pub.publish(face_msg)
                        
                        if faces and frame_count % 30 == 0:  # Log every 30 frames
                            self.get_logger().info(f"Detected {len(faces)} face(s)")
                    
                    # Save photo periodically
                    if self.save_photos and frame_count % (self.fps * 10) == 0:  # Every 10 seconds
                        timestamp = time.strftime('%Y%m%d_%H%M%S')
                        filename = f"pidog_capture_{timestamp}.jpg"
                        filepath = os.path.join(self.photo_path, filename)
                        cv2.imwrite(filepath, frame)
                        self.get_logger().debug(f"Photo saved: {filename}")
                    
                    frame_count += 1
                
            except Exception as e:
                self.get_logger().error(f"Frame capture error: {e}")
            
            # Maintain frame rate
            elapsed = time.time() - start_time
            sleep_time = frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        self.get_logger().info("Camera capture loop ended")
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down camera node...")
        self.running = False
        
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        
        if hasattr(self, 'use_vilib') and self.use_vilib:
            try:
                from vilib import Vilib
                Vilib.camera_close()
                self.get_logger().info("Vilib camera closed")
            except Exception as e:
                self.get_logger().debug(f"Error closing Vilib: {e}")
        elif self.cap:
            self.cap.release()
            self.get_logger().info("OpenCV camera released")
        
        self.get_logger().info("Camera node shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    
    node = PiDogCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera node interrupted")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():  # Check if already shutdown
            rclpy.shutdown()


if __name__ == '__main__':
    main()
