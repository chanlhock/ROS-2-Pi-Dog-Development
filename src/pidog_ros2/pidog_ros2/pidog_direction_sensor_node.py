#!/usr/bin/env python3
"""
ROS 2 Sound Direction Sensor Node for Pi Dog
Detects direction of sound using microphone array or stereo microphones
Uses standard ROS 2 message types (no custom imports required)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Standard ROS 2 imports instead of custom ones
from std_msgs.msg import String
from std_srvs.srv import SetBool

import time
import threading
import math
import queue
import random


class PiDogSoundDirectionNode(Node):
    def __init__(self):
        super().__init__('pidog_direction_sensor_node')
        
        # Parameters
        self.declare_parameter('enabled', True)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('sound_energy_threshold', 500)
        self.declare_parameter('mic_left_device', None)
        self.declare_parameter('mic_right_device', None)
        self.declare_parameter('simulate_sound', True)
        
        self.enabled = self.get_parameter('enabled').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.sound_threshold = self.get_parameter('sound_energy_threshold').value
        self.simulate = self.get_parameter('simulate_sound').value
        
        # Current sound direction
        self.current_angle = -1.0  # -1 = unknown
        self.current_direction = "unknown"
        self.current_confidence = 0.0
        self.last_sound_time = 0
        
        # Audio buffers
        self.left_buffer = queue.Queue(maxsize=10)
        self.right_buffer = queue.Queue(maxsize=10)
        
        # ROS 2 Service (using standard SetBool)
        self.enable_srv = self.create_service(
            SetBool,
            'enable_sound_direction',
            self.enable_sound_direction_callback
        )
        
        # ROS 2 Publisher (using String for direction info)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.direction_pub = self.create_publisher(
            String,
            'sound_direction',
            qos_profile
        )
        
        # Initialize audio
        self.audio_streams = []
        self.audio_running = False
        
        if self.enabled and not self.simulate:
            self.init_audio_hardware()
        
        # Start sound direction detection thread
        if self.enabled:
            self.detection_thread = threading.Thread(target=self.direction_detection_loop, daemon=True)
            self.detection_thread.start()
        
        # Start simulation if needed
        if self.simulate:
            self.sim_thread = threading.Thread(target=self.simulate_sound_loop, daemon=True)
            self.sim_thread.start()
        
        self.get_logger().info(f"PiDog Sound Direction Node Ready (enabled={self.enabled}, simulate={self.simulate})")
        self.get_logger().info(f"Publishing to: /sound_direction")
    
    def enable_sound_direction_callback(self, request, response):
        """Service callback to enable/disable sound direction detection"""
        self.enabled = request.data
        response.success = True
        response.message = f"Sound direction detection {'enabled' if self.enabled else 'disabled'}"
        
        self.get_logger().info(response.message)
        return response
    
    def init_audio_hardware(self):
        """Initialize audio hardware for direction detection"""
        try:
            import pyaudio
            
            self.pyaudio = pyaudio.PyAudio()
            
            # Get left mic device index
            left_mic = self.get_parameter('mic_left_device').value
            right_mic = self.get_parameter('mic_right_device').value
            
            # Use default if not specified
            if left_mic is None:
                default_input = self.pyaudio.get_default_input_device_info()
                left_mic = default_input['index']
                self.get_logger().info(f"Using default microphone device {left_mic}")
            
            # Open audio stream (stereo for direction detection)
            stream = self.pyaudio.open(
                format=pyaudio.paInt16,
                channels=2,  # Stereo for direction detection
                rate=self.sample_rate,
                input=True,
                input_device_index=left_mic,
                frames_per_buffer=self.chunk_size
            )
            
            self.audio_streams.append(stream)
            self.audio_running = True
            
            # Start audio capture thread
            self.capture_thread = threading.Thread(target=self.audio_capture_loop, daemon=True)
            self.capture_thread.start()
            
            self.get_logger().info("Audio hardware initialized for direction detection")
            
        except ImportError:
            self.get_logger().error("PyAudio not available, using simulation")
            self.simulate = True
        except Exception as e:
            self.get_logger().error(f"Failed to initialize audio: {e}")
            self.simulate = True
    
    def calculate_audio_energy(self, audio_data):
        """Calculate audio energy without numpy"""
        if len(audio_data) == 0:
            return 0
        
        # Calculate RMS energy
        total = 0
        for sample in audio_data:
            total += abs(sample)
        
        return total / len(audio_data)
    
    def audio_capture_loop(self):
        """Capture audio from microphones"""
        if not self.audio_streams:
            return
        
        stream = self.audio_streams[0]
        
        while self.audio_running and rclpy.ok():
            try:
                data = stream.read(self.chunk_size, exception_on_overflow=False)
                
                # Convert bytes to int16 list without numpy
                audio_data = []
                for i in range(0, len(data), 2):
                    if i + 1 < len(data):
                        value = int.from_bytes(data[i:i+2], byteorder='little', signed=True)
                        audio_data.append(value)
                
                # Split into left and right channels (interleaved stereo)
                left = []
                right = []
                for i, sample in enumerate(audio_data):
                    if i % 2 == 0:
                        left.append(sample)
                    else:
                        right.append(sample)
                
                # Add to buffers
                if not self.left_buffer.full():
                    self.left_buffer.put(left)
                if not self.right_buffer.full():
                    self.right_buffer.put(right)
                
            except Exception as e:
                self.get_logger().debug(f"Audio capture error: {e}")
    
    def calculate_sound_direction(self, left_signal, right_signal):
        """Calculate sound direction using cross-correlation (TDOA) without numpy"""
        if len(left_signal) == 0 or len(right_signal) == 0:
            return -1, 0.0
        
        # Calculate energy
        left_energy = self.calculate_audio_energy(left_signal)
        right_energy = self.calculate_audio_energy(right_signal)
        
        total_energy = left_energy + right_energy
        
        if total_energy < self.sound_threshold:
            return -1, 0.0  # No significant sound
        
        # Simple cross-correlation for time delay
        # Limit correlation range to reasonable values
        max_delay = min(50, len(left_signal) // 4)
        best_delay = 0
        best_correlation = -1
        
        for delay in range(-max_delay, max_delay + 1):
            correlation = 0
            count = 0
            
            if delay >= 0:
                for i in range(len(left_signal) - delay):
                    if i < len(right_signal):
                        correlation += left_signal[i] * right_signal[i + delay]
                        count += 1
            else:
                for i in range(len(right_signal) + delay):
                    if i < len(left_signal):
                        correlation += left_signal[i - delay] * right_signal[i]
                        count += 1
            
            if count > 0:
                correlation /= count
                if correlation > best_correlation:
                    best_correlation = correlation
                    best_delay = delay
        
        delay = best_delay
        
        # Convert delay to angle
        # delay = (d * sin(theta)) / c * fs
        # where d = microphone spacing (assume 0.1m), c = 343 m/s
        mic_spacing = 0.1  # meters
        speed_of_sound = 343  # m/s
        
        max_delay_samples = (mic_spacing / speed_of_sound) * self.sample_rate
        if max_delay_samples > 0:
            normalized_delay = delay / max_delay_samples
            normalized_delay = max(-1.0, min(1.0, normalized_delay))
        else:
            normalized_delay = 0
        
        # Calculate angle
        try:
            angle_rad = math.asin(normalized_delay)
            angle_deg = math.degrees(angle_rad)
        except ValueError:
            angle_deg = 0
        
        # Convert to 0-360 degrees (0 = front, 90 = right, 180 = back, 270 = left)
        if angle_deg < 0:
            angle_deg = 360 + angle_deg
        
        # Confidence based on correlation strength
        confidence = min(1.0, (best_correlation + 1) / 2)  # Normalize from [-1,1] to [0,1]
        
        return angle_deg, confidence
    
    def direction_to_text(self, angle):
        """Convert angle to direction text"""
        if angle < 0:
            return "unknown"
        
        if angle < 45 or angle >= 315:
            return "front"
        elif 45 <= angle < 135:
            return "right"
        elif 135 <= angle < 225:
            return "back"
        else:
            return "left"
    
    def format_direction_message(self):
        """Format direction as String message"""
        return f"{self.current_angle:.1f}:{self.current_direction}:{self.current_confidence:.2f}"
    
    def direction_detection_loop(self):
        """Main loop for direction detection"""
        self.get_logger().info("Starting sound direction detection loop...")
        
        while rclpy.ok():
            if not self.enabled or self.simulate:
                time.sleep(0.1)
                continue
            
            # Check if we have audio data
            if self.left_buffer.empty() or self.right_buffer.empty():
                time.sleep(0.01)
                continue
            
            try:
                left_data = self.left_buffer.get_nowait()
                right_data = self.right_buffer.get_nowait()
            except queue.Empty:
                continue
            
            # Calculate direction
            angle, confidence = self.calculate_sound_direction(left_data, right_data)
            
            if angle >= 0 and confidence > 0.3:
                self.current_angle = angle
                self.current_direction = self.direction_to_text(angle)
                self.current_confidence = confidence
                self.last_sound_time = time.time()
                
                # Publish only on significant detection
                if confidence > 0.5:
                    self.publish_direction()
            
            # Small sleep to prevent CPU overload
            time.sleep(0.01)
    
    def simulate_sound_loop(self):
        """Simulate sound direction for testing"""
        directions = ["front", "right", "back", "left"]
        angles = [0, 90, 180, 270]
        
        self.get_logger().info("Simulation mode active - generating random sound directions")
        
        while rclpy.ok():
            if not self.enabled:
                time.sleep(0.5)
                continue
            
            # Randomly simulate sound every 5-15 seconds
            time.sleep(random.uniform(5, 15))
            
            idx = random.randint(0, 3)
            self.current_direction = directions[idx]
            self.current_angle = angles[idx]
            self.current_confidence = random.uniform(0.6, 0.95)
            self.last_sound_time = time.time()
            
            self.get_logger().info(f"🎵 Simulated sound from {self.current_direction}")
            self.publish_direction()
    
    def publish_direction(self):
        """Publish sound direction message"""
        # Only publish if recent (within last 2 seconds)
        if time.time() - self.last_sound_time > 2.0:
            return
        
        msg = String()
        msg.data = self.format_direction_message()
        self.direction_pub.publish(msg)
        
        self.get_logger().info(
            f"Sound direction: {self.current_direction} ({self.current_angle:.0f}°), "
            f"confidence={self.current_confidence:.2f}"
        )
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down sound direction node...")
        self.audio_running = False
        
        for stream in self.audio_streams:
            if stream:
                try:
                    stream.stop_stream()
                    stream.close()
                except Exception as e:
                    self.get_logger().debug(f"Error closing stream: {e}")
        
        if hasattr(self, 'pyaudio'):
            try:
                self.pyaudio.terminate()
                self.get_logger().info("PyAudio terminated")
            except Exception as e:
                self.get_logger().debug(f"Error terminating PyAudio: {e}")
        
        self.get_logger().info("Sound direction node shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    
    node = PiDogSoundDirectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Sound direction node interrupted")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
