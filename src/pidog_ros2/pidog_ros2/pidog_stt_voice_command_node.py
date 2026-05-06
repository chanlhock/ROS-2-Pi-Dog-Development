#!/usr/bin/env python3
"""
ROS 2 STT (Speech-to-Text) Voice Command Node for Pi Dog
Publishes recognized commands to voice_command topic
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

import json
import time
import threading
import os
import sys


class PiDogSTTVoiceCommandNode(Node):
    def __init__(self):
        super().__init__('pidog_stt_voice_command_node')
        
        self.get_logger().info("STT node starting...")
        
        # Parameters
        self.declare_parameter('enabled', True)
        self.declare_parameter('model_path', '/pidog/woofer/vosk-model-small-en-us-0.15')
        self.declare_parameter('debug', False)
        
        self.enabled = self.get_parameter('enabled').value
        self.model_path = self.get_parameter('model_path').value
        self.debug = self.get_parameter('debug').value
        
        # Publisher for recognized voice commands
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.voice_pub = self.create_publisher(String, 'voice_command', qos_profile)
        
        # Initialize Vosk
        self.recognizer = None
        self.stream = None
        self.pyaudio = None
        self.listening = False
        
        if self.enabled:
            self.init_vosk()
        
        if self.recognizer:
            self.get_logger().info("Voice commands will be published to /pidog/voice_command")
            self.get_logger().info("Try saying: sit, stand, walk, stop, turn left, turn right")
        else:
            self.get_logger().warning("STT not available - voice commands disabled")
    
    def init_vosk(self):
        """Initialize Vosk speech recognition."""
        try:
            import vosk
            import pyaudio
            
            # Check if model exists
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"Vosk model not found at {self.model_path}")
                self.get_logger().info("Please download from: https://alphacephei.com/vosk/models")
                return
            
            # Suppress Vosk logging
            vosk.SetLogLevel(-1)
            
            # Load model
            self.get_logger().info(f"Loading Vosk model from {self.model_path}...")
            model = vosk.Model(self.model_path)
            self.recognizer = vosk.KaldiRecognizer(model, 16000)
            
            # Initialize PyAudio
            self.pyaudio = pyaudio.PyAudio()
            
            # Find input device
            device_index = None
            for i in range(self.pyaudio.get_device_count()):
                dev_info = self.pyaudio.get_device_info_by_index(i)
                if dev_info['maxInputChannels'] > 0:
                    if 'default' in dev_info['name'].lower() or 'usb' in dev_info['name'].lower():
                        device_index = i
                        break
            
            if device_index is None:
                device_index = self.pyaudio.get_default_input_device_info()['index']
            
            device_name = self.pyaudio.get_device_info_by_index(device_index)['name']
            self.get_logger().info(f"Using audio device: {device_name}")
            
            # Open stream
            self.stream = self.pyaudio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=16000,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=4000
            )
            
            # Start listening thread
            self.listening = True
            self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
            self.listen_thread.start()
            
            self.get_logger().info("Vosk initialized successfully - listening for commands")
            
        except ImportError as e:
            self.get_logger().error(f"Required library not available: {e}")
            self.get_logger().error("Please install: pip3 install vosk pyaudio")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Vosk: {e}")
    
    def listen_loop(self):
        """Main listening loop."""
        self.get_logger().info("Listening for voice commands...")
        
        while rclpy.ok() and self.listening and self.recognizer and self.stream:
            try:
                data = self.stream.read(4000, exception_on_overflow=False)
                
                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(self.recognizer.Result())
                    text = result.get("text", "").strip()
                    
                    if text:
                        self.get_logger().info(f"🎤 Recognized: '{text}'")
                        
                        # Publish to voice_command topic
                        msg = String()
                        msg.data = text
                        self.voice_pub.publish(msg)
                        
            except Exception as e:
                if self.debug:
                    self.get_logger().debug(f"Listen error: {e}")
                time.sleep(0.1)
    
    def shutdown(self):
        """Clean shutdown."""
        self.get_logger().info("Shutting down STT node...")
        self.listening = False
        
        if self.stream:
            try:
                self.stream.stop_stream()
                self.stream.close()
            except:
                pass
        
        if self.pyaudio:
            try:
                self.pyaudio.terminate()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = PiDogSTTVoiceCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("STT node interrupted")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
